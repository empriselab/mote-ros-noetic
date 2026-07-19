#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>

#include <array>
#include <boost/asio.hpp>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <nlohmann/json.hpp>
#include <string>
#include <thread>
#include <vector>

#include "mote-ffi/src/mote_cxx.rs.h"
#include "mote_base/messages.h"
#include "mote_base/scan_rasterizer.h"

using json = nlohmann::json;
using mote_base::ScanRasterizer;

namespace {
constexpr int UDP_PORT = 7475;
constexpr int UDP_BUF_SIZE = 65536;
constexpr std::array<const char *, 2> kJointNames = {"left_wheel", "right_wheel"};
}  // namespace

class MoteHardwareInterface : public hardware_interface::RobotHW {
 public:
  MoteHardwareInterface() : link_(mote::new_mote_link()) {}

  MoteHardwareInterface(const MoteHardwareInterface &) = delete;
  MoteHardwareInterface &operator=(const MoteHardwareInterface &) = delete;
  // Move is unsafe: hardware_interface::JointStateHandle/JointHandle capture
  // raw pointers into pos_/vel_/eff_/cmd_ at registerHandle() time below, so
  // moving this object would invalidate those pointers.
  MoteHardwareInterface(MoteHardwareInterface &&) = delete;
  MoteHardwareInterface &operator=(MoteHardwareInterface &&) = delete;

  ~MoteHardwareInterface() override {
    // Closing the socket cancels the outstanding async_receive with
    // operation_aborted, which stops start_receive() from rearming itself,
    // letting io_ctx_.run() drain naturally.
    boost::system::error_code ec;
    socket_.close(ec);
    io_ctx_.stop();
    if (io_thread_.joinable()) io_thread_.join();
  }

  bool init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh) {
    std::string robot_ip;
    if (!robot_hw_nh.getParam("robot_ip", robot_ip)) {
      ROS_FATAL("mote_node: ~robot_ip parameter is required");
      return false;
    }
    robot_hw_nh.param<std::string>("laser_frame", laser_frame_, "laser");
    robot_hw_nh.param<std::string>("imu_frame", imu_frame_, "imu_link");

    // Register hardware interfaces for left_wheel and right_wheel.
    // Note: JointStateHandle/JointHandle require pointers to persistent
    // storage -- a hardware_interface API constraint, not something to
    // convert to references.
    for (std::size_t i = 0; i < kJointNames.size(); ++i) {
      jnt_state_iface_.registerHandle(
          hardware_interface::JointStateHandle(kJointNames[i], &pos_[i], &vel_[i], &eff_[i]));
      jnt_vel_iface_.registerHandle(
          hardware_interface::JointHandle(jnt_state_iface_.getHandle(kJointNames[i]), &cmd_[i]));
    }
    registerInterface(&jnt_state_iface_);
    registerInterface(&jnt_vel_iface_);

    // Open a UDP socket and connect to the robot.
    try {
      boost::asio::ip::udp::resolver resolver(io_ctx_);
      const auto endpoints =
          resolver.resolve(boost::asio::ip::udp::v4(), robot_ip, std::to_string(UDP_PORT));
      socket_.open(boost::asio::ip::udp::v4());
      socket_.connect(*endpoints.begin());
    } catch (const boost::system::system_error &e) {
      ROS_FATAL("mote_node: failed to connect to %s:%d: %s", robot_ip.c_str(), UDP_PORT, e.what());
      return false;
    }

    // Advertise sensor topics (joint_states and odom published by
    // diff_drive_controller)
    scan_pub_ = root_nh.advertise<sensor_msgs::LaserScan>("scan", 1);
    imu_pub_ = root_nh.advertise<sensor_msgs::Imu>("imu/data", 1);

    // Send a keepalive Ping every second
    keepalive_timer_ =
        root_nh.createTimer(ros::Duration(1.0), &MoteHardwareInterface::keepalive_cb, this);

    // Start the background UDP receive loop.
    start_receive();
    io_thread_ = std::thread([this] { io_ctx_.run(); });

    ROS_INFO("mote_node: connected to %s:%d", robot_ip.c_str(), UDP_PORT);
    return true;
  }

  // Called at the start of each control loop iteration.
  // Drains decoded messages from the link, updates joint state buffers,
  // and publishes sensor topics.
  void read(const ros::Time &time, const ros::Duration & /*period*/) override {
    while (true) {
      mote::ReceiveResult result;
      {
        std::lock_guard<std::mutex> lk(link_mutex_);
        result = link_->poll_receive();
      }
      if (result.error != mote::MoteLinkErrorCode::None) {
        // Corrupt or undeserializable frame — already consumed from the queue.
        // Continue draining rather than dropping all subsequent messages.
        ROS_WARN_THROTTLE(5.0, "mote_node: poll_receive error: %s", result.error_message.c_str());
        continue;
      }
      if (result.json_message.empty()) break;
      try {
        dispatch(json::parse(result.json_message.c_str()), time);
      } catch (const json::exception &e) {
        ROS_WARN_THROTTLE(5.0, "mote_node: JSON parse error: %s", e.what());
      }
    }
  }

  // Called after cm.update(). Sends the velocity commands set by the
  // controller.
  void write(const ros::Time & /*time*/, const ros::Duration & /*period*/) override {
    json cmd = {{"SetDriveBaseVelocity",
                 {{"left_velocity_rad_per_s", cmd_[0]}, {"right_velocity_rad_per_s", cmd_[1]}}}};
    const std::string cmd_str = cmd.dump();

    std::lock_guard<std::mutex> lk(link_mutex_);

    // Reply to any Ping received during read()
    if (pending_pong_) {
      link_->send("\"Pong\"");
      pending_pong_ = false;
    }

    mote::SendResult result = link_->send(cmd_str);
    if (result.error != mote::MoteLinkErrorCode::None)
      ROS_WARN_THROTTLE(5.0, "mote_node: send failed: %s", result.error_message.c_str());

    flush_transmit();
  }

 private:
  // Joint state buffers indexed [0]=left_wheel, [1]=right_wheel
  std::array<double, 2> pos_{};
  std::array<double, 2> vel_{};
  std::array<double, 2> eff_{};
  std::array<double, 2> cmd_{};  // written by diff_drive_controller via VelocityJointInterface

  hardware_interface::JointStateInterface jnt_state_iface_;
  hardware_interface::VelocityJointInterface jnt_vel_iface_;

  ::rust::Box<::mote::MoteLink> link_;
  std::mutex link_mutex_;
  bool pending_pong_ = false;  // set in read(), cleared in write()

  boost::asio::io_context io_ctx_;
  boost::asio::ip::udp::socket socket_{io_ctx_};
  std::thread io_thread_;
  std::array<std::uint8_t, UDP_BUF_SIZE> recv_buf_{};

  ros::Publisher scan_pub_;
  ros::Publisher imu_pub_;
  ros::Timer keepalive_timer_;
  std::string laser_frame_;
  std::string imu_frame_;

  ScanRasterizer scan_rasterizer_;
  ros::Time scan_accum_stamp_;

  // Drain all pending transmit packets from the link and send over UDP.
  // Must be called with link_mutex_ held.
  void flush_transmit() {
    while (true) {
      ::rust::Vec<std::uint8_t> pkt = link_->poll_transmit();
      if (pkt.empty()) break;
      boost::system::error_code ec;
      socket_.send(boost::asio::buffer(pkt.data(), pkt.size()), 0, ec);
      if (ec) ROS_WARN_THROTTLE(5.0, "mote_node: UDP send() failed: %s", ec.message().c_str());
    }
  }

  // Rearms itself after every successful (or recoverably-failed) receive.
  // Stops rearming once the socket is closed (operation_aborted), which is
  // how shutdown terminates this chain from the destructor.
  void start_receive() {
    socket_.async_receive(
        boost::asio::buffer(recv_buf_), [this](const boost::system::error_code &ec, std::size_t n) {
          if (ec == boost::asio::error::operation_aborted) return;
          if (!ec) {
            std::lock_guard<std::mutex> lk(link_mutex_);
            link_->handle_receive(::rust::Slice<const std::uint8_t>(recv_buf_.data(), n));
          } else {
            ROS_WARN_THROTTLE(5.0, "mote_node: async_receive error: %s", ec.message().c_str());
          }
          start_receive();
        });
  }

  // Sends a Ping to the robot once per second to keep the link alive.
  void keepalive_cb(const ros::TimerEvent &) {
    std::lock_guard<std::mutex> lk(link_mutex_);
    link_->send("\"Ping\"");
    flush_transmit();
  }

  // Route a decoded JSON message to the appropriate handler.
  void dispatch(const json &msg, const ros::Time &stamp) {
    if (msg.is_string()) {
      // Ping from robot: queue a Pong reply (sent in write())
      if (msg.get<std::string>() == "Ping") pending_pong_ = true;
      return;
    }
    if (!msg.is_object()) return;

    if (msg.contains("DriveBaseState")) update_joint_state(msg["DriveBaseState"]);
    if (msg.contains("Scan")) publish_scan(msg["Scan"], stamp);
    if (msg.contains("ImuMeasurement")) publish_imu(msg["ImuMeasurement"], stamp);
    if (msg.contains("State"))
      ROS_DEBUG_STREAM("mote_node: device state update: " << msg["State"].dump());
  }

  // Update joint state buffers from a DriveBaseState message.
  void update_joint_state(const json &state) {
    const mote_base::DriveBaseState s = mote_base::parse_drive_base_state(state);
    pos_[0] = s.left_position_rad;
    vel_[0] = s.left_velocity_rad_per_s;
    eff_[0] = s.left_effort_percent;
    pos_[1] = s.right_position_rad;
    vel_[1] = s.right_velocity_rad_per_s;
    eff_[1] = s.right_effort_percent;
  }

  // Feed each point of a Scan message into the rasterizer and publish one
  // LaserScan per full rotation.
  void publish_scan(const json &points, const ros::Time &stamp) {
    for (const auto &raw_pt : mote_base::parse_scan_points(points)) {
      auto completed = scan_rasterizer_.add_point(raw_pt);
      if (completed) scan_pub_.publish(build_laser_scan_msg(*completed, scan_accum_stamp_));
      if (scan_rasterizer_.size() == 1) scan_accum_stamp_ = stamp;
    }
  }

  sensor_msgs::LaserScan build_laser_scan_msg(const std::vector<float> &ranges,
                                              const ros::Time &stamp) const {
    sensor_msgs::LaserScan msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = laser_frame_;
    msg.angle_min = 0.0f;
    msg.angle_max = 2.0f * static_cast<float>(M_PI) - ScanRasterizer::kScanAngleInc;
    msg.angle_increment = ScanRasterizer::kScanAngleInc;
    msg.range_min = ScanRasterizer::kRangeMin;
    msg.range_max = ScanRasterizer::kRangeMax;
    msg.ranges = ranges;
    return msg;
  }

  // Publish an IMU message. Orientation is unknown (covariance[0] = -1 per
  // REP-145).
  void publish_imu(const json &imu, const ros::Time &stamp) {
    const mote_base::ImuMeasurement m = mote_base::parse_imu_measurement(imu);

    sensor_msgs::Imu msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = imu_frame_;

    msg.orientation_covariance[0] = -1.0;  // orientation unknown

    msg.linear_acceleration.x = m.accel_x;
    msg.linear_acceleration.y = m.accel_y;
    msg.linear_acceleration.z = m.accel_z;

    msg.angular_velocity.x = m.gyro_x;
    msg.angular_velocity.y = m.gyro_y;
    msg.angular_velocity.z = m.gyro_z;

    imu_pub_.publish(msg);
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "mote_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  MoteHardwareInterface robot;
  if (!robot.init(nh, pnh)) {
    ROS_FATAL("mote_node: initialization failed, shutting down");
    return 1;
  }

  controller_manager::ControllerManager cm(&robot, nh);

  // Process ROS callbacks (CM service calls, keepalive timer, etc.) in a
  // background thread.  This prevents controller_manager::loadController()'s
  // internal double-buffer busy-wait from blocking the cm.update() calls below.
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Rate rate(50.0);
  ros::Time last = ros::Time::now();

  while (ros::ok()) {
    const ros::Time now = ros::Time::now();
    const ros::Duration dt = now - last;

    robot.read(now, dt);
    cm.update(now, dt);
    robot.write(now, dt);

    last = now;
    rate.sleep();
  }

  spinner.stop();
  return 0;
}
