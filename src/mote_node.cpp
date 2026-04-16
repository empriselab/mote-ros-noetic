#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>

extern "C" {
#include <mote_link.h>
}

#include <nlohmann/json.hpp>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <cmath>
#include <cstring>
#include <limits>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

using json = nlohmann::json;

namespace {
constexpr int SCAN_BINS = 720;
constexpr float SCAN_ANGLE_INC = 2.0f * static_cast<float>(M_PI) / SCAN_BINS;
constexpr int SCAN_ACCUM_MIN =
    800; // publish once this many points are buffered
constexpr int UDP_PORT = 7475;
constexpr int UDP_BUF_SIZE = 65536;
constexpr int LINK_BUF_SIZE = 65536;
constexpr int JSON_BUF_SIZE = 131072;
} // namespace

class MoteHardwareInterface : public hardware_interface::RobotHW {
public:
  MoteHardwareInterface() = default;

  ~MoteHardwareInterface() override {
    running_.store(false);
    if (recv_thread_.joinable())
      recv_thread_.join();
    if (udp_fd_ >= 0) {
      ::close(udp_fd_);
      udp_fd_ = -1;
    }
    if (link_) {
      mote_link_free(link_);
      link_ = nullptr;
    }
  }

  bool init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh) {
    std::string robot_ip;
    if (!robot_hw_nh.getParam("robot_ip", robot_ip)) {
      ROS_FATAL("mote_node: ~robot_ip parameter is required");
      return false;
    }
    robot_hw_nh.param<std::string>("laser_frame", laser_frame_, "laser");
    robot_hw_nh.param<std::string>("imu_frame", imu_frame_, "imu_link");

    // Register hardware interfaces for left_wheel and right_wheel
    const char *joint_names[2] = {"left_wheel", "right_wheel"};
    for (int i = 0; i < 2; ++i) {
      jnt_state_iface_.registerHandle(hardware_interface::JointStateHandle(
          joint_names[i], &pos_[i], &vel_[i], &eff_[i]));
      jnt_vel_iface_.registerHandle(hardware_interface::JointHandle(
          jnt_state_iface_.getHandle(joint_names[i]), &cmd_[i]));
    }
    registerInterface(&jnt_state_iface_);
    registerInterface(&jnt_vel_iface_);

    // Allocate mote-ffi link handle
    link_ = mote_link_new();
    if (!link_) {
      ROS_FATAL("mote_node: mote_link_new() failed");
      return false;
    }

    // Open UDP socket and connect to robot
    udp_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_fd_ < 0) {
      ROS_FATAL("mote_node: socket() failed: %s", std::strerror(errno));
      return false;
    }
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(UDP_PORT));
    if (inet_pton(AF_INET, robot_ip.c_str(), &addr.sin_addr) != 1) {
      ROS_FATAL("mote_node: invalid robot_ip '%s'", robot_ip.c_str());
      return false;
    }
    if (::connect(udp_fd_, reinterpret_cast<const sockaddr *>(&addr),
                  sizeof(addr)) < 0) {
      ROS_FATAL("mote_node: connect() to %s:%d failed: %s", robot_ip.c_str(),
                UDP_PORT, std::strerror(errno));
      return false;
    }

    // Advertise sensor topics (joint_states and odom published by
    // diff_drive_controller)
    scan_pub_ = root_nh.advertise<sensor_msgs::LaserScan>("scan", 1);
    imu_pub_ = root_nh.advertise<sensor_msgs::Imu>("imu/data", 1);

    // Send a keepalive Ping every second
    keepalive_timer_ = root_nh.createTimer(
        ros::Duration(1.0), &MoteHardwareInterface::keepalive_cb, this);

    // Start background UDP receive thread
    running_.store(true);
    recv_thread_ = std::thread(&MoteHardwareInterface::recv_thread_fn, this);

    ROS_INFO("mote_node: connected to %s:%d", robot_ip.c_str(), UDP_PORT);
    return true;
  }

  // Called at the start of each control loop iteration.
  // Drains decoded messages from the link, updates joint state buffers,
  // and publishes sensor topics.
  void read(const ros::Time &time, const ros::Duration & /*period*/) override {
    std::vector<char> buf(JSON_BUF_SIZE);
    while (true) {
      int n;
      {
        std::lock_guard<std::mutex> lk(link_mutex_);
        n = mote_link_poll_receive(link_, buf.data(),
                                   static_cast<int>(buf.size()));
      }
      if (n == 0)
        break;
      if (n < 0) {
        // Corrupt or undeserializable frame — already consumed from the queue.
        // Continue draining rather than dropping all subsequent messages.
        ROS_WARN_THROTTLE(5.0,
                          "mote_node: dropping corrupt frame from mote_link");
        continue;
      }
      try {
        dispatch(json::parse(buf.data()), time);
      } catch (const json::exception &e) {
        ROS_WARN_THROTTLE(5.0, "mote_node: JSON parse error: %s", e.what());
      }
    }
  }

  // Called after cm.update(). Sends the velocity commands set by the
  // controller.
  void write(const ros::Time & /*time*/,
             const ros::Duration & /*period*/) override {
    json cmd = {
        {"DriveBaseCommand",
         {{"left_velocity_rad", cmd_[0]}, {"right_velocity_rad", cmd_[1]}}}};
    const std::string cmd_str = cmd.dump();

    std::lock_guard<std::mutex> lk(link_mutex_);

    // Reply to any Ping received during read()
    if (pending_pong_) {
      mote_link_send(link_, "\"Pong\"");
      pending_pong_ = false;
    }

    if (mote_link_send(link_, cmd_str.c_str()) < 0)
      ROS_WARN_THROTTLE(5.0, "mote_node: mote_link_send failed");

    flush_transmit();
  }

private:
  // Joint state buffers indexed [0]=left_wheel, [1]=right_wheel
  double pos_[2] = {0.0, 0.0};
  double vel_[2] = {0.0, 0.0};
  double eff_[2] = {0.0, 0.0};
  double cmd_[2] = {
      0.0, 0.0}; // written by diff_drive_controller via VelocityJointInterface

  hardware_interface::JointStateInterface jnt_state_iface_;
  hardware_interface::VelocityJointInterface jnt_vel_iface_;

  MoteLinkHandle *link_ = nullptr;
  std::mutex link_mutex_;
  bool pending_pong_ = false; // set in read(), cleared in write()

  int udp_fd_ = -1;
  std::thread recv_thread_;
  std::atomic<bool> running_{false};

  ros::Publisher scan_pub_;
  ros::Publisher imu_pub_;
  ros::Timer keepalive_timer_;
  std::string laser_frame_;
  std::string imu_frame_;

  struct RawScanPoint {
    float angle_rad;
    float distance_mm;
  };
  std::vector<RawScanPoint> scan_accum_;
  ros::Time scan_accum_stamp_;

  // Drain all pending transmit packets from the link and send over UDP.
  // Must be called with link_mutex_ held.
  void flush_transmit() {
    std::vector<uint8_t> pkt(LINK_BUF_SIZE);
    int n;
    while ((n = mote_link_poll_transmit(link_, pkt.data(),
                                        static_cast<int>(pkt.size()))) > 0) {
      if (::send(udp_fd_, pkt.data(), static_cast<std::size_t>(n), 0) < 0)
        ROS_WARN_THROTTLE(5.0, "mote_node: UDP send() failed: %s",
                          std::strerror(errno));
    }
    if (n < 0)
      ROS_WARN_THROTTLE(5.0,
                        "mote_node: mote_link_poll_transmit: buffer too small");
  }

  // Background thread: feeds raw UDP packets into the link for decoding.
  // Uses select() with a 100 ms timeout to allow clean shutdown.
  void recv_thread_fn() {
    std::vector<uint8_t> buf(UDP_BUF_SIZE);
    while (running_.load()) {
      fd_set rfds;
      FD_ZERO(&rfds);
      FD_SET(udp_fd_, &rfds);
      struct timeval tv{0, 100000}; // 100 ms
      const int r = ::select(udp_fd_ + 1, &rfds, nullptr, nullptr, &tv);
      if (r < 0) {
        if (errno == EINTR || errno == EBADF)
          break;
        ROS_WARN_THROTTLE(5.0, "mote_node: select() error: %s",
                          std::strerror(errno));
        continue;
      }
      if (r == 0)
        continue; // timeout — check running_ and loop

      const ssize_t n = ::recv(udp_fd_, buf.data(), buf.size(), 0);
      if (n <= 0)
        continue;

      std::lock_guard<std::mutex> lk(link_mutex_);
      mote_link_handle_receive(link_, buf.data(), static_cast<int>(n));
    }
  }

  // Sends a Ping to the robot once per second to keep the link alive.
  void keepalive_cb(const ros::TimerEvent &) {
    std::lock_guard<std::mutex> lk(link_mutex_);
    mote_link_send(link_, "\"Ping\"");
    flush_transmit();
  }

  // Route a decoded JSON message to the appropriate handler.
  void dispatch(const json &msg, const ros::Time &stamp) {
    if (msg.is_string()) {
      // Ping from robot: queue a Pong reply (sent in write())
      if (msg.get<std::string>() == "Ping")
        pending_pong_ = true;
      return;
    }
    if (!msg.is_object())
      return;

    if (msg.contains("DriveBaseState"))
      update_joint_state(msg["DriveBaseState"]);
    if (msg.contains("Scan"))
      publish_scan(msg["Scan"], stamp);
    if (msg.contains("IMUMeasurement"))
      publish_imu(msg["IMUMeasurement"], stamp);
    if (msg.contains("State"))
      ROS_DEBUG_STREAM(
          "mote_node: device state update: " << msg["State"].dump());
  }

  // Update joint state buffers from a DriveBaseState message.
  void update_joint_state(const json &state) {
    // Note: field name "postition_rad" is a typo in the mote-api schema
    // (double-t). It must match the wire format exactly.
    pos_[0] = state["left"]["postition_rad"].get<double>();
    pos_[1] = state["right"]["postition_rad"].get<double>();
    vel_[0] = state["left"]["velocity_rad_per_s"].get<double>();
    vel_[1] = state["right"]["velocity_rad_per_s"].get<double>();
    eff_[0] = state["left"]["effort_percent"].get<double>();
    eff_[1] = state["right"]["effort_percent"].get<double>();
  }

  // Accumulate raw scan points; publish once SCAN_ACCUM_MIN points are
  // buffered. The stamp of the first contributing packet is used as the message
  // header.
  void publish_scan(const json &points, const ros::Time &stamp) {
    if (scan_accum_.empty())
      scan_accum_stamp_ = stamp;

    for (const auto &pt : points) {
      if (pt["quality"].get<int>() == 0)
        continue;
      scan_accum_.push_back(
          {pt["angle_rad"].get<float>(), pt["distance_mm"].get<float>()});
    }

    if (static_cast<int>(scan_accum_.size()) < SCAN_ACCUM_MIN)
      return;

    // Rasterize accumulated points onto a fixed 0.5° angle grid.
    sensor_msgs::LaserScan msg;
    msg.header.stamp = scan_accum_stamp_;
    msg.header.frame_id = laser_frame_;
    msg.angle_min = 0.0f;
    msg.angle_max = 2.0f * static_cast<float>(M_PI) - SCAN_ANGLE_INC;
    msg.angle_increment = SCAN_ANGLE_INC;
    msg.range_min = 0.05f; // 5 cm
    msg.range_max = 12.0f; // 12 m (RPLiDAR C1 max range)
    msg.ranges.assign(SCAN_BINS, std::numeric_limits<float>::infinity());

    const float two_pi = 2.0f * static_cast<float>(M_PI);
    for (const auto &pt : scan_accum_) {
      float angle = std::fmod(pt.angle_rad, two_pi);
      if (angle < 0.0f)
        angle += two_pi;

      int bin = static_cast<int>(angle / SCAN_ANGLE_INC);
      bin = std::max(0, std::min(bin, SCAN_BINS - 1));

      float dist_m = pt.distance_mm / 1000.0f;
      if (dist_m < msg.ranges[bin])
        msg.ranges[bin] = dist_m;
    }

    scan_pub_.publish(msg);
    scan_accum_.clear();
  }

  // Publish an IMU message. Orientation is unknown (covariance[0] = -1 per
  // REP-145).
  void publish_imu(const json &imu, const ros::Time &stamp) {
    sensor_msgs::Imu msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = imu_frame_;

    msg.orientation_covariance[0] = -1.0; // orientation unknown

    msg.linear_acceleration.x = imu["accel"]["x"].get<double>();
    msg.linear_acceleration.y = imu["accel"]["y"].get<double>();
    msg.linear_acceleration.z = imu["accel"]["z"].get<double>();

    msg.angular_velocity.x = imu["gyro"]["x"].get<double>();
    msg.angular_velocity.y = imu["gyro"]["y"].get<double>();
    msg.angular_velocity.z = imu["gyro"]["z"].get<double>();

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
