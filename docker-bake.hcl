// Empty stub so `inherits` below resolves for a local `docker buildx bake`
// run (built untagged). In CI (.github/workflows/docker-release.yaml),
// docker/metadata-action writes a separate, ephemeral bake-file fragment
// with the real tags/labels; that fragment and this file are both passed to
// `docker buildx bake -f ... -f ...` and merged in memory for that build --
// this file itself is never modified or overwritten.
target "docker-metadata-action" {}

target "image" {
  inherits   = ["docker-metadata-action"]
  context    = "."
  dockerfile = "Dockerfile"
  platforms  = ["linux/amd64", "linux/arm64"]
}
