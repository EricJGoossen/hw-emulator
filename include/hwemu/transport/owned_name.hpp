#pragma once

// A file-system name the hub made (a socket, a /dev/shm object, a symlink),
// removed when the hub is done with it only while it still names what this
// hub made. Every run uses the same names, so a hub still shutting down when
// the next run's hub has started (a rerun right after Ctrl-C) must leave that
// hub's alone: removing them strands its clients ("frame ring never
// appeared", a socket nobody can connect to).
#include <cstddef>
#include <string>
#include <tuple>
#include <utility>

#include <sys/stat.h>
#include <unistd.h>

namespace hwemu::transport {

class OwnedName {
 public:
  OwnedName() = default;
  // Records what `path` names now: make it first. Nothing is recorded (and
  // Remove does nothing) if it doesn't exist.
  explicit OwnedName(std::string path) : path_(std::move(path)) {
    struct stat s {};
    if (::lstat(path_.c_str(), &s) == 0) {
      id_ = Id(s);
      recorded_ = true;
    }
  }

  // Unlinks the name if it still names what was recorded; once.
  void Remove() {
    if (!recorded_) return;
    recorded_ = false;
    struct stat s {};
    if (::lstat(path_.c_str(), &s) == 0 && Id(s) == id_) {
      ::unlink(path_.c_str());
    }
  }

  // The /dev/shm path of a POSIX shared-memory object ("/name" or "name").
  static std::string ShmPath(const std::string& shm_name) {
    return "/dev/shm" + (shm_name.empty() || shm_name[0] == '/' ? shm_name : "/" + shm_name);
  }

 private:
  // Device and inode. A socket's or /dev/shm object's inode stays allocated
  // while its maker holds it open, so it can't come back for another file.
  // A symlink's can (nothing holds it, and on tmpfs the next file made right
  // after an unlink gets it), so a symlink also has its change time and
  // target: the next one points at a different live PTY.
  using FileId = std::tuple<dev_t, ino_t, time_t, long, std::string>;
  FileId Id(const struct stat& s) const {
    if (!S_ISLNK(s.st_mode)) return {s.st_dev, s.st_ino, 0, 0, {}};
    std::string target;
    char buf[4096];
    const ssize_t n = ::readlink(path_.c_str(), buf, sizeof(buf));
    if (n > 0) target.assign(buf, static_cast<std::size_t>(n));
    return {s.st_dev, s.st_ino, s.st_ctim.tv_sec, s.st_ctim.tv_nsec, std::move(target)};
  }

  std::string path_;
  FileId id_{};
  bool recorded_ = false;
};

}  // namespace hwemu::transport
