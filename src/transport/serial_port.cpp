#include "hwemu/transport/serial_port.hpp"

#include <cerrno>
#include <cstring>
#include <filesystem>
#include <stdexcept>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace hwemu::transport {
namespace {

void MakeRaw(int fd) {
  termios t{};
  if (::tcgetattr(fd, &t) == 0) {
    ::cfmakeraw(&t);
    ::tcsetattr(fd, TCSANOW, &t);
  }
}

void NonBlocking(int fd) { ::fcntl(fd, F_SETFL, ::fcntl(fd, F_GETFL) | O_NONBLOCK); }

speed_t BaudConstant(int baud) {
  switch (baud) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 921600: return B921600;
    case 1000000: return B1000000;
    case 2000000: return B2000000;
    default: throw std::invalid_argument("unsupported baud rate " + std::to_string(baud));
  }
}

std::size_t ReadInto(int fd, std::vector<std::uint8_t>* out, std::size_t max) {
  std::size_t total = 0;
  std::uint8_t buf[4096];
  while (total < max) {
    const ssize_t n = ::read(fd, buf, std::min(sizeof(buf), max - total));
    if (n <= 0) {
      break;
    }
    out->insert(out->end(), buf, buf + n);
    total += static_cast<std::size_t>(n);
  }
  return total;
}

std::size_t WriteFrom(int fd, std::span<const std::uint8_t> bytes) {
  std::size_t total = 0;
  while (total < bytes.size()) {
    const ssize_t n = ::write(fd, bytes.data() + total, bytes.size() - total);
    if (n <= 0) {
      break;
    }
    total += static_cast<std::size_t>(n);
  }
  return total;
}

}  // namespace

PtyPort::PtyPort(const std::string& link) : link_(link) {
  master_ = ::posix_openpt(O_RDWR | O_NOCTTY);
  if (master_ < 0 || ::grantpt(master_) != 0 || ::unlockpt(master_) != 0) {
    throw std::runtime_error("PTY: posix_openpt: " + std::string(std::strerror(errno)));
  }
  char name[128];
  if (::ptsname_r(master_, name, sizeof(name)) != 0) {
    throw std::runtime_error("PTY: ptsname_r failed");
  }
  slave_path_ = name;
  // Held open by the hub so the master never sees a hang-up between clients,
  // and raw so nothing echoes or translates.
  slave_ = ::open(slave_path_.c_str(), O_RDWR | O_NOCTTY);
  if (slave_ >= 0) {
    MakeRaw(slave_);
  }
  MakeRaw(master_);
  NonBlocking(master_);
  if (!link_.empty()) {
    namespace fs = std::filesystem;
    const fs::path p(link_);
    if (p.has_parent_path()) {
      fs::create_directories(p.parent_path());
    }
    std::error_code ec;
    fs::remove(p, ec);
    fs::create_symlink(slave_path_, p);
    owned_link_ = OwnedName(link_);
  }
}

PtyPort::~PtyPort() {
  owned_link_.Remove();
  if (slave_ >= 0) ::close(slave_);
  if (master_ >= 0) ::close(master_);
}

std::size_t PtyPort::Write(std::span<const std::uint8_t> bytes) { return WriteFrom(master_, bytes); }
std::size_t PtyPort::Read(std::vector<std::uint8_t>* out, std::size_t max) { return ReadInto(master_, out, max); }

TtyPort::TtyPort(const std::string& path, int baud) : path_(path) {
  fd_ = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    throw std::runtime_error("serial: open " + path + ": " + std::strerror(errno));
  }
  termios t{};
  ::tcgetattr(fd_, &t);
  ::cfmakeraw(&t);
  ::cfsetispeed(&t, BaudConstant(baud));
  ::cfsetospeed(&t, BaudConstant(baud));
  ::tcsetattr(fd_, TCSANOW, &t);
}

TtyPort::~TtyPort() {
  if (fd_ >= 0) ::close(fd_);
}

std::size_t TtyPort::Write(std::span<const std::uint8_t> bytes) { return WriteFrom(fd_, bytes); }
std::size_t TtyPort::Read(std::vector<std::uint8_t>* out, std::size_t max) { return ReadInto(fd_, out, max); }

std::unique_ptr<ISerialPort> OpenSerialPort(const std::string& spec, int default_baud) {
  constexpr std::string_view kPty = "pty:";
  constexpr std::string_view kTty = "tty:";
  if (spec.rfind(kTty, 0) == 0) {
    std::string rest = spec.substr(kTty.size());
    int baud = default_baud;
    if (auto at = rest.rfind('@'); at != std::string::npos) {
      baud = std::stoi(rest.substr(at + 1));
      rest = rest.substr(0, at);
    }
    return std::make_unique<TtyPort>(rest, baud);
  }
  if (spec.rfind(kPty, 0) == 0) {
    return std::make_unique<PtyPort>(spec.substr(kPty.size()));
  }
  return std::make_unique<PtyPort>(spec);
}

}  // namespace hwemu::transport
