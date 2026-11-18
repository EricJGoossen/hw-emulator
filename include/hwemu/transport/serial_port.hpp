#pragma once

// Byte streams the hub's serial devices talk on.
//
//   PtyPort  a pseudo-terminal the hub owns: the far side opens Path() --
//            or a stable symlink to it -- as if it were /dev/ttyUSB0, with
//            any baud rate (a PTY ignores it). The hub keeps the slave side
//            open itself, so a client closing and reopening doesn't hang up.
//   TtyPort  an existing serial device (a real USB-serial adapter), raw,
//            at a baud rate.

#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <vector>

namespace hwemu::transport {

class ISerialPort {
 public:
  virtual ~ISerialPort() = default;
  // Non-blocking; returns bytes written (may be short if the far side isn't
  // reading).
  virtual std::size_t Write(std::span<const std::uint8_t> bytes) = 0;
  // Non-blocking; appends what's waiting to *out, returns how many.
  virtual std::size_t Read(std::vector<std::uint8_t>* out, std::size_t max = 65536) = 0;
  // The path the far side opens.
  virtual std::string Path() const = 0;
  virtual int Fd() const = 0;
};

class PtyPort final : public ISerialPort {
 public:
  // `link`, if non-empty, is a symlink made to the PTY (its directory
  // created), replacing any stale one; removed again on destruction.
  explicit PtyPort(const std::string& link = "");
  ~PtyPort() override;
  PtyPort(const PtyPort&) = delete;
  PtyPort& operator=(const PtyPort&) = delete;

  std::size_t Write(std::span<const std::uint8_t> bytes) override;
  std::size_t Read(std::vector<std::uint8_t>* out, std::size_t max) override;
  std::string Path() const override { return link_.empty() ? slave_path_ : link_; }
  std::string SlavePath() const { return slave_path_; }
  int Fd() const override { return master_; }

 private:
  int master_ = -1;
  int slave_ = -1;
  std::string slave_path_;
  std::string link_;
};

class TtyPort final : public ISerialPort {
 public:
  TtyPort(const std::string& path, int baud);
  ~TtyPort() override;
  std::size_t Write(std::span<const std::uint8_t> bytes) override;
  std::size_t Read(std::vector<std::uint8_t>* out, std::size_t max) override;
  std::string Path() const override { return path_; }
  int Fd() const override { return fd_; }

 private:
  int fd_ = -1;
  std::string path_;
};

// "pty:<link path>" (or a bare path under the runtime directory) makes a
// PtyPort; "tty:<device>@<baud>" opens a TtyPort.
std::unique_ptr<ISerialPort> OpenSerialPort(const std::string& spec, int default_baud);

}  // namespace hwemu::transport
