#pragma once

// The display contract both backends share. A display shows the running sim;
// what it can do beyond that is a capability, asked of Caps():
//
//   - window:          it opens a window at all (always, once Open()ed)
//   - sensor_windows:  a window per sensor (an image, depth, a lidar scan, a
//                      plot) -- SensorWindows() is non-null. Achilles: yes,
//                      native. MuJoCo: no -- its window is MuJoCo's own
//                      `simulate`, with its built-in sensor plot.
//   - pause:           the sim can be paused from the window
//   - builtin_plot:    the window plots sensor data itself
//
// Draw() runs one frame on the caller's thread; ApplyWrites() hands anything
// the window steers (a flown camera) to the sim, and goes before each Step().

#include <cstdint>
#include <string_view>

namespace hwemu::display {

enum class ViewKind { kImage, kDepth, kLidar, kPlot };
using WindowId = std::uint64_t;

struct Capabilities {
  bool window = false;
  bool sensor_windows = false;
  bool pause = false;
  bool builtin_plot = false;
};

class ISensorWindows {
 public:
  virtual ~ISensorWindows() = default;
  // Opens a window on sensor `sensor_name`, as `kind` shows it. Throws
  // std::invalid_argument for a sensor that kind can't show.
  virtual WindowId Show(std::string_view sensor_name, ViewKind kind, std::string_view title) = 0;
  virtual void Hide(WindowId id) = 0;
};

class IDisplay {
 public:
  virtual ~IDisplay() = default;
  virtual void Open() = 0;
  virtual void Close() = 0;
  virtual bool IsOpen() const = 0;
  virtual void Pause(bool paused) = 0;
  virtual bool Paused() const = 0;
  // One frame. False once the user has closed the display.
  virtual bool Draw() = 0;
  virtual void ApplyWrites() = 0;
  virtual Capabilities Caps() const = 0;
  // Null when Caps().sensor_windows is false.
  virtual ISensorWindows* SensorWindows() = 0;
};

ViewKind ParseViewKind(std::string_view name);  // image, depth, lidar, plot

}  // namespace hwemu::display
