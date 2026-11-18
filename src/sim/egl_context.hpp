#pragma once

// A headless OpenGL context through EGL -- no window, no display server --
// for MuJoCo's offscreen camera rendering. It prefers an EGL device (a GPU,
// or Mesa's software device), then the default display.

namespace hwemu::sim {

class EglContext {
 public:
  // Throws std::runtime_error if no EGL display gives an OpenGL context.
  EglContext();
  ~EglContext();
  EglContext(const EglContext&) = delete;
  EglContext& operator=(const EglContext&) = delete;

  // Makes it current on the calling thread.
  void MakeCurrent();
  // Releases it from the calling thread, so another thread can make it
  // current (the hub may step the sim on a thread other than the one that
  // built it).
  void Release();

 private:
  void* display_ = nullptr;
  void* context_ = nullptr;
};

}  // namespace hwemu::sim
