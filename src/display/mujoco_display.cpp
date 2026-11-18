// The MuJoCo display: MuJoCo's own viewer, `simulate`, in passive mode --
// the way MuJoCo's Python passive viewer runs it. Its render loop runs on a
// thread of its own with its own GLFW window and OpenGL context; the hub's
// thread steps the sim and calls Draw(), which syncs the viewer with the
// model and data under the sim's mutex. The viewer has its own sensor plot
// and pause, but no per-sensor windows (Caps().sensor_windows is false).
#include <mujoco/mujoco.h>

#include <atomic>
#include <memory>
#include <mutex>
#include <future>
#include <thread>

#include "glfw_adapter.h"
#include "hwemu/display/display.hpp"
#include "simulate.h"

namespace hwemu::display {
namespace {

class MujocoDisplay final : public IDisplay {
 public:
  MujocoDisplay(mjModel* model, mjData* data, std::recursive_mutex* mutex)
      : model_(model), data_(data), mutex_(mutex) {
    mjv_defaultCamera(&camera_);
    mjv_defaultOption(&options_);
    mjv_defaultPerturb(&perturb_);
  }
  ~MujocoDisplay() override { Close(); }

  void Open() override {
    if (viewer_) return;
    // simulate's RenderLoop draws with the GL context its GlfwAdapter made
    // current, so both live on the display thread; and Load() waits for a
    // running RenderLoop to take the model, so it's called once the loop is
    // up (calling it first, before the thread, deadlocked).
    std::promise<void> created;
    auto ready = created.get_future();
    thread_ = std::thread([this, &created] {
      try {
        viewer_ = std::make_unique<mujoco::Simulate>(std::make_unique<mujoco::GlfwAdapter>(), &camera_, &options_,
                                                     &perturb_, /*is_passive=*/true);
      } catch (...) {
        created.set_exception(std::current_exception());
        return;
      }
      created.set_value();
      viewer_->RenderLoop();
    });
    try {
      ready.get();
    } catch (...) {
      thread_.join();
      throw;
    }
    std::lock_guard<std::recursive_mutex> l(*mutex_);
    viewer_->Load(model_, data_, "hw-emulator");
  }

  void Close() override {
    if (!viewer_) return;
    viewer_->exitrequest.store(1);
    if (thread_.joinable()) thread_.join();
    viewer_.reset();
  }

  bool IsOpen() const override { return viewer_ != nullptr && viewer_->exitrequest.load() == 0; }

  void Pause(bool paused) override {
    if (viewer_) viewer_->run = paused ? 0 : 1;
  }
  bool Paused() const override { return viewer_ != nullptr && viewer_->run == 0; }

  bool Draw() override {
    if (!IsOpen()) return false;
    {
      std::lock_guard<std::recursive_mutex> l(*mutex_);
      std::unique_lock<std::recursive_mutex> viewer_lock(viewer_->mtx);
      viewer_->Sync();
    }
    return IsOpen();
  }

  // Perturbations from the window (ctrl+drag) are applied by Sync() through
  // mjData::xfrc_applied; nothing else to hand over.
  void ApplyWrites() override {}

  Capabilities Caps() const override { return {true, false, true, true}; }
  ISensorWindows* SensorWindows() override { return nullptr; }

 private:
  mjModel* model_;
  mjData* data_;
  std::recursive_mutex* mutex_;
  mjvCamera camera_{};
  mjvOption options_{};
  mjvPerturb perturb_{};
  std::unique_ptr<mujoco::Simulate> viewer_;
  std::thread thread_;
};

}  // namespace

std::unique_ptr<IDisplay> MakeMujocoDisplay(mjModel* model, mjData* data, std::recursive_mutex* mutex) {
  return std::make_unique<MujocoDisplay>(model, data, mutex);
}

}  // namespace hwemu::display
