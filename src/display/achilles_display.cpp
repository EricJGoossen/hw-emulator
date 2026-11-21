// The Achilles display: the shared contract over achilles::display::Renderer.
// A window per sensor is native here (CameraView, DepthView, LidarView,
// ReadingView); a camera's window asked to fly it steers it (FlyCamera), as
// Achilles's own examples do.
#include <map>
#include <stdexcept>
#include <string>

#include "display/controllers/fly_camera.hpp"
#include "display/renderer.hpp"
#include "display/views/camera/camera_view.hpp"
#include "display/views/camera/depth_view.hpp"
#include "display/views/lidar_view.hpp"
#include "display/views/reading_view.hpp"
#include "domain/actuation/dc_motor.hpp"
#include "domain/attachment/sim_quantities.hpp"
#include "domain/sensing/accelerometer.hpp"
#include "domain/sensing/attitude.hpp"
#include "domain/sensing/camera.hpp"
#include "domain/sensing/encoder.hpp"
#include "domain/sensing/gyroscope.hpp"
#include "domain/sensing/lidar.hpp"
#include "domain/sensing/limit_switch.hpp"
#include "domain/sensing/magnetometer.hpp"
#include "hwemu/display/display.hpp"
#include "interface/attachments.hpp"
#include "interface/simulation.hpp"

namespace hwemu::display {
namespace {

namespace ad = achilles::display;
namespace as = achilles::domain::sensing;
namespace aa = achilles::domain::actuation;
using achilles::interface::NamedAttachments;

class AchillesDisplay final : public IDisplay, public ISensorWindows {
 public:
  explicit AchillesDisplay(achilles::interface::Simulation& sim) : sim_(sim) {}

  void Open() override {
    if (!renderer_) renderer_ = std::make_unique<ad::Renderer>("hw-emulator (Achilles)");
  }
  void Close() override {
    renderer_.reset();
    windows_.clear();
  }
  bool IsOpen() const override { return renderer_ != nullptr; }
  void Pause(bool paused) override { paused_ = paused; }
  bool Paused() const override { return paused_; }

  bool Draw() override {
    if (!renderer_) return false;
    const bool any_up = renderer_->Draw();
    // Draw() is false once no window is up: closed by the user if some were.
    if (!any_up && !windows_.empty()) {
      return false;
    }
    return true;
  }

  void ApplyWrites() override {
    if (renderer_) renderer_->ApplyWrites();
  }

  Capabilities Caps() const override { return {true, true, true, false}; }
  ISensorWindows* SensorWindows() override { return this; }

  WindowId Show(std::string_view sensor, ViewKind kind, std::string_view title, bool fly) override {
    Open();
    if (fly && kind != ViewKind::kImage && kind != ViewKind::kDepth) {
      throw std::invalid_argument("'" + std::string(sensor) + "': only a camera's window can fly it");
    }
    const auto lists = sim_.Attachments();
    const std::string name(sensor);
    auto find = [&]<typename K>() -> const achilles::interface::NamedAttachment<K>* {
      for (const auto& a : std::get<NamedAttachments<K>>(lists)) {
        if (a.name == name) return &a;
      }
      return nullptr;
    };
    ad::WindowId id{};
    switch (kind) {
      case ViewKind::kImage:
        if (auto* c = find.template operator()<as::Camera>()) {
          id = renderer_->Attach(ad::CameraView(c->handle), title);
          if (fly) renderer_->Control(id, FlyCameraFor(*c));
        } else {
          throw std::invalid_argument("no camera '" + name + "' to show as an image");
        }
        break;
      case ViewKind::kDepth:
        if (auto* c = find.template operator()<as::Camera>()) {
          id = renderer_->Attach(ad::DepthView(c->handle), title);
          if (fly) renderer_->Control(id, FlyCameraFor(*c));
        } else {
          throw std::invalid_argument("no camera '" + name + "' to show as depth");
        }
        break;
      case ViewKind::kLidar:
        if (auto* l = find.template operator()<as::Lidar>()) {
          id = renderer_->Attach(ad::LidarView(l->handle), title);
        } else {
          throw std::invalid_argument("no lidar '" + name + "'");
        }
        break;
      case ViewKind::kPlot: {
        bool found = false;
        auto plot = [&]<typename K>() {
          if (found) return;
          if (auto* a = find.template operator()<K>()) {
            id = renderer_->Attach(ad::ReadingView<K>(a->handle), title);
            found = true;
          }
        };
        plot.template operator()<as::Encoder>();
        plot.template operator()<as::Gyroscope>();
        plot.template operator()<as::Accelerometer>();
        plot.template operator()<as::Magnetometer>();
        plot.template operator()<as::Attitude>();
        plot.template operator()<as::LimitSwitch>();
        plot.template operator()<aa::DcMotor>();
        if (!found) throw std::invalid_argument("no plottable sensor or motor '" + name + "'");
        break;
      }
    }
    const WindowId ours = next_++;
    windows_[ours] = id;
    return ours;
  }

  void Hide(WindowId id) override {
    auto it = windows_.find(id);
    if (it == windows_.end() || !renderer_) return;
    renderer_->Detach(it->second);
    windows_.erase(it);
  }

  ad::Renderer* Renderer() { return renderer_.get(); }

 private:
  // Steers `camera` by writing the pose of the joint it rides (see
  // achilles::display::FlyCamera), up being the sim's.
  ad::FlyCamera FlyCameraFor(const achilles::interface::NamedAttachment<as::Camera>& camera) const {
    namespace input = achilles::domain::attachment::input;
    return {sim_.State<input::JointPosition>(camera.joint), sim_.State<const input::WorldTransform>(camera.joint),
            camera.handle.Config().GetParams().body_from_sensor, {.up = sim_.Up()}};
  }

  achilles::interface::Simulation& sim_;
  std::unique_ptr<ad::Renderer> renderer_;
  std::map<WindowId, ad::WindowId> windows_;
  WindowId next_ = 1;
  bool paused_ = false;
};

}  // namespace

std::unique_ptr<IDisplay> MakeAchillesDisplay(achilles::interface::Simulation& sim) {
  return std::make_unique<AchillesDisplay>(sim);
}

ad::Renderer* AchillesRenderer(IDisplay* display) {
  auto* a = dynamic_cast<AchillesDisplay*>(display);
  return a == nullptr ? nullptr : a->Renderer();
}

}  // namespace hwemu::display
