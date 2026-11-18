#include "egl_context.hpp"

#include <EGL/egl.h>
#include <EGL/eglext.h>

#include <stdexcept>
#include <string>
#include <vector>

namespace hwemu::sim {
namespace {

EGLDisplay DeviceDisplay() {
  auto query = reinterpret_cast<PFNEGLQUERYDEVICESEXTPROC>(eglGetProcAddress("eglQueryDevicesEXT"));
  auto get = reinterpret_cast<PFNEGLGETPLATFORMDISPLAYEXTPROC>(eglGetProcAddress("eglGetPlatformDisplayEXT"));
  if (query == nullptr || get == nullptr) return EGL_NO_DISPLAY;
  EGLint count = 0;
  if (!query(0, nullptr, &count) || count == 0) return EGL_NO_DISPLAY;
  std::vector<EGLDeviceEXT> devices(static_cast<std::size_t>(count));
  query(count, devices.data(), &count);
  for (EGLint i = 0; i < count; ++i) {
    EGLDisplay d = get(EGL_PLATFORM_DEVICE_EXT, devices[static_cast<std::size_t>(i)], nullptr);
    EGLint major = 0, minor = 0;
    if (d != EGL_NO_DISPLAY && eglInitialize(d, &major, &minor)) return d;
  }
  return EGL_NO_DISPLAY;
}

}  // namespace

EglContext::EglContext() {
  EGLDisplay display = DeviceDisplay();
  if (display == EGL_NO_DISPLAY) {
    display = eglGetDisplay(EGL_DEFAULT_DISPLAY);
    EGLint major = 0, minor = 0;
    if (display == EGL_NO_DISPLAY || !eglInitialize(display, &major, &minor)) {
      throw std::runtime_error("EGL: no display to render MuJoCo cameras on");
    }
  }
  const EGLint attributes[] = {EGL_RED_SIZE,   8,  EGL_GREEN_SIZE,      8, EGL_BLUE_SIZE,       8,
                               EGL_ALPHA_SIZE, 8,  EGL_DEPTH_SIZE,      24, EGL_STENCIL_SIZE,   8,
                               EGL_COLOR_BUFFER_TYPE, EGL_RGB_BUFFER,   EGL_SURFACE_TYPE, EGL_PBUFFER_BIT,
                               EGL_RENDERABLE_TYPE, EGL_OPENGL_BIT,     EGL_NONE};
  EGLConfig config = nullptr;
  EGLint configs = 0;
  if (!eglChooseConfig(display, attributes, &config, 1, &configs) || configs < 1) {
    throw std::runtime_error("EGL: no config for an OpenGL context");
  }
  if (!eglBindAPI(EGL_OPENGL_API)) throw std::runtime_error("EGL: can't bind OpenGL");
  EGLContext context = eglCreateContext(display, config, EGL_NO_CONTEXT, nullptr);
  if (context == EGL_NO_CONTEXT) {
    throw std::runtime_error("EGL: eglCreateContext failed (0x" + std::to_string(eglGetError()) + ")");
  }
  display_ = display;
  context_ = context;
  MakeCurrent();
}

EglContext::~EglContext() {
  if (display_ != nullptr) {
    eglMakeCurrent(display_, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
    if (context_ != nullptr) eglDestroyContext(display_, context_);
    // No eglTerminate: the display is shared process-wide (Achilles renders on it too).
  }
}

void EglContext::MakeCurrent() {
  if (!eglMakeCurrent(display_, EGL_NO_SURFACE, EGL_NO_SURFACE, context_)) {
    throw std::runtime_error("EGL: eglMakeCurrent failed");
  }
}

}  // namespace hwemu::sim
