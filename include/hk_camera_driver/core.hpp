#pragma once
#include <cmath>
#include <condition_variable>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

namespace hk {
struct Settings {
  double exposure = 5000.0, gain = 10.0, fps = 30.0;
  std::string pixel = "BayerRG8";
};
inline void validate(const Settings &s) {
  if (!std::isfinite(s.exposure) || s.exposure <= 0 ||
      !std::isfinite(s.gain) || s.gain < 0 ||
      !std::isfinite(s.fps) || s.fps < 0)
    throw std::invalid_argument("exposure_time>0, gain>=0, frame_rate>=0; finite values required");
  if (s.pixel != "Mono8" && s.pixel != "RGB8Packed" && s.pixel != "BGR8Packed" &&
      s.pixel != "BayerRG8" && s.pixel != "BayerGR8" && s.pixel != "BayerGB8" && s.pixel != "BayerBG8")
    throw std::invalid_argument("Unsupported driver pixel_format; use Mono8, RGB8Packed, BGR8Packed or BayerXX8");
}

// Runs a hardware transaction. Caller commits its desired state only on success.
// If rollback itself fails, caller must close the device and reconnect with old desired state.
template<class Apply, class Rollback, class Invalidate>
std::string transaction(Apply apply, Rollback rollback, Invalidate invalidate) {
  try { apply(); return {}; }
  catch (const std::exception &e) {
    std::string reason = e.what();
    try { rollback(); reason += "; previous settings restored"; }
    catch (const std::exception &r) {
      reason += "; rollback failed: "; reason += r.what();
      invalidate();
    }
    return reason;
  }
}

// A bounded, latest-only queue. Replacements are explicitly counted by the caller.
template<class T> class LatestSlot {
public:
  bool put(T value) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (closed_) return false;
    bool replaced = value_.has_value();
    value_ = std::move(value); ready_.notify_one(); return replaced;
  }
  bool take(T &value) {
    std::unique_lock<std::mutex> lock(mutex_);
    ready_.wait(lock, [&]{return closed_ || value_.has_value();});
    if (closed_) return false;
    value = std::move(*value_); value_.reset(); return true;
  }
  void clear() { std::lock_guard<std::mutex> lock(mutex_); value_.reset(); }
  void close() { std::lock_guard<std::mutex> lock(mutex_); closed_=true; value_.reset(); ready_.notify_all(); }
private:
  std::mutex mutex_;
  std::condition_variable ready_;
  std::optional<T> value_;
  bool closed_=false;
};
} // namespace hk
