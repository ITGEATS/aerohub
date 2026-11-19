#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>

namespace uav_follow_path_execution {
// Executor-local yaw policy. No publisher and no independent motion ownership.
class YawTracker {
public:
  bool active{false};
  std::string task;
  uint64_t revision{0};
  double angle{0}, velocity{0}, target{0}, expires{0}, rate{1.5};
  static double wrap(double value) {return std::atan2(std::sin(value),std::cos(value));}
  static void validate(double yaw, double until, double now, double max_rate) {
    if (!std::isfinite(yaw) || !std::isfinite(until) || until<=now || until>now+5.0 ||
        !std::isfinite(max_rate) || max_rate<=0 || max_rate>3.0) {
      throw std::invalid_argument("invalid or expired continuous yaw target");
    }
  }
  void start(const std::string & owner, double actual, double yaw, double until,
             double now, double max_rate, uint64_t seq) {
    validate(yaw,until,now,max_rate);
    if (owner.empty() || seq==0) {throw std::invalid_argument("missing yaw owner/revision");}
    active=true;task=owner;angle=actual;velocity=0;target=wrap(yaw);
    expires=until;rate=max_rate;revision=seq;
  }
  void update(const std::string & owner, uint64_t seq, double yaw, double until, double now) {
    if (!active || owner!=task || seq<=revision) {throw std::invalid_argument("inactive, wrong owner or stale yaw revision");}
    validate(yaw,until,now,rate);
    target=wrap(yaw);expires=until;revision=seq;
  }
  double step(double dt, double now) {
    if (!active || now>expires || !std::isfinite(dt) || dt<0 || dt>.25) {
      throw std::invalid_argument("yaw target expired or execution clock discontinuity");
    }
    const double error=wrap(target-angle);
    // Velocity and acceleration bounded; proportional approach avoids target chatter.
    const double desired=std::clamp(4.0*error,-rate,rate);
    velocity+=std::clamp(desired-velocity,-4.0*dt,4.0*dt);
    angle=wrap(angle+velocity*dt);
    return angle;
  }
  void reset() {*this=YawTracker();}
};
}
