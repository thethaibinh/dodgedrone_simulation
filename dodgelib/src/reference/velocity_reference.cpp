#include "dodgelib/reference/velocity_reference.hpp"

#include "dodgelib/math/gravity.hpp"

namespace agi {

VelocityReference::VelocityReference(const QuadState& state,
                                     const bool update_from_estimate,
                                     const Scalar timeout)
  : ReferenceBase(state.getHoverState(), INF, "Velocity Reference"),
    update_from_estimate_(update_from_estimate),
    yaw_last_(start_state_.getYaw()),
    timeout_(timeout) {}


Setpoint VelocityReference::getSetpoint(const QuadState& state,
                                        const Scalar t) {
  Setpoint setpoint;
  if (!std::isfinite(state.t)) return setpoint;
  const Scalar t_query = std::isfinite(t) ? t : state.t;
  if (t_query < state.t) return setpoint;
  if (t == state.t) {  // If requested at the query state...
    updateTo(state);
    setpoint.state = start_state_;
    QuadState state_;
    state_.q(reference_yaw_);
    setpoint.state.q(state_.q());
  } else {  // .. or if prediction requested.
    const Scalar dt = t_query - start_state_.t;

    setpoint.state = start_state_;
    setpoint.state.t = t_query;
    setpoint.state.p += dt * v_;
    setpoint.state.v = v_;
    QuadState state_;
    state_.q(reference_yaw_);
    setpoint.state.q(state_.q());
  }

  setpoint.input = Command(t, G, Vector<3>(0, 0, 0));
  return setpoint;
}

void VelocityReference::updateTo(const QuadState& state) {
  if (!state.valid()) return;
  if (state.t <= start_state_.t) return;

  if (std::isnan(t_last_update_)) t_last_update_ = state.t;
  if (state.t - t_last_update_ > timeout_) {
    v_.setZero();
  }

  if (update_from_estimate_) {
    start_state_ = state.getHoverState();
    const Scalar new_yaw = start_state_.getYaw();
    if (std::isfinite(new_yaw)) yaw_last_ = new_yaw;
    start_state_.q(yaw_last_);
    start_state_.v = v_;
  } else {
    const Scalar dt = state.t - start_state_.t;
    start_state_.t = state.t;
    start_state_.p += dt * v_;
    start_state_.v = v_;
    start_state_.q(yaw_last_);
  }
}

bool VelocityReference::update(const Vector<3>& velocity,
                               const Scalar reference_yaw) {
  if (!velocity.allFinite() || !std::isfinite(reference_yaw)) return false;

  v_ = velocity;
  reference_yaw_ = reference_yaw;
  t_last_update_ = start_state_.t;
  return true;
}

}  // namespace agi
