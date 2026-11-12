"""Build per-run visual-servo estimators and state from a validated profile."""
from dataclasses import dataclass

from robot_follow.control.target_filter import KalmanTarget3D
from robot_follow.control.timing import (AdaptiveLeadGate, AxisProcessNoiseGate, FastStartGate,
                          SlowFollowGate, StaticVelocityHold)
from robot_follow.control.state import ServoState


@dataclass
class ServoComponents:
    state: ServoState
    target_filter: KalmanTarget3D
    axis_noise: AxisProcessNoiseGate
    static_hold: StaticVelocityHold
    lead: AdaptiveLeadGate
    fast_start: FastStartGate
    slow_follow: SlowFollowGate

    @classmethod
    def from_configuration(cls, config):
        return cls(
            state=ServoState(ff_taper_now=config.ff_taper_dist),
            target_filter=KalmanTarget3D(q=config.kf_q, r=config.kf_r,
                                         r_xy=config.kf_r_xy, r_z=config.kf_r_z),
            axis_noise=AxisProcessNoiseGate(
                config.kf_axis_adapt_ref_xy, config.kf_axis_adapt_ref_z,
                config.kf_axis_adapt_max_xy, config.kf_axis_adapt_max_z,
                config.kf_axis_adapt_confirm, config.kf_axis_adapt_attack_tau,
                config.kf_axis_adapt_release_tau),
            static_hold=StaticVelocityHold(config.static_vel_hold_speed,
                                           config.static_vel_hold_frames),
            lead=AdaptiveLeadGate(
                config.ff_kf_deadband_off, config.ff_kf_deadband_on,
                config.kf_lead_near_dist, config.kf_lead_full_dist,
                config.kf_lead_dir_full_cos, config.kf_lead_attack_tau,
                config.kf_lead_release_tau),
            fast_start=FastStartGate(config.kf_fast_start_enable, config.kf_fast_start_frames,
                                     config.kf_fast_start_dist, config.kf_fast_start_dir_cos),
            slow_follow=SlowFollowGate(config.kf_slow_follow_dist, config.kf_slow_follow_release,
                                       config.kf_slow_follow_frames, config.kf_slow_follow_dir_cos),
        )
