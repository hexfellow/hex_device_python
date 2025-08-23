from typing import Tuple

SQRT_3 = 1.7320508075688772

# A chassis solution demo using TripleOmniWheelLRDriver chassis as an example

# For a triangular chassis, track_width is the radius of the chassis center
track_width = 0.3274
motor_cnt = 3

def forward_kinematic(self, v: list) -> Tuple[float, float, float]:
    if len(v)!= 3:
        raise ValueError("forward_kinematic: wheel_speeds length error")
    d_inv = 1.0 / self.__track_width  / 3.0
    spd_x = -(SQRT_3 / 3.0) * v[0] + (SQRT_3 / 3.0) * v[2]
    spd_y = (1.0 / 3.0) * v[0] + -(2.0 / 3.0) * v[1] + (1.0 / 3.0) * v[2]
    spd_r = d_inv * v[0] + d_inv * v[1] + d_inv * v[2]
    return (spd_x, spd_y, spd_r)

def inverse_kinematic(self, v_x, v_y, v_r) -> list:
    d = self.__track_width
    v = [0.0, 0.0, 0.0]
    v[0] = -(SQRT_3 / 2.0) * v_x + 0.5 * v_y + v_r * d
    v[1] = -v_y + v_r * d
    v[2] = (SQRT_3 / 2.0) * v_x + 0.5 * v_y + v_r * d
    return v

