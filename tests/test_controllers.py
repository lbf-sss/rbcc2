from __future__ import annotations

import random
import unittest

from relay_control.config import parse_config
from relay_control.controllers import (
    WheelController,
    interpolate_curve,
    seat_control,
    update_assistance,
)
from relay_control.model import Estimate, RuntimeMode, Task
from tests.helpers import synthetic_config_dict


class AdaptiveAssistanceTests(unittest.TestCase):
    def setUp(self) -> None:
        self.config = parse_config(synthetic_config_dict())

    def test_progress_shortfall_increases_assistance(self) -> None:
        result = update_assistance(
            alpha=0.4,
            progress_margin=-0.5,
            quality_gate=1.0,
            dt_s=0.1,
            config=self.config.adaptive,
        )

        self.assertGreater(result, 0.4)

    def test_good_progress_and_quality_withdraws_more_slowly(self) -> None:
        decrease = 0.4 - update_assistance(
            0.4, 0.5, 1.0, 0.1, self.config.adaptive
        )
        increase = update_assistance(
            0.4, -0.5, 1.0, 0.1, self.config.adaptive
        ) - 0.4

        self.assertGreater(increase, decrease)

    def test_degraded_data_freezes_withdrawal(self) -> None:
        result = update_assistance(
            alpha=0.4,
            progress_margin=0.5,
            quality_gate=0.0,
            dt_s=0.1,
            config=self.config.adaptive,
        )

        self.assertEqual(result, 0.4)

    def test_assistance_is_bounded_for_random_finite_inputs(self) -> None:
        rng = random.Random(7)
        for _ in range(1_000):
            result = update_assistance(
                alpha=rng.random(),
                progress_margin=rng.uniform(-10.0, 10.0),
                quality_gate=rng.random(),
                dt_s=rng.uniform(0.0, 0.2),
                config=self.config.adaptive,
            )
            self.assertGreaterEqual(result, self.config.adaptive.alpha_min)
            self.assertLessEqual(result, self.config.adaptive.alpha_max)

    def test_curve_interpolates_and_clamps(self) -> None:
        curve = self.config.seat.device_torque_curve

        self.assertEqual(interpolate_curve(curve, -1.0), 1.0)
        self.assertEqual(interpolate_curve(curve, 2.0), 2.0)
        self.assertAlmostEqual(interpolate_curve(curve, 0.5), 1.5)


class SeatControllerTests(unittest.TestCase):
    def setUp(self) -> None:
        self.config = parse_config(synthetic_config_dict())
        self.phase = Estimate(0.5, 0.9, ("sim.seat.angle",))
        self.intent = Estimate(0.8, 0.9, ("sim.imu.trunk_pitch",))

    def test_stand_torque_separates_device_and_rehabilitation_terms(self) -> None:
        result = seat_control(
            task=Task.STAND_UP,
            seat_angle=0.5,
            seat_angular_velocity=0.1,
            phase=self.phase,
            intent=self.intent,
            alpha=0.5,
            config=self.config.seat,
        )

        self.assertAlmostEqual(result.device_compensation_nm, 1.5)
        self.assertAlmostEqual(result.rehabilitation_assistance_nm, 8.0)
        self.assertAlmostEqual(result.torque_nm, 9.5)

    def test_descent_damping_appears_only_above_safe_speed(self) -> None:
        slow = seat_control(
            Task.SIT_DOWN, 0.5, -0.05, self.phase, self.intent, 0.5, self.config.seat
        )
        fast = seat_control(
            Task.SIT_DOWN, 0.5, -0.3, self.phase, self.intent, 0.5, self.config.seat
        )

        self.assertEqual(slow.descent_damping_nm, 0.0)
        self.assertGreater(fast.descent_damping_nm, 0.0)
        self.assertGreater(fast.torque_nm, slow.torque_nm)


class WheelControllerTests(unittest.TestCase):
    def setUp(self) -> None:
        self.config = parse_config(synthetic_config_dict()).wheel
        self.controller = WheelController(self.config)

    def test_equal_forward_forces_produce_equal_forward_wheel_speed(self) -> None:
        result = self.controller.step(5.0, 5.0, 0.1, RuntimeMode.NORMAL)

        self.assertGreater(result.left_wheel_rad_s, 0.0)
        self.assertAlmostEqual(result.left_wheel_rad_s, result.right_wheel_rad_s)

    def test_force_difference_produces_differential_turn(self) -> None:
        result = self.controller.step(2.0, 8.0, 0.1, RuntimeMode.NORMAL)

        self.assertGreater(result.right_wheel_rad_s, result.left_wheel_rad_s)

    def test_zero_force_smoothly_decays_existing_speed(self) -> None:
        first = self.controller.step(10.0, 10.0, 0.1, RuntimeMode.NORMAL)
        second = self.controller.step(0.0, 0.0, 0.1, RuntimeMode.NORMAL)

        self.assertGreater(second.left_wheel_rad_s, 0.0)
        self.assertLess(second.left_wheel_rad_s, first.left_wheel_rad_s)

    def test_safe_hold_stops_and_requests_brake(self) -> None:
        self.controller.step(10.0, 10.0, 0.1, RuntimeMode.NORMAL)

        result = self.controller.step(10.0, 10.0, 0.1, RuntimeMode.SAFE_HOLD)

        self.assertEqual(result.left_wheel_rad_s, 0.0)
        self.assertEqual(result.right_wheel_rad_s, 0.0)
        self.assertTrue(result.brake_request)

    def test_controlled_stop_decelerates_instead_of_reversing(self) -> None:
        moving = self.controller.step(10.0, 10.0, 0.1, RuntimeMode.NORMAL)

        stopped = self.controller.step(10.0, 10.0, 0.1, RuntimeMode.CONTROLLED_STOP)

        self.assertGreaterEqual(stopped.left_wheel_rad_s, 0.0)
        self.assertLess(stopped.left_wheel_rad_s, moving.left_wheel_rad_s)


if __name__ == "__main__":
    unittest.main()
