#!/usr/bin/env python

# Copyright 2025 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import logging
import time
from typing import Any

from lerobot.teleoperators import Teleoperator

from ..damiao.DM_CAN import DM_Motor_Type, Motor
from ..damiao.damiao import DamiaoMotorsBus, MotorCalibration, MotorNormMode
from .config_cleank_leader import CleankLeaderConfig

from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError

logger = logging.getLogger(__name__)


class CleankLeader(Teleoperator):
    """Teleoperator wrapper around the Damiao-powered CleanK leader arm."""

    config_class = CleankLeaderConfig
    name = "cleank_leader"

    def __init__(self, config: CleankLeaderConfig):
        super().__init__(config)
        self.config = config
        self.bus = DamiaoMotorsBus(
            port=self.config.port,
            motors={
                "shoulder_pan": Motor(DM_Motor_Type.DM6006, 0x01, 0x15),
                "shoulder_lift": Motor(DM_Motor_Type.DM6006, 0x02, 0x15),
                "shoulder_roll": Motor(DM_Motor_Type.DM4310, 0x03, 0x15),
                "elbow_flex": Motor(DM_Motor_Type.DM4310, 0x04, 0x15),
                "wrist_roll": Motor(DM_Motor_Type.DM4310, 0x05, 0x15),
            },
            calibration=self.calibration,
            motor_norm_mode=self.config.motor_norm_mode,
            control_type=self.config.control_type,
        )

    @property
    def action_features(self) -> dict[str, type]:
        """The leader streams normalized joint positions and velocities for each motor."""
        features = {}
        for motor in self.bus.motor_names:
            features[f"{motor}.pos"] = float
            features[f"{motor}.vel"] = float
        return features

    @property
    def feedback_features(self) -> dict[str, type]:
        """No force feedback is implemented for this leader."""
        return {}

    @property
    def is_connected(self) -> bool:
        return self.bus.is_connected

    def connect(self, calibrate: bool = True) -> None:
        """Open the motor bus and optionally run calibration."""
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        self.bus.connect()
        if not self.is_calibrated and calibrate:
            logger.info(
                "Mismatch between calibration values in the motor and the calibration file or no calibration file found"
            )
            self.calibrate()

        self.configure()
        logger.info(f"{self} connected.")

    @property
    def is_calibrated(self) -> bool:
        return self.bus.is_calibrated

    def calibrate(self) -> None:
        """Interactively record offsets/ranges of motion and persist them."""
        if self.calibration:
            user_input = input(
                f"Press ENTER to use provided calibration file associated with the id {self.id}, or type 'c' and press ENTER to run calibration: "
            )
            if user_input.strip().lower() != "c":
                logger.info(f"Writing calibration file associated with the id {self.id} to the motors")
                self.bus.write_calibration(self.calibration)
                return

        logger.info(f"\nRunning calibration of {self}")
        with self.bus.torque_disabled():
            input(f"Move {self} to the middle of its range of motion and press ENTER....")
            homing_offsets = self.bus.reset_offset()

            print(
                "Move all joints sequentially through their entire ranges "
                "of motion.\nRecording positions. Press ENTER to stop..."
            )
            range_mins, range_maxes = self.bus.record_ranges_of_motion()

        self.calibration = {}
        for motor, m in self.bus.motors.items():
            self.calibration[motor] = MotorCalibration(
                id=m.SlaveID,
                motor_offset=homing_offsets[motor],
                range_min=range_mins[motor],
                range_max=range_maxes[motor],
            )

        self.bus.write_calibration(self.calibration)
        self._save_calibration()
        print("Calibration saved to", self.calibration_fpath)

    def configure(self) -> None:
        """Hook reserved for post-connection adjustments."""
        pass

    def get_action(self) -> dict[str, Any]:
        """Return the latest normalized joint positions and velocities from the leader arm."""
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        start = time.perf_counter()
        motor_state = self.bus.sync_read()
        action = {
            key: val
            for key, val in motor_state.items()
            if key.endswith(".pos") or key.endswith(".vel")
        }
        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} read action: {dt_ms:.1f}ms")
        return action

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        """No haptic/force feedback is supported yet."""
        raise NotImplementedError("Force feedback is not implemented for the CleanK leader.")

    def disconnect(self) -> None:
        """Disconnect the teleoperator safely."""
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        self.bus.disconnect(self.config.disable_torque_on_disconnect)
        logger.info(f"{self} disconnected.")
