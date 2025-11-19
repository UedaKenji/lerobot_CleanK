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

from dataclasses import dataclass

from lerobot.teleoperators import TeleoperatorConfig
from ..damiao.damiao import MotorNormMode
from ..damiao.DM_CAN import Control_Type


@TeleoperatorConfig.register_subclass("cleank_leader")
@dataclass
class CleankLeaderConfig(TeleoperatorConfig):
    """Static configuration for the CleanK leader teleoperator (ports, control mode)."""

    port: str
    disable_torque_on_disconnect: bool = True
    control_type: Control_Type = Control_Type.MIT
    motor_norm_mode: MotorNormMode = MotorNormMode.CENTERING
