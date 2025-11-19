from typing import Dict, TypeAlias
from serial import Serial
from functools import cached_property

import logging
import time
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from .DM_CAN import Control_Type, DM_Motor_Type, MotorControl, Motor, DM_variable

DEFAULT_BAUDRATE = 921600

NameOrID: TypeAlias = str | int
Value: TypeAlias = int | float

DEFAULT_KP = 20.0
DEFAULT_KD = 1
DEFAULT_TORQUE_LIMIT = 10.0
DEFAULT_VELOCITY_LIMIT = 2.0

logger = logging.getLogger(__name__)


class DamiaoMotorsBus:
    def __init__(self, port: str, motors: Dict[str, Motor], baudrate: int = DEFAULT_BAUDRATE, control_type=Control_Type.MIT):
        self.port = port
        self.motors = motors
        self.baudrate = baudrate
        self.control_type = control_type
        self.serial_device = Serial(port, baudrate)
        self._last_refresh_time: float | None = None #Track when motor telemetry was last refreshed so we can throttle polling

        self.motorcontrol = MotorControl(self.serial_device)
        self.motorcontrol.serial_.close()
        for motor in self.motors.values():
            self.motorcontrol.addMotor(motor)
            logger.debug(f"Added motor {motor} to DamiaoMotorsBus on port {self.port}")

    def _get_motor_names(self,):
        return list(self.motors.keys())
    
    @cached_property
    def motor_names(self):
        return self._get_motor_names()

    def connect(self):
        
        if self.is_connected:
            raise DeviceAlreadyConnectedError(
                f"{self.__class__.__name__}('{self.port}') is already connected. Do not call `{self.__class__.__name__}.connect()` twice."
            )
        self.motorcontrol.serial_.open()
        logger.debug(f"{self.__class__.__name__} connected.")

        for motor in self.motors.values():
            try:
                self.motorcontrol.switchControlMode(motor, self.control_type)
                logger.info(f"Switched motor {motor} to {self.control_type} control mode")
                self.motorcontrol.enable(motor)
                logger.info(f"Enabled motor {motor}")
            except Exception as e:
                logger.warning(f"Failed to switch motor {motor} to {self.control_type} control mode: {e}")


    def is_connected(self) -> bool:
        return self.motorcontrol.serial_.is_open

    def disconnect(self, disable_torque: bool = True) -> None:
        """Close the serial port (optionally disabling torque first).

        Args:
            disable_torque (bool, optional): If `True` (default) torque is disabled on every motor before
                closing the port. This can prevent damaging motors if they are left applying resisting torque
                after disconnect.
        """
        if not self.is_connected:
            raise DeviceNotConnectedError(
                f"{self.__class__.__name__}('{self.port}') is not connected. Try running `{self.__class__.__name__}.connect()` first."
            )
        
        if disable_torque:
            for motor in self.motors.values():
                self.motorcontrol.disable(motor)
        self.motorcontrol.serial_.close()
        logger.debug(f"{self.__class__.__name__} disconnected.")
    

        
    
    def reset_offset(self,
        motors: str | list[str] | None = None
    ) -> dict[str, int]:
        """Reset the homing offset of several motors to zero.

        Args:
            motors (str | list[str] | None, optional): Motors to reset. Defaults to all motors (`None`).
        Returns:
            dict[str, int]: Mapping *motor name → new offset*.
        """
        if motors is None:
            motors = self.motors
        elif isinstance(motors, str):
            motors = {motors: self.motors[motors]}
        elif isinstance(motors, list):
            motors = {motor_name: self.motors[motor_name] for motor_name in motors}
        else:
            raise TypeError(motors)
        
        offset = {}

        for motor_name, motor in motors.items():
            self.motorcontrol.set_zero_position(motor)

            time.sleep(0.05)  # small delay to ensure command is processed
            offset[motor_name] = self.motorcontrol.read_motor_param(motor, RID=DM_variable.m_off)

        return offset


            

    def sync_read(
        self,
        motors: str | list[str] | None = None,
        *,
        normalize: bool = True,
        num_retry: int = 0,
    ) -> dict[str, Value]:
        """Read the same register from several motors at once.

        Args:
            data_name (str): Register name.
            motors (str | list[str] | None, optional): Motors to query. `None` (default) reads every motor.
            normalize (bool, optional): Normalisation flag.  Defaults to `True`.
            num_retry (int, optional): Retry attempts.  Defaults to `0`.

        Returns:
            dict[str, Value]: Mapping *motor name → value*.
        """

        if motors is None:
            motors = self.motors
        elif isinstance(motors, str):
            motors = {motors: self.motors[motors]}
        elif isinstance(motors, list):
            motors = {motor_name: self.motors[motor_name] for motor_name in motors}
        else:
            raise TypeError(motors)
        
        if not self.is_connected:
            raise DeviceNotConnectedError(
                f"{self.__class__.__name__}('{self.port}') is not connected. You need to run `{self.__class__.__name__}.connect()`."
            )

        now = time.monotonic()
        needs_refresh = (
            self._last_refresh_time is None or (now - self._last_refresh_time) >= 0.1
        )
        # 最後に更新してから100ms以上経っていたら再取得する
        if needs_refresh:
            for motor in motors.values():
                self.motorcontrol.refresh_motor_status(motor)
            self.motorcontrol.recv()
            self._last_refresh_time = now

        res = {}

        res.update({f'{motor_name}.pos': self.motorcontrol.getPosition(motor) for motor_name, motor in motors.items()})
        res.update({f'{motor_name}.vel': self.motorcontrol.getVelocity(motor) for motor_name, motor in motors.items()})
        res.update({f'{motor_name}.tor': self.motorcontrol.getTorque(motor) for motor_name, motor in motors.items()})
        
        return res 
    
    def sync_write(
        self,
        data_dict: dict[str, Value],
        normalize: bool = True,
    ) -> None:
        """Write the same register to several motors at once.

        Args:
            data_dict (dict[str, Value]): Mapping *motor name → value*.
            normalize (bool, optional): Normalisation flag.  Defaults to `True`.
            num_retry (int, optional): Retry attempts.  Defaults to `0`.
        """
        if not self.is_connected:
            raise DeviceNotConnectedError(
                f"{self.__class__.__name__}('{self.port}') is not connected. You need to run `{self.__class__.__name__}.connect()`."
            )
        
        if self.control_type == Control_Type.MIT:
            for motor_name, motor in data_dict.items():

                pos = data_dict[f'{motor_name}.pos']
                vel = data_dict.get(f'{motor_name}.vel', 0.0)
                self.motorcontrol.controlMIT(DM_Motor=motor, kp=DEFAULT_KP, kd=DEFAULT_KD, q=pos, dq=vel, tau=0.0)

        elif self.control_type == Control_Type.POS_VEL:
            for motor_name, motor in data_dict.items():

                pos = data_dict[f'{motor_name}.pos']
                self.motorcontrol.control_Pos_Vel(DM_Motor=motor, position=pos, velocity=DEFAULT_VELOCITY_LIMIT)
        
        else:
            raise NotImplementedError(f"Control type {self.control_type} is not implemented in DamiaoMotorsBus.sync_write()")
        

        
