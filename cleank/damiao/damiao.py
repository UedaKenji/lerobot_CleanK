from typing import Dict, TypeAlias
from serial import Serial
from functools import cached_property

import logging
import time
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from .DM_CAN import Control_Type, DM_Motor_Type, MotorControl, Motor

DEFAULT_BAUDRATE = 921600

NameOrID: TypeAlias = str | int
Value: TypeAlias = int | float

logger = logging.getLogger(__name__)


class DamiaoMotorsBus:
    def __init__(self, port: str, motors: Dict[str, Motor], baudrate: int = DEFAULT_BAUDRATE, control_type=Control_Type.MIT):
        self.port = port
        self.motors = motors
        self.baudrate = baudrate
        self.control_type = control_type
        self.serial_device = Serial(port, baudrate)
        # Track when motor telemetry was last refreshed so we can throttle polling
        self._last_refresh_time: float | None = None

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

        pass

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
        
        if not self.is_connected:
            raise DeviceNotConnectedError(
                f"{self.__class__.__name__}('{self.port}') is not connected. You need to run `{self.__class__.__name__}.connect()`."
            )
        
        now = time.monotonic()
        needs_refresh = (
            self._last_refresh_time is None or (now - self._last_refresh_time) >= 0.5
        )
        if needs_refresh:
            # Only poll the motors again if enough time elapsed since the last refresh
            for motor in self.motors.values():
                self.motorcontrol.refresh_motor_status(motor)
            self.motorcontrol.recv()
            self._last_refresh_time = now

        res = {}

        res.update({f'{motor_name}.pos': self.motorcontrol.getPosition(motor) for motor_name, motor in self.motors.items()})
        res.update({f'{motor_name}.vel': self.motorcontrol.getVelocity(motor) for motor_name, motor in self.motors.items()})
        res.update({f'{motor_name}.tor': self.motorcontrol.getTorque(motor) for motor_name, motor in self.motors.items()})
        
        return res 
