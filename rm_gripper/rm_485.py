"""
Rielman Robotic Arm Modbus Gripper Base Module
Provides reusable Modbus RTU communication functionality
"""

import time
import logging
from dataclasses import dataclass
from typing import List, Optional
import os
import sys

from Robotic_Arm.rm_robot_interface import (
    RoboticArm,
    rm_thread_mode_e,
    rm_peripheral_read_write_params_t
)

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)


@dataclass
class GripperStatus:
    """Unified gripper status data model"""
    position: float = 0.0      # Position (percentage 0-100)
    speed: float = 0.0
    current: float = 0.0
    state: str = "Unknown"
    raw_data: Optional[List[int]] = None
    is_initialized: bool = False


class ModbusGripperBase:
    """Modbus gripper base class - handles all communication logic"""
    
    def __init__(self, robot_ip: str, port: int = 1, slave_addr: int = 0x09, baudrate: int = 115200):
        self.robot_ip = robot_ip
        self.port = port
        self.slave_addr = slave_addr
        self.baudrate = baudrate
        self.arm = None
        self.handle = None
        self._connected = False

    def connect(self) -> None:
        """Connect to robot arm and configure Modbus"""
        if self._connected:
            return
            
        self.arm = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
        self.handle = self.arm.rm_create_robot_arm(self.robot_ip, 8080)
        
        if not self.handle or self.handle.id <= 0:
            raise RuntimeError(f"Failed to connect to arm: {self.robot_ip}")
        
        logger.info(f"Successfully connected to arm (IP: {self.robot_ip})")
        
        self.arm.rm_close_modbus_mode(self.port)
        res = self.arm.rm_set_modbus_mode(port=self.port, baudrate=self.baudrate, timeout=10)
        if res != 0:
            raise RuntimeError(f"Modbus configuration failed (error code: {res})")
        
        self._connected = True
        logger.info(f"Modbus RTU configured (port={self.port}, baud={self.baudrate}, slave=0x{self.slave_addr:02X})")

    def _create_params(self, address: int, num: int = 1):
        """Create communication parameters"""
        return rm_peripheral_read_write_params_t(
            port=self.port, address=address, device=self.slave_addr, num=num
        )

    def write_registers(self, address: int, values: List[int], num: int = 1) -> None:
        """Write registers"""
        self.connect()
        params = self._create_params(address, num)
        res = self.arm.rm_write_registers(params, values)
        if res != 0:
            raise RuntimeError(f"Failed to write register (address: 0x{address:04X}, error: {res})")
        time.sleep(0.1)

    def read_holding_registers(self, address: int, num: int = 1) -> List[int]:
        """Read holding registers"""
        self.connect()
        params = self._create_params(address, num)
        res, data = self.arm.rm_read_holding_registers(params)
        if res != 0:
            raise RuntimeError(f"Failed to read holding register (address: 0x{address:04X}, error: {res})")
        if isinstance(data, (int, float)):
            return [int(data)]
        return data if isinstance(data, list) else [data]

    def read_input_registers(self, address: int, num: int = 1) -> List[int]:
        """Read input registers"""
        self.connect()
        params = self._create_params(address, num)
        res, data = self.arm.rm_read_multiple_input_registers(params)
        if res != 0:
            raise RuntimeError(f"Failed to read input register (address: 0x{address:04X}, error: {res})")
        if isinstance(data, (int, float)):
            return [int(data)]
        return data if isinstance(data, list) else [data]

    def read_multiple_holding_registers(self, address: int, num: int = 6) -> List[int]:
        """Read multiple holding registers (mainly for Hitbot)"""
        self.connect()
        params = self._create_params(address, num)
        res, data = self.arm.rm_read_multiple_holding_registers(params)
        if res != 0:
            raise RuntimeError(f"Failed to read multiple holding registers (address: 0x{address:04X}, error: {res})")
        if isinstance(data, (int, float)):
            return [int(data)]
        return data if isinstance(data, list) else list(data)

    def disconnect(self) -> None:
        """Disconnect Modbus"""
        if self.arm and self._connected:
            try:
                self.arm.rm_close_modbus_mode(self.port)
                self.arm.rm_delete_robot_arm()
            except:
                pass
        self._connected = False
        logger.info("Modbus connection closed")