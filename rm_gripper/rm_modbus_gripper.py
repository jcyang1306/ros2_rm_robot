import threading
import time
import struct
import argparse
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from rm_ros_interfaces.msg import Modbusreaddata, Modbusrtureadparams, Modbusrtuwriteparams, RS485params
from std_msgs.msg import Bool


class RMModbusGripper(Node):
    """Base class for RM Modbus RTU grippers.

    This class only handles transport-level operations:
    - RS485 mode setup
    - Modbus register read/write send and receive
    - generic initialization entrypoint
    """

    def __init__(
        self,
        robot_ip: str,
        port: int,
        slave_addr: int,
        baudrate: int,
        name_prefix: str = "",
        node_name: str = "rm_modbus_gripper",
    ):
        self._owns_rclpy_context = False
        if not rclpy.ok():
            rclpy.init()
            self._owns_rclpy_context = True

        super().__init__(node_name)
        self.robot_ip = robot_ip
        self.port = port
        self.slave_addr = slave_addr
        self.baudrate = baudrate
        self.name_prefix = name_prefix.strip("/")

        self._connected = False
        self._last_rs485_ok: Optional[bool] = None
        self._last_write_ok: Optional[bool] = None
        self._last_read_result: Optional[Modbusreaddata] = None

        self._rs485_event = threading.Event()
        self._write_event = threading.Event()
        self._read_event = threading.Event()

        set_controller_topic = self._rm_driver_topic("set_controller_rs485_mode_cmd")
        set_controller_result_topic = self._rm_driver_topic("set_controller_rs485_mode_result")
        write_registers_topic = self._rm_driver_topic("write_modbus_rtu_registers_cmd")
        read_holding_topic = self._rm_driver_topic("read_modbus_rtu_holding_registers_cmd")
        write_registers_result_topic = self._rm_driver_topic("write_modbus_rtu_registers_result")
        read_holding_result_topic = self._rm_driver_topic("read_modbus_rtu_holding_registers_result")

        self.set_controller_rs485_mode_publisher = self.create_publisher(
            RS485params, set_controller_topic, 10
        )
        self.write_modbus_rtu_registers_publisher = self.create_publisher(
            Modbusrtuwriteparams, write_registers_topic, 10
        )
        self.read_modbus_rtu_holding_registers_publisher = self.create_publisher(
            Modbusrtureadparams, read_holding_topic, 10
        )

        self.create_subscription(Bool, set_controller_result_topic, self._on_rs485_result, 10)
        self.create_subscription(Bool, write_registers_result_topic, self._on_write_result, 10)
        self.create_subscription(Modbusreaddata, read_holding_result_topic, self._on_read_result, 10)

    def _rm_driver_topic(self, topic_name: str) -> str:
        if self.name_prefix:
            return f"/{self.name_prefix}/rm_driver/{topic_name}"
        return f"rm_driver/{topic_name}"

    def _on_rs485_result(self, msg: Bool) -> None:
        self._last_rs485_ok = bool(msg.data)
        self._rs485_event.set()

    def _on_write_result(self, msg: Bool) -> None:
        self._last_write_ok = bool(msg.data)
        self._write_event.set()

    def _on_read_result(self, msg: Modbusreaddata) -> None:
        self._last_read_result = msg
        self._read_event.set()

    def _wait_event(self, event: threading.Event, timeout_s: float) -> bool:
        deadline = time.time() + timeout_s
        while rclpy.ok() and time.time() < deadline:
            if event.is_set():
                return True
            rclpy.spin_once(self, timeout_sec=0.05)
        return event.is_set()

    def _safe_subscription_count(self, publisher) -> int:
        if not rclpy.ok():
            return 0
        try:
            return publisher.get_subscription_count()
        except Exception:
            return 0

    def _wait_for_subscriber(self, publisher, timeout_s: float = 1.0) -> int:
        deadline = time.time() + timeout_s
        while rclpy.ok() and time.time() < deadline:
            count = self._safe_subscription_count(publisher)
            if count > 0:
                return count
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)
        return self._safe_subscription_count(publisher)

    def connect(self, timeout_s: float = 1.0) -> bool:
        """Configure RS485 Modbus mode once and cache connected state."""
        if self._connected:
            return True

        rs485_params = RS485params()
        rs485_params.mode = 1
        rs485_params.baudrate = int(self.baudrate)
        rs485_params.state = False

        if self._wait_for_subscriber(self.set_controller_rs485_mode_publisher, timeout_s=1.5) == 0:
            self.get_logger().error(
                "No subscriber for rm_driver/set_controller_rs485_mode_cmd. "
                "rm_driver may not be running."
            )
            return False

        self._rs485_event.clear()
        self._last_rs485_ok = None
        self.set_controller_rs485_mode_publisher.publish(rs485_params)

        if not self._wait_event(self._rs485_event, timeout_s):
            self.get_logger().warning("No set_controller_rs485_mode_result before timeout.")
            return False
        if not self._last_rs485_ok:
            self.get_logger().error("Failed to set controller RS485 mode.")
            return False

        self._connected = True
        self.get_logger().info("RS485 Modbus mode configured successfully.")
        return True

    def init(self, timeout_s: float = 1.0) -> bool:
        """Base initialization: ensure transport is connected."""
        return self.connect(timeout_s=timeout_s)

    def write_registers(
        self, address: int, values: List[int], num: Optional[int] = None, timeout_s: float = 1.0
    ) -> bool:
        """Write Modbus RTU holding registers."""
        
        verbose = True
        if not self.connect():
            return False

        write_params = Modbusrtuwriteparams()
        write_params.address = int(address)
        write_params.device = int(self.slave_addr)
        write_params.type = int(self.port)
        write_params.num = int(num if num is not None else len(values))
        write_params.data = [int(v) for v in values]

        self._write_event.clear()
        self._last_write_ok = None
        if self._wait_for_subscriber(self.write_modbus_rtu_registers_publisher, timeout_s=1.0) == 0:
            self.get_logger().error(
                "No subscriber for rm_driver/write_modbus_rtu_registers_cmd. "
                "rm_driver may not be running."
            )
            return False
        self.write_modbus_rtu_registers_publisher.publish(write_params)
        if verbose:
            self.get_logger().info(f"Wrote registers to {address} with values {values}")

        if not self._wait_event(self._write_event, timeout_s):
            self.get_logger().warning("No write result before timeout.")
            return False
        if verbose:
            self.get_logger().info(f"Write result: {bool(self._last_write_ok)}")
        return bool(self._last_write_ok)

    def read_registers(self, address: int, num: int, timeout_s: float = 1.0) -> Optional[List[int]]:
        """Read Modbus RTU holding registers."""
        if not self.connect():
            return None

        read_params = Modbusrtureadparams()
        read_params.address = int(address)
        read_params.device = int(self.slave_addr)
        read_params.type = int(self.port)
        read_params.num = int(num)

        self._read_event.clear()
        self._last_read_result = None
        if self._wait_for_subscriber(self.read_modbus_rtu_holding_registers_publisher, timeout_s=1.0) == 0:
            self.get_logger().error(
                "No subscriber for rm_driver/read_modbus_rtu_holding_registers_cmd. "
                "rm_driver may not be running."
            )
            return None
        self.read_modbus_rtu_holding_registers_publisher.publish(read_params)

        if not self._wait_event(self._read_event, timeout_s):
            self.get_logger().warning("No read result before timeout.")
            return None

        if self._last_read_result is None or not self._last_read_result.state:
            return None
        return list(self._last_read_result.read_data)

    def disconnect(self) -> None:
        self._connected = False
        try:
            self.destroy_node()
        except Exception:
            pass
        if self._owns_rclpy_context and rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


class RobotiqGripper(RMModbusGripper):
    """Robotiq gripper protocol implementation on top of RM Modbus base."""

    COMMAND_REGISTER = 0x03E8
    STATUS_REGISTER = 0x07D0

    def __init__(
        self,
        robot_ip: str = "192.168.1.19",
        port: int = 1,
        slave_addr: int = 0x09,
        baudrate: int = 115200,
        name_prefix: str = "left_arm_controller",
    ):
        super().__init__(
            robot_ip=robot_ip,
            port=port,
            slave_addr=slave_addr,
            baudrate=baudrate,
            name_prefix=name_prefix,
            node_name="robotiq_gripper",
        )

    @staticmethod
    def _validate_params(position: float, speed: float, force: float) -> None:
        if not (0 <= position <= 100):
            raise ValueError("position must be between 0 and 100")
        if not (0 <= speed <= 100):
            raise ValueError("speed must be between 0 and 100")
        if not (0 <= force <= 100):
            raise ValueError("force must be between 0 and 100")

    def initialize(self, timeout_s: float = 1.0) -> bool:
        """Initialize Robotiq gripper: reset then activate."""
        if not super().init(timeout_s=timeout_s):
            return False
        time.sleep(0.5)
        return True

    def open(self, position: float = 80, speed: float = 50, force: float = 50) -> bool:
        self._validate_params(position, speed, force)
        pos_val = int((100 - position) / 100.0 * 255)
        speed_val = int(speed / 100.0 * 255)
        force_val = int(force / 100.0 * 255)

        values = [0x09, 0x00, 0x00, pos_val, speed_val, force_val]
        ok = self.write_registers(self.COMMAND_REGISTER, values, num=3)
        self.get_logger().info(
            f"Robotiq open -> position:{position:.1f}%(raw:{pos_val}), "
            f"speed:{speed:.1f}%(raw:{speed_val}), force:{force:.1f}%(raw:{force_val}), ok:{ok}"
        )
        time.sleep(1.0)
        return ok

    def close(self, position: float = 20, speed: float = 50, force: float = 50) -> bool:
        return self.open(position=position, speed=speed, force=force)

    def get_status(self) -> Optional[List[int]]:
        data = self.read_registers(address=self.STATUS_REGISTER, num=3)
        if data is not None:
            self.get_logger().info(f"Robotiq status raw: {data}")
        return data


class HitbotGripper(RMModbusGripper):
    """Hitbot/Lebai gripper protocol implementation on top of RM Modbus base."""

    INIT_REGISTER = 0x0000
    POSITION_REGISTER = 0x0002
    SPEED_REGISTER = 0x0004
    CURRENT_REGISTER = 0x0006
    INIT_STATUS_REGISTER = 0x0040
    GRIP_STATE_REGISTER = 0x0041
    TELEMETRY_REGISTER = 0x0042

    def __init__(
        self,
        robot_ip: str = "192.168.1.18",
        port: int = 1,
        slave_addr: int = 0x07,
        baudrate: int = 9600,
        name_prefix: str = "right_arm_controller",
    ):
        super().__init__(
            robot_ip=robot_ip,
            port=port,
            slave_addr=slave_addr,
            baudrate=baudrate,
            name_prefix=name_prefix,
            node_name="hitbot_gripper",
        )

    @staticmethod
    def _float_to_bytes(value: float) -> List[int]:
        return list(struct.pack(">f", float(value)))

    @staticmethod
    def _validate_params(position: float, speed: float, force: float) -> None:
        if not (0 <= position <= 100):
            raise ValueError("Hitbot position must be between 0 and 100")
        if not (1 <= speed <= 100):
            raise ValueError("Hitbot speed must be between 1 and 100")
        if not (0 <= force <= 100):
            raise ValueError("Hitbot force must be between 0 and 100")

    def initialize(self, timeout_s: float = 1.0) -> bool:
        if not super().init(timeout_s=timeout_s):
            return False
        ok = self.write_registers(self.INIT_REGISTER, [0x00, 0x01], num=1)
        if ok:
            time.sleep(3.0)
        return ok

    def open(self, position: float = 90, speed: float = 50, force: float = 50) -> bool:
        self._validate_params(position, speed, force)
        mm_pos = (position / 100.0) * 50.0
        current_a = 0.1 + (force / 100.0) * 0.4

        ok_pos = self.write_registers(self.POSITION_REGISTER, self._float_to_bytes(mm_pos), num=2)
        ok_speed = self.write_registers(self.SPEED_REGISTER, self._float_to_bytes(speed), num=2)
        ok_current = self.write_registers(self.CURRENT_REGISTER, self._float_to_bytes(current_a), num=2)
        ok = ok_pos and ok_speed and ok_current

        self.get_logger().info(
            f"Hitbot open -> position:{mm_pos:.1f}mm, speed:{speed:.1f}mm/s, "
            f"current:{current_a:.3f}A, ok:{ok}"
        )
        return ok

    def close(self, position: float = 10, speed: float = 50, force: float = 50) -> bool:
        return self.open(position=position, speed=speed, force=force)

    @staticmethod
    def _to_byte_buffer(data: List[int]) -> List[int]:
        if len(data) >= 12:
            return [int(v) & 0xFF for v in data[:12]]
        if len(data) >= 6:
            # Some drivers return 16-bit register values; split into high/low bytes.
            buf: List[int] = []
            for reg in data[:6]:
                value = int(reg) & 0xFFFF
                buf.extend([(value >> 8) & 0xFF, value & 0xFF])
            return buf
        return []

    def get_status(self) -> Optional[Dict[str, object]]:
        init_raw = self.read_registers(self.INIT_STATUS_REGISTER, num=1)
        state_raw = self.read_registers(self.GRIP_STATE_REGISTER, num=1)
        telemetry_raw = self.read_registers(self.TELEMETRY_REGISTER, num=6)
        if init_raw is None or state_raw is None or telemetry_raw is None:
            return None

        init_status = int(init_raw[0])
        grip_state = int(state_raw[0])
        payload = self._to_byte_buffer(telemetry_raw)
        if len(payload) < 12:
            self.get_logger().warning(f"Hitbot telemetry length insufficient: {len(telemetry_raw)}")
            return None

        pos = struct.unpack(">f", bytes(payload[0:4]))[0]
        vel = struct.unpack(">f", bytes(payload[4:8]))[0]
        cur = struct.unpack(">f", bytes(payload[8:12]))[0]
        pos_percent = round((pos / 50.0) * 100.0, 1)

        status = {
            "position": pos_percent,
            "speed": round(vel, 1),
            "current": round(cur, 3),
            "state": f"Init:{init_status} State:{grip_state}",
            "is_initialized": init_status == 5,
            "raw_data": telemetry_raw,
        }
        self.get_logger().info(f"Hitbot status: {status}")
        return status


def _parse_slave_addr(value: str) -> int:
    return int(value, 0)


def main(args=None) -> None:
    parser = argparse.ArgumentParser(description="RM Modbus gripper demo node")
    parser.add_argument(
        "--gripper-type",
        choices=["robotiq", "hitbot"],
        default="robotiq",
        help="Which gripper implementation to run.",
    )
    parser.add_argument("--robot-ip", default=None, help="Override robot IP.")
    parser.add_argument("--port", type=int, default=None, help="Override Modbus port.")
    parser.add_argument(
        "--slave-addr",
        type=_parse_slave_addr,
        default=None,
        help="Override Modbus slave address, e.g. 9 or 0x09.",
    )
    parser.add_argument("--baudrate", type=int, default=None, help="Override Modbus baudrate.")
    parser.add_argument("--name-prefix", default=None, help="Override ROS topic namespace prefix.")
    parsed_args, ros_args = parser.parse_known_args(args=args)

    if not rclpy.ok():
        rclpy.init(args=ros_args)

    init_kwargs = {
        "robot_ip": parsed_args.robot_ip,
        "port": parsed_args.port,
        "slave_addr": parsed_args.slave_addr,
        "baudrate": parsed_args.baudrate,
        "name_prefix": parsed_args.name_prefix,
    }
    init_kwargs = {k: v for k, v in init_kwargs.items() if v is not None}

    if parsed_args.gripper_type == "hitbot":
        gripper = HitbotGripper(**init_kwargs)
    else:
        gripper = RobotiqGripper(**init_kwargs)

    try:
        gripper.initialize()
        gripper.open()
        time.sleep(1.0)
        gripper.close()
        time.sleep(1.0)
        gripper.open()
        time.sleep(1.0)
        gripper.get_status()
    finally:
        gripper.disconnect()


if __name__ == "__main__":
    main()