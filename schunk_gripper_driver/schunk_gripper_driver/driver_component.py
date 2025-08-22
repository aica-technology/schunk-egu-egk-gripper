#!/usr/bin/env python3
# Copyright 2025 SCHUNK SE & Co. KG
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation, either version 3 of the License, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along with
# this program. If not, see <https://www.gnu.org/licenses/>.
# --------------------------------------------------------------------------------

from typing import Optional, TypedDict

import state_representation as sr
import yaml
from modulo_components.lifecycle_component import LifecycleComponent
from sensor_msgs.msg import JointState

from schunk_gripper_interfaces.msg import GripperState
from schunk_gripper_interfaces.srv import ShowGripperSpecification
from schunk_gripper_library.driver import Driver as GripperDriver


class Gripper(TypedDict):
    host: str
    port: int
    serial_port: str
    device_id: int
    driver: GripperDriver


class Driver(LifecycleComponent):
    def __init__(self, node_name: str, **kwargs):
        super().__init__(node_name, **kwargs)

        self.add_parameter(sr.Parameter("host", sr.ParameterType.STRING), "The gripper's TCP/IP host address")
        self.add_parameter(sr.Parameter("port", sr.ParameterType.INT), "The gripper's TCP/IP port")
        self.add_parameter(sr.Parameter("serial_port", sr.ParameterType.STRING), "The gripper's serial port")
        self.add_parameter(sr.Parameter("device_id", sr.ParameterType.INT), "The gripper's Modbus device id")

        self.gripper: Optional[Gripper] = None

        self._joint_state = JointState()
        self._joint_state.name.append("gripper")
        self.add_output("joint_state", "_joint_state", JointState)

        self._gripper_state = GripperState()
        self.add_output("gripper_state", "_gripper_state", GripperState)

        self.add_service("acknowledge", self._acknowledge_cb)
        self.add_service("fast_stop", self._fast_stop_cb)
        self.add_service("move_to_absolute_position", self._move_to_absolute_position_cb)
        self.add_service("grip", self._grip_cb)
        self.add_service("release", self._release_cb)

        self.create_service(ShowGripperSpecification, "~/show_specification", self._show_gripper_specification_cb)

    def add_gripper(self) -> Optional[Gripper]:
        host = self.get_parameter("host")
        port = self.get_parameter("port")
        serial_port = self.get_parameter("serial_port")
        device_id = self.get_parameter("device_id")

        gripper = None
        if all([host.is_empty(), port.is_empty(), serial_port.is_empty(), device_id.is_empty()]):
            self.get_logger().warn("Empty gripper")
        elif host and port:
            gripper = {
                "host": host.get_value(),
                "port": port.get_value(),
                "serial_port": "",
                "device_id": "",
                "driver": GripperDriver(),
            }
        elif (host and port.is_empty()) or (port and host.is_empty()):
            self.get_logger().warn("Missing host or port, both need to be specified")
        elif serial_port and device_id:
            gripper = {
                "host": "",
                "port": "",
                "serial_port": serial_port.get_value(),
                "device_id": device_id.get_value(),
                "driver": GripperDriver(),
            }
        elif (serial_port and device_id.is_empty()) or (device_id and serial_port.is_empty()):
            self.get_logger().warn("Missing serial_port or device_id, both need to be specified")

        return gripper

    def on_configure_callback(self) -> bool:
        self.gripper = self.add_gripper()
        if self.gripper is None:
            self.get_logger().error("No valid gripper specified")
            return False
        driver = GripperDriver()
        if not driver.connect(
            host=self.gripper["host"],
            port=self.gripper["port"],
            serial_port=self.gripper["serial_port"],
            device_id=self.gripper["device_id"],
            update_cycle=self.get_period(),
        ):
            self.get_logger().error(f"Gripper connect failed: {self.gripper}")
            return False
        self.gripper["driver"] = driver

        return True

    def on_activate_callback(self) -> bool:
        return self.gripper["driver"].acknowledge()

    def on_cleanup_callback(self) -> bool:
        if self.gripper:
            self.gripper["driver"].disconnect()
            self.gripper["driver"] = GripperDriver()
        return True

    def on_step_callback(self):
        self._joint_state.header.stamp = self.get_clock().now().to_msg()
        self._joint_state.position = [self.gripper["driver"].get_actual_position() / 1e6]

        self._gripper_state.header.stamp = self.get_clock().now().to_msg()
        status = self.gripper["driver"].get_status_diagnostics().split(",")
        self._gripper_state.error_code = status[0].strip()
        self._gripper_state.warning_code = status[1].strip()
        self._gripper_state.additional_code = status[2].strip()

        self._gripper_state.bit0_ready_for_operation = bool(self.gripper["driver"].get_status_bit(bit=0))
        self._gripper_state.bit1_control_authority_fieldbus = bool(self.gripper["driver"].get_status_bit(bit=1))
        self._gripper_state.bit2_ready_for_shutdown = bool(self.gripper["driver"].get_status_bit(bit=2))
        self._gripper_state.bit3_not_feasible = bool(self.gripper["driver"].get_status_bit(bit=3))
        self._gripper_state.bit4_command_successfully_processed = bool(self.gripper["driver"].get_status_bit(bit=4))
        self._gripper_state.bit5_command_received_toggle = bool(self.gripper["driver"].get_status_bit(bit=5))
        self._gripper_state.bit6_warning = bool(self.gripper["driver"].get_status_bit(bit=6))
        self._gripper_state.bit7_error = bool(self.gripper["driver"].get_status_bit(bit=7))
        self._gripper_state.bit8_released_for_manual_movement = bool(self.gripper["driver"].get_status_bit(bit=8))
        self._gripper_state.bit9_software_limit_reached = bool(self.gripper["driver"].get_status_bit(bit=9))
        self._gripper_state.bit11_no_workpiece_detected = bool(self.gripper["driver"].get_status_bit(bit=11))
        self._gripper_state.bit12_workpiece_gripped = bool(self.gripper["driver"].get_status_bit(bit=12))
        self._gripper_state.bit13_position_reached = bool(self.gripper["driver"].get_status_bit(bit=13))
        self._gripper_state.bit14_workpiece_pre_grip_started = bool(self.gripper["driver"].get_status_bit(bit=14))
        self._gripper_state.bit16_workpiece_lost = bool(self.gripper["driver"].get_status_bit(bit=16))
        self._gripper_state.bit17_wrong_workpiece_gripped = bool(self.gripper["driver"].get_status_bit(bit=17))
        self._gripper_state.bit31_grip_force_and_position_maintenance_activated = bool(
            self.gripper["driver"].get_status_bit(bit=31)
        )

    def _show_gripper_specification_cb(
        self, _: ShowGripperSpecification.Request, response: ShowGripperSpecification.Response
    ):
        spec = self.gripper["driver"].show_specification()
        if not spec:
            response.success = False
            response.message = self.gripper["driver"].get_status_diagnostics()
            return response

        response.specification.max_stroke = spec["max_stroke"]
        response.specification.max_speed = spec["max_speed"]
        response.specification.max_force = spec["max_force"]
        response.specification.serial_number = spec["serial_number"]
        response.specification.firmware_version = spec["firmware_version"]
        response.specification.device_id = spec["device_id"]
        response.specification.ip_address = spec["ip_address"]
        response.success = True
        response.message = self.gripper["driver"].get_status_diagnostics()
        return response

    def _acknowledge_cb(self) -> dict:
        return {
            "success": self.gripper["driver"].acknowledge(),
            "message": self.gripper["driver"].get_status_diagnostics(),
        }

    def _fast_stop_cb(self) -> dict:
        return {
            "success": self.gripper["driver"].fast_stop(),
            "message": self.gripper["driver"].get_status_diagnostics(),
        }

    def _move_to_absolute_position_cb(self, payload) -> dict:
        try:
            request = yaml.safe_load(payload)
        except yaml.YAMLError as e:
            return {"success": False, "message": f"Failed to parse YAML: {e}"}
        try:
            position = int(request["position"]) * 1e6
            velocity = int(request["velocity"]) * 1e6
        except (KeyError, ValueError) as e:
            return {"success": False, "message": f"Failed to parse request: {e}"}
        use_gpe = bool(request.get("use_gpe", False))

        success = self.gripper["driver"].move_to_absolute_position(
            position=position, velocity=velocity, use_gpe=use_gpe
        )
        return {"success": success, "message": self.gripper["driver"].get_status_diagnostics()}

    def _grip_cb(self, payload) -> dict:
        try:
            request = yaml.safe_load(payload)
        except yaml.YAMLError as e:
            return {"success": False, "message": f"Failed to parse YAML: {e}"}
        try:
            force = int(request["force"])
        except (KeyError, ValueError) as e:
            return {"success": False, "message": f"Failed to parse request: {e}"}
        use_gpe = bool(request.get("use_gpe", False))
        outward = bool(request.get("outward", False))
        success = self.gripper["driver"].grip(force=force, use_gpe=use_gpe, outward=outward)
        return {"success": success, "message": self.gripper["driver"].get_status_diagnostics()}

    def _release_cb(self, payload) -> dict:
        use_gpe = True if (payload == "true" or payload == "True") else False
        success = self.gripper["driver"].release(use_gpe=use_gpe)
        return {"success": success, "message": self.gripper["driver"].get_status_diagnostics()}
