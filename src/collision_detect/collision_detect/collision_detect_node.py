#!/usr/bin/env python3
"""
collision_detect_node

Reads simple bumper messages from an ESP32 over serial and commands the
robot to move a short distance in the opposite direction when a bumper
is triggered.

Behavior:
- Accepts ASCII lines from the ESP32 such as "FRONT" or "REAR" (case-insensitive).
- On FRONT: command a backward motion of `move_distance` meters.
- On REAR:  command a forward motion of `move_distance` meters.
- Two stopping modes are supported:
  - time-based (default): publish velocity for duration = distance / speed
  - odom-based: subscribe to `/odom` and stop when the integrated distance
	traveled (linear) reaches the target. Use this for better accuracy.

The node performs serial reads in a background thread so ROS timers and
callbacks remain responsive.
"""

from __future__ import annotations

import time
import threading
import math
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
# odometry not used: time-based motion only

try:
	import serial
except Exception:
	serial = None


class CollisionDetectNode(Node):
	def __init__(self) -> None:
		super().__init__('collision_detect')

		# parameters
		self.declare_parameter('serial_port', '/dev/esp32')
		self.declare_parameter('baudrate', 115200)
		self.declare_parameter('move_speed', 0.2)       # m/s
		self.declare_parameter('move_distance', 0.5)    # meters
		self.declare_parameter('move_duration', 3.0)    # seconds; if >0 overrides distance/speed
		self.declare_parameter('publish_hz', 10)

		self.serial_port: str = self.get_parameter('serial_port').value
		self.baudrate: int = int(self.get_parameter('baudrate').value)
		self.move_speed: float = float(self.get_parameter('move_speed').value)
		self.move_distance: float = float(self.get_parameter('move_distance').value)
		self.move_duration: float = float(self.get_parameter('move_duration').value)
		self.publish_hz: int = int(self.get_parameter('publish_hz').value)

		# publisher (publish to collision topic; use a mux to route to /cmd_vel)
		self.cmd_pub = self.create_publisher(Twist, 'cmd_vel_collision', 10)

		# motion state (time-based only)
		self._motion_lock = threading.Lock()
		self._motion_active = False
		self._motion_direction = 0.0  # +1 forward, -1 backward
		self._motion_end_time = 0.0
		self._motion_target_distance = 0.0

		# timer for publishing cmd_vel
		self._timer = self.create_timer(1.0 / max(1, self.publish_hz), self._on_timer)

		# serial thread
		self._serial = None
		self._serial_thread: Optional[threading.Thread] = None
		self._stop_event = threading.Event()

		if serial is None:
			self.get_logger().error('pyserial not available; install python package "pyserial"')
			return

		try:
			self._serial = serial.Serial(self.serial_port, self.baudrate, timeout=0.5)
			self.get_logger().info(f'Opened serial {self.serial_port} @ {self.baudrate}')
		except Exception as e:
			self.get_logger().error(f'Failed to open serial port {self.serial_port}: {e}')
			self._serial = None

		if self._serial:
			self._serial_thread = threading.Thread(target=self._serial_loop, daemon=True)
			self._serial_thread.start()

	# --------------------- serial reading ---------------------
	def _serial_loop(self) -> None:
		while not self._stop_event.is_set():
			try:
				raw = self._serial.readline()
			except Exception as e:
				self.get_logger().warn(f'Serial read exception: {e}')
				time.sleep(0.5)
				continue

			if not raw:
				continue

			try:
				text = raw.decode('utf-8', errors='ignore').strip()
			except Exception:
				text = str(raw)

			if not text:
				continue

			self.get_logger().info(f'Serial -> "{text}"')
			self._handle_bumper_token(text)

	# --------------------- bumper handling ---------------------
	def _handle_bumper_token(self, token: str) -> None:
		t = token.strip().upper()
		# Accept messages that contain the words 'FRONT' or 'REAR' in any position
		# (e.g. "Rear bumper triggered" or "COLLISION:REAR").
		if 'FRONT' in t:
			direction = -1.0
		elif 'REAR' in t:
			direction = 1.0
		else:
			# ignore unknown tokens
			return

		if self.move_speed == 0.0:
			self.get_logger().warn('move_speed == 0, ignoring bumper event')
			return

		# compute duration (time-based) — allow overriding with move_duration param
		if self.move_duration and self.move_duration > 0.0:
			duration = self.move_duration
		else:
			duration = abs(self.move_distance) / abs(self.move_speed)

		with self._motion_lock:
			now = time.time()
			self._motion_active = True
			self._motion_direction = direction
			self._motion_target_distance = abs(self.move_distance)
			self._motion_end_time = now + duration

		self.get_logger().info(f'Triggered bumper={t}: dir={direction} duration={duration:.2f}s')

	# --------------------- publishing timer ---------------------
	def _on_timer(self) -> None:
		now = time.time()
		publish_twist = Twist()

		with self._motion_lock:
			if not self._motion_active:
				# no active collision motion: do not publish
				return

			# non-odom: stop by time
			if now >= self._motion_end_time:
				self._motion_active = False
				self.get_logger().info('Motion complete (time)')
				# motion finished: publish a single zero Twist to ensure the robot stops,
				# then return (no continuous publishing when idle)
				self.cmd_pub.publish(Twist())
				return

			# still within time window
			publish_twist.linear.x = self._motion_direction * self.move_speed
			self.cmd_pub.publish(publish_twist)

	# --------------------- shutdown ---------------------
	def destroy_node(self) -> None:
		self.get_logger().info('Shutting down collision_detect node')
		self._stop_event.set()
		if self._serial_thread is not None and self._serial_thread.is_alive():
			self._serial_thread.join(timeout=1.0)
		try:
			if self._serial is not None and self._serial.is_open:
				self._serial.close()
		except Exception:
			pass
		super().destroy_node()


def main(args=None):
	rclpy.init(args=args)
	node = CollisionDetectNode()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		try:
			node.destroy_node()
		except Exception:
			pass
		rclpy.shutdown()


if __name__ == '__main__':
	main()

