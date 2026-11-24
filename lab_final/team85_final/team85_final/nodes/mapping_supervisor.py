"""
Node mapping_supervisor: runs Cartographer and can reset the map without quitting.

Cartographer has no "clear the map" service (finish/start trajectory keeps the old
submaps), so a reset restarts cartographer_node + cartographer_occupancy_grid_node.
The new map starts where the robot is at that moment.

Reset by:
  - RViz: click the red "RESET MAP" box above the robot (Interact tool, the default in
    rviz/mapping.rviz). It is an interactive marker on namespace /map_reset.
  - Terminal: ros2 service call /reset_map std_srvs/srv/Trigger

Params: configuration_directory, configuration_basename (Cartographer .lua),
resolution, publish_period_sec (occupancy grid), use_sim_time (passed to both children),
marker_frame (default base_footprint).
"""

import os
import signal
import subprocess
import threading
import time

import rclpy
from ament_index_python.packages import get_package_prefix
from interactive_markers import InteractiveMarkerServer
from rclpy.node import Node
from std_srvs.srv import Trigger
from visualization_msgs.msg import (InteractiveMarker, InteractiveMarkerControl,
                                    InteractiveMarkerFeedback, Marker)


class MappingSupervisor(Node):
    def __init__(self):
        super().__init__('mapping_supervisor')
        p = self.declare_parameters('', [
            ('configuration_directory', ''),
            ('configuration_basename', 'maze_lds_2d.lua'),
            ('resolution', 0.05),
            ('publish_period_sec', 1.0),
            ('marker_frame', 'base_footprint'),
        ])
        self.cfg = {x.name: x.value for x in p}
        self.sim_time = self.get_parameter('use_sim_time').value
        lib = os.path.join(get_package_prefix('cartographer_ros'), 'lib', 'cartographer_ros')
        sim = ['--ros-args', '-p', f'use_sim_time:={str(self.sim_time).lower()}']
        self.commands = [
            [os.path.join(lib, 'cartographer_node'),
             '-configuration_directory', self.cfg['configuration_directory'],
             '-configuration_basename', self.cfg['configuration_basename']] + sim,
            [os.path.join(lib, 'cartographer_occupancy_grid_node'),
             '-resolution', str(self.cfg['resolution']),
             '-publish_period_sec', str(self.cfg['publish_period_sec'])] + sim,
        ]
        self.children: list[subprocess.Popen] = []
        self.lock = threading.Lock()
        self.resets = 0
        self.start_children()

        self.create_service(Trigger, '/reset_map', self.on_service)
        self.markers = InteractiveMarkerServer(self, 'map_reset')
        self.markers.insert(self.button(), feedback_callback=self.on_click)
        self.markers.applyChanges()
        self.get_logger().info('Cartographer started; click RESET MAP in RViz or call /reset_map')

    # ---- children -------------------------------------------------------
    def start_children(self):
        # Direct exec (no shell): macOS SIP would drop DYLD_LIBRARY_PATH via /bin/sh.
        self.children = [subprocess.Popen(cmd) for cmd in self.commands]

    def stop_children(self):
        for c in self.children:
            if c.poll() is None:
                c.send_signal(signal.SIGINT)
        deadline = time.time() + 5.0
        for c in self.children:
            try:
                c.wait(timeout=max(0.1, deadline - time.time()))
            except subprocess.TimeoutExpired:
                c.kill()
                c.wait()
        self.children = []

    def reset(self) -> str:
        with self.lock:
            self.resets += 1
            self.get_logger().warn(f'resetting the map (#{self.resets}): restarting Cartographer')
            self.stop_children()
            self.start_children()
            return f'map reset #{self.resets}; new map starts at the robot'

    # ---- triggers -------------------------------------------------------
    def on_service(self, _req, res):
        res.success, res.message = True, self.reset()
        return res

    def on_click(self, fb: InteractiveMarkerFeedback):
        if fb.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
            # Restarting blocks for up to 5 s; keep the executor free.
            threading.Thread(target=self.reset, daemon=True).start()

    def button(self) -> InteractiveMarker:
        im = InteractiveMarker()
        im.header.frame_id = self.cfg['marker_frame']
        im.name = 'reset_map'
        im.description = 'RESET MAP (click)'
        im.scale = 0.3
        im.pose.position.z = 0.45
        box = Marker(type=Marker.CUBE)
        box.scale.x, box.scale.y, box.scale.z = 0.18, 0.18, 0.08
        box.color.r, box.color.g, box.color.b, box.color.a = 0.9, 0.15, 0.15, 0.9
        ctrl = InteractiveMarkerControl(interaction_mode=InteractiveMarkerControl.BUTTON,
                                        always_visible=True, name='click')
        ctrl.markers.append(box)
        im.controls.append(ctrl)
        return im


def main():
    rclpy.init()
    node = MappingSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_children()
        node.destroy_node()
        rclpy.try_shutdown()
