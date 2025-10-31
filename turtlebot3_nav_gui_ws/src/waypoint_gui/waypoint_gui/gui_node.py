#!/usr/bin/env python3
"""
PyQt5 GUI for sending Nav2 NavigateToPose goals to TurtleBot3.

Usage:
  ros2 run waypoint_gui gui_node

Make sure Nav2 is running and map frame is 'map'.
"""

import sys
import threading
import math
from dataclasses import dataclass
from typing import List

from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QLabel, QPushButton,
    QCheckBox, QHBoxLayout, QMessageBox, QProgressBar
)
from PyQt5.QtCore import pyqtSignal, QObject
from PyQt5.QtGui import QFont

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Quaternion
from action_msgs.msg import GoalStatus

# ---------- Waypoint dataclass ----------
@dataclass
class Waypoint:
    name: str
    x: float
    y: float
    yaw: float

# ---------- Nav2 action client node ----------
class Nav2Client(Node):
    def __init__(self):
        super().__init__('waypoint_gui_nav2_client')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._current_goal_handle = None
        self._goal_lock = threading.Lock()
        self.get_logger().info("Nav2 action client node initialized")

    def _make_pose_stamped(self, wp: Waypoint) -> PoseStamped:
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = float(wp.x)
        ps.pose.position.y = float(wp.y)
        ps.pose.position.z = 0.0
        # Convert yaw to quaternion (only yaw)
        qz = math.sin(wp.yaw / 2.0)
        qw = math.cos(wp.yaw / 2.0)
        ps.pose.orientation = Quaternion(x=0.0, y=0.0, z=qz, w=qw)
        return ps

    def send_goal_and_wait(self, wp: Waypoint, timeout_sec: float = 180.0) -> (bool, str):
        """
        Send NavigateToPose goal and block until result or timeout.
        Returns (success: bool, detail: str)
        """
        with self._goal_lock:
            if not self._action_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error("Action server not available")
                return False, "Action server not available"

            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = self._make_pose_stamped(wp)

            self.get_logger().info(f"Sending goal to {wp.name} at ({wp.x}, {wp.y})")
            
            # send goal
            send_goal_future = self._action_client.send_goal_async(
                goal_msg,
                feedback_callback=self._feedback_callback
            )
            
            rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=10.0)
            
            if not send_goal_future.done():
                self.get_logger().error("Goal send timed out")
                return False, "Goal send timed out"

            goal_handle = send_goal_future.result()
            if not goal_handle.accepted:
                self.get_logger().error("Goal rejected by server")
                return False, "Goal rejected by server"

            self._current_goal_handle = goal_handle
            self.get_logger().info(f"Goal accepted for {wp.name}")

        # wait for result (outside lock)
        result_future = goal_handle.get_result_async()
        
        start = self.get_clock().now()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            
            if result_future.done():
                result = result_future.result()
                status = result.status
                
                self.get_logger().info(f"Goal result status: {status}")
                
                if status == GoalStatus.STATUS_SUCCEEDED:
                    self.get_logger().info(f"Successfully reached {wp.name}")
                    return True, "Succeeded"
                elif status == GoalStatus.STATUS_ABORTED:
                    self.get_logger().warn(f"Goal to {wp.name} was aborted")
                    return False, "Aborted"
                elif status == GoalStatus.STATUS_CANCELED:
                    self.get_logger().warn(f"Goal to {wp.name} was canceled")
                    return False, "Canceled"
                else:
                    self.get_logger().warn(f"Goal to {wp.name} ended with status {status}")
                    return False, f"Status {status}"
            
            # timeout check
            now = self.get_clock().now()
            elapsed = (now - start).nanoseconds / 1e9
            if elapsed > timeout_sec:
                self.get_logger().error(f"Goal to {wp.name} timed out after {elapsed}s")
                try:
                    cancel_future = goal_handle.cancel_goal_async()
                    rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=3.0)
                except Exception as e:
                    self.get_logger().error(f"Error canceling goal: {e}")
                return False, "Timeout"
        
        return False, "Interrupted"

    def _feedback_callback(self, feedback_msg):
        """Optional: Log feedback from Nav2"""
        # feedback = feedback_msg.feedback
        # You can log distance remaining, etc.
        pass

    def cancel_current_goal(self) -> bool:
        """Cancel the currently active goal (if any)"""
        with self._goal_lock:
            if self._current_goal_handle is None:
                return False
            try:
                self.get_logger().info("Canceling current goal")
                cancel_future = self._current_goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=3.0)
                self._current_goal_handle = None
                return True
            except Exception as e:
                self.get_logger().error(f"Error canceling goal: {e}")
                return False

# ---------- Qt signals container ----------
class WorkerSignals(QObject):
    status = pyqtSignal(str)
    progress = pyqtSignal(int, int)  # current, total
    finished = pyqtSignal(bool, str)  # success, message

# ---------- Worker that runs navigation sequence in thread ----------
class NavWorker(threading.Thread):
    def __init__(self, nav_node: Nav2Client, waypoints: List[Waypoint], signals: WorkerSignals):
        super().__init__()
        self.nav_node = nav_node
        self.waypoints = waypoints
        self._stop_requested = False
        self.signals = signals
        self.daemon = True

    def run(self):
        total = len(self.waypoints)
        success_count = 0
        
        for idx, wp in enumerate(self.waypoints):
            if self._stop_requested:
                self.signals.status.emit("Cancelled by user")
                self.signals.finished.emit(False, "Cancelled")
                return
            
            self.signals.progress.emit(idx + 1, total)
            self.signals.status.emit(f"[{idx+1}/{total}] Navigating to {wp.name}...")
            
            ok, detail = self.nav_node.send_goal_and_wait(wp, timeout_sec=180.0)
            
            if ok:
                success_count += 1
                self.signals.status.emit(f"[{idx+1}/{total}] ✓ Reached {wp.name}")
            else:
                self.signals.status.emit(f"[{idx+1}/{total}] ✗ Failed at {wp.name}: {detail}")
                # Optionally continue to next waypoint instead of stopping
                # For now, we'll continue to try remaining waypoints
                self.nav_node.get_logger().warn(f"Failed to reach {wp.name}, continuing to next waypoint...")
        
        # Final status
        if success_count == total:
            final_msg = f"Completed! All {total} waypoints reached successfully."
            self.signals.finished.emit(True, final_msg)
        else:
            final_msg = f"Completed with issues. {success_count}/{total} waypoints reached."
            self.signals.finished.emit(False, final_msg)

    def stop(self):
        self._stop_requested = True
        self.nav_node.cancel_current_goal()

# ---------- Main GUI ----------
class WaypointGUI(QWidget):
    def __init__(self, nav_node: Nav2Client):
        super().__init__()
        self.nav_node = nav_node

        self.setWindowTitle("TurtleBot3 Waypoint Navigation")
        self.setGeometry(100, 100, 400, 450)

        self.layout = QVBoxLayout()
        
        # Title
        title = QLabel("Waypoint Navigation Controller")
        title_font = QFont()
        title_font.setPointSize(12)
        title_font.setBold(True)
        title.setFont(title_font)
        self.layout.addWidget(title)
        
        # Status
        self.status_label = QLabel("Status: Idle")
        self.layout.addWidget(self.status_label)
        
        # Progress bar
        self.progress_bar = QProgressBar()
        self.progress_bar.setMinimum(0)
        self.progress_bar.setMaximum(100)
        self.progress_bar.setValue(0)
        self.layout.addWidget(self.progress_bar)

        # Waypoints section
        waypoints_label = QLabel("Select Waypoints (in order):")
        waypoints_label.setStyleSheet("font-weight: bold; margin-top: 10px;")
        self.layout.addWidget(waypoints_label)

        # define waypoints
        self.waypoints = [
            Waypoint("Home", 0.0, 0.0, 0.0),
            Waypoint("Station 1", 1.5, 2.4, 0.0),
            Waypoint("Station 2", 2.6, 2.2, 0.0),
            Waypoint("Station 3", 3.8, 0.5, 0.0),
            Waypoint("Station 4", 2.8, -1.3, 0.0),
            Waypoint("Docking", 1.6, -2.2, 0.0),
        ]

        # checkboxes in order
        self.checkboxes = []
        for wp in self.waypoints:
            cb = QCheckBox(f"{wp.name} (x={wp.x}, y={wp.y})")
            self.checkboxes.append(cb)
            self.layout.addWidget(cb)

        # Auto-return home checkbox
        self.auto_return_home = QCheckBox("Auto-return to Home at end")
        self.auto_return_home.setChecked(True)
        self.layout.addWidget(self.auto_return_home)

        # Buttons
        btn_layout = QHBoxLayout()
        self.start_btn = QPushButton("Start Navigation")
        self.start_btn.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold; padding: 8px;")
        self.stop_btn = QPushButton("Stop")
        self.stop_btn.setStyleSheet("background-color: #f44336; color: white; font-weight: bold; padding: 8px;")
        self.stop_btn.setEnabled(False)
        btn_layout.addWidget(self.start_btn)
        btn_layout.addWidget(self.stop_btn)
        self.layout.addLayout(btn_layout)
        
        # Select/Deselect all buttons
        select_layout = QHBoxLayout()
        self.select_all_btn = QPushButton("Select All")
        self.deselect_all_btn = QPushButton("Deselect All")
        select_layout.addWidget(self.select_all_btn)
        select_layout.addWidget(self.deselect_all_btn)
        self.layout.addLayout(select_layout)

        self.setLayout(self.layout)

        # signals / worker
        self.worker = None
        self.worker_signals = WorkerSignals()
        self.worker_signals.status.connect(self.update_status)
        self.worker_signals.progress.connect(self.update_progress)
        self.worker_signals.finished.connect(self.on_finished)

        # connect buttons
        self.start_btn.clicked.connect(self.on_start)
        self.stop_btn.clicked.connect(self.on_stop)
        self.select_all_btn.clicked.connect(self.on_select_all)
        self.deselect_all_btn.clicked.connect(self.on_deselect_all)

    def update_status(self, text):
        self.status_label.setText("Status: " + text)

    def update_progress(self, current, total):
        if total > 0:
            percentage = int((current / total) * 100)
            self.progress_bar.setValue(percentage)

    def on_finished(self, success, message):
        self.update_status(message)
        self.progress_bar.setValue(100 if success else 0)
        self.start_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)
        
        # Show completion dialog
        msg_box = QMessageBox()
        if success:
            msg_box.setIcon(QMessageBox.Information)
            msg_box.setWindowTitle("Success")
        else:
            msg_box.setIcon(QMessageBox.Warning)
            msg_box.setWindowTitle("Navigation Completed")
        msg_box.setText(message)
        msg_box.exec_()

    def on_start(self):
        # collect selected waypoints in the same order they appear
        selected = []
        for cb, wp in zip(self.checkboxes, self.waypoints):
            if cb.isChecked():
                selected.append(wp)
        
        if not selected:
            QMessageBox.information(self, "No waypoints", "Please select at least one waypoint.")
            return

        # Auto return to Home at the end (if enabled and last is not home)
        home = self.waypoints[0]
        if self.auto_return_home.isChecked() and selected[-1].name != home.name:
            selected.append(home)

        # Update UI
        self.start_btn.setEnabled(False)
        self.stop_btn.setEnabled(True)
        self.progress_bar.setValue(0)
        
        # Log selected waypoints
        waypoint_names = " → ".join([wp.name for wp in selected])
        self.update_status(f"Starting route: {waypoint_names}")
        self.nav_node.get_logger().info(f"Starting navigation sequence: {waypoint_names}")

        # start worker thread
        self.worker = NavWorker(self.nav_node, selected, self.worker_signals)
        self.worker.start()

    def on_stop(self):
        if self.worker and self.worker.is_alive():
            self.worker.stop()
            self.update_status("Cancelling navigation...")
            self.start_btn.setEnabled(True)
            self.stop_btn.setEnabled(False)

    def on_select_all(self):
        for cb in self.checkboxes:
            cb.setChecked(True)

    def on_deselect_all(self):
        for cb in self.checkboxes:
            cb.setChecked(False)

# ---------- Entrypoint ----------
def main(args=None):
    # Initialize rclpy in this process
    rclpy.init(args=args)
    nav_node = Nav2Client()

    # Start a separate thread to spin rclpy for the Nav2 node
    def spin_thread():
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(nav_node)
        try:
            executor.spin()
        except Exception as e:
            nav_node.get_logger().error(f"Executor error: {e}")
        finally:
            executor.shutdown()

    t = threading.Thread(target=spin_thread, daemon=True)
    t.start()

    # Start Qt app (must be in main thread)
    app = QApplication(sys.argv)
    gui = WaypointGUI(nav_node)
    gui.show()
    
    try:
        exit_code = app.exec_()
    except KeyboardInterrupt:
        pass

    # cleanup
    nav_node.get_logger().info("Shutting down Nav2 client node")
    nav_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
