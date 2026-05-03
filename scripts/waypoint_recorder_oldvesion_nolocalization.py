#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Waypoint Recorder — headless backend for Lichtblick frontend.

Subscribes to /add_waypoint (PoseStamped) from the frontend,
maintains a waypoint list, and publishes visualization markers
to /waypoint_markers (MarkerArray, latched).

Also provides save_json / load_json for persistence.

Execution mode: receives start/stop commands from the frontend,
sequentially sends waypoints to EGO-Planner via /goal_with_id,
and monitors DroneState.reached to advance through the list.
"""

import glob
import json
import os
import signal
import sys

import rospy
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Empty, Int32, String
from visualization_msgs.msg import Marker, MarkerArray
from quadrotor_msgs.msg import GoalSet
from controller_msgs.msg import DroneState
from controller_msgs.msg import cmd as CmdMsg


class WaypointRecorder:
    def __init__(self):
        self.waypoints = []
        self.drone_id = str(rospy.get_param("~drone_id", "1"))
        self.prefix = f"/drone_{self.drone_id}_"
        self.frame_id = rospy.get_param("~frame_id", "world")
        self.json_path = rospy.get_param(
            "~json_path",
            "/home/colman/ego_ws/src/Utils/waypoint_recorder/include/waypoints.json",
        )
        self.include_dir = os.path.dirname(self.json_path)

        # --- Execution state ---
        self.exec_state = "idle"          # "idle" | "executing"
        self.nav_current_idx = 0
        self.drone_reached = False
        self.drone_tookoff = False
        self._nav_sub_state = "sending"   # "sending" | "waiting"
        self._nav_timer = None

        # Publishers — waypoint visualization
        self.marker_pub = rospy.Publisher(
            f"{self.prefix}waypoint_markers", MarkerArray, queue_size=1, latch=True,
        )
        self.project_list_pub = rospy.Publisher(
            f"{self.prefix}waypoint_project_list", String, queue_size=1, latch=True,
        )

        # Publishers — execution
        self.exec_state_pub = rospy.Publisher(
            f"{self.prefix}waypoint_exec_state", String, queue_size=1, latch=True,
        )
        self.goal_pub = rospy.Publisher("/goal_with_id", GoalSet, queue_size=10)
        self.control_pub = rospy.Publisher("/control", CmdMsg, queue_size=10)

        # Publish initial idle state
        self.exec_state_pub.publish(String(data="idle"))

        # Subscribers — waypoint management
        rospy.Subscriber(
            f"{self.prefix}add_waypoint", PoseStamped, self._add_waypoint_cb, queue_size=10,
        )
        rospy.Subscriber(
            f"{self.prefix}remove_waypoint", Int32, self._remove_waypoint_cb, queue_size=10,
        )
        rospy.Subscriber(
            f"{self.prefix}clear_waypoints", Empty, self._clear_waypoints_cb, queue_size=10,
        )
        rospy.Subscriber(
            f"{self.prefix}save_waypoints", String, self._save_waypoints_cb, queue_size=10,
        )
        rospy.Subscriber(
            f"{self.prefix}load_waypoints", String, self._load_waypoints_cb, queue_size=10,
        )
        rospy.Subscriber(
            f"{self.prefix}delete_waypoint_project", String, self._delete_project_cb, queue_size=10,
        )
        rospy.Subscriber(
            f"{self.prefix}reorder_waypoints", String, self._reorder_waypoints_cb, queue_size=10,
        )

        # Subscribers — execution control
        rospy.Subscriber(
            f"{self.prefix}state", DroneState, self._drone_state_cb, queue_size=10,
        )
        rospy.Subscriber(
            f"{self.prefix}start_waypoint_exec", Empty, self._start_exec_cb, queue_size=1,
        )
        rospy.Subscriber(
            f"{self.prefix}stop_waypoint_exec", Empty, self._stop_exec_cb, queue_size=1,
        )

        self._publish_project_list()

        rospy.loginfo("WaypointRecorder ready  drone=%s  prefix=%s  frame=%s  json=%s",
                      self.drone_id, self.prefix, self.frame_id, self.json_path)

    # --------------------------------------------------------- exec guard
    def _check_exec_guard(self, action_name):
        """Return True if the action should be rejected (currently executing)."""
        if self.exec_state == "executing":
            rospy.logwarn("Rejected %s: waypoint execution in progress", action_name)
            return True
        return False

    # --------------------------------------------------------- waypoint callbacks
    def _add_waypoint_cb(self, msg):
        if self._check_exec_guard("add_waypoint"):
            return
        p = msg.pose.position
        idx = len(self.waypoints) + 1
        wp = {
            "idx": idx,
            "x": round(p.x, 3),
            "y": round(p.y, 3),
            "z": round(p.z, 3),
        }
        self.waypoints.append(wp)
        rospy.loginfo("Waypoint added [%d]: x=%s y=%s z=%s",
                      idx, wp["x"], wp["y"], wp["z"])
        self._publish_markers()

    def _remove_waypoint_cb(self, msg):
        if self._check_exec_guard("remove_waypoint"):
            return
        idx = msg.data  # 1-based index from frontend
        if 1 <= idx <= len(self.waypoints):
            removed = self.waypoints.pop(idx - 1)
            # Re-number remaining waypoints
            for i, wp in enumerate(self.waypoints):
                wp["idx"] = i + 1
            rospy.loginfo("Waypoint removed [%d]: x=%s y=%s z=%s",
                          idx, removed["x"], removed["y"], removed["z"])
            self._publish_markers()
        else:
            rospy.logwarn("Remove index %d out of range (have %d)",
                          idx, len(self.waypoints))

    def _clear_waypoints_cb(self, _msg):
        if self._check_exec_guard("clear_waypoints"):
            return
        count = len(self.waypoints)
        self.waypoints = []
        rospy.loginfo("All %d waypoints cleared", count)
        self._publish_markers()

    def _save_waypoints_cb(self, msg):
        # Saving is allowed during execution (snapshot is safe)
        name = msg.data.strip()
        if not name:
            name = "waypoint"
        name = self._resolve_save_name(name)
        path = os.path.join(self.include_dir, name + ".json")
        self.save_json(path)
        self._publish_project_list()

    def _load_waypoints_cb(self, msg):
        if self._check_exec_guard("load_waypoints"):
            return
        name = msg.data.strip()
        if not name:
            rospy.logwarn("Load: empty project name")
            return
        path = os.path.join(self.include_dir, name + ".json")
        self.load_json(path)

    def _delete_project_cb(self, msg):
        names = [n.strip() for n in msg.data.split(",") if n.strip()]
        for name in names:
            path = os.path.join(self.include_dir, name + ".json")
            if os.path.isfile(path):
                os.remove(path)
                rospy.loginfo("Deleted project: %s", name)
            else:
                rospy.logwarn("Delete: file not found: %s", path)
        self._publish_project_list()

    def _reorder_waypoints_cb(self, msg):
        if self._check_exec_guard("reorder_waypoints"):
            return
        try:
            data = json.loads(msg.data)
            order = data.get("order", [])
        except (json.JSONDecodeError, AttributeError) as e:
            rospy.logerr("Reorder: invalid JSON: %s", e)
            return
        if len(order) != len(self.waypoints):
            rospy.logwarn("Reorder: order length %d != waypoints %d",
                          len(order), len(self.waypoints))
            return
        try:
            reordered = [self.waypoints[i - 1] for i in order]
        except IndexError:
            rospy.logerr("Reorder: index out of range in %s", order)
            return
        for i, wp in enumerate(reordered):
            wp["idx"] = i + 1
        self.waypoints = reordered
        rospy.loginfo("Waypoints reordered: %s", order)
        self._publish_markers()

    # --------------------------------------------------------- execution callbacks
    def _drone_state_cb(self, msg):
        """Track drone flight state for execution gating."""
        self.drone_tookoff = msg.tookoff
        if (self.exec_state == "executing"
                and self._nav_sub_state == "waiting"
                and msg.reached):
            self.drone_reached = True

    def _start_exec_cb(self, _msg):
        """Start waypoint execution if all preconditions are met."""
        if self.exec_state != "idle":
            rospy.logwarn("Start exec rejected: already executing")
            return
        if not self.drone_tookoff:
            rospy.logwarn("Start exec rejected: drone not airborne (tookoff=False)")
            return
        if not self.waypoints:
            rospy.logwarn("Start exec rejected: waypoint list is empty")
            return

        self.exec_state = "executing"
        self.nav_current_idx = 0
        self.drone_reached = False
        self._nav_sub_state = "sending"
        self.exec_state_pub.publish(String(data="executing"))

        # Start 5Hz navigation tick timer
        self._nav_timer = rospy.Timer(rospy.Duration(0.2), self._nav_tick)
        rospy.loginfo("Waypoint execution started: %d waypoints", len(self.waypoints))

    def _stop_exec_cb(self, _msg):
        """Stop waypoint execution on frontend command."""
        if self.exec_state != "executing":
            rospy.logwarn("Stop exec ignored: not currently executing")
            return
        self._halt_execution("Frontend stop command")

    def _halt_execution(self, reason=""):
        """Clean shutdown of waypoint execution."""
        # Send Stop to flight controller
        ctrl_msg = CmdMsg()
        ctrl_msg.header.stamp = rospy.Time.now()
        ctrl_msg.cmd = 5  # STOP
        self.control_pub.publish(ctrl_msg)

        # Reset state
        self.exec_state = "idle"
        self.nav_current_idx = 0
        self.drone_reached = False
        self._nav_sub_state = "sending"
        self.exec_state_pub.publish(String(data="idle"))

        # Stop timer
        if self._nav_timer is not None:
            self._nav_timer.shutdown()
            self._nav_timer = None

        rospy.loginfo("Waypoint execution halted: %s", reason)

    def _nav_tick(self, _event=None):
        """Navigation state machine tick (5Hz)."""
        if self.exec_state != "executing":
            return

        if self._nav_sub_state == "sending":
            if self.nav_current_idx >= len(self.waypoints):
                self._halt_execution("All waypoints reached")
                return
            self._send_waypoint(self.nav_current_idx)
            self.drone_reached = False
            self._nav_sub_state = "waiting"

        elif self._nav_sub_state == "waiting":
            if self.drone_reached:
                rospy.loginfo("Waypoint %d/%d reached",
                              self.nav_current_idx + 1, len(self.waypoints))
                self.nav_current_idx += 1
                self._nav_sub_state = "sending"

    def _send_waypoint(self, idx):
        """Send a single waypoint to EGO-Planner via /goal_with_id."""
        wp = self.waypoints[idx]

        # Step 1: send Continue command
        ctrl_msg = CmdMsg()
        ctrl_msg.header.stamp = rospy.Time.now()
        ctrl_msg.cmd = 4  # CONTINUE
        self.control_pub.publish(ctrl_msg)

        rospy.sleep(0.2)

        # Step 2: send goal
        goal_msg = GoalSet()
        goal_msg.drone_id = int(self.drone_id)
        goal_msg.goal = [float(wp["x"]), float(wp["y"]), float(wp["z"])]
        self.goal_pub.publish(goal_msg)

        rospy.loginfo("Sent waypoint [%d/%d] x=%s y=%s z=%s",
                      idx + 1, len(self.waypoints), wp["x"], wp["y"], wp["z"])

    # ------------------------------------------------- project list
    def _publish_project_list(self):
        files = glob.glob(os.path.join(self.include_dir, "*.json"))
        names = sorted(
            os.path.splitext(os.path.basename(f))[0] for f in files
        )
        payload = json.dumps({"projects": names})
        self.project_list_pub.publish(String(data=payload))
        rospy.loginfo("Project list published: %s", names)

    def _resolve_save_name(self, name):
        path = os.path.join(self.include_dir, name + ".json")
        if not os.path.isfile(path):
            return name
        counter = 1
        while True:
            candidate = "{}{}".format(name, counter)
            path = os.path.join(self.include_dir, candidate + ".json")
            if not os.path.isfile(path):
                return candidate
            counter += 1

    # --------------------------------------------------- marker publish
    def _publish_markers(self):
        frame = self.frame_id
        ma = MarkerArray()

        # DELETEALL
        d = Marker()
        d.header.frame_id = frame
        d.header.stamp = rospy.Time.now()
        d.action = Marker.DELETEALL
        d.id = 99999
        ma.markers.append(d)

        # Sphere markers
        for i, wp in enumerate(self.waypoints):
            m = Marker()
            m.header.frame_id = frame
            m.header.stamp = rospy.Time.now()
            m.ns = "waypoints"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position = Point(wp["x"], wp["y"], wp["z"])
            m.pose.orientation.w = 1.0
            m.scale.x = 0.15
            m.scale.y = 0.15
            m.scale.z = 0.15
            m.color.r = 1.0
            m.color.g = 0.0
            m.color.b = 0.0
            m.color.a = 1.0
            ma.markers.append(m)

        # Text labels
        for i, wp in enumerate(self.waypoints):
            m = Marker()
            m.header.frame_id = frame
            m.header.stamp = rospy.Time.now()
            m.ns = "waypoints"
            m.id = 10000 + i
            m.type = Marker.TEXT_VIEW_FACING
            m.action = Marker.ADD
            m.pose.position = Point(wp["x"], wp["y"], wp["z"] + 0.2)
            m.pose.orientation.w = 1.0
            m.scale.z = 0.25
            m.color.r = 1.0
            m.color.g = 1.0
            m.color.b = 0.0
            m.color.a = 1.0
            m.text = str(i + 1)
            ma.markers.append(m)

        # Line strip
        if len(self.waypoints) >= 2:
            line = Marker()
            line.header.frame_id = frame
            line.header.stamp = rospy.Time.now()
            line.ns = "waypoints"
            line.id = 20000
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.pose.orientation.w = 1.0
            line.scale.x = 0.02
            line.color.r = 0.2
            line.color.g = 1.0
            line.color.b = 0.2
            line.color.a = 0.8
            line.points = [Point(wp["x"], wp["y"], wp["z"])
                           for wp in self.waypoints]
            ma.markers.append(line)

        self.marker_pub.publish(ma)

    # --------------------------------------------------------- file I/O
    def save_json(self, path=None):
        path = path or self.json_path
        data = {"waypoints": self.waypoints}
        try:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            with open(path, "w") as f:
                json.dump(data, f, indent=4)
            rospy.loginfo("Saved %d waypoints to %s", len(self.waypoints), path)
        except Exception as e:
            rospy.logerr("Failed to save: %s", e)

    def load_json(self, path=None):
        path = path or self.json_path
        if not os.path.isfile(path):
            rospy.logwarn("File not found: %s", path)
            return
        try:
            with open(path, "r") as f:
                data = json.load(f)
            self.waypoints = data.get("waypoints", [])
            rospy.loginfo("Loaded %d waypoints from %s",
                          len(self.waypoints), path)
            self._publish_markers()
        except Exception as e:
            rospy.logerr("Failed to load: %s", e)


def main():
    rospy.init_node("waypoint_recorder", anonymous=False)
    recorder = WaypointRecorder()

    # Start with an empty working route. Saved projects are loaded only when
    # the frontend explicitly publishes /drone_{id}_load_waypoints.

    def _shutdown(*_args):
        if recorder.exec_state == "executing":
            recorder._halt_execution("Node shutdown")
        rospy.signal_shutdown("exit")
        sys.exit(0)

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    rospy.spin()


if __name__ == "__main__":
    main()

