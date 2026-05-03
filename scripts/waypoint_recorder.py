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
import math
import os
import re
import shutil
import signal
import sys
import threading
import uuid

import rospy
from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
from std_msgs.msg import Empty, Header, Int32, String
from visualization_msgs.msg import Marker, MarkerArray
from quadrotor_msgs.msg import GoalSet
from controller_msgs.msg import DroneState
from controller_msgs.msg import cmd as CmdMsg

SCHEMA_VERSION = 1
ROUTE_KIND = "spikive.localization_route"
NAME_RE = re.compile(r"^[A-Za-z0-9_-]+$")
LOCALIZATION_STATES = {"idle", "recording", "recorded", "loaded"}
LOCALIZATION_PHASES = {"new", "record_start", "record_stop", "save", "load", "delete", "reorder"}
DEFAULT_INCLUDE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "include"))


class StrictCommandError(ValueError):
    pass


def _is_finite_number(value):
    return isinstance(value, (int, float)) and math.isfinite(float(value))


def _round3(value):
    return round(float(value), 3)


def _atomic_write_json(path, data):
    tmp_path = path + ".tmp"
    with open(tmp_path, "w") as f:
        json.dump(data, f, indent=4, sort_keys=True)
    os.replace(tmp_path, path)


def _parse_json_command(raw, required=()):
    try:
        data = json.loads(raw)
    except Exception as exc:
        raise StrictCommandError("invalid JSON command") from exc
    if not isinstance(data, dict):
        raise StrictCommandError("command must be a JSON object")
    if data.get("schema_version") != SCHEMA_VERSION:
        raise StrictCommandError("schema_version must be 1")
    request_id = data.get("request_id")
    route_id = data.get("route_id")
    if not isinstance(request_id, str) or not request_id:
        raise StrictCommandError("request_id is required")
    if route_id is not None and not isinstance(route_id, str):
        raise StrictCommandError("route_id must be a string")
    for key in required:
        if key not in data:
            raise StrictCommandError("{} is required".format(key))
    return data


def _extract_command_ids(raw, fallback_route_id=""):
    try:
        data = json.loads(raw)
    except Exception:
        return "", fallback_route_id
    if not isinstance(data, dict):
        return "", fallback_route_id
    request_id = data.get("request_id")
    route_id = data.get("route_id")
    return (
        request_id if isinstance(request_id, str) else "",
        route_id if isinstance(route_id, str) else fallback_route_id,
    )


def _make_route_id():
    return uuid.uuid4().hex


def _validate_project_name(name):
    if not isinstance(name, str) or not NAME_RE.match(name):
        raise StrictCommandError("invalid project name")
    return name


def _safe_route_file_stem(route_id):
    if not isinstance(route_id, str) or not route_id:
        raise StrictCommandError("route_id is required")
    return re.sub(r"[^A-Za-z0-9_-]", "_", route_id)[:128]


def _validate_waypoints_from_json(data):
    waypoints = data.get("waypoints")
    if not isinstance(waypoints, list):
        raise StrictCommandError("waypoints must be a list")
    result = []
    for expected_idx, wp in enumerate(waypoints, start=1):
        if not isinstance(wp, dict):
            raise StrictCommandError("waypoint must be an object")
        if wp.get("idx") != expected_idx:
            raise StrictCommandError("waypoint idx must be contiguous from 1")
        pos = wp.get("position")
        if not isinstance(pos, dict):
            raise StrictCommandError("waypoint position is required")
        x, y, z = pos.get("x"), pos.get("y"), pos.get("z")
        if not (_is_finite_number(x) and _is_finite_number(y) and _is_finite_number(z)):
            raise StrictCommandError("waypoint position must be finite")
        result.append({"idx": expected_idx, "x": _round3(x), "y": _round3(y), "z": _round3(z)})
    return result



class WaypointRecorder:
    def __init__(self):
        self.waypoints = []
        self.drone_id = str(rospy.get_param("~drone_id", "1"))
        self.prefix = f"/drone_{self.drone_id}_"
        self.frame_id = rospy.get_param("~frame_id", "world")
        self.json_path = rospy.get_param(
            "~json_path",
            os.path.join(DEFAULT_INCLUDE_DIR, "waypoints.json"),
        )
        self.include_dir = os.path.dirname(self.json_path)
        self.cloud_temp_dir = rospy.get_param(
            "~cloud_temp_dir", os.path.join(self.include_dir, ".tmp_clouds"),
        )
        self.localization_pcd_voxel_leaf_size = float(rospy.get_param(
            "~localization_pcd_voxel_leaf_size", 0.05,
        ))
        if self.localization_pcd_voxel_leaf_size <= 0 or not math.isfinite(self.localization_pcd_voxel_leaf_size):
            rospy.logwarn("Invalid localization_pcd_voxel_leaf_size; falling back to 0.05m")
            self.localization_pcd_voxel_leaf_size = 0.05
        self.localization_pcd_max_recorded_points = int(rospy.get_param(
            "~localization_pcd_max_recorded_points", 10000000,
        ))
        if self.localization_pcd_max_recorded_points <= 0:
            rospy.logwarn("Invalid localization_pcd_max_recorded_points; falling back to 10000000")
            self.localization_pcd_max_recorded_points = 10000000
        self.current_route_id = ""
        self.cloud_state = "idle"
        self.cloud_points = []
        self.cloud_fields = None
        self.cloud_frame_id = self.frame_id
        self.cloud_record_started = None
        self.cloud_record_error = None
        self.cloud_record_error_code = None
        self.cloud_record_request_id = ""
        self.cloud_record_route_id = ""
        self.cloud_record_consumed = False
        self.unsaved_cloud_tmp_path = None
        self.unsaved_cloud_point_count = 0
        self.current_cloud_path = None
        self.current_cloud_point_count = 0
        self.saved_project_name = None
        self.cloud_lock = threading.RLock()
        self.command_lock = threading.Lock()

        os.makedirs(self.include_dir, exist_ok=True)
        os.makedirs(self.cloud_temp_dir, exist_ok=True)
        self._cleanup_stale_temp_clouds()

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
        self.localization_status_pub = rospy.Publisher(
            f"{self.prefix}localization_cloud_status", String, queue_size=1, latch=True,
        )
        self.localization_pcl_pub = rospy.Publisher(
            f"{self.prefix}localization_pcl", PointCloud2, queue_size=1, latch=True,
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
        rospy.Subscriber(
            f"{self.prefix}new_waypoint_route", String, self._new_route_cb, queue_size=10,
        )
        rospy.Subscriber(
            f"{self.prefix}localization_cloud_record_start", String, self._cloud_record_start_cb, queue_size=10,
        )
        rospy.Subscriber(
            f"{self.prefix}localization_cloud_record_stop", String, self._cloud_record_stop_cb, queue_size=10,
        )
        rospy.Subscriber(
            f"{self.prefix}cloud_registered", PointCloud2, self._cloud_cb, queue_size=10,
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
        self._publish_localization_status("", "", "new", True)

        rospy.loginfo("WaypointRecorder ready  drone=%s  prefix=%s  frame=%s  json=%s",
                      self.drone_id, self.prefix, self.frame_id, self.json_path)

    # --------------------------------------------------------- exec guard
    def _check_exec_guard(self, action_name):
        """Return True if the action should be rejected (currently executing)."""
        if self.exec_state == "executing":
            rospy.logwarn("Rejected %s: waypoint execution in progress", action_name)
            return True
        return False

    def _check_route_edit_guard(self, action_name):
        if self._check_exec_guard(action_name):
            return True
        if self.cloud_state == "recording":
            rospy.logwarn("Rejected %s: cloud recording in progress", action_name)
            return True
        if not self.current_route_id:
            rospy.logwarn("Rejected %s: no active route; publish new_waypoint_route first", action_name)
            return True
        return False

    def _validate_active_route_id(self, route_id, action_name):
        if not self.current_route_id:
            raise StrictCommandError("start a new route before {}".format(action_name))
        if route_id != self.current_route_id:
            raise StrictCommandError("route_id does not match active route")

    def _check_command_busy_guard(self, request_id, route_id, phase, action_name, project_name=None):
        if self._check_exec_guard(action_name):
            self._publish_localization_status(
                request_id, route_id, phase, False, "executing", "waypoint execution is in progress",
                project_name=project_name,
            )
            return True
        if self.cloud_state == "recording":
            self._publish_localization_status(
                request_id, route_id, phase, False, "recording", "cloud recording is in progress",
                project_name=project_name,
            )
            return True
        return False

    def _try_acquire_command(self, request_id, route_id, phase, action_name, project_name=None):
        if self.command_lock.acquire(False):
            return True
        self._publish_localization_status(
            request_id, route_id, phase, False, "busy", "waypoint recorder is busy",
            project_name=project_name,
        )
        rospy.logwarn("Rejected %s: waypoint recorder is busy", action_name)
        return False

    def _try_acquire_silent_command(self, action_name):
        if self.command_lock.acquire(False):
            return True
        rospy.logwarn("Rejected %s: waypoint recorder is busy", action_name)
        return False

    # --------------------------------------------------------- waypoint callbacks
    def _add_waypoint_cb(self, msg):
        if self._check_route_edit_guard("add_waypoint"):
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
        if self._check_route_edit_guard("remove_waypoint"):
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
        if self._check_route_edit_guard("clear_waypoints"):
            return
        count = len(self.waypoints)
        self.waypoints = []
        rospy.loginfo("All %d waypoints cleared", count)
        self._publish_markers()

    def _strict_command_or_status(self, msg, phase, required=()):
        try:
            return _parse_json_command(msg.data, required=required)
        except StrictCommandError as exc:
            request_id, route_id = _extract_command_ids(msg.data, self.current_route_id)
            rospy.logerr("%s rejected: %s", phase, exc)
            self._publish_localization_status(
                request_id=request_id,
                route_id=route_id,
                phase=phase,
                ok=False,
                error_code="invalid_command",
                error_message=str(exc),
            )
            return None

    def _save_waypoints_cb(self, msg):
        data = self._strict_command_or_status(msg, "save", required=("name",))
        if data is None:
            return
        request_id = data["request_id"]
        route_id = data.get("route_id")
        project_name = data.get("name")
        if not self._try_acquire_command(request_id, route_id, "save", "save_waypoints", project_name=project_name):
            return
        try:
            if self._check_command_busy_guard(
                request_id, route_id, "save", "save_waypoints", project_name=project_name,
            ):
                return
            name = _validate_project_name(data["name"])
            self._save_project(name, route_id)
            self.current_route_id = route_id
            self.saved_project_name = name
            self._publish_project_list()
            self._publish_localization_status(request_id, route_id, "save", True, project_name=name)
        except Exception as exc:
            rospy.logerr("Save rejected: %s", exc)
            self._publish_localization_status(
                request_id, route_id, "save", False, "save_failed", str(exc), project_name=data.get("name"),
            )
        finally:
            self.command_lock.release()

    def _load_waypoints_cb(self, msg):
        data = self._strict_command_or_status(msg, "load", required=("name",))
        if data is None:
            return
        request_id = data["request_id"]
        route_id = data.get("route_id") or _make_route_id()
        project_name = data.get("name")
        if not self._try_acquire_command(request_id, route_id, "load", "load_waypoints", project_name=project_name):
            return
        try:
            if self._check_command_busy_guard(
                request_id, route_id, "load", "load_waypoints", project_name=project_name,
            ):
                return
            name = _validate_project_name(data["name"])
            path = os.path.join(self.include_dir, name + ".json")
            route_data = self._read_route_json(path, expected_name=name, validate_pcd=True)
            next_waypoints = _validate_waypoints_from_json(route_data)
            point_cloud = route_data.get("point_cloud")
            next_cloud_path = None
            next_cloud_count = 0
            next_cloud_frame_id = self.frame_id
            next_cloud_state = "idle"
            if point_cloud is not None:
                next_cloud_count = int(point_cloud["point_count"])
                next_cloud_path = os.path.join(self.include_dir, point_cloud["file"])
                next_cloud_frame_id = point_cloud["frame_id"]
                next_cloud_state = "loaded"
                loaded_fields, loaded_points = self._read_pcd(next_cloud_path, expected_count=next_cloud_count)
            else:
                loaded_fields, loaded_points = None, None
            self._discard_unsaved_cloud(strict=True)
            self.waypoints = next_waypoints
            self._publish_markers()
            self.current_route_id = route_id
            self.saved_project_name = name
            self.cloud_record_consumed = True
            self.current_cloud_path = next_cloud_path
            self.current_cloud_point_count = next_cloud_count
            self.cloud_frame_id = next_cloud_frame_id
            self.cloud_state = next_cloud_state
            if loaded_fields is not None and loaded_points is not None:
                self._publish_localization_pcl(loaded_fields, loaded_points, next_cloud_frame_id)
                del loaded_points
            else:
                self._publish_empty_localization_pcl(next_cloud_frame_id)
            self._publish_localization_status(
                request_id, route_id, "load", True, project_name=name, point_count=next_cloud_count,
            )
        except Exception as exc:
            rospy.logerr("Load rejected: %s", exc)
            self._publish_localization_status(
                request_id, route_id, "load", False, "load_failed", str(exc), project_name=data.get("name"),
            )
        finally:
            self.command_lock.release()

    def _delete_project_cb(self, msg):
        data = self._strict_command_or_status(msg, "delete", required=("names",))
        if data is None:
            return
        request_id = data["request_id"]
        route_id = data.get("route_id") or self.current_route_id
        if not self._try_acquire_command(request_id, route_id, "delete", "delete_waypoint_project"):
            return
        try:
            if self._check_command_busy_guard(request_id, route_id, "delete", "delete_waypoint_project"):
                return
            raw_names = data["names"]
            if not isinstance(raw_names, list) or not raw_names:
                raise StrictCommandError("names must be a non-empty list")
            names = [_validate_project_name(name) for name in raw_names]
            for name in names:
                for ext in (".json", ".pcd"):
                    path = os.path.join(self.include_dir, name + ext)
                    if os.path.isfile(path):
                        os.remove(path)
                        rospy.loginfo("Deleted project file: %s", path)
            if self.saved_project_name in names:
                self._discard_unsaved_cloud(strict=True)
                self.saved_project_name = None
                self.current_route_id = ""
                self.current_cloud_path = None
                self.current_cloud_point_count = 0
                self.cloud_state = "idle"
                self.waypoints = []
                self._publish_markers()
                self._publish_empty_localization_pcl()
            self._publish_project_list()
            self._publish_localization_status(request_id, route_id, "delete", True)
        except Exception as exc:
            rospy.logerr("Delete rejected: %s", exc)
            self._publish_localization_status(
                request_id, route_id, "delete", False, "delete_failed", str(exc),
            )
        finally:
            self.command_lock.release()

    def _reorder_waypoints_cb(self, msg):
        data = self._strict_command_or_status(msg, "reorder", required=("order",))
        if data is None:
            return
        request_id = data["request_id"]
        route_id = data.get("route_id")
        if not self._try_acquire_command(request_id, route_id, "reorder", "reorder_waypoints"):
            return
        try:
            if self._check_command_busy_guard(request_id, route_id, "reorder", "reorder_waypoints"):
                return
            self._validate_active_route_id(route_id, "reorder")
            order = data.get("order")
            if not isinstance(order, list) or len(order) != len(self.waypoints):
                raise StrictCommandError("invalid order length")
            try:
                reordered = [self.waypoints[int(i) - 1] for i in order]
            except (IndexError, ValueError, TypeError) as exc:
                raise StrictCommandError("invalid reorder index") from exc
            for i, wp in enumerate(reordered):
                wp["idx"] = i + 1
            self.waypoints = reordered
            rospy.loginfo("Waypoints reordered: %s", order)
            self._publish_markers()
            self._publish_localization_status(request_id, route_id, "reorder", True)
        except Exception as exc:
            rospy.logerr("Reorder rejected: %s", exc)
            self._publish_localization_status(
                request_id, route_id, "reorder", False, "reorder_failed", str(exc),
            )
        finally:
            self.command_lock.release()

    def _new_route_cb(self, msg):
        data = self._strict_command_or_status(msg, "new")
        if data is None:
            return
        request_id = data["request_id"]
        route_id = data.get("route_id") or _make_route_id()
        if not self._try_acquire_command(request_id, route_id, "new", "new_waypoint_route"):
            return
        try:
            if self._check_command_busy_guard(request_id, route_id, "new", "new_waypoint_route"):
                return
            self._discard_unsaved_cloud(strict=True)
            self.current_route_id = route_id
            self.saved_project_name = None
            self.current_cloud_path = None
            self.current_cloud_point_count = 0
            self.waypoints = []
            self.cloud_state = "idle"
            self.cloud_record_consumed = False
            self._publish_markers()
            self._publish_empty_localization_pcl()
            self._publish_localization_status(request_id, route_id, "new", True)
        except Exception as exc:
            rospy.logerr("New route rejected: %s", exc)
            self._publish_localization_status(
                request_id, route_id, "new", False, "new_failed", str(exc),
            )
        finally:
            self.command_lock.release()

    def _cloud_record_start_cb(self, msg):
        data = self._strict_command_or_status(msg, "record_start")
        if data is None:
            return
        request_id = data["request_id"]
        route_id = data.get("route_id")
        if not self._try_acquire_command(request_id, route_id, "record_start", "localization_cloud_record_start"):
            return
        try:
            if self.exec_state == "executing":
                self._publish_localization_status(
                    request_id, route_id, "record_start", False, "executing", "waypoint execution is in progress",
                )
                return
            if self.cloud_state == "recording":
                self._publish_localization_status(
                    request_id, route_id, "record_start", False, "already_recording", "cloud recording already started",
                )
                return
            if not self.current_route_id or route_id != self.current_route_id:
                self._publish_localization_status(
                    request_id, route_id, "record_start", False, "route_mismatch", "start a new route before recording point cloud",
                )
                return
            if self.saved_project_name is not None or self.cloud_state == "loaded":
                self._publish_localization_status(
                    request_id, route_id, "record_start", False, "loaded_route", "loaded or saved routes cannot record point cloud",
                )
                return
            if self.cloud_record_consumed or self.cloud_state == "recorded":
                self._publish_localization_status(
                    request_id, route_id, "record_start", False, "recording_consumed", "this route already has a completed point cloud recording",
                )
                return
            self._discard_unsaved_cloud()
            self.current_route_id = route_id
            self.saved_project_name = None
            self.current_cloud_path = None
            self.current_cloud_point_count = 0
            with self.cloud_lock:
                self.cloud_points = []
                self.cloud_fields = None
                self.cloud_frame_id = self.frame_id
                self.cloud_record_started = rospy.Time.now()
                self.cloud_record_error = None
                self.cloud_record_error_code = None
                self.cloud_record_request_id = request_id
                self.cloud_record_route_id = route_id
                self.cloud_state = "recording"
            self._publish_localization_status(request_id, route_id, "record_start", True)
        finally:
            self.command_lock.release()

    def _cloud_record_stop_cb(self, msg):
        data = self._strict_command_or_status(msg, "record_stop")
        if data is None:
            return
        request_id = data["request_id"]
        route_id = data.get("route_id")
        if not self._try_acquire_command(request_id, route_id, "record_stop", "localization_cloud_record_stop"):
            return
        tmp_path = None
        status_error_code = "record_stop_failed"
        try:
            with self.cloud_lock:
                if self.cloud_state != "recording":
                    if self.cloud_record_error is not None:
                        error_message = self.cloud_record_error
                        status_error_code = self.cloud_record_error_code or "record_stop_failed"
                        error_route_id = self.cloud_record_route_id
                        self.cloud_record_error = None
                        self.cloud_record_error_code = None
                        self.cloud_record_request_id = ""
                        self.cloud_record_route_id = ""
                        route_id = error_route_id or route_id
                        raise StrictCommandError(error_message)
                    raise StrictCommandError("cloud recording is not active")
                if route_id != self.current_route_id:
                    raise StrictCommandError("route_id does not match active route")
                self.cloud_state = "idle"
                fields = list(self.cloud_fields) if self.cloud_fields is not None else None
                points = self.cloud_points
                self.cloud_points = []
                self.cloud_fields = None
                self.cloud_record_started = None
                self.cloud_record_error = None
                self.cloud_record_error_code = None
                self.cloud_record_request_id = ""
                self.cloud_record_route_id = ""
            if not points or fields is None:
                raise StrictCommandError("no point cloud frames were recorded")
            pcd_fields = self._pcd_fields(fields)
            downsampled_points = self._downsample_points(pcd_fields, points)
            del points
            if not downsampled_points:
                raise StrictCommandError("point cloud is empty after downsampling")
            tmp_path = os.path.join(self.cloud_temp_dir, _safe_route_file_stem(route_id) + ".pcd")
            self._write_pcd(tmp_path, pcd_fields, downsampled_points)
            self._publish_localization_pcl(pcd_fields, downsampled_points, self.cloud_frame_id)
            downsampled_count = len(downsampled_points)
            del downsampled_points
            self.unsaved_cloud_tmp_path = tmp_path
            self.unsaved_cloud_point_count = downsampled_count
            self.current_cloud_path = None
            self.current_cloud_point_count = 0
            self.cloud_record_consumed = True
            self.cloud_state = "recorded"
            self._publish_localization_status(request_id, route_id, "record_stop", True)
        except Exception as exc:
            rospy.logerr("Stop cloud recording rejected: %s", exc)
            if tmp_path and os.path.isfile(tmp_path):
                try:
                    os.remove(tmp_path)
                except OSError as remove_exc:
                    rospy.logwarn("Failed to delete failed temp cloud %s: %s", tmp_path, remove_exc)
            if self.unsaved_cloud_tmp_path == tmp_path:
                self.unsaved_cloud_tmp_path = None
                self.unsaved_cloud_point_count = 0
            if self.cloud_state == "recording":
                self.cloud_state = "idle"
            with self.cloud_lock:
                self.cloud_points = []
                self.cloud_fields = None
                self.cloud_record_started = None
                self.cloud_record_error = None
                self.cloud_record_error_code = None
                self.cloud_record_request_id = ""
                self.cloud_record_route_id = ""
            self._publish_localization_status(
                request_id, route_id, "record_stop", False, status_error_code, str(exc),
            )
        finally:
            self.command_lock.release()

    def _cloud_cb(self, msg):
        with self.cloud_lock:
            if self.cloud_state != "recording":
                return
            if self.cloud_fields is None:
                self.cloud_fields = [(field.name, field.datatype, field.count) for field in msg.fields]
                self.cloud_frame_id = msg.header.frame_id or self.frame_id
            fields = list(self.cloud_fields)
            recorded_count = len(self.cloud_points)
        try:
            x_idx, y_idx, z_idx = self._xyz_indices(fields)
            frame_points = []
            record_error = None
            for point in point_cloud2.read_points(msg, skip_nans=True):
                if (
                    len(point) > max(x_idx, y_idx, z_idx)
                    and all(_is_finite_number(point[index]) for index in (x_idx, y_idx, z_idx))
                ):
                    if recorded_count + len(frame_points) + 1 > self.localization_pcd_max_recorded_points:
                        record_error = "recorded points exceeded max {}".format(
                            self.localization_pcd_max_recorded_points,
                        )
                        break
                    frame_points.append(tuple(float(v) for v in point))
            with self.cloud_lock:
                if self.cloud_state == "recording":
                    if (
                        record_error is None
                        and len(self.cloud_points) + len(frame_points) > self.localization_pcd_max_recorded_points
                    ):
                        record_error = "recorded points exceeded max {}".format(
                            self.localization_pcd_max_recorded_points,
                        )
                    if record_error is not None:
                        self.cloud_state = "idle"
                        self.cloud_points = []
                        self.cloud_fields = None
                        self.cloud_record_started = None
                        self.cloud_record_error_code = "record_too_large"
                        self.cloud_record_error = record_error
                        rospy.logerr(
                            "Cloud recording stopped: recorded points exceeded max %d",
                            self.localization_pcd_max_recorded_points,
                        )
                    elif frame_points:
                        self.cloud_points.extend(frame_points)
            if record_error is not None:
                rospy.logwarn("Cloud recording requires Stop command to ACK failure: %s", record_error)
        except Exception as exc:
            rospy.logwarn("Failed to read point cloud frame: %s", exc)

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

    # ------------------------------------------------- localization/project helpers
    def _localization_capabilities(self):
        is_executing = self.exec_state == "executing"
        is_recording = self.cloud_state == "recording"
        has_route = bool(self.current_route_id)
        unsaved_or_editable_route = has_route and self.saved_project_name is None and self.cloud_state != "loaded"
        can_mutate_route = not is_executing and not is_recording
        can_save = (
            can_mutate_route
            and has_route
            and (bool(self.waypoints) or self.unsaved_cloud_tmp_path is not None or self.current_cloud_path is not None)
        )
        can_record = (
            can_mutate_route
            and unsaved_or_editable_route
            and self.cloud_state == "idle"
            and not self.cloud_record_consumed
        )
        return {
            "can_new_route": can_mutate_route,
            "can_load_project": can_mutate_route,
            "can_delete_project": can_mutate_route,
            "can_edit_waypoints": can_mutate_route and has_route,
            "can_save_project": can_save,
            "can_start_cloud_record": can_record,
            "can_stop_cloud_record": (not is_executing and is_recording),
        }

    def _publish_localization_status(self, request_id, route_id, phase, ok,
                                     error_code=None, error_message=None, project_name=None,
                                     point_count=None):
        if phase not in LOCALIZATION_PHASES:
            phase = "load"
        project = None
        if project_name is not None or point_count is not None or self.cloud_state in ("recorded", "loaded"):
            default_point_count = self.current_cloud_point_count if self.current_cloud_point_count > 0 else self.unsaved_cloud_point_count
            pc_count = int(point_count if point_count is not None else default_point_count)
            safe_project_name = project_name if isinstance(project_name, str) and NAME_RE.match(project_name) else None
            project = {
                "name": safe_project_name,
                "has_point_cloud": pc_count > 0,
                "point_count": pc_count,
            }
        status_route_id = route_id or self.current_route_id
        capabilities = self._localization_capabilities()
        payload = {
            "schema_version": SCHEMA_VERSION,
            "request_id": request_id,
            "route_id": status_route_id,
            "phase": phase,
            "ok": bool(ok),
            "state": self.cloud_state if self.cloud_state in LOCALIZATION_STATES else "idle",
            "can_record_cloud": capabilities["can_start_cloud_record"],
            "capabilities": capabilities,
            "project": project,
            "error": None if ok else {"code": error_code or "error", "message": error_message or "failed"},
        }
        self.localization_status_pub.publish(String(data=json.dumps(payload, sort_keys=True)))

    def _publish_project_list(self):
        projects = []
        for path in sorted(glob.glob(os.path.join(self.include_dir, "*.json"))):
            try:
                route_data = self._read_route_json(path, validate_pcd=True)
                project = route_data["project"]
                point_cloud = route_data.get("point_cloud")
                projects.append({
                    "name": project["name"],
                    "waypoint_count": len(route_data["waypoints"]),
                    "has_point_cloud": point_cloud is not None,
                    "point_count": 0 if point_cloud is None else int(point_cloud["point_count"]),
                })
            except Exception as exc:
                rospy.logwarn("Skipping invalid route json %s: %s", path, exc)
        payload = json.dumps({"schema_version": SCHEMA_VERSION, "projects": projects}, sort_keys=True)
        self.project_list_pub.publish(String(data=payload))
        rospy.loginfo("Project list published: %s", [p["name"] for p in projects])

    def _discard_unsaved_cloud(self, strict=False):
        tmp_path = self.unsaved_cloud_tmp_path
        tmp_removed = True
        if tmp_path and os.path.isfile(tmp_path):
            try:
                os.remove(tmp_path)
            except OSError as exc:
                tmp_removed = False
                rospy.logwarn("Failed to delete temp cloud %s: %s", tmp_path, exc)
                if strict:
                    raise StrictCommandError("failed to delete temporary PCD file") from exc
        preserve_loaded = self.cloud_state == "loaded"
        with self.cloud_lock:
            self.cloud_points = []
            self.cloud_fields = None
            self.cloud_record_started = None
            self.cloud_record_error = None
            self.cloud_record_error_code = None
            self.cloud_record_request_id = ""
            self.cloud_record_route_id = ""
            if not preserve_loaded:
                self.cloud_state = "idle"
        self.unsaved_cloud_point_count = 0
        if not preserve_loaded:
            self.current_cloud_path = None
            self.current_cloud_point_count = 0
        if tmp_removed:
            self.unsaved_cloud_tmp_path = None

    def _cleanup_stale_temp_clouds(self):
        for path in glob.glob(os.path.join(self.cloud_temp_dir, "*.pcd")):
            try:
                os.remove(path)
            except OSError as exc:
                rospy.logwarn("Failed to clean temp cloud %s: %s", path, exc)

    def _pointfield_type_from_pcd(self, type_name, size):
        if type_name == "F" and size == 4:
            return PointField.FLOAT32
        if type_name == "F" and size == 8:
            return PointField.FLOAT64
        if type_name == "I" and size == 1:
            return PointField.INT8
        if type_name == "I" and size == 2:
            return PointField.INT16
        if type_name == "I" and size == 4:
            return PointField.INT32
        if type_name == "U" and size == 1:
            return PointField.UINT8
        if type_name == "U" and size == 2:
            return PointField.UINT16
        if type_name == "U" and size == 4:
            return PointField.UINT32
        raise StrictCommandError("unsupported PCD field type")

    def _read_pcd_header(self, path, expected_count=None):
        if not os.path.isfile(path):
            raise StrictCommandError("PCD file is missing")
        header = {}
        data_start = None
        with open(path, "r") as f:
            while True:
                raw_line = f.readline()
                if raw_line == "":
                    break
                line = raw_line.strip()
                if not line or line.startswith("#"):
                    continue
                parts = line.split()
                key = parts[0].upper()
                values = parts[1:]
                header[key] = values
                if key == "DATA":
                    if values != ["ascii"]:
                        raise StrictCommandError("PCD DATA must be ascii")
                    data_start = f.tell()
                    break
        if data_start is None:
            raise StrictCommandError("PCD DATA section is missing")
        field_names = header.get("FIELDS")
        sizes = [int(v) for v in header.get("SIZE", [])]
        types = header.get("TYPE")
        counts = [int(v) for v in header.get("COUNT", [])]
        if not field_names or not sizes or not types or not counts:
            raise StrictCommandError("PCD field metadata is incomplete")
        if not (len(field_names) == len(sizes) == len(types) == len(counts)):
            raise StrictCommandError("PCD field metadata length mismatch")
        if any(count != 1 for count in counts):
            raise StrictCommandError("PCD COUNT values must be 1")
        try:
            point_count = int(header.get("POINTS", ["-1"])[0])
        except (TypeError, ValueError) as exc:
            raise StrictCommandError("PCD POINTS is invalid") from exc
        if point_count <= 0:
            raise StrictCommandError("PCD POINTS must be positive")
        if expected_count is not None and point_count != int(expected_count):
            raise StrictCommandError("PCD point count does not match route JSON")
        if point_count > self.localization_pcd_max_recorded_points:
            raise StrictCommandError("PCD point count exceeds max {}".format(
                self.localization_pcd_max_recorded_points,
            ))
        try:
            xyz_indices = [field_names.index(name) for name in ("x", "y", "z")]
        except ValueError as exc:
            raise StrictCommandError("PCD must contain x/y/z fields") from exc
        fields = [
            (name, self._pointfield_type_from_pcd(type_name, size), count)
            for name, type_name, size, count in zip(field_names, types, sizes, counts)
        ]
        return data_start, field_names, fields, xyz_indices, point_count

    def _iter_pcd_points(self, path, data_start, field_names, xyz_indices):
        with open(path, "r") as f:
            f.seek(data_start)
            for raw_line in f:
                line = raw_line.strip()
                if not line or line.startswith("#"):
                    continue
                values = line.split()
                if len(values) != len(field_names):
                    raise StrictCommandError("PCD point field count mismatch")
                try:
                    point = tuple(float(value) for value in values)
                except ValueError as exc:
                    raise StrictCommandError("PCD point contains non-numeric value") from exc
                if not all(_is_finite_number(point[index]) for index in xyz_indices):
                    raise StrictCommandError("PCD point contains non-finite x/y/z")
                yield point

    def _validate_pcd_file(self, path, expected_count=None):
        data_start, field_names, _fields, xyz_indices, point_count = self._read_pcd_header(
            path, expected_count=expected_count,
        )
        read_count = 0
        for _point in self._iter_pcd_points(path, data_start, field_names, xyz_indices):
            read_count += 1
            if read_count > point_count:
                raise StrictCommandError("PCD data line count does not match POINTS")
        if read_count != point_count:
            raise StrictCommandError("PCD data line count does not match POINTS")

    def _read_pcd(self, path, expected_count=None):
        data_start, field_names, fields, xyz_indices, point_count = self._read_pcd_header(
            path, expected_count=expected_count,
        )
        points = []
        for point in self._iter_pcd_points(path, data_start, field_names, xyz_indices):
            points.append(point)
            if len(points) > point_count:
                raise StrictCommandError("PCD data line count does not match POINTS")
        if len(points) != point_count:
            raise StrictCommandError("PCD data line count does not match POINTS")
        return fields, points

    def _xyz_indices(self, fields):
        names = [field[0] for field in fields]
        try:
            return names.index("x"), names.index("y"), names.index("z")
        except ValueError as exc:
            raise StrictCommandError("point cloud fields must contain x/y/z") from exc

    def _pcd_fields(self, fields):
        return [(field[0], PointField.FLOAT32, 1) for field in fields]

    def _downsample_points(self, fields, points):
        leaf = self.localization_pcd_voxel_leaf_size
        x_idx, y_idx, z_idx = self._xyz_indices(fields)
        seen_voxels = set()
        downsampled = []
        for point in points:
            if len(point) < len(fields):
                continue
            x, y, z = point[x_idx], point[y_idx], point[z_idx]
            if not (_is_finite_number(x) and _is_finite_number(y) and _is_finite_number(z)):
                continue
            key = (math.floor(float(x) / leaf), math.floor(float(y) / leaf), math.floor(float(z) / leaf))
            if key in seen_voxels:
                continue
            seen_voxels.add(key)
            downsampled.append(tuple(float(value) for value in point[:len(fields)]))
        rospy.loginfo(
            "Localization cloud downsampled: raw=%d final=%d leaf=%.3fm",
            len(points), len(downsampled), leaf,
        )
        return downsampled

    def _ros_fields_from_cloud_fields(self, fields):
        ros_fields = []
        offset = 0
        size_by_type = {
            PointField.INT8: 1,
            PointField.UINT8: 1,
            PointField.INT16: 2,
            PointField.UINT16: 2,
            PointField.INT32: 4,
            PointField.UINT32: 4,
            PointField.FLOAT32: 4,
            PointField.FLOAT64: 8,
        }
        for name, datatype, count in fields:
            field = PointField()
            field.name = str(name)
            field.offset = offset
            field.datatype = int(datatype)
            field.count = int(count)
            ros_fields.append(field)
            offset += size_by_type.get(field.datatype, 4) * field.count
        return ros_fields

    def _publish_localization_pcl(self, fields, points, frame_id=None):
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = frame_id or self.cloud_frame_id or self.frame_id
        msg = point_cloud2.create_cloud(header, self._ros_fields_from_cloud_fields(fields), points)
        self.localization_pcl_pub.publish(msg)
        rospy.loginfo("Published localization PCL: points=%d frame=%s", len(points), header.frame_id)

    def _publish_empty_localization_pcl(self, frame_id=None):
        fields = [("x", PointField.FLOAT32, 1), ("y", PointField.FLOAT32, 1), ("z", PointField.FLOAT32, 1)]
        self._publish_localization_pcl(fields, [], frame_id or self.frame_id)

    def _save_project(self, name, route_id):
        self._validate_active_route_id(route_id, "saving")
        if not self.waypoints and self.unsaved_cloud_tmp_path is None and self.current_cloud_path is None:
            raise StrictCommandError("cannot save an empty route")
        json_path = os.path.join(self.include_dir, name + ".json")
        pcd_path = os.path.join(self.include_dir, name + ".pcd")
        pcd_tmp_path = None
        pcd_backup_path = None
        point_cloud_meta = None
        was_loaded = self.cloud_state == "loaded"

        try:
            if self.unsaved_cloud_tmp_path is not None:
                if not os.path.isfile(self.unsaved_cloud_tmp_path):
                    raise StrictCommandError("temporary PCD file is missing")
                pcd_tmp_path = pcd_path + ".tmp"
                shutil.copyfile(self.unsaved_cloud_tmp_path, pcd_tmp_path)
                point_cloud_meta = {
                    "file": name + ".pcd",
                    "frame_id": self.cloud_frame_id,
                    "point_count": int(self.unsaved_cloud_point_count),
                }
            elif self.current_cloud_path is not None:
                if not os.path.isfile(self.current_cloud_path):
                    raise StrictCommandError("loaded PCD file is missing")
                if self.current_cloud_point_count <= 0:
                    raise StrictCommandError("loaded PCD point count is invalid")
                if os.path.abspath(self.current_cloud_path) != os.path.abspath(pcd_path):
                    pcd_tmp_path = pcd_path + ".tmp"
                    shutil.copyfile(self.current_cloud_path, pcd_tmp_path)
                point_cloud_meta = {
                    "file": name + ".pcd",
                    "frame_id": self.cloud_frame_id,
                    "point_count": int(self.current_cloud_point_count),
                }

            data = {
                "schema_version": SCHEMA_VERSION,
                "kind": ROUTE_KIND,
                "project": {"name": name, "drone_id": self.drone_id, "frame_id": self.frame_id},
                "anchor": {"position": {"x": 0, "y": 0, "z": 0}},
                "waypoints": [
                    {"idx": wp["idx"], "position": {"x": wp["x"], "y": wp["y"], "z": wp["z"]}}
                    for wp in self.waypoints
                ],
                "point_cloud": point_cloud_meta,
            }
            self._validate_route_data(data, validate_pcd=False)

            if pcd_tmp_path is not None:
                pcd_backup_path = pcd_path + ".bak"
                if os.path.exists(pcd_backup_path):
                    os.remove(pcd_backup_path)
                if os.path.exists(pcd_path):
                    os.replace(pcd_path, pcd_backup_path)
                os.replace(pcd_tmp_path, pcd_path)
            elif point_cloud_meta is None and os.path.isfile(pcd_path):
                pcd_backup_path = pcd_path + ".bak"
                if os.path.exists(pcd_backup_path):
                    os.remove(pcd_backup_path)
                os.replace(pcd_path, pcd_backup_path)

            self._validate_route_data(data, validate_pcd=True)
            if point_cloud_meta is not None:
                self._validate_pcd_file(pcd_path, expected_count=point_cloud_meta["point_count"])
            _atomic_write_json(json_path, data)
        except Exception:
            if pcd_tmp_path is not None and os.path.isfile(pcd_tmp_path):
                try:
                    os.remove(pcd_tmp_path)
                except OSError as exc:
                    rospy.logwarn("Failed to delete failed temp PCD %s: %s", pcd_tmp_path, exc)
            if pcd_backup_path is not None:
                if os.path.isfile(pcd_path):
                    try:
                        os.remove(pcd_path)
                    except OSError as exc:
                        rospy.logwarn("Failed to roll back PCD %s: %s", pcd_path, exc)
                if os.path.isfile(pcd_backup_path):
                    os.replace(pcd_backup_path, pcd_path)
            raise

        if pcd_backup_path is not None and os.path.isfile(pcd_backup_path):
            try:
                os.remove(pcd_backup_path)
            except OSError as exc:
                rospy.logwarn("Failed to delete PCD backup %s: %s", pcd_backup_path, exc)

        if self.unsaved_cloud_tmp_path and os.path.isfile(self.unsaved_cloud_tmp_path):
            try:
                os.remove(self.unsaved_cloud_tmp_path)
            except OSError as exc:
                rospy.logwarn("Failed to delete saved temp cloud %s: %s", self.unsaved_cloud_tmp_path, exc)
        self.unsaved_cloud_tmp_path = None
        self.cloud_points = []
        self.cloud_fields = None

        self.cloud_record_consumed = True
        if point_cloud_meta is not None:
            self.current_cloud_path = pcd_path
            self.current_cloud_point_count = int(point_cloud_meta["point_count"])
            self.unsaved_cloud_point_count = 0
            self.cloud_state = "loaded" if was_loaded else "recorded"
        else:
            self.current_cloud_path = None
            self.current_cloud_point_count = 0
            self.unsaved_cloud_point_count = 0
            if not was_loaded:
                self.cloud_state = "idle"
        rospy.loginfo("Saved route %s with %d waypoints", name, len(self.waypoints))

    def _read_route_json(self, path, expected_name=None, validate_pcd=True):
        if not os.path.isfile(path):
            raise StrictCommandError("route JSON not found: {}".format(path))
        with open(path, "r") as f:
            data = json.load(f)
        self._validate_route_data(data, expected_name=expected_name, validate_pcd=validate_pcd)
        return data

    def _validate_route_data(self, data, expected_name=None, validate_pcd=True):
        if not isinstance(data, dict):
            raise StrictCommandError("route JSON must be an object")
        if data.get("schema_version") != SCHEMA_VERSION or data.get("kind") != ROUTE_KIND:
            raise StrictCommandError("invalid route JSON schema")
        project = data.get("project")
        if not isinstance(project, dict):
            raise StrictCommandError("project is required")
        name = _validate_project_name(project.get("name"))
        if expected_name is not None and name != expected_name:
            raise StrictCommandError("project name does not match file name")
        if project.get("drone_id") != self.drone_id:
            raise StrictCommandError("project drone_id does not match recorder")
        if project.get("frame_id") != self.frame_id:
            raise StrictCommandError("project frame_id does not match recorder")
        anchor = data.get("anchor")
        if not isinstance(anchor, dict) or not isinstance(anchor.get("position"), dict):
            raise StrictCommandError("anchor.position is required")
        pos = anchor["position"]
        if pos.get("x") != 0 or pos.get("y") != 0 or pos.get("z") != 0:
            raise StrictCommandError("anchor must be origin")
        _validate_waypoints_from_json(data)
        if "point_cloud" not in data:
            raise StrictCommandError("point_cloud is required")
        point_cloud = data.get("point_cloud")
        if point_cloud is not None:
            if not isinstance(point_cloud, dict):
                raise StrictCommandError("point_cloud must be null or object")
            file_name = point_cloud.get("file")
            if file_name != name + ".pcd":
                raise StrictCommandError("point_cloud file must match project name")
            if point_cloud.get("frame_id") != self.frame_id:
                raise StrictCommandError("point_cloud frame_id does not match recorder")
            if not isinstance(point_cloud.get("point_count"), int) or point_cloud["point_count"] <= 0:
                raise StrictCommandError("point_cloud point_count must be positive")
            if validate_pcd and not os.path.isfile(os.path.join(self.include_dir, file_name)):
                raise StrictCommandError("declared point cloud file is missing")

    def _write_pcd(self, path, fields, points):
        if not points:
            raise StrictCommandError("no points to write")
        field_names = [field[0] for field in fields]
        with open(path, "w") as f:
            f.write("# .PCD v0.7 - Point Cloud Data file format\n")
            f.write("VERSION 0.7\n")
            f.write("FIELDS {}\n".format(" ".join(field_names)))
            f.write("SIZE {}\n".format(" ".join("4" for _ in field_names)))
            f.write("TYPE {}\n".format(" ".join("F" for _ in field_names)))
            f.write("COUNT {}\n".format(" ".join("1" for _ in field_names)))
            f.write("WIDTH {}\n".format(len(points)))
            f.write("HEIGHT 1\n")
            f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
            f.write("POINTS {}\n".format(len(points)))
            f.write("DATA ascii\n")
            for point in points:
                f.write(" ".join("{:.6f}".format(float(value)) for value in point[:len(field_names)]) + "\n")

    # --------------------------------------------------------- file I/O
    def save_json(self, path=None):
        path = path or self.json_path
        self._save_project(os.path.splitext(os.path.basename(path))[0], self.current_route_id or "manual")

    def load_json(self, path=None, expected_name=None):
        path = path or self.json_path
        data = self._read_route_json(path, expected_name=expected_name, validate_pcd=True)
        self.waypoints = _validate_waypoints_from_json(data)
        self._publish_markers()
        rospy.loginfo("Loaded %d waypoints from %s", len(self.waypoints), path)
        return data

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



def main():
    rospy.init_node("waypoint_recorder", anonymous=False)
    recorder = WaypointRecorder()

    # Start with an empty working route. Saved projects are loaded only when
    # the frontend explicitly publishes /drone_{id}_load_waypoints.

    def _shutdown(*_args):
        if recorder.exec_state == "executing":
            recorder._halt_execution("Node shutdown")
        recorder._discard_unsaved_cloud()
        rospy.signal_shutdown("exit")
        sys.exit(0)

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    rospy.spin()


if __name__ == "__main__":
    main()
