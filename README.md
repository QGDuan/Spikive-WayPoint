# Spikive WayPoint

`Spikive-WayPoint` 是 Spikive 地面站使用的 ROS1 航点与 Localization 路线后端包。当前 ROS package 名称为 `waypoint_recorder`，主节点为 `scripts/waypoint_recorder.py`。

当前固定 tag：`spikive-waypoint-v1`。

## 设计原则

- 后端是路线、航点、PCD、ACK、`capabilities` 的唯一业务状态源。
- 前端只发送低频用户命令、渲染后端 status，并回显后端 ACK 错误。
- Topic 使用 `/drone_{id}_<base>` 命名，避免不同无人机之间状态串扰。
- `localization_pcl` 不持续发布，只在 Record Stop、Load 成功、New、Delete 当前项目、Load 无点云路线时发布一次。
- 不引入 `python-pcl`、Open3D 等额外依赖；点云保存前使用纯 Python voxel grid 降采样。

## 数据路径

默认数据路径跟随本 package，而不是旧的 `ego_ws` 绝对路径：

- 默认 JSON：`include/waypoints.json`
- 项目 JSON/PCD：`include/<project>.json`、`include/<project>.pcd`
- 临时未保存 PCD：`include/.tmp_clouds/<route_id>.pcd`

可通过 ROS 参数覆盖：

- `~json_path`
- `~cloud_temp_dir`
- `~frame_id`，默认 `world`
- `~drone_id`，默认 `1`
- `~localization_pcd_voxel_leaf_size`，默认 `0.05`
- `~localization_pcd_max_recorded_points`，默认 `10000000`

## 主要 Topic

航点编辑：

- `/drone_{id}_add_waypoint`：`geometry_msgs/PoseStamped`
- `/drone_{id}_remove_waypoint`：`std_msgs/Int32`
- `/drone_{id}_clear_waypoints`：`std_msgs/Empty`
- `/drone_{id}_waypoint_markers`：`visualization_msgs/MarkerArray`，latched

项目与 Localization 点云：

- `/drone_{id}_new_waypoint_route`：`std_msgs/String`，严格 JSON
- `/drone_{id}_save_waypoints`：`std_msgs/String`，严格 JSON
- `/drone_{id}_load_waypoints`：`std_msgs/String`，严格 JSON
- `/drone_{id}_delete_waypoint_project`：`std_msgs/String`，严格 JSON
- `/drone_{id}_reorder_waypoints`：`std_msgs/String`，严格 JSON
- `/drone_{id}_waypoint_project_list`：`std_msgs/String`，latched
- `/drone_{id}_localization_cloud_status`：`std_msgs/String`，latched
- `/drone_{id}_localization_cloud_record_start`：`std_msgs/String`，严格 JSON
- `/drone_{id}_localization_cloud_record_stop`：`std_msgs/String`，严格 JSON
- `/drone_{id}_cloud_registered`：`sensor_msgs/PointCloud2`
- `/drone_{id}_localization_pcl`：`sensor_msgs/PointCloud2`，latched

航线执行：

- `/drone_{id}_start_waypoint_exec`：`std_msgs/Empty`
- `/drone_{id}_stop_waypoint_exec`：`std_msgs/Empty`
- `/drone_{id}_waypoint_exec_state`：`std_msgs/String`，latched
- `/goal_with_id`：`quadrotor_msgs/GoalSet`
- `/control`：`controller_msgs/cmd`

## 点云闭环

Record Stop 时，节点会取走录制内存、执行 voxel 降采样、写入临时 PCD，并发布一次 `/drone_{id}_localization_pcl`。Save 只复制已降采样的临时 PCD；Save 失败时保留临时 PCD，允许重试；Save 成功后清理临时文件。

Load 会先验证路线 JSON、PCD 文件和点数，全部通过后才切换当前路线并发布点云。Load 失败不会破坏当前路线。New、Delete 当前项目、Load 无点云路线会发布空 `PointCloud2`，用于清理前端和下游的 latched 大消息。

超过 `~localization_pcd_max_recorded_points` 时，后端停止录制并清空 raw 点云缓存，后续 Record Stop 会返回匹配当前请求的失败 ACK，避免前端等到 timeout。

## 校验

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -c "from pathlib import Path; compile(Path('scripts/waypoint_recorder.py').read_text(), 'scripts/waypoint_recorder.py', 'exec'); print('syntax ok')"
```
