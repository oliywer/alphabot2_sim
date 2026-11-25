#!/usr/bin/env python3
"""Waypoint patrol for the AlphaBot2 digital twin.

Sends the robot through a list of waypoints (Nav2 NavigateToPose, one goal at a
time) for a given number of laps and logs every goal to a CSV file:
result, navigation time (simulation and wall clock), straight-line distance and number of recoveries.

Run AFTER sim.launch.py and nav.launch.py:
    ros2 run alphabot2_sim waypoint_patrol.py
    ros2 run alphabot2_sim waypoint_patrol.py --ros-args -p laps:=3

Waypoints are given as a flat list [x1, y1, yaw1, x2, y2, yaw2, ...] in the
`map` frame (metres, radians). With SLAM started at the spawn pose (0, 0, 0)
the map frame coincides with the Gazebo world frame of the world file.
"""
import csv
import math
import os
import time
from datetime import datetime

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# Free spots in worlds/office.sdf (world frame == map frame when spawned at 0,0,0)
DEFAULT_WAYPOINTS = [
    -2.0, 1.0, 0.0,     # A: N1 office
    -0.6, 1.75, 0.0,    # B: N2 meeting room (via the N1-N2 inner door or the corridor)
    2.0, 0.9, 0.0,      # C: N3 lounge, next to the sofa
    2.4, -0.8, 0.0,     # D: S2 lab
    2.6, -1.6, 0.0,     # E: S2 alcove behind the inner wall (dead end)
    -2.2, -0.6, 0.0,    # F: S1 storage
    0.0, 0.0, 0.0,      # home (corridor)
]


def make_pose(x, y, yaw):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    # stamp left at 0 = "latest available transform" (avoids wall-time vs sim-time mismatch)
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def main():
    rclpy.init()
    nav = BasicNavigator(node_name='waypoint_patrol')

    nav.declare_parameter('waypoints', DEFAULT_WAYPOINTS)
    nav.declare_parameter('laps', 1)
    nav.declare_parameter('goal_timeout', 120.0)   # [s] per waypoint
    nav.declare_parameter('behavior_tree', '')     # optional custom BT XML for each goal
    nav.declare_parameter('results_dir', os.path.expanduser('~/ros2_ws/results'))

    flat = list(nav.get_parameter('waypoints').value)
    laps = int(nav.get_parameter('laps').value)
    goal_timeout = float(nav.get_parameter('goal_timeout').value)
    bt_xml = str(nav.get_parameter('behavior_tree').value)
    results_dir = str(nav.get_parameter('results_dir').value)

    if len(flat) % 3 != 0:
        nav.get_logger().error('waypoints must be a flat list of x, y, yaw triplets')
        return
    waypoints = [tuple(flat[i:i + 3]) for i in range(0, len(flat), 3)]

    # SLAM (slam_toolbox) is used instead of AMCL -> do not wait for an 'amcl' node
    # and do not set an initial pose.
    nav.waitUntilNav2Active(localizer='robot_localization')

    os.makedirs(results_dir, exist_ok=True)
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    csv_path = os.path.join(results_dir, f'patrol_{stamp}.csv')

    rows = []
    prev_xy = (0.0, 0.0)
    for lap in range(1, laps + 1):
        for idx, (x, y, yaw) in enumerate(waypoints):
            goal = make_pose(x, y, yaw)
            nav.get_logger().info(f'Lap {lap}/{laps}  waypoint {idx + 1}/{len(waypoints)} -> ({x:.2f}, {y:.2f})')
            t0 = time.monotonic()
            if not nav.goToPose(goal, behavior_tree=bt_xml):
                nav.get_logger().error('Goal was rejected')
                rows.append([lap, idx + 1, x, y, 'REJECTED', 0.0, 0.0, 0.0, 0])
                continue

            recoveries = 0
            sim_time = 0.0
            while not nav.isTaskComplete():
                fb = nav.getFeedback()
                if fb is not None:
                    recoveries = fb.number_of_recoveries
                    # navigation_time is measured by Nav2 in SIMULATION time
                    sim_time = fb.navigation_time.sec + fb.navigation_time.nanosec * 1e-9
                if time.monotonic() - t0 > goal_timeout:
                    nav.get_logger().warn('Timeout - cancelling goal')
                    nav.cancelTask()
                time.sleep(0.1)

            elapsed = time.monotonic() - t0
            result = nav.getResult()
            name = {TaskResult.SUCCEEDED: 'SUCCEEDED',
                    TaskResult.CANCELED: 'CANCELED',
                    TaskResult.FAILED: 'FAILED'}.get(result, 'UNKNOWN')
            dist = math.hypot(x - prev_xy[0], y - prev_xy[1])
            prev_xy = (x, y)
            nav.get_logger().info(f'  {name} in {sim_time:.1f} s sim ({elapsed:.1f} s wall), recoveries: {recoveries}')
            rows.append([lap, idx + 1, x, y, name, round(sim_time, 2), round(elapsed, 2), round(dist, 2), recoveries])

    with open(csv_path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['lap', 'waypoint', 'x', 'y', 'result', 'sim_time_s', 'wall_time_s', 'straight_dist_m', 'recoveries'])
        w.writerows(rows)

    ok = sum(1 for r in rows if r[4] == 'SUCCEEDED')
    sim_t = sum(r[5] for r in rows)
    wall_t = sum(r[6] for r in rows)
    rec = sum(r[8] for r in rows)
    nav.get_logger().info(f'Done: {ok}/{len(rows)} goals succeeded, total {sim_t:.1f} s sim '
                          f'({wall_t:.1f} s wall, RTF ~{sim_t / wall_t if wall_t else 0:.2f}), recoveries: {rec}')
    nav.get_logger().info(f'Results saved to {csv_path}')

    nav.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
