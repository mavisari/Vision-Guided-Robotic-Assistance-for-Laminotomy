#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import numpy as np
import csv
import time
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.srv import GetCartesianPath
from visualization_msgs.msg import Marker

class CsvIkNodeFinal(Node):
    def __init__(self):
        super().__init__('csv_ik_node_final')

        self.planning_group = 'arm'
        self.ee_link = 'lbr_link_ee'
        self.base_frame = 'lbr_link_0'
        self.safe_height = 0.10  
        self.pause_time = 2.0     

        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 10)
        self.move_action = ActionClient(self, MoveGroup, '/lbr/move_action')
        self.cartesian_client = self.create_client(GetCartesianPath, '/lbr/compute_cartesian_path')
        self.execute_client = ActionClient(self, ExecuteTrajectory, '/lbr/execute_trajectory')

        csv_path = "/home/beatr/lbr-stack/src/lbr_utils/data/captures.csv"
        with open(csv_path, newline='') as csvfile:
            self.rows = list(csv.DictReader(csvfile))

        idx = int(input(f"Scegli la posa (1-{len(self.rows)}): ")) - 1
        row = self.rows[idx]

        self.px, self.py, self.pz = float(row["px"]), float(row["py"]), float(row["pz"])
        self.qx, self.qy, self.qz, self.qw = float(row["qx"]), float(row["qy"]), float(row["qz"]), float(row["qw"])
        
        T_base_ee = self.pose_to_T(self.px, self.py, self.pz, self.qx, self.qy, self.qz, self.qw)
        T_cam_to_ee = np.array([[0.99997, -0.00314, -0.00725, 0.00858], [0.00607, 0.89345, 0.44913, -0.44913], [0.01713, -0.44960, 0.89344, 0.02955], [0.0, 0.0, 0.0, 1.0]])
        p_base = (T_base_ee @ T_cam_to_ee @ np.array([float(row["Xf"]), float(row["Yf"]), float(row["Zf"]), 1.0]))[:3]
        self.target_x, self.target_y, self.target_z = p_base

        self.cartesian_client.wait_for_service()
        self.execute_client.wait_for_server()
        
        self.publish_marker()
        self.execute_mission()

    def pose_to_T(self, px, py, pz, qx, qy, qz, qw):
        T = np.eye(4)
        q = np.array([qx, qy, qz, qw])
        q /= np.linalg.norm(q)
        qx, qy, qz, qw = q
        T[:3, :3] = np.array([
            [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
            [2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qx*qw)],
            [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx**2 + qy**2)]
        ])
        T[:3, 3] = [px, py, pz]
        return T

    def create_pose(self, x, y, z):
        p = Pose()
        p.position.x, p.position.y, p.position.z = x, y, z
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = self.qx, self.qy, self.qz, self.qw
        return p

    def execute_mission(self):
        # 1. POSA INIZIALE
        self.get_logger().info("Fase 1: Posa iniziale")
        self.send_standard_goal(self.create_pose(self.px, self.py, self.pz))
        time.sleep(self.pause_time)

        # 2. MOVIMENTO XY
        self.get_logger().info("Fase 2: Spostamento XY")
        self.execute_cartesian_path([self.create_pose(self.target_x, self.target_y, self.pz)], slowdown=1.0)
        time.sleep(self.pause_time)

        # 3. DISCESA VERTICALE 
        self.get_logger().info("Fase 3: Discesa")
        # Slowdown moderato con molti punti intermedi
        self.execute_cartesian_path([self.create_pose(self.target_x, self.target_y, self.target_z + self.safe_height)], slowdown=2.5)

    def send_standard_goal(self, pose):
        goal = MoveGroup.Goal()
        goal.request.group_name = self.planning_group
        goal.request.max_velocity_scaling_factor = 0.15
        from moveit_msgs.msg import Constraints, PositionConstraint
        from shape_msgs.msg import SolidPrimitive
        c = Constraints(); pc = PositionConstraint()
        pc.header.frame_id = self.base_frame; pc.link_name = self.ee_link
        pc.constraint_region.primitive_poses.append(pose)
        pc.constraint_region.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.001, 0.001, 0.001]))
        c.position_constraints.append(pc)
        goal.request.goal_constraints.append(c)
        self.move_action.send_goal_async(goal)

    def execute_cartesian_path(self, waypoints, slowdown=1.0):
        req = GetCartesianPath.Request()
        req.header.frame_id = self.base_frame
        req.group_name = self.planning_group
        req.link_name = self.ee_link
        req.waypoints = waypoints
        # PASSO MOLTO PICCOLO PER LA FLUIDITÀ
        req.max_step = 0.002 # 2 millimetri tra i punti (più risoluzione)
        req.jump_threshold = 0.0 
        
        future = self.cartesian_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        
        if res and res.fraction > 0.5:
            traj = res.solution
            # Dilatazione temporale
            for point in traj.joint_trajectory.points:
                point.time_from_start.sec = int(point.time_from_start.sec * slowdown)
                point.time_from_start.nanosec = int(point.time_from_start.nanosec * slowdown)

            goal = ExecuteTrajectory.Goal()
            goal.trajectory = traj
            send_future = self.execute_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, send_future)
            result_future = send_future.result().get_result_async()
            rclpy.spin_until_future_complete(self, result_future)

    def publish_marker(self):
        now = self.get_clock().now().to_msg()
        # Centroide GIALLO
        m_s = Marker()
        m_s.header.frame_id = self.base_frame; m_s.header.stamp = now
        m_s.ns = "centroid"; m_s.id = 0; m_s.type = Marker.SPHERE; m_s.action = Marker.ADD
        m_s.scale.x = m_s.scale.y = m_s.scale.z = 0.05
        m_s.color.r, m_s.color.g, m_s.color.b, m_s.color.a = 1.0, 1.0, 0.0, 1.0
        m_s.pose.position.x, m_s.pose.position.y, m_s.pose.position.z = self.target_x, self.target_y, self.target_z
        self.marker_pub.publish(m_s)
        # Box ROSSA
        m_b = Marker()
        m_b.header.frame_id = self.base_frame; m_b.header.stamp = now
        m_b.ns = "centroid"; m_b.id = 1; m_b.type = Marker.CUBE; m_b.action = Marker.ADD
        m_b.scale.x, m_b.scale.y, m_b.scale.z = 0.10, 0.30, 0.07
        m_b.color.r, m_b.color.g, m_b.color.b, m_b.color.a = 1.0, 0.0, 0.0, 1.0
        m_b.pose.position.x, m_b.pose.position.y = self.target_x, self.target_y
        m_b.pose.position.z = self.target_z - 0.02
        m_b.pose.orientation.w = 1.0
        self.marker_pub.publish(m_b)

def main():
    rclpy.init()
    node = CsvIkNodeFinal()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()