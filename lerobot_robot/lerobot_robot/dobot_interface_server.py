'''
Dobot Nova5 dual-arm robot interface server.
Provides zerorpc interface for dual-arm control.
'''

import zerorpc
import numpy as np
import logging

log = logging.getLogger(__name__)


class DobotDualArmServer:
    """Dual-arm Dobot Nova5 server interface."""
    
    def __init__(self, gripper_enabled: bool = True):
        self.gripper_enabled = gripper_enabled
        
        try:
            from dobot_ros_wrapper import DobotMoveitWrapper
            self._robot = DobotMoveitWrapper()
            log.info("[SERVER] Dobot wrapper initialized")
        except ImportError:
            log.warning("[SERVER] Wrapper not found, using mock mode")
            self._robot = None
            self._mock_left_joints = np.zeros(6)
            self._mock_right_joints = np.zeros(6)
            self._mock_left_pose = np.zeros(6)
            self._mock_right_pose = np.zeros(6)
        
        log.info("=" * 50)
        log.info("Dobot Dual-Arm Server Ready")
        log.info("=" * 50)
    
    # ==================== State Query ====================
    
    def left_robot_get_joint_positions(self) -> list:
        if self._robot is None:
            return self._mock_left_joints.tolist()
        return self._robot.left_get_joint_positions()
    
    def right_robot_get_joint_positions(self) -> list:
        if self._robot is None:
            return self._mock_right_joints.tolist()
        return self._robot.right_get_joint_positions()
    
    def left_robot_get_ee_pose(self) -> list:
        if self._robot is None:
            return self._mock_left_pose.tolist()
        return self._robot.left_get_ee_pose()
    
    def right_robot_get_ee_pose(self) -> list:
        if self._robot is None:
            return self._mock_right_pose.tolist()
        return self._robot.right_get_ee_pose()
    
    # ==================== MoveIt Motion ====================
    
    def left_robot_move_to_joint_positions(self, positions: list, delta: bool = False):
        if self._robot is None:
            if delta:
                self._mock_left_joints = np.array(self._mock_left_joints) + np.array(positions)
            else:
                self._mock_left_joints = np.array(positions)
            return
        self._robot.left_move_to_joint_positions(positions, delta=delta, wait=True)
    
    def right_robot_move_to_joint_positions(self, positions: list, delta: bool = False):
        if self._robot is None:
            if delta:
                self._mock_right_joints = np.array(self._mock_right_joints) + np.array(positions)
            else:
                self._mock_right_joints = np.array(positions)
            return
        self._robot.right_move_to_joint_positions(positions, delta=delta, wait=True)
    
    def left_robot_move_to_ee_pose(self, pose: list, delta: bool = False):
        if self._robot is None:
            if delta:
                self._mock_left_pose = np.array(self._mock_left_pose) + np.array(pose)
            else:
                self._mock_left_pose = np.array(pose)
            return
        self._robot.left_move_to_ee_pose(pose, delta=delta, wait=True)
    
    def right_robot_move_to_ee_pose(self, pose: list, delta: bool = False):
        if self._robot is None:
            if delta:
                self._mock_right_pose = np.array(self._mock_right_pose) + np.array(pose)
            else:
                self._mock_right_pose = np.array(pose)
            return
        self._robot.right_move_to_ee_pose(pose, delta=delta, wait=True)
    
    def dual_robot_move_to_ee_pose(self, left_pose: list, right_pose: list, delta: bool = False):
        if self._robot is None:
            return
        self._robot.dual_move_to_ee_pose(left_pose, right_pose, delta=delta, wait=True)
    
    # ==================== Go Home ====================
    
    def left_robot_go_home(self):
        if self._robot is None:
            self._mock_left_joints = np.zeros(6)
            return
        self._robot.left_go_home()
    
    def right_robot_go_home(self):
        if self._robot is None:
            self._mock_right_joints = np.zeros(6)
            return
        self._robot.right_go_home()
    
    def robot_go_home(self):
        """Move both arms to home position."""
        if self._robot is None:
            self._mock_left_joints = np.zeros(6)
            self._mock_right_joints = np.zeros(6)
            return
        self._robot.dual_go_home(wait=True)
    
    # ==================== ServoJ Control (Joint Servo) ====================
    
    def servo_j(self, arm_name: str, joints: list, t: float = 0.1, 
                lookahead_time: float = 0.05, gain: float = 300) -> bool:
        """
        Send ServoJ with ABSOLUTE joint angles (radians).
        Args:
            joints: Joint angles in RADIANS
        """
        if self._robot is None:
            return True
        # Convert radians to degrees for ROS wrapper
        joints_deg = np.degrees(joints).tolist()
        return self._robot.servo_j(arm_name, joints_deg, t, lookahead_time, gain)
    
    def servo_j_delta(self, arm_name: str, delta_joints: list, t: float = 0.1,
                      lookahead_time: float = 0.05, gain: float = 300) -> bool:
        """
        Send ServoJ with RELATIVE joint increments (radians).
        Args:
            delta_joints: Joint increments in RADIANS
        """
        if self._robot is None:
            return True
        # Convert radians to degrees for ROS wrapper
        delta_joints_deg = np.degrees(delta_joints).tolist()
        return self._robot.servo_j_delta(arm_name, delta_joints_deg, t, lookahead_time, gain)
    
    # ==================== ServoP Control (Pose Servo) ====================
    
    def servo_p(self, arm_name: str, pose: list) -> bool:
        """
        Send ServoP with target pose [x, y, z, rx, ry, rz] (m, radians).
        Args:
            pose: Target pose in METERS and RADIANS
        """
        if self._robot is None:
            return True
        # Convert m to mm, radians to degrees for ROS wrapper
        pose_array = np.array(pose)
        pose_dobot = np.concatenate([
            pose_array[:3] * 1000,  # m -> mm
            np.degrees(pose_array[3:])  # rad -> deg
        ]).tolist()
        return self._robot.servo_p(arm_name, pose_dobot)
    
    def servo_p_delta(self, arm_name: str, delta_pose: list) -> bool:
        """
        Send ServoP with RELATIVE pose increments (m, radians).
        Args:
            delta_pose: Pose increments in METERS and RADIANS
        """
        if self._robot is None:
            return True
        # Convert m to mm, radians to degrees for ROS wrapper
        delta_array = np.array(delta_pose)
        delta_dobot = np.concatenate([
            delta_array[:3] * 1000,  # m -> mm
            np.degrees(delta_array[3:])  # rad -> deg
        ]).tolist()
        return self._robot.servo_p_delta(arm_name, delta_dobot)
    
    # ==================== Inverse Kinematics ====================
    
    def inverse_kinematics(self, arm_name: str, pose: list, current_joints: list = None):
        """
        Solve IK using Dobot controller.
        Args:
            arm_name: 'left' or 'right'
            pose: Target pose [x, y, z, rx, ry, rz] (m, radians)
            current_joints: Current joints for reference (radians)
        Returns:
            Joint angles (radians) or None if failed
        """
        if self._robot is None:
            return [0.0] * 6
        
        # ROS wrapper expects mm and degrees for IK
        pose_array = np.array(pose)
        pose_dobot = np.concatenate([
            pose_array[:3] * 1000,  # m -> mm
            np.degrees(pose_array[3:])  # rad -> deg
        ]).tolist()
        
        # Convert current joints to degrees
        current_joints_deg = None
        if current_joints is not None:
            current_joints_deg = np.degrees(current_joints).tolist()
        
        # Solve IK (ROS wrapper returns degrees)
        result_deg = self._robot.inverse_kinematics(arm_name, pose_dobot, current_joints_deg)
        
        # Convert back to radians
        if result_deg is not None:
            return np.radians(result_deg).tolist()
        return None
    
    # ==================== Gripper (Placeholder) ====================
    
    def left_gripper_initialize(self): pass
    def left_gripper_goto(self, *args, **kwargs): pass
    def left_gripper_grasp(self, *args, **kwargs): pass
    def left_gripper_get_state(self): return {"width": 0.04, "is_moving": False, "is_grasped": False}
    
    def right_gripper_initialize(self): pass
    def right_gripper_goto(self, *args, **kwargs): pass
    def right_gripper_grasp(self, *args, **kwargs): pass
    def right_gripper_get_state(self): return {"width": 0.04, "is_moving": False, "is_grasped": False}
    
    def gripper_initialize(self):
        self.left_gripper_initialize()
        self.right_gripper_initialize()
    
    # ==================== Utility ====================
    
    def stop(self, arm_name: str):
        if self._robot is None:
            return
        self._robot.stop(arm_name)


def start_server(port: int = 4242, gripper_enabled: bool = True):
    server = zerorpc.Server(DobotDualArmServer())
    server.bind(f"tcp://0.0.0.0:{port}")
    log.info(f"[SERVER] Listening on port {port}")
    server.run()


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=4242)
    parser.add_argument('--no-gripper', action='store_true')
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(message)s')
    start_server(port=args.port, gripper_enabled=not args.no_gripper)
