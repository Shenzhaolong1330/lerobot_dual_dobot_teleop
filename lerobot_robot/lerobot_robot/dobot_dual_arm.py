"""
Dobot Nova5 dual-arm robot implementation.
Each arm has 6 DOF with Robotiq 2F-85 gripper as end effector.
Uses Oculus Quest for teleoperation control.
"""

import logging
import time
import threading
from pathlib import Path
from typing import Any, Dict, Optional
import numpy as np

from lerobot.cameras import make_cameras_from_configs
from lerobot.utils.errors import DeviceNotConnectedError, DeviceAlreadyConnectedError
from lerobot.robots.robot import Robot

from .config_dobot import DobotDualArmConfig
from .dobot_interface_client import DobotDualArmClient
from .dual_gripper_client import DualGripperClient

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


class DobotDualArm(Robot):
    """
    Dual-arm Dobot Nova5 robot with two Robotiq 2F-85 grippers.
    Each arm has 6 DOF, total 12 DOF.
    """
    
    config_class = DobotDualArmConfig
    name = "dobot_dual_arm"
    
    def __init__(self, config: DobotDualArmConfig):
        super().__init__(config)
        self.cameras = make_cameras_from_configs(config.cameras)
        
        self.config = config
        self._is_connected = False
        self._robot: Optional[DobotDualArmClient] = None
        self._gripper_client: Optional[DualGripperClient] = None
        self._prev_observation = None
        self._num_joints_per_arm = 6
        
        # Gripper settings
        self._gripper_force = config.gripper_force
        self._gripper_speed = config.gripper_speed
        self._left_gripper_position = 1.0
        self._right_gripper_position = 1.0
        self._last_left_gripper_position = 1.0
        self._last_right_gripper_position = 1.0
        
        # Action smoothing
        self._smoothing_alpha = 0.4
        self._left_smoothed_delta = None
        self._right_smoothed_delta = None
    
    def connect(self) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self.name} is already connected.")
        
        logger.info("\n" + "=" * 60)
        logger.info("[ROBOT] Connecting to Dobot Nova5 Dual-Arm System")
        logger.info("=" * 60)
        
        # Connect to dual-arm server (single port)
        self._robot = self._check_dobot_connection()
        
        # Connect to gripper server
        self._gripper_client = self._check_gripper_connection()
        
        # Initialize grippers
        if self.config.use_gripper:
            self._check_grippers_connection()
        
        # Connect cameras
        logger.info("\n===== [CAM] Initializing Cameras =====")
        for cam_name, cam in self.cameras.items():
            cam.connect()
            logger.info(f"[CAM] {cam_name} connected successfully.")
        logger.info("===== [CAM] Cameras Initialized Successfully =====\n")
        
        self.is_connected = True
        logger.info(f"[INFO] {self.name} initialization completed successfully.\n")
    
    def _check_dobot_connection(self) -> DobotDualArmClient:
        """Connect to Dobot dual-arm server via zerorpc (single port)."""
        try:
            logger.info("\n===== [ROBOT] Connecting to Dobot dual-arm =====")
            
            robot = DobotDualArmClient(
                ip='127.0.0.1',
                port=self.config.robot_port
            )
            
            # Get joint positions for both arms
            left_joints = robot.left_robot_get_joint_positions()
            right_joints = robot.right_robot_get_joint_positions()
            
            if left_joints is not None and len(left_joints) == self._num_joints_per_arm:
                logger.info(f"[LEFT ARM] Joint positions: {[round(j, 4) for j in left_joints]}")
            if right_joints is not None and len(right_joints) == self._num_joints_per_arm:
                logger.info(f"[RIGHT ARM] Joint positions: {[round(j, 4) for j in right_joints]}")
            
            logger.info("===== [ROBOT] Dobot dual-arm connected successfully =====\n")
            return robot
            
        except Exception as e:
            logger.error("===== [ERROR] Failed to connect to Dobot dual-arm =====")
            logger.error(f"Exception: {e}\n")
            raise
    
    def _check_gripper_connection(self) -> DualGripperClient:
        """Connect to dual gripper server via zerorpc."""
        try:
            logger.info("\n===== [GRIPPER] Connecting to dual gripper server =====")
            
            gripper_client = DualGripperClient(
                ip='127.0.0.1',
                port=4243
            )
            
            logger.info("===== [GRIPPER] Dual gripper server connected successfully =====\n")
            return gripper_client
            
        except Exception as e:
            logger.error("===== [ERROR] Failed to connect to dual gripper server =====")
            logger.error(f"Exception: {e}\n")
            return DualGripperClient()

    def _check_grippers_connection(self):
        """Initialize both Robotiq 2F-85 grippers."""
        logger.info("\n===== [GRIPPER] Initializing Robotiq 2F-85 grippers...")
        
        # Initialize left gripper
        self._gripper_client.left_gripper_initialize()
        self._gripper_client.left_gripper_goto(
            width=self.config.gripper_max_open,
            speed=self._gripper_speed,
            force=self._gripper_force,
            blocking=True
        )
        logger.info("[LEFT GRIPPER] Initialized successfully")
        
        # Initialize right gripper
        self._gripper_client.right_gripper_initialize()
        self._gripper_client.right_gripper_goto(
            width=self.config.gripper_max_open,
            speed=self._gripper_speed,
            force=self._gripper_force,
            blocking=True
        )
        logger.info("[RIGHT GRIPPER] Initialized successfully")
        logger.info("===== [GRIPPER] Both grippers initialized successfully =====\n")
    
    def reset(self) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self.name} is not connected.")
        
        logger.info("[ROBOT] Resetting dual-arm system...")
        self._robot.robot_go_home()
        
        if self.config.use_gripper:
            self._gripper_client.left_gripper_goto(
                width=self.config.gripper_max_open,
                speed=self._gripper_speed,
                force=self._gripper_force,
                blocking=True
            )
            self._gripper_client.right_gripper_goto(
                width=self.config.gripper_max_open,
                speed=self._gripper_speed,
                force=self._gripper_force,
                blocking=True
            )
        
        logger.info("===== [ROBOT] Dual-arm system reset successfully =====\n")
    
    @property
    def _motors_ft(self) -> dict[str, type]:
        """Motor features for dual-arm system."""
        features = {}
        
        # Left arm joint positions and velocities
        for i in range(self._num_joints_per_arm):
            features[f"left_joint_{i+1}.pos"] = float
            features[f"left_joint_{i+1}.vel"] = float
        
        # Right arm joint positions and velocities
        for i in range(self._num_joints_per_arm):
            features[f"right_joint_{i+1}.pos"] = float
            features[f"right_joint_{i+1}.vel"] = float
        
        # Left arm end effector pose
        for axis in ["x", "y", "z", "rx", "ry", "rz"]:
            features[f"left_ee_pose.{axis}"] = float
        
        # Right arm end effector pose
        for axis in ["x", "y", "z", "rx", "ry", "rz"]:
            features[f"right_ee_pose.{axis}"] = float
        
        # Gripper states
        if self.config.use_gripper:
            features["left_gripper_state_norm"] = float
            features["left_gripper_cmd_bin"] = float
            features["right_gripper_state_norm"] = float
            features["right_gripper_cmd_bin"] = float
        
        return features
    
    @property
    def action_features(self) -> dict[str, type]:
        """Return action features based on control mode."""
        if self.config.control_mode == "isoteleop":
            features = {}
            # Left arm joints
            for i in range(self._num_joints_per_arm):
                features[f"left_joint_{i+1}.pos"] = float
            # Right arm joints
            for i in range(self._num_joints_per_arm):
                features[f"right_joint_{i+1}.pos"] = float
            if self.config.use_gripper:
                features["left_gripper_position"] = float
                features["right_gripper_position"] = float
            return features
        elif self.config.control_mode == "oculus":
            features = {}
            # Left arm delta pose
            for axis in ["x", "y", "z", "rx", "ry", "rz"]:
                features[f"left_delta_ee_pose.{axis}"] = float
            # Right arm delta pose
            for axis in ["x", "y", "z", "rx", "ry", "rz"]:
                features[f"right_delta_ee_pose.{axis}"] = float
            if self.config.use_gripper:
                features["left_gripper_cmd_bin"] = float
                features["right_gripper_cmd_bin"] = float
            return features
        else:
            raise ValueError(f"Unsupported control mode: {self.config.control_mode}")
    
    def _handle_gripper(self, arm_side: str, gripper_value: float, is_binary: bool = True) -> None:
        """Handle gripper control for specified arm."""
        if not self.config.use_gripper:
            return
        
        last_pos_attr = f"_last_{arm_side}_gripper_position"
        gripper_pos_attr = f"_{arm_side}_gripper_position"
        
        if is_binary:
            gripper_position = gripper_value
        else:
            gripper_position = 0.0 if gripper_value < self.config.close_threshold else 1.0
        
        if self.config.gripper_reverse:
            gripper_position = 1 - gripper_position
        
        try:
            last_pos = getattr(self, last_pos_attr)
            if gripper_position != last_pos:
                if arm_side == "left":
                    self._gripper_client.left_gripper_goto(
                        width=gripper_position * self.config.gripper_max_open,
                        speed=self._gripper_speed,
                        force=self._gripper_force,
                    )
                else:
                    self._gripper_client.right_gripper_goto(
                        width=gripper_position * self.config.gripper_max_open,
                        speed=self._gripper_speed,
                        force=self._gripper_force,
                    )
                setattr(self, last_pos_attr, gripper_position)
            
            if arm_side == "left":
                gripper_state = self._gripper_client.left_gripper_get_state()
            else:
                gripper_state = self._gripper_client.right_gripper_get_state()
            
            gripper_state_norm = max(0.0, min(1.0, gripper_state["width"] / self.config.gripper_max_open))
            if self.config.gripper_reverse:
                gripper_state_norm = 1 - gripper_state_norm
            setattr(self, gripper_pos_attr, gripper_state_norm)
        except Exception as e:
            logger.warning(f"[{arm_side.upper()} GRIPPER] zerorpc error: {e}")
    
    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        
        if self.config.control_mode == "isoteleop":
            self._send_action_isoteleop(action)
        elif self.config.control_mode == "oculus":
            self._send_action_cartesian(action)
        else:
            raise ValueError(f"Unsupported control mode: {self.config.control_mode}")
        
        return action
    
    def _send_action_isoteleop(self, action: dict[str, Any]) -> None:
        """Send action in isoteleop mode (joint positions)."""
        # Left arm
        left_target_joints = np.array([
            action[f"left_joint_{i+1}.pos"] for i in range(self._num_joints_per_arm)
        ])
        # Right arm
        right_target_joints = np.array([
            action[f"right_joint_{i+1}.pos"] for i in range(self._num_joints_per_arm)
        ])
        
        if not self.config.debug:
            try:
                left_joints = self._robot.left_robot_get_joint_positions()
                right_joints = self._robot.right_robot_get_joint_positions()
                
                left_max_delta = (np.abs(left_joints - left_target_joints)).max()
                right_max_delta = (np.abs(right_joints - right_target_joints)).max()
                
                if left_max_delta > self.config.max_joint_delta or right_max_delta > self.config.max_joint_delta:
                    logger.warning("MOVING TOO FAST! SLOW DOWN!")
                
                self._robot.left_robot_update_desired_joint_positions(left_target_joints)
                self._robot.right_robot_update_desired_joint_positions(right_target_joints)
                
            except Exception as e:
                logger.warning(f"[ROBOT] isoteleop action failed: {e}")
        
        if "left_gripper_position" in action:
            self._handle_gripper("left", action["left_gripper_position"], is_binary=False)
        if "right_gripper_position" in action:
            self._handle_gripper("right", action["right_gripper_position"], is_binary=False)
    
    def _send_action_cartesian(self, action: dict[str, Any]) -> None:
        """Send action in oculus mode (delta ee pose)."""
        # Check for reset request
        if action.get("reset_requested", False):
            logger.info("[ROBOT] Reset requested for dual-arm system...")
            self._robot.robot_go_home()
            if self.config.use_gripper:
                self._gripper_client.left_gripper_goto(
                    width=self.config.gripper_max_open,
                    speed=self._gripper_speed,
                    force=self._gripper_force,
                    blocking=True
                )
                self._gripper_client.right_gripper_goto(
                    width=self.config.gripper_max_open,
                    speed=self._gripper_speed,
                    force=self._gripper_force,
                    blocking=True
                )
            return
        
        # Get delta poses for both arms
        left_delta = np.array([
            action[f"left_delta_ee_pose.{axis}"] for axis in ["x", "y", "z", "rx", "ry", "rz"]
        ])
        right_delta = np.array([
            action[f"right_delta_ee_pose.{axis}"] for axis in ["x", "y", "z", "rx", "ry", "rz"]
        ])
        
        # EMA smoothing for both arms
        for arm_side, delta, smoothed_attr in [
            ("left", left_delta, "_left_smoothed_delta"),
            ("right", right_delta, "_right_smoothed_delta")
        ]:
            if np.linalg.norm(delta) < 1e-6:
                setattr(self, smoothed_attr, None)
            else:
                smoothed = getattr(self, smoothed_attr)
                if smoothed is None:
                    smoothed = delta.copy()
                else:
                    alpha = self._smoothing_alpha
                    smoothed = alpha * delta + (1 - alpha) * smoothed
                setattr(self, smoothed_attr, smoothed)
        
        if not self.config.debug:
            import scipy.spatial.transform as st
            
            for arm_side, delta, smoothed_attr in [
                ("left", left_delta, "_left_smoothed_delta"),
                ("right", right_delta, "_right_smoothed_delta")
            ]:
                smoothed = getattr(self, smoothed_attr)
                
                if smoothed is None or np.linalg.norm(smoothed) < 0.01:
                    continue
                
                try:
                    if arm_side == "left":
                        ee_pose = self._robot.left_robot_get_ee_pose()
                    else:
                        ee_pose = self._robot.right_robot_get_ee_pose()
                    
                    target_position = ee_pose[:3] + smoothed[:3]
                    current_rot = st.Rotation.from_rotvec(ee_pose[3:])
                    delta_rot = st.Rotation.from_rotvec(smoothed[3:])
                    target_rotation = delta_rot * current_rot
                    target_rotvec = target_rotation.as_rotvec()
                    target_ee_pose = np.concatenate([target_position, target_rotvec])
                    
                    if arm_side == "left":
                        self._robot.left_robot_update_desired_ee_pose(target_ee_pose)
                    else:
                        self._robot.right_robot_update_desired_ee_pose(target_ee_pose)
                except Exception as e:
                    logger.warning(f"[{arm_side.upper()} ARM] zerorpc error: {e}")
        
        if "left_gripper_cmd_bin" in action:
            self._handle_gripper("left", action["left_gripper_cmd_bin"], is_binary=True)
        if "right_gripper_cmd_bin" in action:
            self._handle_gripper("right", action["right_gripper_cmd_bin"], is_binary=True)
    
    def get_observation(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        
        try:
            left_joint_pos = self._robot.left_robot_get_joint_positions()
            left_joint_vel = self._robot.left_robot_get_joint_velocities()
            left_ee_pose = self._robot.left_robot_get_ee_pose()
            
            right_joint_pos = self._robot.right_robot_get_joint_positions()
            right_joint_vel = self._robot.right_robot_get_joint_velocities()
            right_ee_pose = self._robot.right_robot_get_ee_pose()
            
        except Exception as e:
            logger.warning(f"[ROBOT] zerorpc error in get_observation: {e}")
            if self._prev_observation is not None:
                return self._prev_observation
            else:
                raise
        
        obs_dict = {}
        
        # Left arm observations
        for i in range(len(left_joint_pos)):
            obs_dict[f"left_joint_{i+1}.pos"] = float(left_joint_pos[i])
            obs_dict[f"left_joint_{i+1}.vel"] = float(left_joint_vel[i])
        for i, axis in enumerate(["x", "y", "z", "rx", "ry", "rz"]):
            obs_dict[f"left_ee_pose.{axis}"] = float(left_ee_pose[i])
        
        # Right arm observations
        for i in range(len(right_joint_pos)):
            obs_dict[f"right_joint_{i+1}.pos"] = float(right_joint_pos[i])
            obs_dict[f"right_joint_{i+1}.vel"] = float(right_joint_vel[i])
        for i, axis in enumerate(["x", "y", "z", "rx", "ry", "rz"]):
            obs_dict[f"right_ee_pose.{axis}"] = float(right_ee_pose[i])
        
        # Gripper states
        if self.config.use_gripper:
            obs_dict["left_gripper_state_norm"] = self._left_gripper_position
            obs_dict["left_gripper_cmd_bin"] = self._last_left_gripper_position
            obs_dict["right_gripper_state_norm"] = self._right_gripper_position
            obs_dict["right_gripper_cmd_bin"] = self._last_right_gripper_position
        else:
            obs_dict["left_gripper_state_norm"] = None
            obs_dict["left_gripper_cmd_bin"] = None
            obs_dict["right_gripper_state_norm"] = None
            obs_dict["right_gripper_cmd_bin"] = None
        
        # Camera images
        for cam_key, cam in self.cameras.items():
            start = time.perf_counter()
            obs_dict[cam_key] = cam.read()
            dt_ms = (time.perf_counter() - start) * 1e3
            logger.debug(f"{self} read {cam_key}: {dt_ms:.1f}ms")
        
        self._prev_observation = obs_dict
        return obs_dict
    
    def disconnect(self) -> None:
        if not self.is_connected:
            return
        
        for cam in self.cameras.values():
            cam.disconnect()
        
        if self._robot is not None:
            self._robot.close()
        
        if self._gripper_client is not None:
            self._gripper_client.close()
        
        self.is_connected = False
        logger.info(f"[INFO] ===== {self.name} disconnected =====")
    
    def calibrate(self) -> None:
        pass
    
    def is_calibrated(self) -> bool:
        return self.is_connected
    
    def configure(self) -> None:
        pass
    
    @property
    def is_connected(self) -> bool:
        return self._is_connected
    
    @is_connected.setter
    def is_connected(self, value: bool) -> None:
        self._is_connected = value
    
    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        return {
            cam: (self.cameras[cam].height, self.cameras[cam].width, 3) 
            for cam in self.cameras
        }
    
    @property
    def observation_features(self) -> dict[str, Any]:
        return {**self._motors_ft, **self._cameras_ft}