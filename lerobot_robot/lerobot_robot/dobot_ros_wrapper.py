import numpy as np
import logging

log = logging.getLogger(__name__)

class DobotMoveitWrapper:
    def __init__(self):
        import sys
        import rospy
        import moveit_commander
        
        rospy.loginfo("Initializing MoveIt Commander...")
        if not rospy.core.is_initialized():
            moveit_commander.roscpp_initialize(sys.argv)
            rospy.init_node('dual_arm_zerorpc_server_backend', anonymous=True, disable_signals=True)
            
        self.left_arm = moveit_commander.MoveGroupCommander('left_arm')
        self.right_arm = moveit_commander.MoveGroupCommander('right_arm')
        rospy.loginfo("Successfully connected to MoveIt Core.")
        
    def _pose_to_list(self, pose):
        import tf.transformations as tf
        euler = tf.euler_from_quaternion([
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
        ])
        return [pose.position.x, pose.position.y, pose.position.z, euler[0], euler[1], euler[2]]

    def left_get_joint_positions(self):
        return self.left_arm.get_current_joint_values()
        
    def right_get_joint_positions(self):
        return self.right_arm.get_current_joint_values()
        
    def left_get_joint_velocities(self):
        return [0.0] * 6
        
    def right_get_joint_velocities(self):
        return [0.0] * 6
        
    def left_get_ee_pose(self):
        return self._pose_to_list(self.left_arm.get_current_pose().pose)
        
    def right_get_ee_pose(self):
        return self._pose_to_list(self.right_arm.get_current_pose().pose)
        
    def left_move_to_joint_positions(self, positions, time_to_go=None, delta=False, Kq=None, Kqd=None):
        if delta:
            curr = self.left_arm.get_current_joint_values()
            positions = [c + p for c, p in zip(curr, positions)]
        self.left_arm.go(positions, wait=True)
        self.left_arm.stop()
        
    def right_move_to_joint_positions(self, positions, time_to_go=None, delta=False, Kq=None, Kqd=None):
        if delta:
            curr = self.right_arm.get_current_joint_values()
            positions = [c + p for c, p in zip(curr, positions)]
        self.right_arm.go(positions, wait=True)
        self.right_arm.stop()

    def _list_to_pose_target(self, arm, pose_list, delta=False):
        import tf.transformations as tf
        from geometry_msgs.msg import Pose
        if delta:
            curr = self._pose_to_list(arm.get_current_pose().pose)
            pose_list = [c + p for c, p in zip(curr, pose_list)]
            
        p = Pose()
        p.position.x, p.position.y, p.position.z = pose_list[0], pose_list[1], pose_list[2]
        q = tf.quaternion_from_euler(pose_list[3], pose_list[4], pose_list[5])
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = q[0], q[1], q[2], q[3]
        arm.set_pose_target(p)
        arm.go(wait=True)
        arm.stop()
        arm.clear_pose_targets()

    def left_move_to_ee_pose(self, pose_list, time_to_go=None, delta=False, Kx=None, Kxd=None, op_space_interp=True):
        self._list_to_pose_target(self.left_arm, pose_list, delta)
        
    def right_move_to_ee_pose(self, pose_list, time_to_go=None, delta=False, Kx=None, Kxd=None, op_space_interp=True):
        self._list_to_pose_target(self.right_arm, pose_list, delta)

    def left_go_home(self):
        """Move left arm to default home pose."""
        import geometry_msgs.msg
        pose = geometry_msgs.msg.Pose()
        pose.position.x = -0.5
        pose.position.y = 1.0
        pose.position.z = 1.3
        pose.orientation.x = 0.0
        pose.orientation.y = 1.0
        pose.orientation.z = 0.0
        pose.orientation.w = 0.0
        self.left_arm.set_pose_target(pose)
        self.left_arm.go(wait=True)
        self.left_arm.stop()
        self.left_arm.clear_pose_targets()

    def right_go_home(self):
        """Move right arm to default home pose."""
        import geometry_msgs.msg
        pose = geometry_msgs.msg.Pose()
        pose.position.x = 0.5
        pose.position.y = 1.0
        pose.position.z = 1.3
        pose.orientation.x = 0.0
        pose.orientation.y = 1.0
        pose.orientation.z = 0.0
        pose.orientation.w = 0.0
        self.right_arm.set_pose_target(pose)
        self.right_arm.go(wait=True)
        self.right_arm.stop()
        self.right_arm.clear_pose_targets()

    def left_gripper_initialize(self): pass
    def left_gripper_goto(self, *args, **kwargs): pass
    def left_gripper_grasp(self, *args, **kwargs): pass
    def left_gripper_get_state(self): return {"width": 0.04, "is_moving": False, "is_grasped": False}

    def right_gripper_initialize(self): pass
    def right_gripper_goto(self, *args, **kwargs): pass
    def right_gripper_grasp(self, *args, **kwargs): pass
    def right_gripper_get_state(self): return {"width": 0.04, "is_moving": False, "is_grasped": False}

    # ----- New Methods from MoveIt Reference Script -----
    def get_current_pose(self, arm_name: str):
        arm = self.left_arm if arm_name == 'left_arm' else self.right_arm
        pose = arm.get_current_pose().pose
        return {
            'x': pose.position.x, 'y': pose.position.y, 'z': pose.position.z,
            'qx': pose.orientation.x, 'qy': pose.orientation.y, 
            'qz': pose.orientation.z, 'qw': pose.orientation.w
        }

    def go_to_pose(self, arm_name: str, x: float, y: float, z: float, qx: float, qy: float, qz: float, qw: float, wait=True):
        arm = self.left_arm if arm_name == 'left_arm' else self.right_arm
        arm.set_pose_target([x, y, z, qx, qy, qz, qw])
        result = arm.go(wait=wait)
        arm.stop()
        arm.clear_pose_targets()
        return result

    def sync_go_to_pose(self, left_pose: dict, right_pose: dict):
        import actionlib
        from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
        import rospy
        
        self.left_arm.set_pose_target([left_pose['x'], left_pose['y'], left_pose['z'], 
                                       left_pose['qx'], left_pose['qy'], left_pose['qz'], left_pose['qw']])
        self.right_arm.set_pose_target([right_pose['x'], right_pose['y'], right_pose['z'], 
                                        right_pose['qx'], right_pose['qy'], right_pose['qz'], right_pose['qw']])

        left_plan_tuple = self.left_arm.plan()
        right_plan_tuple = self.right_arm.plan()

        left_plan = left_plan_tuple[1] if type(left_plan_tuple) is tuple else left_plan_tuple
        right_plan = right_plan_tuple[1] if type(right_plan_tuple) is tuple else right_plan_tuple

        left_client = actionlib.SimpleActionClient('/left_nova5_robot/joint_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        right_client = actionlib.SimpleActionClient('/right_nova5_robot/joint_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        
        l_conn = left_client.wait_for_server(rospy.Duration(5.0))
        r_conn = right_client.wait_for_server(rospy.Duration(5.0))

        if not (l_conn and r_conn): return False

        left_goal = FollowJointTrajectoryGoal(trajectory=left_plan.joint_trajectory)
        right_goal = FollowJointTrajectoryGoal(trajectory=right_plan.joint_trajectory)

        left_client.send_goal(left_goal)
        right_client.send_goal(right_goal)

        left_client.wait_for_result()
        right_client.wait_for_result()
        
        self.left_arm.stop()
        self.right_arm.stop()
        self.left_arm.clear_pose_targets()
        self.right_arm.clear_pose_targets()
        return True

    def stop(self, arm_name: str):
        arm = self.left_arm if arm_name == 'left_arm' else self.right_arm
        arm.stop()
        arm.clear_pose_targets()
        
