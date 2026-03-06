import time
import logging
import numpy as np
from dobot_interface_client import DobotDualArmClient
from dual_gripper_client import DualGripperClient

logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")
log = logging.getLogger(__name__)

def test_dobot_interface():
    log.info("开始进行 Dobot 接口全面测试...")
    
    # 连接机械臂客户端
    robot_client = DobotDualArmClient(ip="127.0.0.1", port=4242)
    # 连接夹爪客户端
    gripper_client = DualGripperClient(ip="127.0.0.1", port=4243)
    
    try:
        # 1. 初始化夹爪
        log.info("\n--- 1. 初始化夹爪 ---")
        gripper_client.gripper_initialize()
        time.sleep(1)
        
        # 2. 获取初始状态
        log.info("\n--- 2. 获取初始状态 ---")
        l_joints = robot_client.left_robot_get_joint_positions()
        r_joints = robot_client.right_robot_get_joint_positions()
        l_ee = robot_client.left_robot_get_ee_pose()
        r_ee = robot_client.right_robot_get_ee_pose()
        
        log.info(f"左臂当前关节位置: {l_joints}")
        log.info(f"右臂当前关节位置: {r_joints}")
        log.info(f"左臂当前 EE 位姿 [x,y,z, rx,ry,rz]: {l_ee}")
        log.info(f"右臂当前 EE 位姿 [x,y,z, rx,ry,rz]: {r_ee}")
        time.sleep(1)

        # 3. 测试连贯动作序列
        log.info("\n--- 3. 执行连贯动作序列 ---")
        
        # 3.1 左臂向下移动，准备抓取
        log.info("3.1 左臂向下移动，准备抓取...")
        if l_ee is not None and len(l_ee) == 6:
            target_pose = l_ee.copy()
            target_pose[2] -= 0.1  # 向下移动 0.1m
            robot_client.left_robot_move_to_ee_pose(target_pose, time_to_go=2.0)
        time.sleep(2)
        
        # 3.2 左臂夹爪打开
        log.info("3.2 左臂夹爪打开...")
        gripper_client.left_gripper_goto(width=0.085, speed=0.1, force=10.0, blocking=True)
        time.sleep(1)
        
        # 3.3 右臂向下移动，准备抓取
        log.info("3.3 右臂向下移动，准备抓取...")
        if r_ee is not None and len(r_ee) == 6:
            target_pose = r_ee.copy()
            target_pose[2] -= 0.1  # 向下移动 0.1m
            robot_client.right_robot_move_to_ee_pose(target_pose, time_to_go=2.0)
        time.sleep(2)
        
        # 3.4 右臂夹爪打开
        log.info("3.4 右臂夹爪打开...")
        gripper_client.right_gripper_goto(width=0.085, speed=0.1, force=10.0, blocking=True)
        time.sleep(1)
        
        # 3.5 双臂夹爪闭合（模拟抓取）
        log.info("3.5 双臂夹爪闭合（模拟抓取）...")
        gripper_client.left_gripper_goto(width=0.04, speed=0.1, force=10.0, blocking=True)
        gripper_client.right_gripper_goto(width=0.04, speed=0.1, force=10.0, blocking=True)
        time.sleep(1)
        
        # 3.6 双臂向上移动
        log.info("3.6 双臂向上移动...")
        if l_ee is not None and len(l_ee) == 6:
            target_pose = l_ee.copy()
            target_pose[2] += 0.1  # 向上移动 0.1m
            robot_client.left_robot_move_to_ee_pose(target_pose, time_to_go=2.0)
        if r_ee is not None and len(r_ee) == 6:
            target_pose = r_ee.copy()
            target_pose[2] += 0.1  # 向上移动 0.1m
            robot_client.right_robot_move_to_ee_pose(target_pose, time_to_go=2.0)
        time.sleep(2)
        
        # 3.7 双臂向外移动
        log.info("3.7 双臂向外移动...")
        if l_ee is not None and len(l_ee) == 6:
            target_pose = l_ee.copy()
            target_pose[0] -= 0.1  # 向左移动 0.1m
            robot_client.left_robot_move_to_ee_pose(target_pose, time_to_go=2.0)
        if r_ee is not None and len(r_ee) == 6:
            target_pose = r_ee.copy()
            target_pose[0] += 0.1  # 向右移动 0.1m
            robot_client.right_robot_move_to_ee_pose(target_pose, time_to_go=2.0)
        time.sleep(2)
        
        # 3.8 双臂夹爪打开
        log.info("3.8 双臂夹爪打开...")
        gripper_client.left_gripper_goto(width=0.085, speed=0.1, force=10.0, blocking=True)
        gripper_client.right_gripper_goto(width=0.085, speed=0.1, force=10.0, blocking=True)
        time.sleep(1)
        
        # 3.9 双臂回到初始位置
        log.info("3.9 双臂回到初始位置...")
        if l_ee is not None and len(l_ee) == 6:
            robot_client.left_robot_move_to_ee_pose(l_ee, time_to_go=2.0)
        if r_ee is not None and len(r_ee) == 6:
            robot_client.right_robot_move_to_ee_pose(r_ee, time_to_go=2.0)
        time.sleep(2)

        # 4. 测试抓取功能
        log.info("\n--- 4. 测试抓取功能 ---")
        log.info("执行左臂抓取动作...")
        gripper_client.left_gripper_grasp(speed=0.1, force=10.0, grasp_width=0.01)
        time.sleep(1)
        log.info(f"左夹爪状态: {gripper_client.left_gripper_get_state()}")
        
        log.info("执行右臂抓取动作...")
        gripper_client.right_gripper_grasp(speed=0.1, force=10.0, grasp_width=0.01)
        time.sleep(1)
        log.info(f"右夹爪状态: {gripper_client.right_gripper_get_state()}")

        # 5. 返回原点
        log.info("\n--- 5. 返回原点 ---")
        log.info("发送回原点 (Home) 命令...")
        robot_client.robot_go_home()
        time.sleep(3)
        
        # 6. 最终状态检查
        log.info("\n--- 6. 最终状态检查 ---")
        l_joints_final = robot_client.left_robot_get_joint_positions()
        r_joints_final = robot_client.right_robot_get_joint_positions()
        l_ee_final = robot_client.left_robot_get_ee_pose()
        r_ee_final = robot_client.right_robot_get_ee_pose()
        
        log.info(f"左臂最终关节位置: {l_joints_final}")
        log.info(f"右臂最终关节位置: {r_joints_final}")
        log.info(f"左臂最终 EE 位姿: {l_ee_final}")
        log.info(f"右臂最终 EE 位姿: {r_ee_final}")
        
        # 7. 打开夹爪
        log.info("\n--- 7. 打开夹爪 ---")
        gripper_client.left_gripper_goto(width=0.085, speed=0.1, force=10.0, blocking=True)
        gripper_client.right_gripper_goto(width=0.085, speed=0.1, force=10.0, blocking=True)
        time.sleep(1)
        
    finally:
        # 关闭连接
        log.info("\n--- 8. 关闭连接 ---")
        robot_client.close()
        gripper_client.close()
        log.info("全部测试完成!")

if __name__ == "__main__":
    test_dobot_interface()