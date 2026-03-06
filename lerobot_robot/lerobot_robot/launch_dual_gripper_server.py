import zerorpc
from polymetis import GripperInterface

class DualGripperServer:
    def __init__(self):
        self.gripper_enabled = True
        self._robot_wrapper = None  # 使用GripperInterface直接控制
        self._mock_left_gripper = {"width": 0.085, "is_grasped": False}
        self._mock_right_gripper = {"width": 0.085, "is_grasped": False}
        # 连接两个 gripper server
        try:
            self.gripper1 = GripperInterface(ip_address="localhost", port=50052)
            self.gripper2 = GripperInterface(ip_address="localhost", port=50053)
            print("成功连接到夹爪服务器")
        except Exception as e:
            print(f"连接夹爪服务器失败: {e}")
            self.gripper1 = None
            self.gripper2 = None

    def left_gripper_initialize(self):
        """Initialize left gripper."""
        if not self.gripper_enabled:
            return
        if self.gripper1 is None:
            self._mock_left_gripper = {"width": 0.085, "is_grasped": False}
            return
        # 尝试调用initialize方法（如果存在）
        try:
            self.gripper1.initialize()
        except AttributeError:
            pass

    def left_gripper_goto(
        self,
        width: float,
        speed: float,
        force: float,
        epsilon_inner: float = -1.0,
        epsilon_outer: float = -1.0,
        blocking: bool = True
    ):
        """Move left gripper to specified width."""
        if not self.gripper_enabled:
            return
        if self.gripper1 is None:
            self._mock_left_gripper["width"] = width
            return
        self.gripper1.goto(
            width=width,
            speed=speed,
            force=force,
            epsilon_inner=epsilon_inner,
            epsilon_outer=epsilon_outer,
            blocking=blocking
        )

    def left_gripper_grasp(
        self,
        speed: float,
        force: float,
        grasp_width: float = 0.0,
        epsilon_inner: float = -1.0,
        epsilon_outer: float = -1.0,
        blocking: bool = True,
    ):
        """Grasp with left gripper."""
        if not self.gripper_enabled:
            return
        if self.gripper1 is None:
            self._mock_left_gripper["is_grasped"] = True
            return
        self.gripper1.grasp(
            speed=speed,
            force=force,
            grasp_width=grasp_width,
            epsilon_inner=epsilon_inner,
            epsilon_outer=epsilon_outer,
            blocking=blocking
        )

    def left_gripper_get_state(self) -> dict:
        """Get left gripper state."""
        if not self.gripper_enabled:
            return {"width": 0.085, "is_moving": False, "is_grasped": False}
        if self.gripper1 is None:
            return self._mock_left_gripper
        return self.gripper1.get_state()

    def right_gripper_initialize(self):
        """Initialize right gripper."""
        if not self.gripper_enabled:
            return
        if self.gripper2 is None:
            self._mock_right_gripper = {"width": 0.085, "is_grasped": False}
            return
        # 尝试调用initialize方法（如果存在）
        try:
            self.gripper2.initialize()
        except AttributeError:
            pass

    def right_gripper_goto(
        self,
        width: float,
        speed: float,
        force: float,
        epsilon_inner: float = -1.0,
        epsilon_outer: float = -1.0,
        blocking: bool = True
    ):
        """Move right gripper to specified width."""
        if not self.gripper_enabled:
            return
        if self.gripper2 is None:
            self._mock_right_gripper["width"] = width
            return
        self.gripper2.goto(
            width=width,
            speed=speed,
            force=force,
            epsilon_inner=epsilon_inner,
            epsilon_outer=epsilon_outer,
            blocking=blocking
        )

    def right_gripper_grasp(
        self,
        speed: float,
        force: float,
        grasp_width: float = 0.0,
        epsilon_inner: float = -1.0,
        epsilon_outer: float = -1.0,
        blocking: bool = True,
    ):
        """Grasp with right gripper."""
        if not self.gripper_enabled:
            return
        if self.gripper2 is None:
            self._mock_right_gripper["is_grasped"] = True
            return
        self.gripper2.grasp(
            speed=speed,
            force=force,
            grasp_width=grasp_width,
            epsilon_inner=epsilon_inner,
            epsilon_outer=epsilon_outer,
            blocking=blocking
        )

    def right_gripper_get_state(self) -> dict:
        """Get right gripper state."""
        if not self.gripper_enabled:
            return {"width": 0.085, "is_moving": False, "is_grasped": False}
        if self.gripper2 is None:
            return self._mock_right_gripper
        return self.gripper2.get_state()

# 启动zerorpc服务器
if __name__ == "__main__":
    server = zerorpc.Server(DualGripperServer())
    server.bind("tcp://0.0.0.0:4243")
    print("双夹爪服务器已启动，监听端口4243")
    server.run()