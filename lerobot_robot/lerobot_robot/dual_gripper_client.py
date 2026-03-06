import logging
import zerorpc
from typing import Dict, Any

log = logging.getLogger(__name__)


class DualGripperClient:
    """
    Client for dual gripper system.
    Connects to a zerorpc server that controls both grippers.
    """
    
    def __init__(self, ip: str = '127.0.0.1', port: int = 4243):
        """
        Initialize dual gripper client.
        
        Args:
            ip: Server IP address
            port: Server port
        """
        self.ip = ip
        self.port = port
        
        log.info("=" * 60)
        log.info("Initializing Dual Gripper Client")
        log.info("=" * 60)
        
        try:
            self.server = zerorpc.Client(heartbeat=20)
            self.server.connect(f"tcp://{ip}:{port}")
            log.info(f"[DUAL-GRIPPER] Connected to server at {ip}:{port}")
        except Exception as e:
            log.error(f"[DUAL-GRIPPER] Failed to connect to server: {e}")
            self.server = None
        
        log.info("=" * 60)
        log.info("Dual Gripper Client Initialized")
        log.info("=" * 60)

    # ==================== Left Gripper Interface ====================
    
    def left_gripper_initialize(self):
        """Initialize left gripper."""
        if self.server is None:
            log.error("[LEFT GRIPPER] Not connected to server")
            return
        try:
            self.server.left_gripper_initialize()
            log.info("[LEFT GRIPPER] Gripper initialized")
        except Exception as e:
            log.error(f"[LEFT GRIPPER] Failed to initialize gripper: {e}")

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
        if self.server is None:
            return
        try:
            self.server.left_gripper_goto(width, speed, force, epsilon_inner, epsilon_outer, blocking)
        except Exception as e:
            log.error(f"[LEFT GRIPPER] gripper_goto failed: {e}")

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
        if self.server is None:
            return
        try:
            self.server.left_gripper_grasp(
                speed,
                force,
                grasp_width,
                epsilon_inner,
                epsilon_outer,
                blocking,
            )
        except Exception as e:
            log.error(f"[LEFT GRIPPER] gripper_grasp failed: {e}")

    def left_gripper_get_state(self) -> dict:
        """Get left gripper state."""
        if self.server is None:
            return {"width": 0.085, "is_moving": False, "is_grasped": False}
        try:
            return self.server.left_gripper_get_state()
        except Exception as e:
            log.error(f"[LEFT GRIPPER] gripper_get_state failed: {e}")
            return {"width": 0.085, "is_moving": False, "is_grasped": False}

    # ==================== Right Gripper Interface ====================
    
    def right_gripper_initialize(self):
        """Initialize right gripper."""
        if self.server is None:
            log.error("[RIGHT GRIPPER] Not connected to server")
            return
        try:
            self.server.right_gripper_initialize()
            log.info("[RIGHT GRIPPER] Gripper initialized")
        except Exception as e:
            log.error(f"[RIGHT GRIPPER] Failed to initialize gripper: {e}")

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
        if self.server is None:
            return
        try:
            self.server.right_gripper_goto(width, speed, force, epsilon_inner, epsilon_outer, blocking)
        except Exception as e:
            log.error(f"[RIGHT GRIPPER] gripper_goto failed: {e}")

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
        if self.server is None:
            return
        try:
            self.server.right_gripper_grasp(
                speed,
                force,
                grasp_width,
                epsilon_inner,
                epsilon_outer,
                blocking,
            )
        except Exception as e:
            log.error(f"[RIGHT GRIPPER] gripper_grasp failed: {e}")

    def right_gripper_get_state(self) -> dict:
        """Get right gripper state."""
        if self.server is None:
            return {"width": 0.085, "is_moving": False, "is_grasped": False}
        try:
            return self.server.right_gripper_get_state()
        except Exception as e:
            log.error(f"[RIGHT GRIPPER] gripper_get_state failed: {e}")
            return {"width": 0.085, "is_moving": False, "is_grasped": False}

    # ==================== Dual-Gripper Convenience Methods ====================
    
    def gripper_initialize(self):
        """Initialize both grippers."""
        self.left_gripper_initialize()
        self.right_gripper_initialize()

    def close(self):
        """Close connection to server."""
        if self.server is not None:
            try:
                self.server.close()
            except:
                pass


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    
    # Test dual gripper client
    client = DualGripperClient(ip="127.0.0.1", port=4243)
    
    # Initialize grippers
    client.gripper_initialize()
    
    # Test left gripper
    client.left_gripper_goto(width=0.04, speed=0.1, force=10.0)
    left_gripper_state = client.left_gripper_get_state()
    print(f"Left gripper state: {left_gripper_state}")
    
    # Test right gripper
    client.right_gripper_goto(width=0.04, speed=0.1, force=10.0)
    right_gripper_state = client.right_gripper_get_state()
    print(f"Right gripper state: {right_gripper_state}")