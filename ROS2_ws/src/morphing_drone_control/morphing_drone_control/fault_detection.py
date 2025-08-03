from std_msgs.msg import Int32

class FaultDetection:
    def __init__(self, node, state):
        self.state = state
        self.failnum = 0
        # node: rclpy.Node 인스턴스
        self._sub = node.create_subscription(
            Int32,
            '/motor_fail',
            self._fail_callback,
            10
        )

    def _fail_callback(self, msg: Int32):
        # 받는 즉시 최신값 저장
        self.failnum = msg.data
        # 상태(state)에도 동기화
        self.state.failnum = msg.data
