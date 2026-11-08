"""ROS 2 node: /cmd_vel in, 18 position targets on /joint_commands out, at a fixed rate. It reads no ground truth."""
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger

from hexapod_controller.core import Config, Controller, State
from hexapod_kinematics import RobotModel, VelocityCommand


class ControllerNode(Node):
    def __init__(self):
        super().__init__('hexapod_controller')
        defaults = Config()
        for name in defaults.__dataclass_fields__:
            self.declare_parameter(name, getattr(defaults, name))
        self.declare_parameter('autostart', True)
        values = {f: self.get_parameter(f).value for f in defaults.__dataclass_fields__}
        self.core = Controller(RobotModel.load(), Config(**values))
        self.autostart = self.get_parameter('autostart').value
        self.command_pub = self.create_publisher(Float64MultiArray, '/joint_commands', 10)
        self.state_pub = self.create_publisher(String, '/controller_state', 10)
        self.safety_pub = self.create_publisher(String, '/controller_safety', 10)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel, 10)
        self.create_service(Trigger, '/hexapod/stand', self.on_stand)
        self.create_service(Trigger, '/hexapod/sit', self.on_sit)
        self.create_service(Trigger, '/hexapod/clear_fault', self.on_clear)
        self.last_state = None
        self.create_timer(self.core.dt, self.tick)
        self.get_logger().info(f'{self.core.params.gait} gait, up to {self.core.gait.max_speed() * 1000:.0f} mm/s')

    def on_cmd_vel(self, msg):
        self.core.set_velocity(VelocityCommand(msg.linear.x, msg.linear.y, msg.angular.z))

    def on_stand(self, _request, response):
        response.success = self.core.request_stand()
        response.message = self.core.state.value
        return response

    def on_sit(self, _request, response):
        response.success = self.core.request_sit()
        response.message = self.core.state.value
        return response

    def on_clear(self, _request, response):
        self.core.clear_fault()
        response.success = True
        response.message = 'cleared'
        return response

    def tick(self):
        if self.core.state == State.LAYDOWN and self.autostart and self.command_pub.get_subscription_count() > 0:
            self.autostart = False
            self.core.request_stand()
        out = self.core.step()
        self.command_pub.publish(Float64MultiArray(data=[float(v) for v in out.q.ravel()]))
        label = out.state.value + (' (fault: walking stopped)' if out.fault else '')
        self.state_pub.publish(String(data=label))
        if label != self.last_state:
            self.get_logger().info(f'state: {label}')
            self.last_state = label
        for issue in out.issues:
            self.safety_pub.publish(String(data=f'{issue.kind}: {issue.message}'))
            self.get_logger().warn(f'{issue.kind}: {issue.message}', throttle_duration_sec=1.0)


def main():
    rclpy.init()
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
