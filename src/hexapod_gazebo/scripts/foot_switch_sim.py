#!/usr/bin/env python3
"""Turn raw Gazebo contacts into what the real microswitches report: /foot_contact/leg_N, std_msgs/Bool."""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ros_gz_interfaces.msg import Contacts
from std_msgs.msg import Bool

from hexapod_gazebo.foot_switch import FootSwitch, contact_force
from hexapod_kinematics import RobotModel


class FootSwitchSim(Node):
    def __init__(self):
        super().__init__('foot_switch_sim')
        model = RobotModel.load()
        c = model.raw['foot_contact']
        self.rate = float(c['update_rate_hz'])
        self.switches = {}
        self.pubs = {}
        self.last_force = {}
        self.last_seen = {}
        for leg in model.legs:
            self.switches[leg.id] = FootSwitch(c['sim_force_threshold'], c['sim_release_force'], c['sim_delay'])
            self.pubs[leg.id] = self.create_publisher(Bool, c['topic'].format(id=leg.id), 10)
            self.last_force[leg.id] = 0.0
            self.last_seen[leg.id] = -1.0
            self.create_subscription(Contacts, c['raw_topic'].format(id=leg.id), lambda m, i=leg.id: self.on_contact(i, m), qos_profile_sensor_data)
        self.create_timer(1.0 / self.rate, self.tick)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_contact(self, leg, msg):
        self.last_force[leg] = contact_force(msg)
        self.last_seen[leg] = self.now()

    def tick(self):
        t = self.now()
        for leg, switch in self.switches.items():
            # Gazebo only publishes while touching, so silence means no force.
            force = self.last_force[leg] if t - self.last_seen[leg] < 3.0 / self.rate else 0.0
            self.pubs[leg].publish(Bool(data=switch.update(t, force)))


def main():
    rclpy.init()
    rclpy.spin(FootSwitchSim())


if __name__ == '__main__':
    main()
