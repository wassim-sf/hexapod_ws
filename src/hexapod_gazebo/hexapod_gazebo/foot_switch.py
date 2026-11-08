"""The simulated microswitch: force threshold, release hysteresis and reporting delay."""
import math
from collections import deque


def contact_force(msg):
    """Largest contact force magnitude in a ros_gz_interfaces/Contacts message, in newtons."""
    best = 0.0
    for contact in msg.contacts:
        total = [0.0, 0.0, 0.0]
        for wrench in contact.wrenches:
            f = wrench.body_1_wrench.force
            total[0] += f.x
            total[1] += f.y
            total[2] += f.z
        best = max(best, math.sqrt(sum(v * v for v in total)))
    return best


class FootSwitch:
    def __init__(self, threshold, release, delay):
        if not 0.0 <= release <= threshold:
            raise ValueError('release force must be between 0 and the threshold')
        self.threshold = threshold
        self.release = release
        self.delay = delay
        self.pressed = False
        self.queue = deque()
        self.output = False

    def update(self, t, force):
        """Feed the current force at time t and return the reported state."""
        if self.pressed and force < self.release:
            self.pressed = False
        elif not self.pressed and force >= self.threshold:
            self.pressed = True
        self.queue.append((t, self.pressed))
        while self.queue and t - self.queue[0][0] >= self.delay:
            self.output = self.queue.popleft()[1]
        return self.output
