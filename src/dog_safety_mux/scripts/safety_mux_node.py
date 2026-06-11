#!/usr/bin/env python3
import threading

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String


def clamp(value, low, high):
    return max(low, min(high, value))


class SafetyMuxNode:
    """Simple manual/auto command multiplexer with rate limiting."""

    def __init__(self):
        self.lock = threading.Lock()

        self.manual_cmd_topic = rospy.get_param("~topics/manual_cmd", "/web_cmd_vel")
        self.nav_cmd_topic = rospy.get_param("~topics/nav_cmd", "/nav_cmd_vel")
        self.output_cmd_topic = rospy.get_param("~topics/output_cmd", "/cmd_vel")
        self.status_topic = rospy.get_param("~topics/status", "/dog_safety_mux/status")

        self.rate = float(rospy.get_param("~control/rate", 20.0))
        self.command_timeout = rospy.Duration(float(rospy.get_param("~control/command_timeout", 0.35)))
        self.max_linear_x = abs(float(rospy.get_param("~control/max_linear_x", 0.35)))
        self.max_linear_y = abs(float(rospy.get_param("~control/max_linear_y", 0.25)))
        self.max_angular_z = abs(float(rospy.get_param("~control/max_angular_z", 0.8)))

        self.manual_cmd = Twist()
        self.nav_cmd = Twist()
        self.last_manual_stamp = rospy.Time(0)
        self.last_nav_stamp = rospy.Time(0)
        self.last_status = "waiting"

        self.cmd_pub = rospy.Publisher(self.output_cmd_topic, Twist, queue_size=1)
        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=1)

        rospy.Subscriber(self.manual_cmd_topic, Twist, self.on_manual_cmd, queue_size=1)
        rospy.Subscriber(self.nav_cmd_topic, Twist, self.on_nav_cmd, queue_size=1)

        rospy.Timer(rospy.Duration(1.0 / max(1.0, self.rate)), self.on_timer)
        rospy.loginfo(
            "safety_mux started (simple mux). manual=%s nav=%s output=%s",
            self.manual_cmd_topic,
            self.nav_cmd_topic,
            self.output_cmd_topic,
        )

    def on_manual_cmd(self, msg):
        with self.lock:
            self.manual_cmd = msg
            self.last_manual_stamp = rospy.Time.now()

    def on_nav_cmd(self, msg):
        with self.lock:
            self.nav_cmd = msg
            self.last_nav_stamp = rospy.Time.now()

    def on_timer(self, _event):
        if rospy.is_shutdown():
            return

        now = rospy.Time.now()
        with self.lock:
            manual_cmd = self.copy_twist(self.manual_cmd)
            nav_cmd = self.copy_twist(self.nav_cmd)
            manual_age = now - self.last_manual_stamp
            nav_age = now - self.last_nav_stamp

        cmd, mode = self.select_command(manual_cmd, manual_age, nav_cmd, nav_age)
        cmd = self.clamp_twist(cmd)

        status = "%s: vx=%.2f vy=%.2f wz=%.2f" % (
            mode,
            cmd.linear.x,
            cmd.linear.y,
            cmd.angular.z,
        )

        try:
            self.cmd_pub.publish(cmd)
            if status != self.last_status:
                rospy.loginfo("safety_mux: %s", status)
                self.last_status = status
            self.status_pub.publish(String(data=status))
        except rospy.ROSException:
            if not rospy.is_shutdown():
                raise

    def select_command(self, manual_cmd, manual_age, nav_cmd, nav_age):
        if manual_age <= self.command_timeout and self.is_active(manual_cmd):
            return manual_cmd, "manual"
        if nav_age <= self.command_timeout and self.is_active(nav_cmd):
            return nav_cmd, "nav"
        return Twist(), "idle"

    def clamp_twist(self, cmd):
        out = Twist()
        out.linear.x = clamp(cmd.linear.x, -self.max_linear_x, self.max_linear_x)
        out.linear.y = clamp(cmd.linear.y, -self.max_linear_y, self.max_linear_y)
        out.angular.z = clamp(cmd.angular.z, -self.max_angular_z, self.max_angular_z)
        return out

    @staticmethod
    def is_active(cmd):
        return abs(cmd.linear.x) > 1e-4 or abs(cmd.linear.y) > 1e-4 or abs(cmd.angular.z) > 1e-4

    @staticmethod
    def copy_twist(cmd):
        out = Twist()
        out.linear.x = cmd.linear.x
        out.linear.y = cmd.linear.y
        out.linear.z = cmd.linear.z
        out.angular.x = cmd.angular.x
        out.angular.y = cmd.angular.y
        out.angular.z = cmd.angular.z
        return out


def main():
    rospy.init_node("dog_safety_mux")
    SafetyMuxNode()
    rospy.spin()


if __name__ == "__main__":
    main()