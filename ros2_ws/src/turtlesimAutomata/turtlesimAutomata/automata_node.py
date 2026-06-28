"""
Turtlesim Automata Assignment.

This node controls multiple turtles in turtlesim. Each active turtle moves to
the furthest corner. Older turtles follow the next turtle in the chain.
When turtle6 is spawned, the program terminates and prints distances.

Author: Thomas Kleemayr
Assignment: Mobile Robotics - Turtlesim Automata
Date: 28.06.2026
"""

import math
import random

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from turtlesim.msg import Pose
from turtlesim.srv import SetPen
from turtlesim.srv import Spawn
from turtlesim.srv import TeleportAbsolute


class TurtlesimAutomata(Node):
    """Main controller for the turtlesim automata."""

    def __init__(self):
        """Initialize publishers, subscribers, service clients and state."""
        super().__init__("turtlesim_automata")

        self.world_min = 0.6
        self.world_max = 10.4
        self.max_turtles = 6

        self.poses = {}
        self.previous_poses = {}
        self.distance_x = {}
        self.distance_y = {}

        self.targets = {}
        self.turtles = ["turtle1"]
        self.active_index = 0

        self.turtle1_started = False
        self.finished = False

        self.pose_subscribers = []

        self.cmd_publishers = {
            "turtle1": self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        }

        self.pose_subscribers.append(
            self.create_subscription(
                Pose,
                "/turtle1/pose",
                lambda msg: self.pose_callback(msg, "turtle1"),
                10
            )
        )

        self.teleport_client = self.create_client(
            TeleportAbsolute,
            "/turtle1/teleport_absolute"
        )

        self.spawn_client = self.create_client(Spawn, "/spawn")

        self.pen_client = self.create_client(
            SetPen,
            "/turtle1/set_pen"
        )

        self.timer = self.create_timer(0.1, self.control_loop)

    def pose_callback(self, msg, turtle_name):
        """
        Store pose and update travelled x and y distances.

        Parameters
        ----------
        msg : turtlesim.msg.Pose
            Current turtle pose.
        turtle_name : str
            Name of the turtle.
        """
        if turtle_name not in self.distance_x:
            self.distance_x[turtle_name] = 0.0
            self.distance_y[turtle_name] = 0.0

        if turtle_name in self.previous_poses:
            old_pose = self.previous_poses[turtle_name]

            dx = abs(msg.x - old_pose.x)
            dy = abs(msg.y - old_pose.y)

            if dx < 1.0 and dy < 1.0:
                self.distance_x[turtle_name] += dx
                self.distance_y[turtle_name] += dy

        self.previous_poses[turtle_name] = msg
        self.poses[turtle_name] = msg

    def set_turtle1_pen(self, off):
        """
        Switch turtle1 pen on or off.

        Parameters
        ----------
        off : bool
            True disables drawing, False enables drawing.
        """
        if not self.pen_client.service_is_ready():
            return

        request = SetPen.Request()
        request.r = 255
        request.g = 255
        request.b = 255
        request.width = 2
        request.off = int(off)

        self.pen_client.call_async(request)

    def teleport_turtle1_randomly(self):
        """Teleport turtle1 to a random start position without drawing."""
        if not self.teleport_client.service_is_ready():
            return

        self.set_turtle1_pen(True)

        request = TeleportAbsolute.Request()
        request.x = random.uniform(self.world_min, self.world_max)
        request.y = random.uniform(self.world_min, self.world_max)
        request.theta = random.uniform(-math.pi, math.pi)

        self.teleport_client.call_async(request)

        self.get_logger().info(
            f"turtle1 spawned at x={request.x:.2f}, y={request.y:.2f}"
        )

        self.set_turtle1_pen(False)
        self.turtle1_started = True

    def spawn_turtle(self, turtle_name):
        """
        Spawn a new turtle at a random position.

        Parameters
        ----------
        turtle_name : str
            Name of the new turtle.
        """
        if not self.spawn_client.service_is_ready():
            return

        request = Spawn.Request()
        request.name = turtle_name
        request.x = random.uniform(self.world_min, self.world_max)
        request.y = random.uniform(self.world_min, self.world_max)
        request.theta = random.uniform(-math.pi, math.pi)

        self.spawn_client.call_async(request)

        self.cmd_publishers[turtle_name] = self.create_publisher(
            Twist,
            f"/{turtle_name}/cmd_vel",
            10
        )

        self.pose_subscribers.append(
            self.create_subscription(
                Pose,
                f"/{turtle_name}/pose",
                lambda msg, name=turtle_name: self.pose_callback(msg, name),
                10
            )
        )

        self.turtles.append(turtle_name)

        self.get_logger().info(
            f"{turtle_name} spawned at x={request.x:.2f}, y={request.y:.2f}"
        )

    def calculate_furthest_corner(self, turtle_name):
        """
        Calculate the furthest corner from the turtle position.

        Parameters
        ----------
        turtle_name : str
            Name of the turtle.

        Returns
        -------
        tuple
            Furthest corner as (x, y).
        """
        pose = self.poses[turtle_name]

        corners = [
            (self.world_min, self.world_min),
            (self.world_min, self.world_max),
            (self.world_max, self.world_min),
            (self.world_max, self.world_max)
        ]

        distances = []

        for corner_x, corner_y in corners:
            distance = math.sqrt(
                (corner_x - pose.x) ** 2 +
                (corner_y - pose.y) ** 2
            )
            distances.append(distance)

        max_distance = max(distances)

        best_corners = [
            corners[index]
            for index, distance in enumerate(distances)
            if abs(distance - max_distance) < 0.001
        ]

        return random.choice(best_corners)

    def drive_to_target(self, turtle_name):
        """
        Drive the active turtle to its target corner.

        Parameters
        ----------
        turtle_name : str
            Name of the active turtle.

        Returns
        -------
        bool
            True if the turtle reached its target.
        """
        pose = self.poses[turtle_name]
        target_x, target_y = self.targets[turtle_name]

        dx = target_x - pose.x
        dy = target_y - pose.y

        distance = math.sqrt(dx ** 2 + dy ** 2)
        target_angle = math.atan2(dy, dx)

        angle_error = target_angle - pose.theta
        angle_error = math.atan2(
            math.sin(angle_error),
            math.cos(angle_error)
        )

        msg = Twist()

        if distance < 0.15:
            self.cmd_publishers[turtle_name].publish(msg)
            return True

        msg.angular.z = 4.0 * angle_error

        if abs(angle_error) < 0.05:
            msg.angular.z = 0.0
            msg.linear.x = min(2.0, 2.0 * distance)

        self.cmd_publishers[turtle_name].publish(msg)
        return False

    def follow_turtle(self, follower_name, target_name):
        """
        Make one turtle follow another turtle.

        Parameters
        ----------
        follower_name : str
            Name of the following turtle.
        target_name : str
            Name of the turtle being followed.
        """
        if follower_name not in self.poses or target_name not in self.poses:
            return

        follower_pose = self.poses[follower_name]
        target_pose = self.poses[target_name]

        dx = target_pose.x - follower_pose.x
        dy = target_pose.y - follower_pose.y

        distance = math.sqrt(dx ** 2 + dy ** 2)
        target_angle = math.atan2(dy, dx)

        angle_error = target_angle - follower_pose.theta
        angle_error = math.atan2(
            math.sin(angle_error),
            math.cos(angle_error)
        )

        msg = Twist()
        msg.angular.z = 4.0 * angle_error

        if distance > 1.0 and abs(angle_error) < 0.5:
            msg.linear.x = min(2.0, distance)

        self.cmd_publishers[follower_name].publish(msg)

    def stop_all_turtles(self):
        """Stop all turtles."""
        for turtle_name in self.turtles:
            self.cmd_publishers[turtle_name].publish(Twist())

    def print_results(self):
        """Print the final x and y distance results."""
        print()
        print("The program has finished, The results are as follows:", flush=True)
        print()
        print("Turtle         x         y")
        print("----------------------------")

        for turtle_name in self.turtles:
            x_distance = self.distance_x.get(turtle_name, 0.0)
            y_distance = self.distance_y.get(turtle_name, 0.0)

            print(
                f"{turtle_name:<10} "
                f"{x_distance:>8.2f} "
                f"{y_distance:>8.2f}"
            )

    def control_loop(self):
        """Run the main state machine."""
        if self.finished:
            return

        if "turtle1" not in self.poses:
            return

        if not self.turtle1_started:
            self.teleport_turtle1_randomly()
            return

        active_turtle = self.turtles[self.active_index]

        if active_turtle not in self.poses:
            return

        if active_turtle not in self.targets:
            self.targets[active_turtle] = self.calculate_furthest_corner(
                active_turtle
            )
            self.get_logger().info("Furthest point detected!")
            return

        reached_target = self.drive_to_target(active_turtle)

        for index in range(self.active_index - 1, -1, -1):
            follower = self.turtles[index]
            target = self.turtles[index + 1]
            self.follow_turtle(follower, target)

        if not reached_target:
            return

        self.get_logger().info(
            f"{active_turtle} reached the wall point."
        )

        next_number = self.active_index + 2

        next_turtle = f"turtle{next_number}"
        self.spawn_turtle(next_turtle)

        if next_number == self.max_turtles:
            self.get_logger().info(
                "There are too many turtles... Program will terminate..."
            )
            self.stop_all_turtles()
            self.print_results()
            self.finished = True
            return

        self.active_index += 1


def main(args=None):
    """Start the turtlesim automata node."""
    rclpy.init(args=args)

    node = TurtlesimAutomata()

    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)

    except KeyboardInterrupt:
        pass

    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()