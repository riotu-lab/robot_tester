#!/usr/bin/env python

import pika
import threading
import json
import math

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped


class RabbitmqConsumer(threading.Thread):
    def __init__(self, host, port, *args, **kwargs):
        super(RabbitmqConsumer, self).__init__(*args, **kwargs)

        self._host = host
        self._port = port

        cmd_vel_topic = "/cmd_vel"
        self.velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
        position_topic = "/amcl_pose"
        self.pose_subscriber = rospy.Subscriber(
            position_topic, PoseWithCovarianceStamped, self.poseCallback
        )

    def poseCallback(self, pose_message):
        print("pose_message: ", pose_message)
        global x
        global y
        x = pose_message.pose.pose.position.x
        y = pose_message.pose.pose.position.y

    def move(self, speed, distance):
        # declare a Twist message to send velocity commands
        velocity_message = Twist()
        # get current location from the global variable before entering the loop
        x0 = x
        y0 = y
        # z0=z;
        # yaw0=yaw;
        velocity_message.linear.x = speed
        distance_moved = 0.0
        loop_rate = rospy.Rate(
            10
        )  # we publish the velocity at 10 Hz (10 times a second)

        while True:
            rospy.loginfo("Turtlesim moves forwards")
            self.velocity_publisher.publish(velocity_message)

            loop_rate.sleep()

            # rospy.Duration(1.0)

            distance_moved = distance_moved + abs(
                0.5 * math.sqrt(((x - x0) ** 2) + ((y - y0) ** 2))
            )
            print(distance_moved)
            if not (distance_moved < distance):
                rospy.loginfo("reached")
                break

        # finally, stop the robot when the distance is moved
        velocity_message.linear.x = 0
        self.velocity_publisher.publish(velocity_message)

    # Not necessarily a method.
    def callback_func(self, channel, method, properties, body):
        print("{} received '{}'".format(self.name, body))
        json_message = body.decode()
        json_message = json.loads(json_message)
        print("json_message: ", json_message)
        if "move.app.key" in method.routing_key:
            self.move(json_message["speed"], json_message["distance"])
            print("")

    def run(self):
        credentials = pika.PlainCredentials("myuser", "mypassword")

        connection = pika.BlockingConnection(
            pika.ConnectionParameters(
                host=self._host,
                port=self._port,
                virtual_host="/",
                credentials=credentials,
            )
        )

        channel = connection.channel()

        result = channel.queue_declare(
            queue="",
            exclusive=True,
        )

        channel.queue_bind(
            result.method.queue,
            exchange="riotu.topic",
            routing_key="move.app.key",
        )

        channel.basic_consume(
            on_message_callback=self.callback_func,
            queue=result.method.queue,
            auto_ack=False,
        )

        channel.start_consuming()


if __name__ == "__main__":
    rospy.init_node("RabbitmqConsumer", anonymous=True)
    host = rospy.get_param("~host", "localhost")
    port = rospy.get_param("~port", 5762)
    thread = RabbitmqConsumer(host=host, port=port)
    # thread.daemon = True
    thread.start()
