#!/usr/bin/env python

import pika
import json
import rospy
from threading import Thread


class HeartbeatPublisher:
    def __init__(self, config):
        self.config = config

    def publish(self, routing_key, message):
        connection = self.create_connection()
        # Create a new channel with the next available channel number or pass in a channel number to use

        channel = connection.channel()
        # Creates an exchange if it does not already exist, and if the exchange exists,

        # verifies that it is of the correct and expected class.
        channel.exchange_declare(
            exchange=self.config["exchange"],
            exchange_type=self.config["exchange_type"],
            durable=True,
        )
        
        # Publishes message to the exchange with the given routing key
        channel.basic_publish(
            exchange=self.config["exchange"],
            routing_key=routing_key,
            body=json.dumps(message),
        )
        print(" [x] Sent message %r for %r" % (message, routing_key))

    def create_connection(self):
        credentials = pika.PlainCredentials(
            self.config["user"], self.config["password"]
        )
        param = pika.ConnectionParameters(
            host=self.config["heartbeat_host"],
            port=self.config["heartbeat_port"],
            virtual_host=self.config["virtual_host"],
            credentials=credentials,
        )
        return pika.BlockingConnection(param)


# Create new connection
if __name__ == "__main__":
    rospy.init_node("HeartbeatPublisher", anonymous=True)
    host = rospy.get_param("~host", "localhost")
    port = rospy.get_param("~port", 5672)
    config = {
        "heartbeat_host": host,
        "heartbeat_port": port,
        "user": "myuser",
        "password": "mypassword",
        "virtual_host": "/",
        "exchange": "riotu.topic",
        "exchange_type": "topic",
    }
    publisher = HeartbeatPublisher(config)
    rate = rospy.Rate(1)  # 10hz

    def heartbeat_publisher():
        while not rospy.is_shutdown():
            publisher.publish("heartbeat.app.key", {"heartbeat": 1})
            rate.sleep()

    thr = Thread(name="heartbeat", target=heartbeat_publisher())
    thr.daemon = True
    thr.start()
