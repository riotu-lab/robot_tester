#!/usr/bin/env python

import pika
import json
import rospy
from threading import Thread


class HP:
    def __init__(self, config):
        self.config = config

    def p(self, routing_key, message):
        co = self.c_c()

        channel = co.channel()

        channel.exchange_declare(
            exchange=self.config["ex"],
            exchange_type=self.config["ex_t"],
            durable=True,
        )

        channel.basic_publish(
            exchange=self.config["ex"],
            routing_key=routing_key,
            body=json.dumps(message),
        )
        # print(" [x] Sent message %r for %r" % (message, routing_key))

    def c_c(self):
        credentials = pika.PlainCredentials(
            self.config["user"], self.config["password"]
        )
        param = pika.ConnectionParameters(
            host=self.config["host"],
            port=self.config["port"],
            virtual_host=self.config["virtual_host"],
            credentials=credentials,
        )
        return pika.BlockingConnection(param)


if __name__ == "__main__":
    rospy.init_node("HP", anonymous=True)
    host = rospy.get_param("~host", "localhost")
    port = rospy.get_param("~port", 5672)
    config = {
        "host": host,
        "port": port,
        "user": "myuser",
        "password": "mypassword",
        "virtual_host": "/",
        "ex": "riotu.topic",
        "ex_t": "topic",
    }
    pu = HP(config)
    rate = rospy.Rate(1)

    def h_p():
        while not rospy.is_shutdown():
            pu.p("heartbeat.app.key", {"heartbeat": 1})
            rate.sleep()

    thr = Thread(name="heartbeat", target=h_p())
    thr.daemon = True
    thr.start()
