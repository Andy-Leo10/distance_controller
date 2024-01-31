#!/usr/bin/env python3
import threading
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time

# Global variables to store the x and y positions
x_pos = []
y_pos = []
time_list = []

class OdomSubscriber(Node):
    def __init__(self):
        super().__init__('odom_subscriber')
        self.subscription = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.callback,
            10)

    def callback(self, msg):
        global x_pos, y_pos, time_list
        x_pos.append(msg.pose.pose.position.x)
        y_pos.append(msg.pose.pose.position.y)
        time_list.append(time.time())  # store the current time
        x_pos = x_pos[-200:]
        y_pos = y_pos[-200:]
        time_list = time_list[-200:]

def animate(i):
    ax1.clear()
    ax1.plot(time_list, x_pos, label='x position')
    ax1.plot(time_list, y_pos, label='y position')
    ax1.set_ylim(-5, 5)
    ax1.grid(True)
    ax1.set_xlabel('Time')
    ax1.set_ylabel('Position')
    ax1.set_title('Odometry visualizer')
    ax1.legend()

def main(args=None):
    rclpy.init(args=args)
    odom_subscriber = OdomSubscriber()
    rclpy.spin(odom_subscriber)
    odom_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    fig = plt.figure()
    ax1 = fig.add_subplot(1,1,1)
    ax1.set_ylim(-1, 5)
    ani = animation.FuncAnimation(fig, animate, interval=1000)
    thread = threading.Thread(target=main)
    thread.start()
    ax1.set_title('Odometry visualizer')
    plt.show()