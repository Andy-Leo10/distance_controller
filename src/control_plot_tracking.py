import threading
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Global variables to store the x and y positions
x_pos = []
y_pos = []

class OdomSubscriber(Node):
    def __init__(self):
        super().__init__('odom_subscriber')
        self.subscription = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.callback,
            10)

    def callback(self, msg):
        global x_pos, y_pos
        x_pos.append(msg.pose.pose.position.x)
        y_pos.append(msg.pose.pose.position.y)
        x_pos = x_pos[-500:]
        y_pos = y_pos[-500:]

def animate(i):
    ax1.clear()
    ax1.plot(x_pos, y_pos)  # plot y against x
    ax1.set_xlim(-5, 5)
    ax1.set_ylim(-5, 5)
    ax1.grid(True)
    ax1.set_xlabel('x position')
    ax1.set_ylabel('y position')
    ax1.set_title('Tracking the robot position')

def main(args=None):
    rclpy.init(args=args)
    odom_subscriber = OdomSubscriber()
    rclpy.spin(odom_subscriber)
    odom_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    fig = plt.figure()
    ax1 = fig.add_subplot(1,1,1)
    ani = animation.FuncAnimation(fig, animate, interval=1000)
    thread = threading.Thread(target=main)
    thread.start()
    plt.show()