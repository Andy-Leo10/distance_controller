import threading
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Global variable to store the x position
x_pos = []

# This class is a ROS2 node that subscribes to the '/odometry/filtered' topic
class OdomSubscriber(Node):
    def __init__(self):
        # Initialize the node with the name 'odom_subscriber'
        super().__init__('odom_subscriber')
        # Create a subscription to the '/odometry/filtered' topic
        # The callback function is called whenever a new message is published on this topic
        self.subscription = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.callback,
            10)

    # This function is called whenever a new Odometry message is published on the '/odometry/filtered' topic
    def callback(self, msg):
        global x_pos
        # Append the x position from the Odometry message to the global list
        x_pos.append(msg.pose.pose.position.x)
        # Keep only the last 100 x positions
        x_pos = x_pos[-200:]
        # Log the x position
        # self.get_logger().info('x: %f' % msg.pose.pose.position.x)

# This function is called every 1000ms to update the plot
def animate(i):
    # Clear the previous plot
    ax1.clear()
    # Plot the x positions
    ax1.plot(x_pos)
    
    # Set the limits for Y-axis
    ax1.set_ylim(-1, 5)
    # add a grid to the plot
    ax1.grid(True)
    # Add a title to the plot
    ax1.set_title('Odometry visualizer')

def main(args=None):
    # Initialize the ROS2 library
    rclpy.init(args=args)
    # Create an instance of the OdomSubscriber node
    odom_subscriber = OdomSubscriber()
    # Spin the node so it can process callbacks
    rclpy.spin(odom_subscriber)
    # Destroy the node when it's done
    odom_subscriber.destroy_node()
    # Shutdown the ROS2 library
    rclpy.shutdown()

if __name__ == '__main__':
    # Create a figure for the plot
    fig = plt.figure()
    # Add a subplot to the figure
    ax1 = fig.add_subplot(1,1,1)
    # Set the limits for Y-axis
    ax1.set_ylim(-1, 5)
    
    # Create an animation that calls the 'animate' function every 1000ms
    ani = animation.FuncAnimation(fig, animate, interval=1000)

    # Start the ROS2 node in a separate thread
    thread = threading.Thread(target=main)
    thread.start()

    # Add a title to the plot
    ax1.set_title('Odometry visualizer')
    # Show the plot
    plt.show()