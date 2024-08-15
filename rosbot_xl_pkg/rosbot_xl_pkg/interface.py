import tkinter as tk
import threading
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node

class RobotControlNode(Node):
    def __init__(self):
        super().__init__('robot_control_node')
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.orientation_x = 0.0
        self.orientation_y = 0.0
        self.orientation_z = 0.0
        self.orientation_w = 1.0

        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.pose_callback,
            10)
        self.twist = Twist()

        # Timer to publish movement commands
        self.timer = self.create_timer(0.1, self.move_callback)

    def move_callback(self):
        self.publisher_.publish(self.twist)

    def pose_callback(self, msg):
        # Update pose information from the received message
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.position.z
        self.orientation_x = msg.pose.pose.orientation.x
        self.orientation_y = msg.pose.pose.orientation.y
        self.orientation_z = msg.pose.pose.orientation.z
        self.orientation_w = msg.pose.pose.orientation.w

    def get_position(self):
        return f"({self.x:.2f}, {self.y:.2f}, {self.z:.2f})"
    
    def get_orientation(self):
        return f"({self.orientation_x:.2f}, {self.orientation_y:.2f}, {self.orientation_z:.2f}, {self.orientation_w:.2f})"
    
    def move(self, linear_x, linear_y, angular):
        self.twist.linear.x = linear_x
        self.twist.linear.y = linear_y
        self.twist.angular.z = angular

    def stop(self):
        self.move(0.0, 0.0, 0.0)

class Application(tk.Frame):
    def __init__(self, master=None, robot_node=None):
        super().__init__(master)
        self.master = master
        self.robot_node = robot_node
        self.grid(sticky="nsew")
        self.createWidget()
        self.configure_grid_weights()
        self.update_ui_periodically()

    def createWidget(self):
        # Create a frame for the left controls
        self.left_frame = tk.Frame(self, borderwidth=2, relief="solid", width=300, height=400)
        self.left_frame.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")

        # Create a frame for the right controls
        self.right_frame = tk.Frame(self, borderwidth=2, relief="solid", width=300, height=400)
        self.right_frame.grid(row=0, column=1, padx=10, pady=10, sticky="nsew")

        # Ensure the frames maintain their size
        self.left_frame.grid_propagate(False)
        self.right_frame.grid_propagate(False)

        # Velocity slider with label on the left side
        self.velocity_label = tk.Label(self.left_frame, text="Velocity")
        self.velocity_label.grid(row=1, column=0, padx=5, pady=5)

        # Set the scale to go from 1 to 0 with a step of 0.1
        self.velocity_slider = tk.Scale(
            self.left_frame,
            from_=1,
            to=0,
            orient=tk.VERTICAL,
            resolution=0.1
        )
        self.velocity_slider.grid(row=1, column=1, padx=5, pady=5)

        # Turn buttons placed symmetrically with velocity
        self.button_turn_left = tk.Button(self.left_frame, text="↺", command=self.turn_left)
        self.button_turn_left.grid(row=1, column=2, padx=5, pady=5)

        self.button_turn_right = tk.Button(self.left_frame, text="↻", command=self.turn_right)
        self.button_turn_right.grid(row=1, column=3, padx=5, pady=5)

        # Direction buttons centered and grouped like arrow keys
        self.button_up = tk.Button(self.left_frame, text="↑", command=self.move_up)
        self.button_up.grid(row=4, column=2, padx=5, pady=5)

        self.button_down = tk.Button(self.left_frame, text="↓", command=self.move_down)
        self.button_down.grid(row=5, column=2, padx=5, pady=5)

        self.button_left = tk.Button(self.left_frame, text="←", command=self.move_left)
        self.button_left.grid(row=5, column=1, padx=5, pady=5)

        self.button_right = tk.Button(self.left_frame, text="→", command=self.move_right)
        self.button_right.grid(row=5, column=3, padx=5, pady=5)

        # Stop button in the left bottom corner
        self.button_stop = tk.Button(self.left_frame, text="Stop", bg="red", command=self.stop)
        self.button_stop.grid(row=7, column=0, padx=5, pady=5, sticky="w")

        # Right panel labels
        self.angular_velocity_label = tk.Label(self.right_frame, text="Angular Velocity:")
        self.angular_velocity_label.grid(row=0, column=0, padx=5, pady=5, sticky="e")
        self.angular_velocity_value = tk.Label(self.right_frame, text="0 rad/s", borderwidth=2, relief="solid")
        self.angular_velocity_value.grid(row=0, column=1, padx=5, pady=5, sticky="w")

        self.linear_velocity_label = tk.Label(self.right_frame, text="Linear Velocity:")
        self.linear_velocity_label.grid(row=1, column=0, padx=5, pady=5, sticky="e")
        self.linear_velocity_value = tk.Label(self.right_frame, text="(0,0) m/s", borderwidth=2, relief="solid")
        self.linear_velocity_value.grid(row=1, column=1, padx=5, pady=5, sticky="w")

        self.moving_direction_label = tk.Label(self.right_frame, text="Moving Direction:")
        self.moving_direction_label.grid(row=2, column=0, padx=5, pady=5, sticky="e")
        self.moving_direction_value = tk.Label(self.right_frame, text="Stopped", borderwidth=2, relief="solid")
        self.moving_direction_value.grid(row=2, column=1, padx=5, pady=5, sticky="w")

        self.current_state_label = tk.Label(self.right_frame, text="Current State:")
        self.current_state_label.grid(row=3, column=0, padx=5, pady=5, sticky="e")
        self.current_state_value = tk.Label(self.right_frame, text="On", borderwidth=2, relief="solid")
        self.current_state_value.grid(row=3, column=1, padx=5, pady=5, sticky="w")

        # Position label
        self.position_label = tk.Label(self.right_frame, text="Position:")
        self.position_label.grid(row=4, column=0, padx=5, pady=5, sticky="e")
        self.position_value = tk.Label(self.right_frame, text="(0, 0, 0)", borderwidth=2, relief="solid")
        self.position_value.grid(row=4, column=1, padx=5, pady=5, sticky="w")

        self.orientation_label = tk.Label(self.right_frame, text="Orientation:")
        self.orientation_label.grid(row=5, column=0, padx=5, pady=5, sticky="e")
        self.orientation_value = tk.Label(self.right_frame, text="(0, 0, 0, 1)", borderwidth=2, relief="solid")
        self.orientation_value.grid(row=5, column=1, padx=5, pady=5, sticky="w")

    def configure_grid_weights(self):
        self.master.rowconfigure(0, weight=1)
        self.master.columnconfigure(0, weight=1)
        self.master.columnconfigure(1, weight=1)
        self.rowconfigure(0, weight=1)
        self.columnconfigure(0, weight=1)
        self.columnconfigure(1, weight=1)

        # Adjust left_frame weights
        self.left_frame.rowconfigure(0, weight=1)
        self.left_frame.rowconfigure(1, weight=1)
        self.left_frame.rowconfigure(2, weight=1)
        self.left_frame.rowconfigure(3, weight=1)
        self.left_frame.rowconfigure(4, weight=1)
        self.left_frame.rowconfigure(5, weight=1)
        self.left_frame.rowconfigure(6, weight=1)
        self.left_frame.columnconfigure(0, weight=1)
        self.left_frame.columnconfigure(1, weight=1)
        self.left_frame.columnconfigure(2, weight=1)
        self.left_frame.columnconfigure(3, weight=1)

        # Adjust right_frame weights
        self.right_frame.rowconfigure(0, weight=1)
        self.right_frame.rowconfigure(1, weight=1)
        self.right_frame.rowconfigure(2, weight=1)
        self.right_frame.rowconfigure(3, weight=1)
        self.right_frame.rowconfigure(4, weight=1)
        self.right_frame.rowconfigure(5, weight=1)
        self.right_frame.columnconfigure(0, weight=1)
        self.right_frame.columnconfigure(1, weight=1)

    def move_up(self):
        linear_velocity_x = self.velocity_slider.get()
        linear_velocity_y = 0.0
        angular_velocity = 0.0
        self.robot_node.move(linear_velocity_x, linear_velocity_y, angular_velocity)
        self.update_display("forward", linear_velocity_x, linear_velocity_y, angular_velocity)

    def move_down(self):
        linear_velocity_x = -self.velocity_slider.get()
        linear_velocity_y = 0.0
        angular_velocity = 0.0
        self.robot_node.move(linear_velocity_x, linear_velocity_y, angular_velocity)
        self.update_display("backward", linear_velocity_x, linear_velocity_y, angular_velocity)

    def move_left(self):
        linear_velocity_x = 0.0
        linear_velocity_y = -self.velocity_slider.get()
        angular_velocity = 0.0
        self.robot_node.move(linear_velocity_x, linear_velocity_y, angular_velocity)
        self.update_display("left", linear_velocity_x, linear_velocity_y, angular_velocity)

    def move_right(self):
        linear_velocity_x = 0.0
        linear_velocity_y = self.velocity_slider.get()
        angular_velocity = 0.0
        self.robot_node.move(linear_velocity_x, linear_velocity_y, angular_velocity)
        self.update_display("right", linear_velocity_x, linear_velocity_y, angular_velocity)

    def turn_left(self):
        angular_velocity = 0.1  # Turning left
        self.robot_node.move(0.0, 0.0, angular_velocity)
        self.update_display("turn left", 0.0, 0.0, angular_velocity)

    def turn_right(self):
        angular_velocity = -0.1  # Turning right
        self.robot_node.move(0.0, 0.0, angular_velocity)
        self.update_display("turn right", 0.0, 0.0, angular_velocity)

    def stop(self):
        self.robot_node.stop()
        self.update_display("stopped", 0.0, 0.0, 0.0)

    def update_display(self, direction, linear_velocity_x, linear_velocity_y, angular_velocity):
        self.moving_direction_value.config(text=direction)
        self.linear_velocity_value.config(text=f"({linear_velocity_x:.2f},{linear_velocity_y:.2f}) m/s")
        self.angular_velocity_value.config(text=f"{angular_velocity:.2f} rad/s")

    def update_ui_periodically(self):
        # Update UI elements with latest data from robot node
        self.position_value.config(text=self.robot_node.get_position())
        self.orientation_value.config(text=self.robot_node.get_orientation())
        # Schedule this function to run again after 100 ms
        self.after(100, self.update_ui_periodically)

def main():
    rclpy.init()
    robot_node = RobotControlNode()
    root = tk.Tk()
    root.title("Control Panel")

    # Create a thread for the ROS 2 node spinning
    ros_thread = threading.Thread(target=rclpy.spin, args=(robot_node,), daemon=True)
    ros_thread.start()

    # Create the Tkinter application
    app = Application(master=root, robot_node=robot_node)

    # Start the Tkinter main loop
    root.mainloop()

    # Clean up ROS 2
    robot_node.destroy_node()
    rclpy.shutdown()

    # Wait for the ROS 2 thread to finish
    ros_thread.join()

if __name__ == "__main__":
    main()
