from geometry_msgs.msg import Twist

def go_to_goal(self, distance, angle):
        cmd_vel_msg = Twist()

        goal_distance = 200.0  # desired distance to the ball in mm
        goal_angle = 0.0  # desired angle in degrees

        k_linear = 0.001  # proportional constant for linear velocity
        k_angular = 0.01  # proportional constant for angular velocity

        error_distance = distance - goal_distance
        error_angle = angle - goal_angle

        linear_speed = k_linear * error_distance
        angular_speed = k_angular * error_angle

        cmd_vel_msg.linear.x = linear_speed
        cmd_vel_msg.angular.z = angular_speed

        twist = Twist()

        # Distance threshold and angle threshold to stop the robot
        distance_threshold = 250.0  # in mm
        angle_threshold = 4.0  # in degrees

        if abs(self.angle_degrees) > angle_threshold:
            # Rotate to align with the ball
            twist.angular.z = -150.0 if self.angle_degrees < 0 else 150.0  # Adjust rotation speed as needed
        else:
            twist.angular.z = 0.0  # Go straight if within angle threshold
            if self.ball_distance > distance_threshold:
                # Move forward
                twist.linear.x = 200.0  # Adjust speed as needed

            else:
                twist.linear.x = 0.0  # Stop if within distance threshold
                twist.angular.z = 0.0  # Stop rotating
        self.cmd_vel_publisher_.publish(twist)

        if self.ball_distance < 250 and self.angle_degrees < 5:
            # Robot is considered to have reached the goal if the error is within thresholds
            self.reached_goal = True
            self.get_logger().info('Reached goal, stopping and moving linear actuator.')
            self.move_linear_actuator()
            self.state = "silo_tracking"  # Switch to silo tracking state

        else:
            self.reached_goal = False