from .jacobian_A_P_function import *
from .ur_position_vector import *
from std_msgs.msg import Float64MultiArray as F64M
import rclpy
from rclpy.node import Node

class MyInverseNode(Node):
    
    def __init__(self, q_real, q0, alpha, number_of_iterations):

        super().__init__("my_ik_position_publisher")
        self.publisher_ = self.create_publisher(F64M, '/forward_position_controller/commands', 10)
        node_period = 1
        self.timer = self.create_timer(node_period, self.timer_callback)
        self.q_real = np.copy(q_real)
        self.q = np.copy(q0)
        self.alpha = alpha
        self.number_of_iterations = number_of_iterations
        self.pos_msg = self.calculate_gradient_descent()

    def timer_callback(self):

        pos_msg = F64M()
        pos_msg.data = self.pos_msg
        self.publisher_.publish(pos_msg)
        self.get_logger().info('Publishing: "%s"' % pos_msg.data)
        
    def calculate_error(self):

        return ur_position_vector(self.q_real.flatten()) - ur_position_vector(self.q.flatten())

    def calculate_gradient_descent(self):

        for k in range(self.number_of_iterations):

            update_value_first_three = self.alpha * jacobian_A_P(self.q.flatten())[:,:3].T @ self.calculate_error()
            update_value_last_three = self.alpha * jacobian_A_P(self.q.flatten())[:,3:].T @ self.calculate_error()
            update_value_first_three = update_value_first_three.reshape(-1, 1)
            update_value_last_three = update_value_last_three.reshape(-1,1)
            self.q = np.vstack((self.q[:-3] + update_value_first_three, self.q[3:] + update_value_last_three))

        q_final = self.q
        return q_final.flatten().tolist()
