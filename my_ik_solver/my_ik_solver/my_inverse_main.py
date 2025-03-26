from .my_inverse_node import *

def main(args=None):
    rclpy.init(args=args)
    q_real = np.array([[0 , -1.57, 0, -1.57, 0, 0]]).T
    q0 = np.array([[0, 0, 0, 0, 0, 0]]).T
    alpha = 1
    number_of_iteration = 1000
    my_inverse_publisher = MyInverseNode(q_real=q_real, q0=q0, alpha=alpha, number_of_iterations=number_of_iteration)
    rclpy.spin(my_inverse_publisher)
    my_inverse_publisher.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
