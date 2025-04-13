import rclpy
from rclpy.node import Node
from rclpy.time import Time

from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
import tf_transformations
import numpy as np

class TransFormFinder(Node):
    def __init__(self):
        super().__init__('tf_finder_node')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        try:
            trans: TransformStamped = self.tf_buffer.lookup_transform(
                target_frame='robot_base_link',
                source_frame='robot_front_laser_link',
                time=Time()
            )
            translation = trans.transform.translation
            rotation = trans.transform.rotation

            T = tf_transformations.translation_matrix([translation.x, translation.y, translation.z])
            R = tf_transformations.quaternion_matrix([rotation.x, rotation.y, rotation.z, rotation.w])

            transformation_matrix = T @ R

            print("Transformation Matrix:")
            print(np.round(transformation_matrix, 4))
            np.save("front_laser_transform.npy", transformation_matrix)
        except Exception as e:
            self.get_logger().error(f"Could not get the transform: {e}")

def main(args=None):
    rclpy.init(args=args)
    transform_finder = TransFormFinder()

    rclpy.spin(transform_finder)

if __name__ == '__main__':
    main()