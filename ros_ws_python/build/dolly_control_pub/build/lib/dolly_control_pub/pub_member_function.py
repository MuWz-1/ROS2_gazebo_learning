import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Vector3

class DollyControlPub(Node):
    
    def __init__(self):
        super().__init__('dolly_control_pub')
        # super()用来调用父类(基类)的方法，__init__()是类的构造方法，
        # super().__init__() 就是调用父类的init方法， 同样可以使用super()去调用父类的其他方法。
        self.publishers_ = self.create_publisher(Vector3, "cmd_vel", 10)
        
        self.set_vel()

    def set_vel(self):
        msg = Vector3()
        msg.x = 36.0
        msg.y = 0.0
        msg.z = 0.0

        self.publishers_.publish(msg)
        self.get_logger().info('Publishing velocity: "%d"' % msg.x)

def main(args=None):
    rclpy.init(args=args)

    dolly_control_pub = DollyControlPub()

    rclpy.spin_once(dolly_control_pub)

    dolly_control_pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


