import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature
from geometry_msgs.msg import Pose

class NeuralNetworkNode(Node):

    def __init__(self):
        super().__init__('neural_network_node')
        self.subscription = self.create_subscription(
            Temperature,
            'sensor_topic',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(Pose, 'control_topic', 10)

    def listener_callback(self, msg):
        self.get_logger().info('Received sensor data: "%f"' % msg.temperature)
        # 模拟神经网络处理，生成轨迹路径
        pose = Pose()
        pose.position.x = 1.0
        pose.position.y = 1.0
        pose.position.z = 1.0
        self.publisher_.publish(pose)
        self.get_logger().info('Published control command')

def main(args=None):
    rclpy.init(args=args)
    neural_network_node = NeuralNetworkNode()
    rclpy.spin(neural_network_node)
    neural_network_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
