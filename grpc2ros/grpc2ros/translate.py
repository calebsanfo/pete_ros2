import grpc
from concurrent import futures
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from nav_msgs.msg import Path
from std_msgs.msg import Header

import path2D_pb2
import path2D_pb2_grpc

# Define the gRPC server handler
class PathService(path2D_pb2_grpc.PathServiceServicer):
    def __init__(self, node):
        self.node = node  # ROS 2 node instance

    def SendPath(self, request, context):
        path_message = Path()
        path_message.header = Header(frame_id="map", stamp=self.node.get_clock().now().to_msg())

         # Logging the receipt of the path and its length
        self.node.get_logger().info(f'Received path with {len(request.poses)} poses.')

        for pose in request.poses:
            ros_pose_stamped = PoseStamped()
            ros_pose_stamped.header = Header(frame_id="map", stamp=self.node.get_clock().now().to_msg())
            # Correct way to initialize Point with named arguments
            ros_pose_stamped.pose.position = Point(x=pose.pose.x, y=pose.pose.y, z=0.0)  # Assuming the z-coordinate is 0
            # Assuming no rotation, using an identity quaternion
            ros_pose_stamped.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            path_message.poses.append(ros_pose_stamped)

        self.node.publish_path(path_message)
        return path2D_pb2.Acknowledge(success=True, message="Path received and published")

class PathPublisherNode(Node):
    def __init__(self):
        super().__init__('path_publisher_node')
        self.publisher = self.create_publisher(Path, 'path_topic', 10)
        self.grpc_server = grpc.server(futures.ThreadPoolExecutor(max_workers=2))
        path2D_pb2_grpc.add_PathServiceServicer_to_server(PathService(self), self.grpc_server)
        self.grpc_server.add_insecure_port('[::]:50051')
        self.grpc_server.start()
        self.get_logger().info('gRPC server started on port 50051')

    def publish_path(self, path):
        self.publisher.publish(path)

    def shutdown(self):
        self.grpc_server.stop(None)
        self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = PathPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
