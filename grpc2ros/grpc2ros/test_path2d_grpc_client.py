import grpc
import path2D_pb2
import path2D_pb2_grpc
from google.protobuf.timestamp_pb2 import Timestamp

def create_pose_stamped(x, y, theta):
    """ Helper function to create a PoseStamped protobuf message. """
    pose_stamped = path2D_pb2.PoseStamped()
    pose_stamped.header.frame_id = "map"
    pose_stamped.header.stamp.GetCurrentTime()  # Set the current timestamp
    pose_stamped.pose.x = x
    pose_stamped.pose.y = y
    pose_stamped.pose.theta = theta
    return pose_stamped

def run():
    # Connect to the gRPC server
    channel = grpc.insecure_channel('localhost:50051')
    stub = path2D_pb2_grpc.PathServiceStub(channel)

    # Create a Path2D message
    path = path2D_pb2.Path2D()
    poses = [
        (1.0, 2.0, 0.5),
        (2.0, 3.0, 1.0),
        (3.0, 4.0, 1.5),
        (4.0, 5.0, 2.0)
    ]
    for x, y, theta in poses:
        pose_stamped = create_pose_stamped(x, y, theta)
        path.poses.append(pose_stamped)

    # Send the message
    response = stub.SendPath(path)
    print("Server response:", response.message)

if __name__ == '__main__':
    run()
