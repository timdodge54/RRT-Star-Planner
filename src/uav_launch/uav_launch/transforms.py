import rclpy
from rclpy.node import Node
from uav_interfaces.msg import UavState
from tf2_ros import TransformBroadcaster
from tf2_ros import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from mav_sim.chap2.transforms import rot_v_to_v1, rot_v1_to_v2, rot_v2_to_b, rot_b_to_s, rot_s_to_w
from mav_sim.tools.types import RotMat, Vector, NP_MAT

import numpy as np

import tf_transformations


class TransformPublisher(Node):
    """ Given the UavState, this class publishes the transforms 
        inertial -> vehicle
        vehicle -> vehicle1
        vehicle1 -> vehicle2
        vehicle2 -> body
        body -> stability
        stability -> wind

    Each transform is prefixed by the given namespace
    """
    def __init__(self) -> None:
        """ Initializes the subscription to the uav state
        """

        # Initialize the node
        super().__init__(node_name="uav_transformer")

        # Create the subsciber for the uav state
        self.sub_state = self.create_subscription(UavState, "uav_state", self.state_callback, 1)
        
        # Create variables for the transforms
        self.br = TransformBroadcaster(self)

        ### Broadcast the transform from map to ned ###
        # Initialize transform
        self._static_br = StaticTransformBroadcaster(self)
        static_tf = TransformStamped()
        static_tf.header.stamp = self.get_clock().now().to_msg()
        static_tf.header.frame_id = 'enu'
        static_tf.child_frame_id = 'ned'

        # Set zero translation
        static_tf.transform.translation.x = 0.
        static_tf.transform.translation.y = 0.
        static_tf.transform.translation.z = 0.

        # Set rotation
        rot = np.array([[0., 1., 0.], [1., 0., 0.], [0., 0., -1.]]).T
        q = tf_transformations.quaternion_from_matrix(to_4x4(rot))
        static_tf.transform.rotation.x = q[0]
        static_tf.transform.rotation.y = q[1]
        static_tf.transform.rotation.z = q[2]
        static_tf.transform.rotation.w = q[3]
        self._static_br.sendTransform(static_tf)


    def state_callback(self, msg: UavState) -> None:
        """ Stores the state
        """
        self.publish_transforms(msg)

    def publish_transforms(self, state: UavState) -> None:
        """Publishes the transforms as recorded in the state

        args:
            state: State from which to publish transforms
        """
        
        # Inertial to vehicle
        stamp = self.get_clock().now().to_msg() # state.pose.header.stamp
        trans = np.array([[state.pose.pose.position.x], [state.pose.pose.position.y], [state.pose.pose.position.z]])
        self.publish_transform(stamp, "ned", "vehicle", np.identity(3), trans)

        # Vehicle to vehicle 1
        self.publish_transform(stamp, 'vehicle', 'vehicle1', rot_v_to_v1(state.psi))

        # Vehicle 1 to vehicle 2
        self.publish_transform(stamp, 'vehicle1', 'vehicle2', rot_v1_to_v2(state.theta))

        # Vehicle 2 to body
        self.publish_transform(stamp, 'vehicle2', 'body', rot_v2_to_b(state.phi))

        # Body to stability
        self.publish_transform(stamp, 'body', 'stability', rot_b_to_s(state.alpha))

        # Stability to wind
        self.publish_transform(stamp, 'stability', 'wind', rot_s_to_w(state.beta))


    def publish_transform(self, stamp, header_frame: str, child_frame: str, rot: RotMat, trans: Vector = np.zeros([3,1])) -> None:
        """Publishes a single transform

        args:
            stamp: Time stamp of transform
            header_frame: Frame from which the transform originates
            child_frame: Frame to which the transform extends
            rot: Rotation matrix defining the rotation from header to child
            trans: Translation vector defining the translation in the header frame to the child frame origin
        """

        # Initialize transform
        t = TransformStamped()
        t.header.stamp = stamp
        
        # Set frames
        t.header.frame_id = header_frame
        t.child_frame_id = child_frame

        # Set translation
        t.transform.translation.x = trans.item(0)
        t.transform.translation.y = trans.item(1)
        t.transform.translation.z = trans.item(2)

        # Set orientation
        q = tf_transformations.quaternion_from_matrix(to_4x4(rot.T))
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # Send the transformation
        self.br.sendTransform(t)

def to_4x4(rot: RotMat) -> NP_MAT:
    """ Converts a 3x3 rotation matrix to a 4x4 transformation matrix

    args:
        rot: 3x3 rotation matrix

    returns:
        4x4 rotation matrix
    """
    T = np.zeros([4,4])
    T[0:3,0:3] = rot
    return T

def main(args=None):
    rclpy.init(args=args)

    # Create the transform
    transform_publisher = TransformPublisher()
    rclpy.spin(transform_publisher)

    # # Destroy node instead of waiting for garbage collector
    # transform_publisher.destroy_node()
    # rclpy.shutdown()   


if __name__ == '__main__':
    main()
