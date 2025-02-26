#!/usr/bin/env python3

from multiverse_client_py import MultiverseClient, MultiverseMetaData

class JointStateSender(MultiverseClient):
    def __init__(self, port: str, multiverse_meta_data: MultiverseMetaData) -> None:
        super().__init__(port, multiverse_meta_data)

    def loginfo(self, message: str) -> None:
        print(f"INFO: {message}")

    def logwarn(self, message: str) -> None:
        print(f"WARN: {message}")

    def _run(self) -> None:
        self.loginfo("Start logging.")
        self._connect_and_start()

    def send_and_receive_meta_data(self) -> None:
        self.loginfo("Sending request meta data: " + str(self.request_meta_data))
        self._communicate(True)
        self.loginfo("Received response meta data: " + str(self.response_meta_data))

    def send_and_receive_data(self) -> None:
        self._communicate(False)

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState

object_attributes = {
    "actuator_1": ["cmd_joint_rvalue"],
    "actuator_2": ["cmd_joint_rvalue"],
    "actuator_3": ["cmd_joint_rvalue"],
    "actuator_4": ["cmd_joint_rvalue"],
    "actuator_5": ["cmd_joint_rvalue"],
    "actuator_6": ["cmd_joint_rvalue"],
    "actuator_7": ["cmd_joint_rvalue"],
    "fingers_actuator": ["cmd_joint_tvalue"],
}

joint_actuator_map = {
    "joint_1": "actuator_1",
    "joint_2": "actuator_2",
    "joint_3": "actuator_3",
    "joint_4": "actuator_4",
    "joint_5": "actuator_5",
    "joint_6": "actuator_6",
    "joint_7": "actuator_7",
    "robotiq_85_left_knuckle_joint": "fingers_actuator",
}

actuator_ids = {
    "actuator_1": 1,
    "actuator_2": 2,
    "actuator_3": 3,
    "actuator_4": 4,
    "actuator_5": 5,
    "actuator_6": 6,
    "actuator_7": 7,
    "fingers_actuator": 8,
}

class JointStateSubscriber(Node):
    def __init__(self):
        multiverse_meta_data = MultiverseMetaData(
            world_name="world",
            simulation_name="joint_state_sender",
            length_unit="m",
            angle_unit="rad",
            mass_unit="kg",
            time_unit="s",
            handedness="rhs",
        )
        self.joint_state_sender = JointStateSender(port="5000",
                                            multiverse_meta_data=multiverse_meta_data)
        self.joint_state_sender.run()

        self.joint_state_sender.request_meta_data["send"] = {}
        self.joint_state_sender.request_meta_data["receive"] = {}
        for object_name, attribute_names in object_attributes.items():
            self.joint_state_sender.request_meta_data["send"][object_name] = attribute_names
        self.joint_state_sender.send_and_receive_meta_data()

        self.joint_state_sender.send_data = [0.0] + [0.0, 0.262, -3.14, -2.27, 0.0, 0.96, 1.57, 0.0]
        self.joint_state_sender.send_and_receive_data()

        super().__init__('joint_state_subscriber')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.callback,
            10)

    def callback(self, msg: JointState):
        send_data = [self.joint_state_sender.sim_time] + [0.0] * len(actuator_ids)
        for i, joint_name in enumerate(msg.name):
            joint_value = msg.position[i]
            actuator_name = joint_actuator_map[joint_name]
            actuator_id = actuator_ids[actuator_name]
            send_data[actuator_id] = joint_value
        if send_data[8] > 0.1:
            send_data[8] = 255.0
        self.joint_state_sender.send_data = send_data
        self.joint_state_sender.send_and_receive_data()


if __name__ == "__main__":
    rclpy.init()

    joint_state_subscriber = JointStateSubscriber()

    rclpy.spin(joint_state_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    joint_state_subscriber.destroy_node()
    rclpy.shutdown()