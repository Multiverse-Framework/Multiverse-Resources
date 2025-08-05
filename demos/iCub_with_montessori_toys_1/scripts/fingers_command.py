from multiverse_client_py import MultiverseClient, MultiverseMetaData

class FingersCommand(MultiverseClient):
    def __init__(self, port: str, multiverse_meta_data: MultiverseMetaData) -> None:
        super().__init__(port, multiverse_meta_data)

    def loginfo(self, message: str) -> None:
        print(f"INFO: {message}")

    def logwarn(self, message: str) -> None:
        print(f"WARN: {message}")

    def _run(self) -> None:
        self.loginfo("Start running the client.")
        self._connect_and_start()

    def send_and_receive_meta_data(self) -> None:
        self.loginfo("Sending request meta data: " + str(self.request_meta_data))
        self._communicate(True)
        self.loginfo("Received response meta data: " + str(self.response_meta_data))

    def send_and_receive_data(self) -> None:
        self._communicate(False)

import time

right_fingers_state_dict = {
    "open": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
    "close": [0.0, 0.7, 0.7, 0.7, 0.0, 0.7, 0.7, 0.7, 0.0, 0.7, 0.7, 0.7, 0.0, 0.7, 0.7, 0.7, 1.0, 0.7, 0.7, 0.7]
}

left_fingers_state_dict = {
    "open": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
    "close": [0.0, 0.7, 0.7, 0.7, 0.0, 0.7, 0.7, 0.7, 0.0, 0.7, 0.7, 0.7, 0.0, 0.7, 0.7, 0.7, 1.0, 0.7, 0.7, 0.7]
}

if __name__ == "__main__":
        multiverse_meta_data = MultiverseMetaData(
            world_name="world",
            simulation_name="gripper_command",
            length_unit="m",
            angle_unit="rad",
            mass_unit="kg",
            time_unit="s",
            handedness="rhs",
        )
        my_connector = FingersCommand(port="5000",
                                      multiverse_meta_data=multiverse_meta_data)
        my_connector.run()

        my_connector.request_meta_data["send"] = {}
        gripper_names = [
            "l_hand_index_0_actuator",
            "l_hand_index_1_actuator",
            "l_hand_index_2_actuator",
            "l_hand_index_3_actuator",
            "l_hand_little_0_actuator",
            "l_hand_little_1_actuator",
            "l_hand_little_2_actuator",
            "l_hand_little_3_actuator",
            "l_hand_middle_0_actuator",
            "l_hand_middle_1_actuator",
            "l_hand_middle_2_actuator",
            "l_hand_middle_3_actuator",
            "l_hand_ring_0_actuator",
            "l_hand_ring_1_actuator",
            "l_hand_ring_2_actuator",
            "l_hand_ring_3_actuator",
            "l_hand_thumb_0_actuator",
            "l_hand_thumb_1_actuator",
            "l_hand_thumb_2_actuator",
            "l_hand_thumb_3_actuator", 
            "r_hand_index_0_actuator", 
            "r_hand_index_1_actuator", 
            "r_hand_index_2_actuator", 
            "r_hand_index_3_actuator", 
            "r_hand_little_0_actuator", 
            "r_hand_little_1_actuator", 
            "r_hand_little_2_actuator", 
            "r_hand_little_3_actuator", 
            "r_hand_middle_0_actuator", 
            "r_hand_middle_1_actuator", 
            "r_hand_middle_2_actuator", 
            "r_hand_middle_3_actuator", 
            "r_hand_ring_0_actuator", 
            "r_hand_ring_1_actuator", 
            "r_hand_ring_2_actuator", 
            "r_hand_ring_3_actuator", 
            "r_hand_thumb_0_actuator", 
            "r_hand_thumb_1_actuator", 
            "r_hand_thumb_2_actuator", 
            "r_hand_thumb_3_actuator"]
        for gripper_name in gripper_names:
            my_connector.request_meta_data["send"][gripper_name] = ["cmd_joint_angular_position"]
        my_connector.send_and_receive_meta_data()

        # First: open the fingers
        my_connector.send_data = [0.1] + left_fingers_state_dict["open"] + right_fingers_state_dict["open"]
        my_connector.send_and_receive_data()

        time.sleep(1)

        # Second: change the state of the fingers
        left_fingers_state = "close"
        right_fingers_state = "open"
        
        while my_connector.receive_data[0] >= 0.0:
            fingers_state = left_fingers_state_dict[left_fingers_state] + right_fingers_state_dict[right_fingers_state]
            my_connector.send_data = [0.1] + fingers_state
            my_connector.send_and_receive_data()
            time.sleep(0.1)
