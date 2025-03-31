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
        my_connector = FingersCommand(port="1856",
                                      multiverse_meta_data=multiverse_meta_data)
        my_connector.run()

        my_connector.request_meta_data["send"] = {}
        my_connector.request_meta_data["send"]["actuator8"] = ["cmd_joint_tvalue"]
        my_connector.request_meta_data["receive"] = {}
        my_connector.request_meta_data["receive"]["gripper_trigger"] = ["cmd_joint_tvalue"]
        my_connector.send_and_receive_meta_data()

        # First: open the fingers
        finger_value = 255.0
        my_connector.send_data = [my_connector.sim_time, finger_value]
        my_connector.send_and_receive_data()

        while my_connector.receive_data[0] >= 0.0:
            receive_data = my_connector.receive_data
            gripper_trigger = receive_data[1]
            if gripper_trigger > 50.0:
                finger_value = 0.0
            else:
                finger_value = 255.0
            my_connector.send_data = [my_connector.sim_time, finger_value]
            my_connector.send_and_receive_data()
            time.sleep(0.1)
