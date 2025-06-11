from multiverse_client_py import MultiverseClient, MultiverseMetaData

from std_srvs.srv import SetBool, SetBoolResponse
import rospy

import threading
import numpy
from scipy.spatial.transform import Rotation

class FingersCommand(MultiverseClient):
    left_hand_grasped = False
    right_hand_grasped = False
    grasp_radius = 0.1
    object_states = {
        "montessori_object_2": {"position": [0.0, 0.0, 0.0], "quaternion": [1.0, 0.0, 0.0, 0.0]},
        "montessori_object_3": {"position": [0.0, 0.0, 0.0], "quaternion": [1.0, 0.0, 0.0, 0.0]},
        "montessori_object_5": {"position": [0.0, 0.0, 0.0], "quaternion": [1.0, 0.0, 0.0, 0.0]},
        "montessori_object_6": {"position": [0.0, 0.0, 0.0], "quaternion": [1.0, 0.0, 0.0, 0.0]},
    }
    hand_states = {
        "l_hand": {"position": [0.0, 0.0, 0.0], "quaternion": [1.0, 0.0, 0.0, 0.0], "grasped_objects": {}},
        "r_hand": {"position": [0.0, 0.0, 0.0], "quaternion": [1.0, 0.0, 0.0, 0.0], "grasped_objects": {}}
    }
    grasp_thread = threading.Thread()
    grasp_stop = False

    def __init__(self, port: str, multiverse_meta_data: MultiverseMetaData) -> None:
        rospy.Service('grasp_with_left_hand', SetBool, self.grasp_with_left_hand)
        rospy.Service('grasp_with_right_hand', SetBool, self.grasp_with_right_hand)
        super().__init__(port, multiverse_meta_data)

    def init(self) -> None:
        self.run()

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

    def can_grasp(self, left=False, right=False) -> bool:
        self.request_meta_data["send"] = {}
        for object_name in self.object_states.keys():
            self.request_meta_data["send"][f"{object_name}_ref"] = ["position", "quaternion"]
        self.request_meta_data["receive"] = {}
        for object_name in self.object_states.keys():
            self.request_meta_data["receive"][f"{object_name}"] = ["position", "quaternion"]
        if left:
            self.request_meta_data["receive"]["l_hand"] = ["position", "quaternion"]
        if right:
            self.request_meta_data["receive"]["r_hand"] = ["position", "quaternion"]
        self.send_and_receive_meta_data()
        response_meta_data = self.response_meta_data
        if left:
            self.hand_states["l_hand"]["position"] = response_meta_data["receive"]["l_hand"]["position"]
            self.hand_states["l_hand"]["quaternion"] = response_meta_data["receive"]["l_hand"]["quaternion"]
        if right:
            self.hand_states["r_hand"]["position"] = response_meta_data["receive"]["r_hand"]["position"]
            self.hand_states["r_hand"]["quaternion"] = response_meta_data["receive"]["r_hand"]["quaternion"]

        grasped_success = False
        for object_name in self.object_states.keys():
            self.object_states[object_name]["position"] = response_meta_data["receive"][f"{object_name}"]["position"]
            self.object_states[object_name]["quaternion"] = response_meta_data["receive"][f"{object_name}"]["quaternion"]
            if left and object_name not in self.hand_states["l_hand"]["grasped_objects"]:
                left_distance = ((self.object_states[object_name]["position"][0] - self.hand_states["l_hand"]["position"][0]) ** 2 +
                                (self.object_states[object_name]["position"][1] - self.hand_states["l_hand"]["position"][1]) ** 2 +
                                (self.object_states[object_name]["position"][2] - self.hand_states["l_hand"]["position"][2]) ** 2) ** 0.5
                self.loginfo(f"Checking grasp for object {object_name} with left hand. Distance: {left_distance:.2f} m")
                if left_distance < self.grasp_radius:
                    self.loginfo(f"Object {object_name} is within grasping range for left hand. Distance: {left_distance:.2f} m")
                    # Calculate relative position of the object to the hand
                    hand_rot = Rotation.from_quat(self.hand_states["l_hand"]["quaternion"])
                    object_rot = Rotation.from_quat(self.object_states[object_name]["quaternion"])
                    # Compute relative position
                    relative_position = numpy.array(self.object_states[object_name]["position"]) - numpy.array(self.hand_states["l_hand"]["position"])
                    object_relative_position = hand_rot.inv().apply(relative_position)
                    object_relative_rotation = hand_rot.inv() * object_rot
                    object_relative_quaternion = object_relative_rotation.as_quat()
                    self.hand_states["l_hand"]["grasped_objects"][object_name] = [object_relative_position, object_relative_quaternion]
                    grasped_success = True
            if right and object_name not in self.hand_states["r_hand"]["grasped_objects"]:
                right_distance = ((self.object_states[object_name]["position"][0] - self.hand_states["r_hand"]["position"][0]) ** 2 +
                                (self.object_states[object_name]["position"][1] - self.hand_states["r_hand"]["position"][1]) ** 2 +
                                (self.object_states[object_name]["position"][2] - self.hand_states["r_hand"]["position"][2]) ** 2) ** 0.5
                self.loginfo(f"Checking grasp for object {object_name} with right hand. Distance: {right_distance:.2f} m")
                if right_distance < self.grasp_radius:
                    self.loginfo(f"Object {object_name} is within grasping range for right hand. Distance: {right_distance:.2f} m")
                    # Calculate relative position of the object to the hand
                    hand_rot = Rotation.from_quat(self.hand_states["r_hand"]["quaternion"])
                    object_rot = Rotation.from_quat(self.object_states[object_name]["quaternion"])
                    # Compute relative position
                    relative_position = numpy.array(self.object_states[object_name]["position"]) - numpy.array(self.hand_states["r_hand"]["position"])
                    object_relative_position = hand_rot.inv().apply(relative_position)
                    object_relative_rotation = hand_rot.inv() * object_rot
                    object_relative_quaternion = object_relative_rotation.as_quat()
                    self.hand_states["r_hand"]["grasped_objects"][object_name] = [object_relative_position, object_relative_quaternion]
                    grasped_success = True
        return grasped_success
        
    def grasp(self) -> None:
        send_data = [self.sim_time]
        for object_name, object_attributes in self.response_meta_data["send"].items():
            for attribute_name, attribute_values in object_attributes.items():
                send_data += self.object_states[object_name[:-4]][attribute_name]
        self.send_data = send_data
        self.send_and_receive_data()
        while not rospy.is_shutdown() and not self.grasp_stop:
            receive_data = self.receive_data[1:]
            idx = 0
            for object_name, object_attributes in self.response_meta_data["receive"].items():
                for attribute_name, attribute_values in object_attributes.items():
                    if object_name in ["l_hand", "r_hand"]:
                        self.hand_states[object_name][attribute_name] = receive_data[idx:idx+len(attribute_values)]
                    idx += len(attribute_values)

            for hand_name in ["l_hand", "r_hand"]:
                for object_name, object_data in self.hand_states[hand_name]["grasped_objects"].items():
                    relative_position, relative_quaternion = object_data
                    hand_position = numpy.array(self.hand_states[hand_name]["position"])
                    hand_rotation = Rotation.from_quat(self.hand_states[hand_name]["quaternion"])
                    self.object_states[object_name]["position"] = (hand_position + hand_rotation.apply(relative_position)).tolist()
                    self.object_states[object_name]["quaternion"] = ((hand_rotation * Rotation.from_quat(relative_quaternion)).as_quat()).tolist()

            send_data = [self.sim_time]
            for object_name, object_attributes in self.response_meta_data["send"].items():
                for attribute_name, attribute_values in object_attributes.items():
                    send_data += self.object_states[object_name[:-4]][attribute_name]
            self.send_data = send_data
            self.send_and_receive_data()
            time.sleep(0.001)

    def grasp_with_left_hand(self, req):
        self.grasp_stop = True
        if self.grasp_thread.is_alive():
            self.grasp_thread.join()
        if req.data:
            self.left_hand_grasped = self.can_grasp(left=True, right=self.right_hand_grasped)
            if not self.left_hand_grasped:
                return SetBoolResponse(success=False, message="Left hand cannot grasp the object.")
            self.grasp_stop = False
            self.grasp_thread = threading.Thread(target=self.grasp)
            self.grasp_thread.start()
            return SetBoolResponse(success=True, message="Left hand grasped successfully.")
        else:
            self.left_hand_grasped = False
            self.hand_states["l_hand"]["grasped_objects"] = {}
            return SetBoolResponse(success=True, message="Left hand released successfully.")

    def grasp_with_right_hand(self, req):
        self.grasp_stop = True
        if self.grasp_thread.is_alive():
            self.grasp_thread.join()
        if req.data:
            self.right_hand_grasped = self.can_grasp(left=self.left_hand_grasped, right=True)
            if not self.right_hand_grasped:
                return SetBoolResponse(success=False, message="Right hand cannot grasp the object.")
            self.grasp_stop = False
            self.grasp_thread = threading.Thread(target=self.grasp)
            self.grasp_thread.start()
            return SetBoolResponse(success=True, message="Right hand grasped successfully.")
        else:
            self.right_hand_grasped = False
            self.hand_states["r_hand"]["grasped_objects"] = {}
            return SetBoolResponse(success=True, message="Right hand released successfully.")

import time

if __name__ == "__main__":
    rospy.init_node('iCub_grasp_server')

    multiverse_meta_data = MultiverseMetaData(
        world_name="world",
        simulation_name="gripper_command",
        length_unit="m",
        angle_unit="rad",
        mass_unit="kg",
        time_unit="s",
        handedness="rhs",
    )
    my_connector = FingersCommand(port="1856", multiverse_meta_data=multiverse_meta_data)
    my_connector.init()

    print("Ready to grasp.")
    rospy.spin()
