from multiverse_client_py import MultiverseClient, MultiverseMetaData

from typing import Dict
from std_srvs.srv import SetBool, SetBoolResponse
import rospy

import threading
import numpy
from scipy.spatial.transform import Rotation

class SwichtingCommand(MultiverseClient):
    object_physics = {
        "montessori_object_2": True,
        "montessori_object_3": True,
        "montessori_object_5": True,
        "montessori_object_6": True,
    }
    check_physics_thread = threading.Thread()
    lock = threading.Lock()

    def __init__(self, port: str, multiverse_meta_data: MultiverseMetaData) -> None:
        super().__init__(port, multiverse_meta_data)

    def init(self) -> None:
        self.run()
        self.check_physics_thread = threading.Thread(target=self.check_physics)
        self.check_physics_thread.start()

    def check_physics(self) -> None:
        while not rospy.is_shutdown():
            self.lock.acquire()
            self.request_meta_data["meta_data"]["simulation_name"] = "switching_command"
            self.request_meta_data["send"] = {}
            self.request_meta_data["receive"] = {}
            self.request_meta_data["receive"]["montessori_object_2_unreal"] = ["scalar"]
            self.request_meta_data["receive"]["montessori_object_3_unreal"] = ["scalar"]
            self.request_meta_data["receive"]["montessori_object_5_unreal"] = ["scalar"]
            self.request_meta_data["receive"]["montessori_object_6_unreal"] = ["scalar"]
            self.send_and_receive_meta_data()
            need_switch = False
            for object_ref_name, object_attributes in self.response_meta_data["receive"].items():
                object_name = object_ref_name[:-7]  # Remove '_unreal' suffix
                if object_attributes["scalar"][0] > 0.0 and self.object_physics[object_name]:
                    self.object_physics[object_name] = False
                    need_switch = True
                elif object_attributes["scalar"][0] < 0.0 and not self.object_physics[object_name]:
                    self.object_physics[object_name] = True
                    need_switch = True
            if need_switch:
                self.logwarn("Physics state changed, switching physics.")
                self.switch_physics()
            self.lock.release()
            time.sleep(0.1)

    def switch_physics(self) -> None:
        self.request_meta_data["meta_data"]["simulation_name"] = "montessori_toys_2"
        self.request_meta_data["send"] = {}
        self.request_meta_data["receive"] = {}
        for object_name, mujoco_takes_over in self.object_physics.items():
            if mujoco_takes_over:
                self.request_meta_data["send"][f"{object_name}_ref"] = ["position", "quaternion"]
                self.request_meta_data["receive"][f"{object_name}_ref"] = []
            else:
                self.request_meta_data["send"][f"{object_name}_ref"] = []
                self.request_meta_data["receive"][f"{object_name}_ref"] = ["position", "quaternion", "relative_velocity"]
        self.send_and_receive_meta_data()
        send_data = [self.sim_time]
        if "send" in self.response_meta_data:
            idx = 0
            for _, object_attributes in self.response_meta_data["send"].items():
                for _, attribute_values in object_attributes.items():
                    send_data += attribute_values
                    idx += len(attribute_values)
        self.send_data = send_data
        self.send_and_receive_data()

    def loginfo(self, message: str) -> None:
        print(f"INFO: {message}")

    def logwarn(self, message: str) -> None:
        print(f"WARN: {message}")

    def _run(self) -> None:
        self.loginfo("Start running the client.")
        self._connect_and_start()

    def send_and_receive_meta_data(self) -> None:
        # self.loginfo("Sending request meta data: " + str(self.request_meta_data))
        self._communicate(True)
        # self.loginfo("Received response meta data: " + str(self.response_meta_data))

    def send_and_receive_data(self) -> None:
        self._communicate(False)

class GraspingCommand(MultiverseClient):
    left_hand_grasped = False
    right_hand_grasped = False
    grasp_radius = 0.05
    object_states = {
        "montessori_object_2": {"position": [0.0, 0.0, 0.0], "quaternion": [1.0, 0.0, 0.0, 0.0]},
        "montessori_object_3": {"position": [0.0, 0.0, 0.0], "quaternion": [1.0, 0.0, 0.0, 0.0]},
        "montessori_object_5": {"position": [0.0, 0.0, 0.0], "quaternion": [1.0, 0.0, 0.0, 0.0]},
        "montessori_object_6": {"position": [0.0, 0.0, 0.0], "quaternion": [1.0, 0.0, 0.0, 0.0]},
    }
    hand_states = {
        "l_gripper_tool_frame": {"position": [0.0, 0.0, 0.0], "quaternion": [1.0, 0.0, 0.0, 0.0], "grasped_objects": {}},
        "r_gripper_tool_frame": {"position": [0.0, 0.0, 0.0], "quaternion": [1.0, 0.0, 0.0, 0.0], "grasped_objects": {}}
    }
    grasp_thread = threading.Thread()
    grasp_stop = False

    def __init__(self, port: str, multiverse_meta_data: MultiverseMetaData, switching_connector: SwichtingCommand) -> None:
        self.switching_connector = switching_connector
        rospy.Service('grasp_with_left_hand', SetBool, self.grasp_with_left_hand)
        rospy.Service('grasp_with_right_hand', SetBool, self.grasp_with_right_hand)
        rospy.Service('mujoco_takes_over', SetBool, self.mujoco_takes_over)
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
            self.request_meta_data["receive"]["l_gripper_tool_frame"] = ["position", "quaternion"]
        if right:
            self.request_meta_data["receive"]["r_gripper_tool_frame"] = ["position", "quaternion"]
        self.send_and_receive_meta_data()
        response_meta_data = self.response_meta_data
        if left:
            self.hand_states["l_gripper_tool_frame"]["position"] = response_meta_data["receive"]["l_gripper_tool_frame"]["position"]
            self.hand_states["l_gripper_tool_frame"]["quaternion"] = response_meta_data["receive"]["l_gripper_tool_frame"]["quaternion"]
        if right:
            self.hand_states["r_gripper_tool_frame"]["position"] = response_meta_data["receive"]["r_gripper_tool_frame"]["position"]
            self.hand_states["r_gripper_tool_frame"]["quaternion"] = response_meta_data["receive"]["r_gripper_tool_frame"]["quaternion"]

        grasped_success = False
        left_min_distance = float('inf')
        right_min_distance = float('inf')
        objects_to_grasp = {}
        for object_name in self.object_states.keys():
            self.object_states[object_name]["position"] = response_meta_data["receive"][f"{object_name}"]["position"]
            self.object_states[object_name]["quaternion"] = response_meta_data["receive"][f"{object_name}"]["quaternion"]
            if left and object_name not in self.hand_states["l_gripper_tool_frame"]["grasped_objects"]:
                left_distance = ((self.object_states[object_name]["position"][0] - self.hand_states["l_gripper_tool_frame"]["position"][0]) ** 2 +
                                (self.object_states[object_name]["position"][1] - self.hand_states["l_gripper_tool_frame"]["position"][1]) ** 2 +
                                (self.object_states[object_name]["position"][2] - self.hand_states["l_gripper_tool_frame"]["position"][2]) ** 2) ** 0.5
                self.loginfo(f"Checking grasp for object {object_name} with left hand. Distance: {left_distance:.2f} m")
                if left_distance < self.grasp_radius:
                    self.loginfo(f"Object {object_name} is within grasping range for left hand. Distance: {left_distance:.2f} m")
                    grasped_success = True
                    if left_distance < left_min_distance:
                        left_min_distance = left_distance
                        objects_to_grasp["l_gripper_tool_frame"] = object_name
            if right and object_name not in self.hand_states["r_gripper_tool_frame"]["grasped_objects"]:
                right_distance = ((self.object_states[object_name]["position"][0] - self.hand_states["r_gripper_tool_frame"]["position"][0]) ** 2 +
                                (self.object_states[object_name]["position"][1] - self.hand_states["r_gripper_tool_frame"]["position"][1]) ** 2 +
                                (self.object_states[object_name]["position"][2] - self.hand_states["r_gripper_tool_frame"]["position"][2]) ** 2) ** 0.5
                self.loginfo(f"Checking grasp for object {object_name} with right hand. Distance: {right_distance:.2f} m")
                if right_distance < self.grasp_radius:
                    self.loginfo(f"Object {object_name} is within grasping range for right hand. Distance: {right_distance:.2f} m")
                    grasped_success = True
                    if right_distance < right_min_distance:
                        right_min_distance = right_distance
                        objects_to_grasp["r_gripper_tool_frame"] = object_name

        if grasped_success:
            for hand_name, object_name in objects_to_grasp.items():
                # Calculate relative position of the object to the hand
                hand_rot = Rotation.from_quat(self.hand_states[hand_name]["quaternion"], scalar_first=True)
                object_rot = Rotation.from_quat(self.object_states[object_name]["quaternion"], scalar_first=True)
                # Compute relative position
                relative_position = numpy.array(self.object_states[object_name]["position"]) - numpy.array(self.hand_states[hand_name]["position"])
                object_relative_position = hand_rot.inv().apply(relative_position)
                object_relative_rotation = hand_rot.inv() * object_rot
                object_relative_quaternion = object_relative_rotation.as_quat(scalar_first=True)
                self.hand_states[hand_name]["grasped_objects"][object_name] = [object_relative_position, object_relative_quaternion]
                self.switching_connector.object_physics[object_name] = False

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
                    if object_name in ["l_gripper_tool_frame", "r_gripper_tool_frame"]:
                        self.hand_states[object_name][attribute_name] = receive_data[idx:idx+len(attribute_values)]
                    idx += len(attribute_values)

            for hand_name in ["l_gripper_tool_frame", "r_gripper_tool_frame"]:
                for object_name, object_data in self.hand_states[hand_name]["grasped_objects"].items():
                    relative_position, relative_quaternion = object_data
                    hand_position = numpy.array(self.hand_states[hand_name]["position"])
                    hand_rotation = Rotation.from_quat(self.hand_states[hand_name]["quaternion"], scalar_first=True)
                    self.object_states[object_name]["position"] = (hand_position + hand_rotation.apply(relative_position)).tolist()
                    self.object_states[object_name]["quaternion"] = ((hand_rotation * Rotation.from_quat(relative_quaternion, scalar_first=True))).as_quat(scalar_first=True).tolist()

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
        self.switching_connector.lock.acquire()
        if req.data:
            self.left_hand_grasped = self.can_grasp(left=True, right=self.right_hand_grasped)
            if not self.left_hand_grasped:
                self.switching_connector.lock.release()
                return SetBoolResponse(success=False, message="Left hand cannot grasp the object.")
            self.switching_connector.switch_physics()
            self.grasp_stop = False
            self.grasp_thread = threading.Thread(target=self.grasp)
            self.grasp_thread.start()
            return SetBoolResponse(success=True, message="Left hand grasped successfully.")
        else:
            self.left_hand_grasped = False
            for object_name in self.hand_states["l_gripper_tool_frame"]["grasped_objects"].keys():
                self.switching_connector.object_physics[object_name] = True
            self.switching_connector.switch_physics()
            self.hand_states["l_gripper_tool_frame"]["grasped_objects"] = {}
            self.switching_connector.lock.release()
            return SetBoolResponse(success=True, message="Left hand released successfully.")

    def grasp_with_right_hand(self, req):
        self.grasp_stop = True
        if self.grasp_thread.is_alive():
            self.grasp_thread.join()
        self.switching_connector.lock.acquire()
        if req.data:
            self.right_hand_grasped = self.can_grasp(left=self.left_hand_grasped, right=True)
            if not self.right_hand_grasped:
                self.switching_connector.lock.release()
                return SetBoolResponse(success=False, message="Right hand cannot grasp the object.")
            self.switching_connector.switch_physics()
            self.grasp_stop = False
            self.grasp_thread = threading.Thread(target=self.grasp)
            self.grasp_thread.start()
            return SetBoolResponse(success=True, message="Right hand grasped successfully.")
        else:
            self.right_hand_grasped = False
            for object_name in self.hand_states["r_gripper_tool_frame"]["grasped_objects"].keys():
                self.switching_connector.object_physics[object_name] = True
            self.switching_connector.switch_physics()
            self.hand_states["r_gripper_tool_frame"]["grasped_objects"] = {}
            self.switching_connector.lock.release()
            return SetBoolResponse(success=True, message="Right hand released successfully.")

    def mujoco_takes_over(self, req):
        self.switching_connector.lock.acquire()
        for object_name in self.switching_connector.object_physics.keys():
            self.switching_connector.object_physics[object_name] = req.data
        self.switching_connector.switch_physics()
        self.switching_connector.lock.release()
        return SetBoolResponse(success=True, message="Mujoco physics switched successfully.")

import time
import argparse

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="iCub Grasping Server")
    parser.add_argument("--port", type=str, default="1855", help="Port for the switching command")
    args = parser.parse_args()
    rospy.init_node('iCub_grasp_server')

    multiverse_meta_data = MultiverseMetaData(
        world_name="world",
        simulation_name="switching_command",
        length_unit="m",
        angle_unit="rad",
        mass_unit="kg",
        time_unit="s",
        handedness="rhs",
    )
    switching_connector = SwichtingCommand(port=args.port, multiverse_meta_data=multiverse_meta_data)
    switching_connector.init()
    rospy.loginfo("Ready to switch physics.")

    multiverse_meta_data = MultiverseMetaData(
        world_name="world",
        simulation_name="grasping_command",
        length_unit="m",
        angle_unit="rad",
        mass_unit="kg",
        time_unit="s",
        handedness="rhs",
    )
    grasping_connector = GraspingCommand(port="1856", multiverse_meta_data=multiverse_meta_data, switching_connector=switching_connector)
    grasping_connector.init()

    rospy.loginfo("Ready to grasp.")
    rospy.spin()
