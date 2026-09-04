import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path


SOURCE_ROOT = Path(__file__).resolve().parents[1]
XACRO = SOURCE_ROOT / "urdf" / "marvin_pika_bimanual.urdf.xacro"


def _expand(*xacro_args):
    result = subprocess.run(
        ["xacro", str(XACRO), *xacro_args],
        check=True,
        capture_output=True,
        text=True,
    )
    return ET.fromstring(result.stdout)


def _joints_by_name(root):
    return {joint.get("name"): joint for joint in root.findall("joint")}


def _hardware_params(control):
    hardware = control.find("hardware")
    return {
        param.get("name"): param.text
        for param in hardware.findall("param")
    }


def test_default_model_is_complete_and_uses_three_fake_systems():
    root = _expand()
    link_names = [link.get("name") for link in root.findall("link")]
    joint_names = [joint.get("name") for joint in root.findall("joint")]
    controls = root.findall("ros2_control")

    assert len(link_names) == len(set(link_names))
    assert len(joint_names) == len(set(joint_names))
    assert {control.get("name") for control in controls} == {
        "MarvinBimanualArmHardware",
        "LeftPikaGripperHardware",
        "RightPikaGripperHardware",
    }
    assert {
        control.find("hardware/plugin").text for control in controls
    } == {"mock_components/GenericSystem"}

    assert {f"Joint{number}_{side}" for side in ("L", "R") for number in range(1, 8)} <= set(
        joint_names
    )
    assert {
        "left_gripper_left_joint",
        "left_gripper_right_joint",
        "right_gripper_left_joint",
        "right_gripper_right_joint",
    } <= set(joint_names)


def test_each_pika_is_attached_to_the_matching_marvin_flange():
    joints = _joints_by_name(_expand())

    assert joints["left_pika_adaptor_joint"].find("parent").get("link") == "flange_L"
    assert joints["left_pika_adaptor_joint"].find("child").get("link") == (
        "left_pika_adaptor_link"
    )
    assert joints["left_pika_gripper_mount_joint"].find("parent").get("link") == (
        "left_pika_adaptor_link"
    )
    assert joints["right_pika_adaptor_joint"].find("parent").get("link") == "flange_R"
    assert joints["right_pika_adaptor_joint"].find("child").get("link") == (
        "right_pika_adaptor_link"
    )
    assert joints["right_pika_gripper_mount_joint"].find("parent").get("link") == (
        "right_pika_adaptor_link"
    )


def test_real_hardware_arguments_reach_the_three_plugins():
    root = _expand(
        "use_fake_arm_hardware:=false",
        "use_fake_left_gripper_hardware:=false",
        "use_fake_right_gripper_hardware:=false",
        "robot_ip:=192.0.2.10",
        "left_gripper_serial_port:=/dev/test_pika_left",
        "right_gripper_serial_port:=/dev/test_pika_right",
    )
    controls = {control.get("name"): control for control in root.findall("ros2_control")}

    assert controls["MarvinBimanualArmHardware"].find("hardware/plugin").text == (
        "marvin_hardware_interface/MarvinBimanualArmHardware"
    )
    assert _hardware_params(controls["MarvinBimanualArmHardware"])["robot_ip"] == (
        "192.0.2.10"
    )

    for name, serial_port in (
        ("LeftPikaGripperHardware", "/dev/test_pika_left"),
        ("RightPikaGripperHardware", "/dev/test_pika_right"),
    ):
        assert controls[name].find("hardware/plugin").text == (
            "pika_gripper_hardware_interface/PikaGripperInterface"
        )
        assert _hardware_params(controls[name])["serial_port"] == serial_port


def test_ros2_control_can_be_omitted_for_description_only_consumers():
    root = _expand("ros2_control:=false")

    assert root.findall("ros2_control") == []
    assert root.find(".//link[@name='left_pika_gripper_tcp']") is not None
    assert root.find(".//link[@name='right_pika_gripper_tcp']") is not None
