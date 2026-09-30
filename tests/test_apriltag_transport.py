"""Composite-message transport contracts, using generated ROS messages; no UDP host."""

from io import BytesIO
from pathlib import Path
import xml.etree.ElementTree as ET

import rospy
from apriltag_ros.msg import AprilTagDetection, AprilTagDetectionArray
from cooperation_landing.msg import GR2UAV_1, GR2UAV_2, UAV2GR_1, UAV2GR_2
from jsk_network_tools.silverhammer_util import (
    LargeDataUDPPacket, decomposeLargeMessage, separateBufferIntoPackets,
    subscribersFromMessage,
)


ROOT = Path(__file__).resolve().parents[1]


def test_detection_field_mapping_and_other_fields():
    assert GR2UAV_2.__slots__ == ['qilin__tag_detections']
    assert GR2UAV_2._slot_types == ['apriltag_ros/AprilTagDetectionArray']
    assert subscribersFromMessage(GR2UAV_2()) == [
        ('/qilin/tag_detections', AprilTagDetectionArray)]
    assert dict(subscribersFromMessage(GR2UAV_1())) == {
        '/xuanwu/target_pose/info': type(GR2UAV_1().xuanwu__target_pose__info),
        '/xuanwu/servo/target_states/info':
        type(GR2UAV_1().xuanwu__servo__target_states__info),
    }
    assert GR2UAV_1.__slots__ == [
        'xuanwu__target_pose__info', 'xuanwu__servo__target_states__info']
    assert UAV2GR_1.__slots__ == [
        'xuanwu__uav__cog__odom', 'xuanwu__flight_state',
        'xuanwu__servo__states', 'xuanwu__mocap__pose',
        'xuanwu__visual_landing__state']
    assert UAV2GR_2.__slots__ == ['xuanwu__tag_detections']
    assert '/xuanwu/visual_landing/state' in decomposeLargeMessage(UAV2GR_1())
    assert '/xuanwu/tag_detections' in decomposeLargeMessage(UAV2GR_2())
    assert not any('visual_landing__info' in field for cls in (GR2UAV_1, GR2UAV_2)
                   for field in cls.__slots__)


def test_full_detection_serialization_and_empty_frame():
    observation = AprilTagDetectionArray()
    observation.header.seq = 21
    observation.header.stamp = rospy.Time(123, 456)
    observation.header.frame_id = 'usb_cam'
    detection = AprilTagDetection()
    detection.id = [0, 1]
    detection.size = [0.19, 0.21]
    detection.pose.header.seq = 22
    detection.pose.header.stamp = rospy.Time(123, 455)
    detection.pose.header.frame_id = 'usb_cam'
    pose = detection.pose.pose.pose
    pose.position.x, pose.position.y, pose.position.z = 0.1, -0.2, 1.3
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = 0.1, 0.2, 0.3, 0.9
    detection.pose.pose.covariance = [float(i) / 10 for i in range(36)]
    observation.detections = [detection]
    for original in (observation, AprilTagDetectionArray(header=observation.header)):
        encoded = GR2UAV_2(qilin__tag_detections=original)
        buffer = BytesIO()
        # Exercise the same ROS framing and packet helpers used by Silverhammer.
        rospy.msg.serialize_message(buffer, 0, encoded)
        packets = separateBufferIntoPackets(7, buffer.getvalue(), 80)
        if original.detections:
            assert len(packets) > 1
        received_packets = [LargeDataUDPPacket.fromData(packet.pack(), 80)
                            for packet in packets]
        payload = b''.join(packet.data for packet in received_packets)
        decoded_messages = []
        receive_buffer = BytesIO()
        receive_buffer.write(payload)  # Receiver leaves the cursor at the end.
        rospy.msg.deserialize_messages(receive_buffer, decoded_messages, GR2UAV_2)
        decoded = decoded_messages[0]
        received = decomposeLargeMessage(decoded)['/qilin/tag_detections']
        before, after = BytesIO(), BytesIO()
        original.serialize(before)
        received.serialize(after)
        assert after.getvalue() == before.getvalue()
        assert received.header == original.header
        assert len(received.detections) == len(original.detections)


def test_network_launches_only_transport_and_ground_legacy_is_opt_in():
    for name in ('GR2UAV.launch', 'UAV2GR.launch'):
        launch = ET.parse(ROOT / 'launch' / name).getroot()
        assert not launch.findall(".//node[@type='apriltag_relative_pose.py']")
        assert not launch.findall(".//node[@type='visual_landing_controller.py']")
    ground = ET.parse(ROOT / 'launch/apriltag_relative_pose.launch').getroot()
    assert ground.find("arg[@name='start_estimator']").get('default') == 'false'
    assert ground.find("node[@type='apriltag_relative_pose.py']").get('if') == '$(arg start_estimator)'
    receiver = ET.parse(ROOT / 'launch/UAV2GR.launch').getroot().find("node[@name='receiver_2']")
    assert receiver.find("param[@name='message']").get('value') == 'cooperation_landing/GR2UAV_2'
    assert not receiver.findall("rosparam[@param='timestamp_overwrite_topics']")
    assert not receiver.findall("rosparam[@param='publish_only_if_updated_topics']")
