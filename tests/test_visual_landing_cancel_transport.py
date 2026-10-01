"""The explicit cancel event uses its own low-speed Silverhammer port."""

from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]


def test_start_and_cancel_have_distinct_low_speed_paths():
    ground = ET.parse(ROOT / 'launch/GR2UAV.launch').getroot()
    uav = ET.parse(ROOT / 'launch/UAV2GR.launch').getroot()
    for launch in (ground, uav):
        defaults = {arg.get('name'): int(arg.get('default'))
                    for arg in launch.findall('arg') if arg.get('name', '').startswith('LOW_PORT_')}
        assert len(defaults) == 9 and len(set(defaults.values())) == 9
        assert defaults['LOW_PORT_8'] == 1065
        assert defaults['LOW_PORT_9'] == 1066
        assert not launch.findall('.//include')
        assert all(node.get('pkg') == 'jsk_network_tools' for node in launch.findall('node'))

    streamer = ground.find("node[@name='lowspeed_streamer_visual_landing_cancel']")
    receiver = uav.find("node[@name='lowspeed_receiver_visual_landing_cancel']")
    start_streamer = ground.find("node[@name='lowspeed_streamer_visual_landing']")
    start_receiver = uav.find("node[@name='lowspeed_receiver_visual_landing']")
    assert all(node is not None for node in (streamer, receiver, start_streamer, start_receiver))
    assert streamer.get('type') == 'silverhammer_lowspeed_streamer.py'
    assert receiver.get('type') == 'silverhammer_lowspeed_receiver.py'
    assert streamer.find("remap[@from='~input']").get('to') == '/$(arg drone_ns)/visual_landing/cancel'
    assert receiver.find("remap[@from='~output']").get('to') == '/$(arg drone_ns)/visual_landing/cancel'
    assert start_streamer.find("remap[@from='~input']").get('to') == '/$(arg drone_ns)/visual_landing/trigger'
    assert start_receiver.find("remap[@from='~output']").get('to') == '/$(arg drone_ns)/visual_landing/trigger'

    ground_params = {p.get('name'): p.get('value') for p in streamer.findall('param')}
    uav_params = {p.get('name'): p.get('value') for p in receiver.findall('param')}
    assert ground_params == {
        'message': 'std_msgs/Empty', 'to_ip': '$(arg UAV_IP)',
        'to_port': '$(arg LOW_PORT_9)', 'send_rate': '1', 'event_driven': 'True',
    }
    assert uav_params == {
        'message': 'std_msgs/Empty', 'receive_ip': '$(arg UAV_IP)',
        'receive_port': '$(arg LOW_PORT_9)', 'receive_buffer_size': '1000',
    }
    assert start_streamer.find("param[@name='to_port']").get('value') == '$(arg LOW_PORT_8)'
    assert start_receiver.find("param[@name='receive_port']").get('value') == '$(arg LOW_PORT_8)'
