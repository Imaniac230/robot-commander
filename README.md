# robot-voice-command

## Requirements
To be able to access the openai API you must have an account at [OpenAI](https://openai.com) with a valid payment method set 💩.

## Support
The `geometry_msgs/PoseStamped` and `geometry_msgs/Twist` message types should work most of the time.

## Setup
* define your openai API key:
```
export OPENAI_API_KEY=<secret-key>
```

* install required dependencies:
```bash
sudo apt install portaudio19-dev && pip install roslibpy pyaudio openai
```

* install the ros bridge:
```
sudo apt install ros-<ros-distro>-rosbridge-suite
```

## Usage
* source ROS and start the ros bridge node:
```
. /opt/ros/<ros-distro>/setup.bash && ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

* start the commander:
```
python3 voice_control.py --topic <ros-topic-name> --type <ros-message-type>
```

* start providing voice commands in your native natural language and be explicit rather than too abstract