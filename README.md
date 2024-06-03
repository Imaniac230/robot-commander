# Robot Commander

>TODO: Update outdated README:
> * add steps for preparing all dependency libraries (llama_cpp, whisper_cpp, bark)
> * add notes for downloading and preparing local models
> * add examples for using the local interfaces in addition to the openai API

## Requirements

### Local models

This project is currently conceived for inference usage only. If you want to fine-tune any of the models, you will have to research and implement the required steps yourself.

Local model support will be relying heavily on [llama.cpp](https://github.com/ggerganov/llama.cpp), [whisper.cpp](https://github.com/ggerganov/whisper.cpp), and their derivatives when possible. The comon interface of this project is created in python, however, to also allow easy integration and testing of native pytorch models and ML libraries.

### External API

This project is currently developed to use the public openai API server requests. To be able to access their API you must have an account at [OpenAI](https://openai.com) with a valid payment method set.

## Setup

### ROS

To integrate with ROS2, the current proof-of-concept examples simply utilize the `roslibpy` library together with `rosbridge`. In order to use all the current examples, you should install the following dependencies:
```bash
sudo apt install portaudio19-dev && pip install roslibpy pyaudio openai
```

and the ros bridge:
```
sudo apt install ros-<ros-distro>-rosbridge-suite
```

### Local models

#### Build

To install and build the main required libraries, run the setup script:
```bash
./setup.sh
```
>NOTE: If you want to compile `llama.cpp` and `whisper.cpp` with different options, please refer to their respective documentation for more detailed compilation explanations.

#### Download and quantize

This project requires all models to be downloaded and/or quantized manually before running any examples.
>TODO: Add simplified steps

### External API

To use the external openai API examples, export your API key to your environment:
```
export OPENAI_API_KEY=<secret-key>
```
or use it directly when running the examples.

## Usage
>TODO: This section is outdated and will be updated once the project is more fledged.
* source ROS and start the ros bridge node:
```
. /opt/ros/<ros-distro>/setup.bash && ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

* start the commander:
```
python3 voice_control.py --topic <ros-topic-name> --type <ros-message-type>
```

* start providing voice commands in your native natural language and be explicit rather than too abstract