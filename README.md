# Robot Commander

>TODO: Update outdated README:
> * add usage examples for the local interfaces in addition to the openai API
> * add some example demonstration results (video?)

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
>NOTE: If you want to compile `llama.cpp` and `whisper.cpp` with different options, please refer to their respective instructions for more detailed compilation steps.

#### Download and quantize

This project requires all models to be downloaded and/or quantized manually before running any examples. To get the most comprehensive and up-to-date instructions, please always follow the instructions in the [llama.cpp](https://github.com/ggerganov/llama.cpp?tab=readme-ov-file#prepare-and-quantize) and [whisper.cpp](https://github.com/ggerganov/whisper.cpp/tree/master/models#whisper-model-files-in-custom-ggml-format) README. Here are basic simplified example steps to prepare the `whisper`, `llama3`, and `bark` models for use:

***Whisper***
1. Download the official `whisper` repo from OpenAI:
   ```
   git clone https://github.com/openai/whisper.git
   ```
2. Download one of the official models from OpenAI, ex:
   ```
   wget https://openaipublic.azureedge.net/main/whisper/models/e5b1a55b89c1367dacf97e3e19bfd829a01529dbfdeefa8caeb59b3f1b81dadb/large-v3.pt
   ```
   > Or download from OpenAI on huggingface, ex:
   > ```
   > git clone https://huggingface.co/openai/whisper-large-v3
   > ```
   > Make sure that you have `git lfs` installed: `git lfs install`
3. Use the `convert-pt-to-ggml.py` script from `whisper.cpp` to convert, ex:
   ```
   python libs/whisper_cpp/models/convert-pt-to-ggml.py <path-to-downloaded-model-file>/large-v3.pt <path-to-official-whisper-repo> <path-to-converted-file> && mv <path-to-converted-file>/ggml-model.bin <path-to-converted-file>/ggml-model-f16-large-v3.bin
   ```
   > To convert the model downloaded from huggingface, use `convert-h5-to-ggml.py` instead, ex:
   > ```
   > python libs/whisper_cpp/models/convert-h5-to-ggml.py <path-to-downloaded-model-files> <path-to-official-whisper-repo> <path-to-converted-file> && mv <path-to-converted-file>/ggml-model.bin <path-to-converted-file>/ggml-model-f16-large-v3.bin
   > ```
4. Quantize the converted model, ex.:
   ```
   ./libs/whisper_cpp/build/bin/quantize <path-to-converted-file>/ggml-model-f16-large-v3.bin <path-to-quantized-file>/ggml-model-q4_0-large-v3.bin q4_0
   ```

 ***Llama3***
1. Download one of the official models from Meta on huggingface, ex.:
   ```
   git clone https://huggingface.co/meta-llama/Meta-Llama-3-8B-Instruct
   ```
   > Make sure that you have `git lfs` installed: `git lfs install`
2. Use the `convert-hf-to-gguf.py` script from `llama.cpp` to convert, ex.:
   ```
   python libs/llama_cpp/convert-hf-to-gguf.py <path-to-downloaded-model-files> --outtype f16
   ```
3. Quantize the converted model, ex.:
   ```
   ./libs/llama_cpp/build/bin/quantize <path-to-converted-file>/ggml-model-f16.gguf <path-to-quantized-file>/ggml-model-q4_0.gguf Q4_0
   ```

***Bark***
1. Download the official model files from SunoAI on huggingface, ex:
   ```
   git clone https://huggingface.co/suno/bark
   ```
   > Make sure that you have `git lfs` installed: `git lfs install`
> The GGML format and quantization for bark are currently not supported in this project.

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