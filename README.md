# Robot Commander

## Requirements

### Local models

This project is currently conceived for inference usage only. If you want to fine-tune any of the models, you will have to research and implement the required steps yourself.

Local model support will be relying heavily on [llama.cpp](https://github.com/ggerganov/llama.cpp), [whisper.cpp](https://github.com/ggerganov/whisper.cpp), and their derivatives when possible. The comon interface of this project is created in python, however, to also allow easy integration and testing of native pytorch models and ML libraries.

### External API

This project currently supports requests to public OpenAI and Anthropic API servers. To be able to access their APIs you must have an account at [OpenAI](https://auth.openai.com/authorize?issuer=auth0.openai.com&client_id=DRivsnm2Mu42T3KOpqdtwB3NYviHYzwD&audience=https%3A%2F%2Fapi.openai.com%2Fv1&redirect_uri=https%3A%2F%2Fplatform.openai.com%2Fauth%2Fcallback&device_id=a903c544-9857-457c-b6e4-6368120a61bf&max_age=0&scope=openid+profile+email+offline_access&response_type=code&response_mode=query&state=NWxSdi5CNmxQZGpfekFIc0o5QnJVTmlaekhvTHdDMkdZSUh5OGp2RHB2Nw%3D%3D&nonce=R2pzTnVaV3FXQl9FQWZWLmZuUH5RUWU2a29qY3EwQWVLZnV3TjFCMmh3aQ%3D%3D&code_challenge=XEjn1HVfmPdUSE-8GAuCi0WrvryWEWqSWtI82gt4BFQ&code_challenge_method=S256&auth0Client=eyJuYW1lIjoiYXV0aDAtc3BhLWpzIiwidmVyc2lvbiI6IjEuMjEuMCJ9&flow=control), and/or [Anthropic](https://console.anthropic.com/login) with a valid payment method set.

## Setup

### Build

1. Install system dependencies:
   ```bash
   sudo apt install ninja-build portaudio19-dev
   ```
   >NOTE: Also make sure you have installed ROS and the following dependencies: `python3-colcon-common-extensions`, `python3-rosdep`, and `python3-vcstool`.
2. Download this repo and update ROS dependencies:
   ```
   git clone https://github.com/Imaniac230/robot-commander.git && . /opt/ros/<ros-distro>/setup.bash && rosdep install --from-paths robot-commander/ --ignore-src -r -y
   ```
3. This project relies on the independent `robot_commander_library` package located in `library_vendor/`. The other external projects listed in `library_vendor/libraries.repos` are required for usage with local models only (if you're only going to make requests to external servers, you only need `robot_commander_library`). The `input` package is also optional, and provides a gamepad interface with support for additional interactions with the DualSense PS5 controller. To download all the external libraries use the `library_vendor_setup.sh` script (or run the included commands manually):
   ```bash
   cd robot-commander/ && ./library_vendor_setup.sh
   ```
4. Build the packages:
   ```bash
    colcon build
   ```
   >NOTE: If you want to build the .cpp libraries with CUDA support enable the USE_CUDA option:
   ```bash
   colcon build --cmake-args -DUSE_CUDA=ON
   ```
   >NOTE: If you want to compile `llama.cpp` and `whisper.cpp` with different options than defined in this project, please refer to their respective instructions for more detailed compilation steps.
5. Source the current ROS workspace:
   ```bash
   . install/setup.bash
   ```

### Local models

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
   python3 whisper-convert-pt-to-ggml.py <path-to-downloaded-model-file>/large-v3.pt <path-to-official-whisper-repo> <path-to-converted-file> && mv <path-to-converted-file>/ggml-model.bin <path-to-converted-file>/ggml-model-f16-large-v3.bin
   ```
   > To convert the model downloaded from huggingface, use `convert-h5-to-ggml.py` instead, ex:
   > ```
   > python3 whisper-convert-h5-to-ggml.py <path-to-downloaded-model-files> <path-to-official-whisper-repo> <path-to-converted-file> && mv <path-to-converted-file>/ggml-model.bin <path-to-converted-file>/ggml-model-f16-large-v3.bin
   > ```
4. Quantize the converted model, ex.:
   ```
   whisper-quantize <path-to-converted-file>/ggml-model-f16-large-v3.bin <path-to-quantized-file>/ggml-model-q4_0-large-v3.bin q4_0
   ```

 ***Llama3***
1. Download one of the official models from Meta on huggingface, ex.:
   ```
   git clone https://huggingface.co/meta-llama/Meta-Llama-3-8B-Instruct
   ```
   > Make sure that you have `git lfs` installed: `git lfs install`
2. Use the `convert_hf_to_gguf.py` script from `llama.cpp` to convert, ex.:
   ```
   python3 llama-convert_hf_to_gguf.py <path-to-downloaded-model-files> --outtype f16
   ```
3. Quantize the converted model, ex.:
   ```
   llama-quantize <path-to-converted-file>/ggml-model-f16.gguf <path-to-quantized-file>/ggml-model-q4_0.gguf Q4_0
   ```

***Bark***
1. Download the official model files from SunoAI on huggingface, ex:
   ```
   git clone https://huggingface.co/suno/bark
   ```
   > Make sure that you have `git lfs` installed: `git lfs install`
> NOTE: The GGML format and quantization for bark are currently only experimental. Use the full pytorch models if you want the best results.
2. Use the `convert.py` script from `bark.cpp` to convert, ex.:
   ```
   python3 bark-convert.py --dir-model <path-to-downloaded-model-files> --use-f16 && mv <path-to-converted-file>/ggml_weights.bin <path-to-converted-file>/ggml-model-f16.bin
   ```
3. Quantize the converted model, ex.:
   ```
   bark-quantize <path-to-converted-file>/ggml-model-f16.bin <path-to-quantized-file>/ggml-model-q4_0.bin q4_0
   ```

## Usage

1. Specify your "keyword" contexts for the `PoseStamped` ROS message in `messages/contexts/posestamped.txt`.
2. Specify your commander parameters in `params/commander_params.yaml`.
3. Launch the ROS commanders:
   ```bash
   SUNO_OFFLOAD_CPU=True SUNO_USE_SMALL_MODELS=True ros2 launch robot_commander_py commanders.launch.py
   ```
   > If you have enough memory to hold the full Bark models, you can disable the `SUNO_USE_SMALL_MODELS` option. If you want to use CUDA support with Bark, you can disable the `SUNO_OFFLOAD_CPU` option. Please refer to the [Bark](https://github.com/suno-ai/bark?tab=readme-ov-file#how-much-vram-do-i-need) README for more details.

   > NOTE: Local quantized Bark TTS agent server is currently only experimental. For best results, use the raw pytorch models, which do not support a server mode, and must be loaded dynamically for each request. To use the TTS server host, disable the `use_pytorch` parameter for the chat commander `text_to_speech` section.
4. Start providing voice commands in natural language using a "push-to-talk" interface exposed through the `/record_prompt` topic.
   >NOTE: If you want to use the included gamepad interface node, use the `launch_with_gamepad:=True` argument.

### Local model servers

To launch the locally hosted agent servers:
1. Specify your "keyword" contexts for the `PoseStamped` ROS message in `messages/contexts/posestamped.txt`.
2. Specify your agent parameters in `params/agent_params.yaml`.
3. Launch the local agent servers:
   ```bash
   ros2 launch robot_commander_py agents.launch.py
   ```

## Results

>TODO: add some example demonstration results (video?)

## Other examples

Some other examples, such as the `voice_prompted_image.py` script use a wrapper around the `openai` python library. If you want to use that, you will also have to install the library: `pip install openai`.
