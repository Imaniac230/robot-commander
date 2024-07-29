# Robot Commander

## Requirements

### Local models

This project is currently conceived for inference usage only. If you want to fine-tune any of the models, you will have to research and implement the required steps yourself.

Local model support will be relying heavily on [llama.cpp](https://github.com/ggerganov/llama.cpp), [whisper.cpp](https://github.com/ggerganov/whisper.cpp), and their derivatives when possible. The comon interface of this project is created in python, however, to also allow easy integration and testing of native pytorch models and ML libraries.

### External API

This project is currently developed to use the public openai API server requests. To be able to access their API you must have an account at [OpenAI](https://openai.com) with a valid payment method set.

## Setup

Install dependencies:
```bash
sudo apt install portaudio19-dev && pip install pyaudio
```

### Local models

#### Build

To install the base `robot-commander-library` project and install/build its main dependency libraries, run the setup script:
```bash
./setup.sh
```
>NOTE: This will download all external dependencies listed in `libraries.repos`, compile the cpp libraries with predefined options and install the python libraries locally. If you want to compile `llama.cpp` and `whisper.cpp` with different options, please refer to their respective instructions for more detailed compilation steps. Exporting environment variables `ROBOT_COMMANDER_WHISPER_CPP_PATH`, `ROBOT_COMMANDER_LLAMA_CPP_PATH`, and `ROBOT_COMMANDER_BARK_CPP_PATH` will also be setup in `~/.bashrc`.

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
   python3 "$ROBOT_COMMANDER_WHISPER_CPP_PATH"/models/convert-pt-to-ggml.py <path-to-downloaded-model-file>/large-v3.pt <path-to-official-whisper-repo> <path-to-converted-file> && mv <path-to-converted-file>/ggml-model.bin <path-to-converted-file>/ggml-model-f16-large-v3.bin
   ```
   > To convert the model downloaded from huggingface, use `convert-h5-to-ggml.py` instead, ex:
   > ```
   > python3 "$ROBOT_COMMANDER_WHISPER_CPP_PATH"/models/convert-h5-to-ggml.py <path-to-downloaded-model-files> <path-to-official-whisper-repo> <path-to-converted-file> && mv <path-to-converted-file>/ggml-model.bin <path-to-converted-file>/ggml-model-f16-large-v3.bin
   > ```
4. Quantize the converted model, ex.:
   ```
   "$ROBOT_COMMANDER_WHISPER_CPP_PATH"/build/bin/quantize <path-to-converted-file>/ggml-model-f16-large-v3.bin <path-to-quantized-file>/ggml-model-q4_0-large-v3.bin q4_0
   ```

 ***Llama3***
1. Download one of the official models from Meta on huggingface, ex.:
   ```
   git clone https://huggingface.co/meta-llama/Meta-Llama-3-8B-Instruct
   ```
   > Make sure that you have `git lfs` installed: `git lfs install`
2. Use the `convert-hf-to-gguf.py` script from `llama.cpp` to convert, ex.:
   ```
   python3 "$ROBOT_COMMANDER_LLAMA_CPP_PATH"/convert_hf_to_gguf.py <path-to-downloaded-model-files> --outtype f16
   ```
3. Quantize the converted model, ex.:
   ```
   "$ROBOT_COMMANDER_LLAMA_CPP_PATH"/build/bin/llama-quantize <path-to-converted-file>/ggml-model-f16.gguf <path-to-quantized-file>/ggml-model-q4_0.gguf Q4_0
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
   python3 "$ROBOT_COMMANDER_BARK_CPP_PATH"/convert.py --dir-model <path-to-downloaded-model-files> --use-f16 && mv <path-to-converted-file>/ggml_weights.bin <path-to-converted-file>/ggml-model-f16.bin
   ```
3. Quantize the converted model, ex.:
   ```
   "$ROBOT_COMMANDER_BARK_CPP_PATH"/build/examples/quantize/quantize <path-to-converted-file>/ggml-model-f16.bin <path-to-quantized-file>/ggml-model-q4_0.bin q4_0
   ```

### ROS

1. Source ROS and build the main package:
   ```
   . /opt/ros/<ros-distro>/setup.bash && colcon build
   ```
2. Source the current package workspace:
   ```bash
   . install/setup.bash
   ```

## Usage

1. Specify your "keyword" contexts for the `PoseStamped` ROS message in `messages/contexts/posestamped.txt`.
2. Specify your commander parameters in `params/commander_params.yaml`.
3. Launch the ROS commanders:
   ```bash
   SUNO_USE_SMALL_MODELS=True ros2 launch robot_commander commanders.launch.py
   ```
   > The local raw pytorch Bark implementation will attempt to use CUDA by default, if you want run it on CPU only, also specify `SUNO_OFFLOAD_CPU=True`. If you have enough memory to hold the full Bark models, you can disable the `SUNO_USE_SMALL_MODELS` option. Please refer to the [Bark](https://github.com/suno-ai/bark?tab=readme-ov-file#how-much-vram-do-i-need) README for more details.

   > NOTE: Local quantized Bark TTS agent server is currently only experimental. For best results, use the raw pytorch models, which do not support a server mode, and must be loaded dynamically for each request. To use the TTS server host, disable the `use_pytorch` parameter for the chat commander `text_to_speech` section.
4. Start providing voice commands in natural language using a "push-to-talk" interface exposed through the `/record_prompt` topic.

### Local model servers

To launch the locally hosted agent servers:
1. Specify your "keyword" contexts for the `PoseStamped` ROS message in `messages/contexts/posestamped.txt`.
2. Specify your agent parameters in `params/agent_params.yaml`.
3. Launch the local agent servers:
   ```bash
   ros2 launch robot_commander agents.launch.py
   ```

## Results

>TODO: add some example demonstration results (video?)

## Other examples

Some other examples, such as the `voice_prompted_image.py` script use a wrapper around the `openai` python library. If you want to use that, you will also have to install the library: `pip install openai`.
