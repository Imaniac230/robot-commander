import threading as th
from robot_commander_library.ai_interface import openai_interface as oai


def voice_prompted_image():
    ai = oai.OpenAI(oai.OpenAIParams(chat_model="gpt-4o",
                                     chat_initial_prompt=
                                     "You are the conversational part of an image generating agent that will be "
                                     "creating images from prompts. You do not create the images yourself, but act "
                                     "as if you can and provide witty responses to the prompting user."))
    ai.synthesize(ai.respond("Hi, image wizard..."))

    while True:
        try:
            print("Hold F10 to record your prompt ...")
            prompt = ai.transcribe()
            t = th.Thread(target=ai.generate_image, kwargs={"prompt": prompt})
            t.start()
            ai.synthesize(ai.respond(prompt))
            t.join()
        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"Failed to complete all tasks, error: {e}")
            continue


if __name__ == "__main__":
    voice_prompted_image()
