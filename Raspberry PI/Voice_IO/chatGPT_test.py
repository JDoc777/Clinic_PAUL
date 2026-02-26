from openai import OpenAI
client = OpenAI(api_key="")

client = OpenAI()

# def gpt5_nano_process(command_text: str) -> str:
#     """
#     Sends text to GPT-5 nano and returns a short, speakable reply.
#     """
#     try:
#         resp = client.responses.create(
#             model="gpt-5-nano",   # ✅ GPT-5 Nano
#             input=[
#                 {
#                     "role": "system",
#                     "content": (
#                         "You are PAUL, a voice-controlled assistant. "
#                         "You only receive commands after the wake phrase 'Hey Paul'. "
#                         "Respond briefly and clearly in plain text."
#                     )
#                 },
#                 {"role": "user", "content": command_text},
#             ],
#         )

#         return (resp.output_text or "").strip()

#     except Exception as e:
#         print("GPT error:", e)
#         return "Sorry, something went wrong."

from openai import OpenAI

client = OpenAI()

r = client.responses.create(
    model="gpt-5-nano",
    input="Say hello"
)

print(r.output_text)