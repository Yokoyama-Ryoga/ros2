import ollama

MODEL = "gemma3:1b"

response1 = ollama.generate(
    model=MODEL, prompt="What is the capital of France? (Only capital name)"
)
response2 = ollama.generate(
    model=MODEL, prompt="And what about Germany? (Only capital name)"
)

print(response1["response"])
print(response2["response"])
