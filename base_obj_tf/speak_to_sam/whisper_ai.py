import whisper
import torch 

class STT():
    def __init__(self):
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.model = None

    def load_model(self, weight="large"):
        self.model = whisper.load_model(weight, device=self.device)

    def get_stt(self, audio_data) -> str:
        result = self.model.transcribe(
            str(audio_data),
            language="ko",
            task="transcribe",
            fp16=(self.device == "cuda"),
            verbose=True,
        )
        return result["text"].strip()
