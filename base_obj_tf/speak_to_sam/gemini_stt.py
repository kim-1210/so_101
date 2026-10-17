from pathlib import Path

from google import genai
from google.genai import types


class GeminiSTT:
    def __init__(self, model="gemini-2.5-flash"):
        self.client = None
        self.model = model
        self.prompt = "이 음성 파일을 한국어 문장으로 받아쓰기해줘. 답은 받아쓰기 결과만 줘."

    def load_api(self, api_key: str):
        self.client = genai.Client(api_key=api_key)

    def set_model(self, model: str):
        self.model = model

    def set_prompt(self, prompt: str):
        self.prompt = prompt

    def get_stt(self, audio_path) -> str:
        if self.client is None:
            raise ValueError("API 키가 로드되지 않았습니다. load_api()를 먼저 호출하세요.")

        audio_path = Path(audio_path)
        audio_part = types.Part.from_bytes(
            data=audio_path.read_bytes(),
            mime_type=self._get_mime_type(audio_path),
        )

        response = self.client.models.generate_content(
            model=self.model,
            contents=[self.prompt, audio_part],
        )
        return response.text.strip()

    @staticmethod
    def _get_mime_type(audio_path: Path) -> str:
        suffix = audio_path.suffix.lower()
        mime_types = {
            ".wav": "audio/wav",
            ".mp3": "audio/mpeg",
            ".m4a": "audio/mp4",
            ".aac": "audio/aac",
            ".ogg": "audio/ogg",
            ".flac": "audio/flac",
            ".webm": "audio/webm",
        }
        return mime_types.get(suffix, "audio/wav")


__all__ = ["GeminiSTT"]
