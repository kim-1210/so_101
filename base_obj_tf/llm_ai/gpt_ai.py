from openai import OpenAI

import json

from typing import Literal, List
from pydantic import BaseModel, Field

class AnswerShape(BaseModel):
    target_obj:List[str] = Field(description="집을 물체들")
    target_place:str = Field(description="물체를 놓을 공간")


class GetTarget():
    def __init__(self):
        self.gpt = None
        self.model = "gpt-4o-mini"
        self.default_msg = (
            "사용자의 한국어 명령에서 집을 물체와 놓을 장소를 찾아줘. "
            "SAM3에서 찾기 쉬운 영어 단어로만 답해줘."
        )

    def load_api(self, api_key:str):
        self.gpt = OpenAI(api_key=api_key)

    def set_default_msg(self, new_msg:str):
        self.default_msg = new_msg

    def get_default_msg(self):
        return self.default_msg

    def set_model(self, model):
        self.model = model

    def get_model(self,):
        return self.model

    def get_answer(self, msg:str):
        if self.gpt is None:
            raise ValueError("API 키가 로드되지 않았습니다. load_api()를 먼저 호출하세요.")

        # beta.chat.completions.parse를 사용하여 Pydantic 스키마를 직접 주입합니다.
        response = self.gpt.beta.chat.completions.parse(
            model=self.model,
            messages=[
                {
                    "role": "system", 
                    "content": f"지시사항: {self.default_msg}"
                },
                {
                    "role": "user", 
                    "content": msg
                }
            ],
            response_format=AnswerShape, # Pydantic 모델 클래스를 그대로 입력
        )
        
        result = response.choices[0].message.parsed
        
        if result is None:
            raise ValueError("응답을 지정된 형식으로 파싱하지 못했습니다.")

        return result.target_obj, result.target_place

__all__ = ['GetTarget']