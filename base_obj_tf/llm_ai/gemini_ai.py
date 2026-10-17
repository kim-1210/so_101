from google import genai

import json

from typing import Literal
from pydantic import BaseModel, Field

class AnswerShape(BaseModel):
    target_obj:str = Field(description="집을 물체")
    target_place:str = Field(description="물체를 놓을 공간")


class GetTarget():
    def __init__(self):
        self.gemini = None
        self.model = "gemini-3.8-flash"
        self.default_msg = "Sam3에서 찾을 수 있는 영어단어로 해줘."

    def load_api(self, api_key:str):
        self.gemini = genai.Client(
            api_key=api_key
        )

    def set_default_msg(self, new_msg:str):
        self.default_msg = new_msg

    def get_default_msg(self):
        return self.default_msg

    def set_model(self, model):
        self.model = model

    def get_model(self,):
        return self.model

    def get_answer(self, msg:str):
        answer = self.gemini.interactions.create(
            model=self.model,
            input=f"'{msg}' {self.default_msg}",
            response_format={
                "type": "text",
                "mime_type": "application/json",
                "schema": AnswerShape.model_json_schema(), # 출력형태를 지정
            },
        )
        result = AnswerShape.model_validate_json(answer.output_text)

        return result.target_obj, result.target_place

__all__ = ['GetTarget']