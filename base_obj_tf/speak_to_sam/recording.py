import signal
import shutil
import subprocess
from datetime import datetime
from pathlib import Path


class AudioRecorder:
    def __init__(self, output_dir="recordings", sample_rate=16000, channels=1, device="default"):
        self.output_dir = Path(output_dir)
        self.sample_rate = sample_rate
        self.channels = channels
        self.device = device
        self.process = None
        self.audio_path = None

    @property
    def is_recording(self):
        return self.process is not None and self.process.poll() is None

    def start(self):
        if self.is_recording:
            return self.audio_path

        if shutil.which("arecord") is None:
            raise RuntimeError("arecord 명령을 찾지 못했습니다. Ubuntu라면 alsa-utils가 필요합니다.")

        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.audio_path = self.output_dir / f"recording_{datetime.now():%Y%m%d_%H%M%S}.wav"

        command = [
            "arecord",
            "-D",
            self.device,
            "-f",
            "S16_LE",
            "-r",
            str(self.sample_rate),
            "-c",
            str(self.channels),
            str(self.audio_path),
        ]
        self.process = subprocess.Popen(command)
        return self.audio_path

    def stop(self):
        if not self.is_recording:
            return self.audio_path

        self.process.send_signal(signal.SIGINT)
        self.process.wait()
        self.process = None
        return self.audio_path

    def toggle(self):
        if self.is_recording:
            return self.stop()
        return self.start()

    def close(self):
        if self.is_recording:
            self.stop()


def make_record_button(recorder=None):
    import ipywidgets as widgets
    from IPython.display import Audio, clear_output, display

    recorder = recorder or AudioRecorder()
    button = widgets.Button(description="녹음 시작", button_style="success", icon="microphone")
    out = widgets.Output()

    def on_click(_):
        if recorder.is_recording:
            audio_path = recorder.stop()
            button.description = "녹음 시작"
            button.button_style = "success"
            button.icon = "microphone"
            with out:
                clear_output()
                print(f"저장 완료: {audio_path}")
                display(Audio(str(audio_path)))
        else:
            recorder.start()
            button.description = "녹음 중지"
            button.button_style = "danger"
            button.icon = "stop"

    button.on_click(on_click)
    display(button, out)
    return recorder, button, out
