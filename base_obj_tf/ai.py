"""이전 import 경로를 사용하는 코드를 위한 호환 파일.

새 코드는 ``base_obj_tf.py``를 실행한다. 모델과 카메라가 필요하면
``rf_detr_model.py``와 ``realsense_data.py``에서 직접 import한다.
"""

from realsense_data import RealSenseCamera, Realsense_Data
from rf_detr_model import CubeRFDetr

__all__ = ["CubeRFDetr", "RealSenseCamera", "Realsense_Data"]
