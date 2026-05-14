from __future__ import annotations

import base64
import io
from typing import Optional

from PIL import Image
from sensor_msgs.msg import CompressedImage, Image as RosImage


def ros_image_to_pil(msg: RosImage | CompressedImage) -> Optional[Image.Image]:
    if isinstance(msg, CompressedImage):
        try:
            return Image.open(io.BytesIO(bytes(msg.data))).convert('RGB')
        except Exception:
            return None
    width = int(msg.width)
    height = int(msg.height)
    if width <= 0 or height <= 0:
        return None
    data = bytes(msg.data)
    try:
        if msg.encoding == 'rgb8':
            return Image.frombytes('RGB', (width, height), data)
        if msg.encoding == 'bgr8':
            return Image.frombuffer('RGB', (width, height), data, 'raw', 'BGR', 0, 1)
        if msg.encoding == 'mono8':
            return Image.frombytes('L', (width, height), data).convert('RGB')
    except Exception:
        return None
    return None


def image_to_data_url(image: Image.Image, max_side: int = 1024, quality: int = 85) -> str:
    resized = image.copy()
    resized.thumbnail((max_side, max_side))
    buf = io.BytesIO()
    resized.save(buf, format='JPEG', quality=quality)
    payload = base64.b64encode(buf.getvalue()).decode('ascii')
    return f'data:image/jpeg;base64,{payload}'
