from __future__ import annotations

import base64
import binascii
import json
from pathlib import Path

from sensor_msgs.msg import CompressedImage


def decode_image_data_url(image_data_url: str) -> tuple[str, bytes] | None:
    raw = image_data_url.strip()
    if not raw.startswith("data:image/"):
        return None
    header, separator, payload = raw.partition(",")
    if separator != ",":
        return None
    if ";base64" not in header:
        return None
    mime = header[len("data:") : header.index(";base64")]
    image_format = mime.split("/")[-1] or "jpeg"
    try:
        image_bytes = base64.b64decode(payload, validate=True)
    except (binascii.Error, ValueError):
        return None
    if not image_bytes:
        return None
    return image_format, image_bytes


def encode_compressed_image_data_url(msg: CompressedImage) -> str:
    if not msg.data:
        return ""
    image_format = (msg.format or "jpeg").strip().lower()
    if "/" in image_format:
        image_format = image_format.split("/")[-1]
    if image_format == "jpg":
        image_format = "jpeg"
    encoded = base64.b64encode(bytes(msg.data)).decode("ascii")
    return f"data:image/{image_format};base64,{encoded}"


def load_demo_measure_image_data_url(current_file: str) -> str | None:
    repo_root = Path(current_file).resolve().parents[2]
    demo_image_path = repo_root / "extender_ui" / "src" / "assets" / "image_measures.png"
    if not demo_image_path.is_file():
        return None

    try:
        image_bytes = demo_image_path.read_bytes()
    except OSError:
        return None

    if not image_bytes:
        return None

    encoded = base64.b64encode(image_bytes).decode("ascii")
    return f"data:image/png;base64,{encoded}"


def is_legacy_fake_measure_vectors(vectors_json: str | None) -> bool:
    if not vectors_json:
        return False
    try:
        parsed = json.loads(vectors_json)
    except json.JSONDecodeError:
        return False
    source = parsed.get("source") if isinstance(parsed, dict) else None
    return isinstance(source, str) and source.startswith("fake_opencv")
