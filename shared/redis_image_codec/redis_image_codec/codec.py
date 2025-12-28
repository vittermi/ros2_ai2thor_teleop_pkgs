from __future__ import annotations

from typing import Any

import time
import numpy as np
import cv2
import msgpack
import lz4.frame


class RedisImageCodec:

    def __init__(
        self,
        resize = 144,
        jpeg_quality: int = 70,
        depth_max_m: float = 10.0,
        depth_quantization: str = "mm",
        lz4_level: int = 0,  
        add_seq: bool = True,
    ):
        self.resize = resize
        self.jpeg_quality = int(jpeg_quality)
        self.depth_max_m = float(depth_max_m)
        if depth_quantization not in ("mm", "cm"):
            raise ValueError("depth_quantization must be 'mm' or 'cm'")
        self.depth_quantization = depth_quantization
        self.lz4_level = int(lz4_level)
        self.add_seq = bool(add_seq)

        self._rgb_seq = 0
        self._depth_seq = 0


    def encode_rgb(
        self,
        rgb_frame: np.ndarray,
        timestamp: float | None,
    ) -> bytes:

        if timestamp is None:
            timestamp = time.time()

        if rgb_frame.dtype != np.uint8 or rgb_frame.ndim != 3 or rgb_frame.shape[2] != 3:
            raise ValueError("rgb_frame must be HxWx3 uint8")

        resized = self._resize_keep_aspect(rgb_frame, self.resize)

        bgr = cv2.cvtColor(resized, cv2.COLOR_RGB2BGR)

        ok, enc = cv2.imencode(
            ".jpg",
            bgr,
            [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
        )
        if not ok:
            raise RuntimeError("cv2.imencode(.jpg) failed")

        payload: dict[str, Any] = {
            "type": "rgb",
            "timestamp": float(timestamp),
            "height": int(resized.shape[0]),
            "width": int(resized.shape[1]),
            "enc": "jpeg",
            "data": enc.tobytes(),
        }

        if self.add_seq:
            self._rgb_seq += 1
            payload["seq"] = self._rgb_seq

        return msgpack.packb(payload, use_bin_type=True)

    def encode_depth(
        self,
        depth_frame_m: np.ndarray,
        timestamp: float | None,
    ) -> bytes:
        if timestamp is None:
            timestamp = time.time()

        if depth_frame_m.ndim != 2:
            raise ValueError("depth_frame_m must be HxW")

        depth = np.asarray(depth_frame_m, dtype=np.float32)

        depth_resized = self._resize_keep_aspect(depth, self.resize, is_depth=True)

        depth_q, q_unit = self._quantize_depth(depth_resized)

        raw_bytes = depth_q.tobytes(order="C")
        comp = lz4.frame.compress(raw_bytes, compression_level=self.lz4_level)

        payload: dict[str, Any] = {
            "type": "depth",
            "timestamp": float(timestamp),
            "height": int(depth_q.shape[0]),
            "width": int(depth_q.shape[1]),
            "enc": "u16-" + q_unit + "+lz4",
            "dtype": "uint16",
            "depth_max_m": self.depth_max_m,
            "data": comp,
        }

        if self.add_seq:
            self._depth_seq += 1
            payload["seq"] = self._depth_seq

        return msgpack.packb(payload, use_bin_type=True)


    @staticmethod
    def decode(payload_bytes: bytes) -> dict[str, Any]:

        msg = msgpack.unpackb(payload_bytes, raw=False)

        typ = msg.get("type")
        if typ == "rgb":
            jpg = msg["data"]
            arr = np.frombuffer(jpg, dtype=np.uint8)
            bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if bgr is None:
                raise RuntimeError("cv2.imdecode failed for rgb")
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            msg["data"] = rgb
            return msg

        if typ == "depth":
            comp = msg["data"]
            raw = lz4.frame.decompress(comp)
            h, w = int(msg["height"]), int(msg["width"])
            u16 = np.frombuffer(raw, dtype=np.uint16).reshape((h, w))

            encoding = str(msg.get("enc", ""))
            if "u16-mm" in encoding:
                depth_m = u16.astype(np.float32) / 1000.0
            elif "u16-cm" in encoding:
                depth_m = u16.astype(np.float32) / 100.0
            else:
                depth_m = u16.astype(np.float32) / 1000.0

            msg["data"] = depth_m
            return msg

        raise ValueError(f"Unknown payload type: {typ}")


    def _quantize_depth(self, depth_m: np.ndarray) -> tuple[np.ndarray, str]:
        depth = np.nan_to_num(depth_m, nan=self.depth_max_m, posinf=self.depth_max_m, neginf=0.0)
        depth = np.clip(depth, 0.0, self.depth_max_m)

        if self.depth_quantization == "mm":
            scale = 1000.0
            unit = "mm"
        else:
            scale = 100.0
            unit = "cm"

        q = np.round(depth * scale).astype(np.uint16)
        return q, unit

    @staticmethod
    def _resize_keep_aspect(
        img: np.ndarray,
        short_side_size: int,
        is_depth: bool = False,
    ) -> np.ndarray:
        h, w = img.shape[:2]
        if h <= 0 or w <= 0:
            raise ValueError("Invalid image shape")

        short = min(h, w)
        if short == short_side_size:
            return img

        scale = short_side_size / float(short)
        new_w = max(1, int(round(w * scale)))
        new_h = max(1, int(round(h * scale)))

        interpolation = cv2.INTER_NEAREST if is_depth else cv2.INTER_AREA

        resized = cv2.resize(img, (new_w, new_h), interpolation=interpolation)
        return resized
