"""Render NumPy | GL | TRUTH montages for visual GL verification.

Usage: python3 scripts/gl_montage.py  ->  writes gl_montage.png
"""

import sys
import os

# Ensure the project root is on sys.path when run as a script from any cwd.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

from tpsprojector.app import Engine
from tpsprojector.gl_context import gl_available
from tpsprojector.presets import PRESET_NAMES, get_preset
from tpsprojector.validate import psnr


def _to_u8(img):
    return (np.clip(img, 0, 1) * 255).astype(np.uint8)


def main():
    assert gl_available(), "no GL context — cannot build GL montage"
    W, H, mode = 240, 180, "bowl"
    np_eng = Engine.from_defaults(width=W, height=H, mode=mode, backend="numpy")
    gl_eng = Engine.from_defaults(width=W, height=H, mode=mode, backend="gl")

    rows = []
    for name in PRESET_NAMES[:4]:
        shot = get_preset(name)
        npr = np_eng.synthesize(shot)
        glr = gl_eng.synthesize(shot)
        p = psnr(glr.synth, npr.synth, np.ones(glr.synth.shape[:2], bool))
        row = np.concatenate([_to_u8(npr.synth), _to_u8(glr.synth),
                              _to_u8(npr.truth)], axis=1)
        rows.append(row)
        print(f"{name:10s}  GL-vs-NumPy PSNR = {p:5.2f} dB")
    montage = np.concatenate(rows, axis=0)

    try:
        from PIL import Image
        Image.fromarray(montage).save("gl_montage.png")
    except ImportError:
        import pygame
        pygame.image.save(
            pygame.surfarray.make_surface(np.transpose(montage, (1, 0, 2))),
            "gl_montage.png")
    print("wrote gl_montage.png  (cols: NumPy | GL | TRUTH)")


if __name__ == "__main__":
    main()
