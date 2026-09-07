# -*- coding: utf-8 -*-
"""Cuts the primary robot out of the green field and exports the component photos."""
import os
import numpy as np
from PIL import Image, ImageFilter

SRC = r'C:\Users\amirr\AppData\Local\Temp\claude\media'


def largest_component(keep):
    h, w = keep.shape
    lab = np.where(keep, np.arange(h * w).reshape(h, w), -1)
    for _ in range(500):
        prev = lab
        m = lab.copy()
        m[1:, :] = np.maximum(m[1:, :], np.where(lab[:-1, :] >= 0, lab[:-1, :], -1) * keep[1:, :])
        m[:-1, :] = np.maximum(m[:-1, :], np.where(lab[1:, :] >= 0, lab[1:, :], -1) * keep[:-1, :])
        m[:, 1:] = np.maximum(m[:, 1:], np.where(lab[:, :-1] >= 0, lab[:, :-1], -1) * keep[:, 1:])
        m[:, :-1] = np.maximum(m[:, :-1], np.where(lab[:, 1:] >= 0, lab[:, 1:], -1) * keep[:, :-1])
        lab = np.where(keep, m, -1)
        if np.array_equal(lab, prev):
            break
    vals, counts = np.unique(lab[lab >= 0], return_counts=True)
    return lab == vals[counts.argmax()]


def cut_robot():
    im = Image.open(os.path.join(SRC, 'image20.jpeg')).convert('RGB')
    a = np.asarray(im).astype(int)
    r, g, b = a[..., 0], a[..., 1], a[..., 2]
    green = (g > r + 28) & (g > b + 18) & (g > 70)
    mask = largest_component(~green)

    # Despill: pull green back toward the red/blue average wherever it dominates.
    rgb = a.astype(float)
    cap = (rgb[..., 0] + rgb[..., 2]) / 2 * 1.12
    spill = rgb[..., 1] > cap
    rgb[..., 1] = np.where(spill, cap, rgb[..., 1])

    alpha = Image.fromarray((mask * 255).astype(np.uint8))
    # Shrink one pixel, then soften, so no keyed fringe survives.
    alpha = alpha.filter(ImageFilter.MinFilter(3)).filter(ImageFilter.GaussianBlur(0.8))

    out = Image.merge('RGBA', list(Image.fromarray(rgb.clip(0, 255).astype(np.uint8)).split()) + [alpha])
    bbox = alpha.point(lambda v: 255 if v > 8 else 0).getbbox()
    out = out.crop(bbox)
    out = out.resize((out.width * 2, out.height * 2), Image.LANCZOS)
    out.save('robot-primary.png')
    print('robot-primary.png', out.size, round(os.path.getsize('robot-primary.png') / 1024, 1), 'KB')


def part(src, dst, box=None, width=300, quality=72):
    im = Image.open(os.path.join(SRC, src))
    if im.mode != 'RGB':
        bg = Image.new('RGB', im.size, (255, 255, 255))
        rgba = im.convert('RGBA')
        bg.paste(rgba, mask=rgba.split()[-1])
        im = bg
    if box:
        im = im.crop(box)
    if im.width > width:
        im = im.resize((width, round(im.height * width / im.width)), Image.LANCZOS)
    im.save(dst, 'JPEG', quality=quality, optimize=True, progressive=True)
    print(dst, im.size, round(os.path.getsize(dst) / 1024, 1), 'KB')


cut_robot()
part('image5.jpeg', 'p-chassis.jpg', box=(120, 280, 900, 1080), width=300)
part('image10.jpeg', 'p-wheel.jpg', width=300)
part('image12.jpeg', 'p-motor.jpg', width=300)
part('image8.png', 'p-pcb.jpg', box=(272, 82, 872, 568), width=380)
part('image11.png', 'p-stm32.jpg', width=300)
part('image13.jpeg', 'p-driver.jpg', width=300)
part('image14.jpeg', 'p-tsop.jpg', width=300)
part('image17.jpeg', 'p-sharp.jpg', width=300)
part('image3.png', 'robocup-logo.jpg', width=300, quality=80)
