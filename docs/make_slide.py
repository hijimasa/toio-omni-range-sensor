#!/usr/bin/env python3
"""概要スライド（16:9・1 枚）を生成する。

事前に docs/make_figs.py を実行して figs/*.png を作っておくこと。
出力: docs/toio_omni_range_overview.pptx
"""
import os

from PIL import Image
from pptx import Presentation
from pptx.dml.color import RGBColor
from pptx.enum.text import MSO_ANCHOR, PP_ALIGN
from pptx.util import Emu, Inches, Pt

HERE = os.path.dirname(os.path.abspath(__file__))
FIGS = os.path.join(HERE, 'figs')
OUT = os.path.join(HERE, 'toio_omni_range_overview.pptx')

NAVY = RGBColor(0x1B, 0x2A, 0x41)
TEAL = RGBColor(0x2A, 0x9D, 0x8F)
ORANGE = RGBColor(0xE7, 0x6F, 0x51)
GRAY = RGBColor(0x6C, 0x75, 0x7D)
LIGHT = RGBColor(0xF2, 0xF5, 0xF8)
WHITE = RGBColor(0xFF, 0xFF, 0xFF)

JP = 'Noto Sans CJK JP'

W, H = 13.333, 7.5


def textbox(slide, x, y, w, h, anchor=MSO_ANCHOR.TOP):
    tb = slide.shapes.add_textbox(Inches(x), Inches(y), Inches(w), Inches(h))
    tf = tb.text_frame
    tf.word_wrap = True
    tf.margin_left = tf.margin_right = Emu(0)
    tf.margin_top = tf.margin_bottom = Emu(0)
    tf.vertical_anchor = anchor
    return tf


def para(tf, text, size, color, bold=False, align=PP_ALIGN.LEFT,
         space_before=0, space_after=0, first=False, line=None):
    p = tf.paragraphs[0] if first else tf.add_paragraph()
    p.alignment = align
    p.space_before = Pt(space_before)
    p.space_after = Pt(space_after)
    if line:
        p.line_spacing = line
    r = p.add_run()
    r.text = text
    r.font.size = Pt(size)
    r.font.bold = bold
    r.font.color.rgb = color
    r.font.name = JP
    return p


def rect(slide, x, y, w, h, fill, line=None, lw=1.0):
    from pptx.enum.shapes import MSO_SHAPE
    sh = slide.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE,
                                Inches(x), Inches(y), Inches(w), Inches(h))
    sh.adjustments[0] = 0.06
    sh.fill.solid()
    sh.fill.fore_color.rgb = fill
    if line is None:
        sh.line.fill.background()
    else:
        sh.line.color.rgb = line
        sh.line.width = Pt(lw)
    sh.shadow.inherit = False
    sh.text_frame.text = ''
    return sh


def plain_rect(slide, x, y, w, h, fill):
    from pptx.enum.shapes import MSO_SHAPE
    sh = slide.shapes.add_shape(MSO_SHAPE.RECTANGLE,
                                Inches(x), Inches(y), Inches(w), Inches(h))
    sh.fill.solid()
    sh.fill.fore_color.rgb = fill
    sh.line.fill.background()
    sh.shadow.inherit = False
    return sh


def picture(slide, name, x, y, box_w, box_h):
    """box に収まる最大サイズで中央寄せに貼る。"""
    path = os.path.join(FIGS, name)
    iw, ih = Image.open(path).size
    scale = min(box_w / iw, box_h / ih)
    w, h = iw * scale, ih * scale
    slide.shapes.add_picture(path, Inches(x + (box_w - w) / 2),
                             Inches(y + (box_h - h) / 2),
                             Inches(w), Inches(h))
    return w, h


def build():
    prs = Presentation()
    prs.slide_width = Inches(W)
    prs.slide_height = Inches(H)
    slide = prs.slides.add_slide(prs.slide_layouts[6])

    # ---------------------------------------------------------- ヘッダ
    plain_rect(slide, 0, 0, W, 1.12, NAVY)
    plain_rect(slide, 0, 1.12, W, 0.045, TEAL)

    tf = textbox(slide, 0.42, 0.13, 8.6, 0.9)
    para(tf, 'toio 用 全周障害物検知センサ  Omni Range Sensor',
         24, WHITE, bold=True, first=True)
    para(tf, 'カメラ 1 台 + 凸面鏡だけで、可動部なしに 5° 刻み・72 方向の距離を測る',
         11.5, RGBColor(0xB7, 0xC6, 0xD6), space_before=4)

    chip = rect(slide, 9.35, 0.24, 3.55, 0.62, RGBColor(0x2A, 0x3D, 0x59))
    ctf = chip.text_frame
    ctf.word_wrap = True
    ctf.vertical_anchor = MSO_ANCHOR.MIDDLE
    para(ctf, 'Step 5/6 まで実機確認済み', 11, TEAL, bold=True,
         align=PP_ALIGN.CENTER, first=True)
    para(ctf, 'ROS 2 ドライバは実機 BLE 未確認', 8.5,
         RGBColor(0xB7, 0xC6, 0xD6), align=PP_ALIGN.CENTER, space_before=1)

    # ---------------------------------------------------------- 3 パネル
    top = 1.34
    pic_h = 2.62
    picture(slide, 'hw.png', 0.30, top, 4.62, pic_h)
    picture(slide, 'unwrap.png', 5.15, top, 3.55, 4.30)
    picture(slide, 'scan.png', 8.90, top, 4.15, 4.30)

    # 左下: 特徴
    bx = 0.30
    by = top + pic_h + 0.14
    box = rect(slide, bx, by, 4.62, 1.60, LIGHT)
    btf = box.text_frame
    btf.word_wrap = True
    btf.margin_left = Inches(0.16)
    btf.margin_right = Inches(0.12)
    btf.margin_top = Inches(0.10)
    para(btf, 'この構成の利点', 11.5, NAVY, bold=True, first=True)
    for t in ('可動部なし・カメラ 1 台。回転 LiDAR より小さく安い',
              '72 本が同一フレーム＝同時刻。走査歪みがない',
              'ESP32 上で検出まで完結し、BLE で 150 B/スキャンだけ送る',
              'LaserScan で出すので RViz / Nav2 がそのまま使える'):
        para(btf, '▪ ' + t, 9.5, NAVY, space_before=3.5, line=1.05)

    # ---------------------------------------------------------- スペック帯
    sy = 5.80
    sh_h = 0.86
    specs = [('72 方向', '5° 刻み・全周 360°'),
             ('2.2 – 23.3 cm', '測距範囲（実測裏付け 4.5–21 cm）'),
             ('4.1 Hz', 'BLE 単独・実測（642 B/s）'),
             ('55 ms', '1 スキャンの ESP32 処理時間'),
             ('約 1.9 h', 'バッテリ駆動（BLE 単独・実測）')]
    gap = 0.16
    total = W - 0.60
    tw = (total - gap * (len(specs) - 1)) / len(specs)
    for i, (big, small) in enumerate(specs):
        x = 0.30 + i * (tw + gap)
        card = rect(slide, x, sy, tw, sh_h, WHITE, line=RGBColor(0xD5, 0xDD, 0xE5))
        ctf = card.text_frame
        ctf.word_wrap = True
        ctf.vertical_anchor = MSO_ANCHOR.MIDDLE
        para(ctf, big, 15, ORANGE if i == 1 else NAVY, bold=True,
             align=PP_ALIGN.CENTER, first=True)
        para(ctf, small, 8.5, GRAY, align=PP_ALIGN.CENTER, space_before=1)

    # ---------------------------------------------------------- フッタ
    ftf = textbox(slide, 0.32, 6.86, 12.7, 0.45)
    para(ftf, '検出原理: 床と物体の彩度／明度の差（床の MAD 比 3.0σ / 6.0σ）'
              '  ・  半径→距離: r = a + b·arctan(d/c)（6 点フィット, 残差 rms 0.62 px）'
              '  ・  空の床での誤検出 0/72',
         9, GRAY, first=True)
    para(ftf, 'H/W: M5Stack Timer Camera F（ESP32 + OV3660）+ 凸面鏡  /  '
              'S/W: PlatformIO ファーム + ROS 2 ドライバ（sensor_msgs/LaserScan, BatteryState, Diagnostics）',
         9, GRAY, space_before=2)

    prs.save(OUT)
    print(OUT)


if __name__ == '__main__':
    build()
