#!/usr/bin/env python3
"""概要スライドの簡易版（16:9・1 枚）。

詳細版（make_figs.py / make_slide.py）から数値と処理の内訳を落とし、
図の文字も大きくしたもの。

出力: docs/figs/{hw_simple,scan_simple}.png, docs/toio_omni_range_simple.pptx
"""
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle, FancyBboxPatch
from PIL import Image
from pptx import Presentation
from pptx.dml.color import RGBColor
from pptx.enum.shapes import MSO_SHAPE
from pptx.enum.text import MSO_ANCHOR, PP_ALIGN
from pptx.util import Emu, Inches, Pt

plt.rcParams['font.family'] = 'Noto Sans CJK JP'
plt.rcParams['axes.unicode_minus'] = False

HERE = os.path.dirname(os.path.abspath(__file__))
FIGS = os.path.join(HERE, 'figs')
OUT = os.path.join(HERE, 'toio_omni_range_simple.pptx')

NAVY_H = '#1B2A41'
TEAL_H = '#2A9D8F'
ORANGE_H = '#E76F51'
AMBER_H = '#E9C46A'
GRAY_H = '#8D99AE'
FLOOR_H = '#DDE3EA'

NAVY = RGBColor(0x1B, 0x2A, 0x41)
TEAL = RGBColor(0x2A, 0x9D, 0x8F)
ORANGE = RGBColor(0xE7, 0x6F, 0x51)
GRAY = RGBColor(0x6C, 0x75, 0x7D)
WHITE = RGBColor(0xFF, 0xFF, 0xFF)

JP = 'Noto Sans CJK JP'
W, H = 13.333, 7.5


# ------------------------------------------------------------------ 図 1
def fig_hw():
    fig, ax = plt.subplots(figsize=(5.4, 3.4))
    ax.set_xlim(-7.6, 7.6)
    ax.set_ylim(-1.1, 5.9)
    ax.set_aspect('equal')
    ax.axis('off')

    ax.add_patch(FancyBboxPatch((-7.5, -0.9), 15.0, 0.9, boxstyle='square,pad=0',
                                fc=FLOOR_H, ec='none'))
    ax.plot([-7.5, 7.5], [0, 0], color=GRAY_H, lw=1.4)

    ax.add_patch(FancyBboxPatch((-0.95, 0.0), 1.9, 1.25,
                                boxstyle='round,pad=0.02,rounding_size=0.18',
                                fc='#FFFFFF', ec=NAVY_H, lw=2.0))
    ax.text(0, 0.6, 'toio', ha='center', va='center', fontsize=13,
            color=NAVY_H, fontweight='bold')

    ax.add_patch(FancyBboxPatch((-0.62, 1.25), 1.24, 0.85,
                                boxstyle='round,pad=0.02,rounding_size=0.1',
                                fc=NAVY_H, ec=NAVY_H))
    ax.add_patch(Circle((0, 2.12), 0.18, fc=AMBER_H, ec='#FFFFFF', lw=1.0, zorder=5))
    ax.annotate('カメラ\n（上向き）', xy=(-0.66, 1.72), xytext=(-2.0, 0.7),
                ha='right', va='center', fontsize=12, color=NAVY_H,
                arrowprops=dict(arrowstyle='-', color=GRAY_H, lw=1.1,
                                shrinkA=0, shrinkB=4))

    for x in (-1.55, 1.55):
        ax.plot([x, x], [0.0, 4.25], color=GRAY_H, lw=2.4, solid_capstyle='round')

    th = np.linspace(0, np.pi, 200)
    my, R = 4.15, 1.75
    ax.fill(R * np.cos(th), my - 0.85 * np.sin(th),
            color='#C9D6E3', ec=NAVY_H, lw=1.8, zorder=4)
    ax.plot([-R, R], [my, my], color=NAVY_H, lw=1.8, zorder=4)
    ax.annotate('凸面鏡', xy=(1.5, 3.95), xytext=(2.9, 4.5), ha='left', va='center',
                fontsize=13, color=NAVY_H, fontweight='bold',
                arrowprops=dict(arrowstyle='-', color=GRAY_H, lw=1.1,
                                shrinkA=0, shrinkB=3))

    for ox, oc in ((-6.0, ORANGE_H), (6.0, TEAL_H)):
        ax.add_patch(FancyBboxPatch((ox - 0.5, 0.0), 1.0, 1.1,
                                    boxstyle='round,pad=0.01,rounding_size=0.08',
                                    fc=oc, ec='none', alpha=0.92))
        ax.text(ox, 1.3, '障害物', ha='center', va='bottom', fontsize=12, color=oc)
        hitx = np.sign(ox) * 1.18
        hity = my - 0.85 * np.sqrt(max(0.0, 1 - (hitx / R) ** 2))
        ax.annotate('', xy=(hitx, hity), xytext=(ox, 0.85),
                    arrowprops=dict(arrowstyle='-|>', color=oc, lw=2.2,
                                    shrinkA=2, shrinkB=0))
        ax.annotate('', xy=(0.0, 2.2), xytext=(hitx, hity),
                    arrowprops=dict(arrowstyle='-|>', color=oc, lw=2.2,
                                    shrinkA=0, shrinkB=2))

    ax.text(0, 5.45, '周囲 360° が 1 枚の画像に写る', ha='center', va='center',
            fontsize=13.5, color=NAVY_H, fontweight='bold')
    return fig


# ------------------------------------------------------------------ 図 2
def fig_scan():
    fig = plt.figure(figsize=(4.6, 3.9))
    ax = fig.add_subplot(111, projection='polar')
    ax.set_theta_zero_location('N')
    ax.set_theta_direction(1)

    rmin, rmax = 4.0, 11.0
    ang = np.radians(np.arange(0, 360, 5))
    ax.bar(0, rmax - rmin, width=2 * np.pi, bottom=rmin,
           color=TEAL_H, alpha=0.10, edgecolor='none')
    for k in ang:
        ax.plot([k, k], [rmin, rmax], color=GRAY_H, lw=0.5, alpha=0.35)

    det = {25: 6.2, 30: 6.0, 35: 6.4, 110: 8.6, 115: 8.9,
           250: 5.2, 255: 5.0, 260: 5.4, 265: 5.9, 315: 9.8}
    for deg, d in det.items():
        a = np.radians(deg)
        ax.plot([a, a], [rmin, d], color=ORANGE_H, lw=2.8, solid_capstyle='round')
        ax.plot([a], [d], marker='o', ms=6.5, color=ORANGE_H,
                mec='#FFFFFF', mew=1.0, zorder=6)

    ax.text(0, 0, 'toio', ha='center', va='center', fontsize=11,
            color=NAVY_H, fontweight='bold')
    ax.set_rlim(0, 12.2)
    ax.set_rticks([4.0, 11.0])
    ax.set_yticklabels(['2 cm', '23 cm'], fontsize=10.5, color=NAVY_H)
    ax.set_rlabel_position(192)
    ax.set_xticks(np.radians([0, 90, 180, 270]))
    ax.set_xticklabels(['前', '左', '後', '右'], fontsize=13, color=NAVY_H)
    ax.tick_params(pad=1.0)
    ax.grid(color=GRAY_H, lw=0.6, alpha=0.5)
    ax.spines['polar'].set_color(GRAY_H)

    fig.text(0.5, -0.035, '橙 = 見つけた障害物までの距離',
             ha='center', fontsize=12, color=ORANGE_H)
    return fig


# ---------------------------------------------------------------- スライド
def para(tf, text, size, color, bold=False, align=PP_ALIGN.LEFT,
         space_before=0, first=False):
    p = tf.paragraphs[0] if first else tf.add_paragraph()
    p.alignment = align
    p.space_before = Pt(space_before)
    p.space_after = Pt(0)
    r = p.add_run()
    r.text = text
    r.font.size = Pt(size)
    r.font.bold = bold
    r.font.color.rgb = color
    r.font.name = JP
    return p


def textbox(slide, x, y, w, h, anchor=MSO_ANCHOR.TOP):
    tb = slide.shapes.add_textbox(Inches(x), Inches(y), Inches(w), Inches(h))
    tf = tb.text_frame
    tf.word_wrap = True
    tf.margin_left = tf.margin_right = Emu(0)
    tf.margin_top = tf.margin_bottom = Emu(0)
    tf.vertical_anchor = anchor
    return tf


def shape(slide, kind, x, y, w, h, fill, line=None):
    sh = slide.shapes.add_shape(kind, Inches(x), Inches(y), Inches(w), Inches(h))
    sh.fill.solid()
    sh.fill.fore_color.rgb = fill
    if line is None:
        sh.line.fill.background()
    else:
        sh.line.color.rgb = line
        sh.line.width = Pt(1.0)
    sh.shadow.inherit = False
    return sh


def picture(slide, name, x, y, box_w, box_h):
    path = os.path.join(FIGS, name)
    iw, ih = Image.open(path).size
    s = min(box_w / iw, box_h / ih)
    w, h = iw * s, ih * s
    slide.shapes.add_picture(path, Inches(x + (box_w - w) / 2),
                             Inches(y + (box_h - h) / 2), Inches(w), Inches(h))


def build():
    prs = Presentation()
    prs.slide_width = Inches(W)
    prs.slide_height = Inches(H)
    slide = prs.slides.add_slide(prs.slide_layouts[6])

    shape(slide, MSO_SHAPE.RECTANGLE, 0, 0, W, 1.30, NAVY)
    shape(slide, MSO_SHAPE.RECTANGLE, 0, 1.30, W, 0.06, TEAL)

    tf = textbox(slide, 0.55, 0.20, 12.3, 1.0)
    para(tf, 'toio 用 全周障害物検知センサ', 30, WHITE, bold=True, first=True)
    para(tf, 'カメラ 1 台と凸面鏡だけで、周囲 360° の障害物までの距離を測る',
         15, RGBColor(0xBF, 0xCE, 0xDC), space_before=6)

    cap_y, img_y, img_h = 1.58, 2.10, 3.45
    for x, w, text in ((0.55, 5.85, '① カメラの上に凸面鏡を置く'),
                       (6.95, 5.85, '② まわり 72 方向の距離を出す')):
        c = textbox(slide, x, cap_y, w, 0.45)
        para(c, text, 17, NAVY, bold=True, align=PP_ALIGN.CENTER, first=True)
    picture(slide, 'hw_simple.png', 0.55, img_y, 5.85, img_h)
    picture(slide, 'scan_simple.png', 6.95, img_y, 5.85, img_h)

    tiles = [('全周 360°', '5° ごと 72 方向'),
             ('2 〜 23 cm', '測れる距離'),
             ('可動部なし', 'カメラ 1 台だけ'),
             ('ROS 2 対応', 'そのまま地図・自律走行へ')]
    ty, th_ = 5.72, 1.20
    gap, total = 0.22, W - 1.10
    tw = (total - gap * (len(tiles) - 1)) / len(tiles)
    for i, (big, small) in enumerate(tiles):
        card = shape(slide, MSO_SHAPE.ROUNDED_RECTANGLE,
                     0.55 + i * (tw + gap), ty, tw, th_,
                     WHITE, line=RGBColor(0xD5, 0xDD, 0xE5))
        card.adjustments[0] = 0.12
        ctf = card.text_frame
        ctf.word_wrap = True
        ctf.vertical_anchor = MSO_ANCHOR.MIDDLE
        para(ctf, big, 20, ORANGE if i == 1 else NAVY, bold=True,
             align=PP_ALIGN.CENTER, first=True)
        para(ctf, small, 11.5, GRAY, align=PP_ALIGN.CENTER, space_before=3)

    prs.save(OUT)
    print(OUT)


if __name__ == '__main__':
    os.makedirs(FIGS, exist_ok=True)
    for f, name in ((fig_hw(), 'hw_simple.png'), (fig_scan(), 'scan_simple.png')):
        p = os.path.join(FIGS, name)
        f.savefig(p, dpi=220, facecolor='#FFFFFF', bbox_inches='tight',
                  pad_inches=0.06)
        plt.close(f)
        print(p)
    build()
