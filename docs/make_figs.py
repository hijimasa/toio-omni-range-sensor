#!/usr/bin/env python3
"""概要スライド用の図を生成する。

出力: docs/figs/{hw,unwrap,scan}.png
"""
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Arc, Circle, FancyArrowPatch, FancyBboxPatch, Wedge

plt.rcParams['font.family'] = 'Noto Sans CJK JP'
plt.rcParams['axes.unicode_minus'] = False

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'figs')

NAVY = '#1B2A41'
TEAL = '#2A9D8F'
ORANGE = '#E76F51'
AMBER = '#E9C46A'
GRAY = '#8D99AE'
FLOOR = '#DDE3EA'
BG = '#FFFFFF'


def save(fig, name):
    path = os.path.join(OUT, name)
    fig.savefig(path, dpi=220, facecolor=BG, bbox_inches='tight', pad_inches=0.06)
    plt.close(fig)
    print(path)


# ---------------------------------------------------------------- 1. 構成図
def fig_hw():
    fig, ax = plt.subplots(figsize=(4.6, 3.7))
    ax.set_xlim(-7.6, 7.6)
    ax.set_ylim(-1.1, 7.5)
    ax.set_aspect('equal')
    ax.axis('off')

    # 床
    ax.add_patch(FancyBboxPatch((-7.5, -0.9), 15.0, 0.9, boxstyle='square,pad=0',
                                fc=FLOOR, ec='none'))
    ax.plot([-7.5, 7.5], [0, 0], color=GRAY, lw=1.2)

    # toio コアキューブ
    ax.add_patch(FancyBboxPatch((-0.95, 0.0), 1.9, 1.25, boxstyle='round,pad=0.02,rounding_size=0.18',
                                fc='#FFFFFF', ec=NAVY, lw=1.8))
    ax.text(0, 0.62, 'toio', ha='center', va='center', fontsize=10, color=NAVY, fontweight='bold')

    # カメラ（上向き）
    ax.add_patch(FancyBboxPatch((-0.62, 1.25), 1.24, 0.85, boxstyle='round,pad=0.02,rounding_size=0.1',
                                fc=NAVY, ec=NAVY))
    ax.add_patch(Circle((0, 2.12), 0.17, fc=AMBER, ec='#FFFFFF', lw=0.8, zorder=5))
    ax.annotate('Timer Camera F\n（上向き）',
                xy=(-0.66, 1.72), xytext=(-1.9, 0.62), ha='right', va='center',
                fontsize=7.8, color=NAVY,
                arrowprops=dict(arrowstyle='-', color=GRAY, lw=0.9,
                                shrinkA=0, shrinkB=4))

    # 支柱
    for x in (-1.55, 1.55):
        ax.plot([x, x], [0.0, 4.25], color=GRAY, lw=2.0, solid_capstyle='round')

    # 凸面鏡（下向きドーム）
    th = np.linspace(0, np.pi, 200)
    mx, my, R = 0.0, 4.15, 1.75
    ax.fill(mx + R * np.cos(th), my + 0.85 * np.sin(th) * -1 + 0.0,
            color='#C9D6E3', ec=NAVY, lw=1.6, zorder=4)
    ax.plot([mx - R, mx + R], [my, my], color=NAVY, lw=1.6, zorder=4)
    ax.annotate('凸面鏡', xy=(1.5, 3.95), xytext=(2.9, 4.35), ha='left', va='center',
                fontsize=9.5, color=NAVY, fontweight='bold',
                arrowprops=dict(arrowstyle='-', color=GRAY, lw=0.9,
                                shrinkA=0, shrinkB=3))

    # 障害物
    for ox, oc in ((-6.0, ORANGE), (6.0, TEAL)):
        ax.add_patch(FancyBboxPatch((ox - 0.45, 0.0), 0.9, 1.0,
                                    boxstyle='round,pad=0.01,rounding_size=0.08',
                                    fc=oc, ec='none', alpha=0.9))
    ax.text(-6.0, 1.18, '障害物', ha='center', va='bottom', fontsize=8.5, color=ORANGE)
    ax.text(6.0, 1.18, '障害物', ha='center', va='bottom', fontsize=8.5, color=TEAL)

    # 光路: 物体 -> 鏡 -> カメラ
    for ox, oc in ((-6.0, ORANGE), (6.0, TEAL)):
        hitx = np.sign(ox) * 1.18
        hity = my - 0.85 * np.sqrt(max(0.0, 1 - (hitx / R) ** 2))
        ax.annotate('', xy=(hitx, hity), xytext=(ox, 0.8),
                    arrowprops=dict(arrowstyle='-|>', color=oc, lw=1.7,
                                    shrinkA=2, shrinkB=0))
        ax.annotate('', xy=(0.0, 2.18), xytext=(hitx, hity),
                    arrowprops=dict(arrowstyle='-|>', color=oc, lw=1.7,
                                    shrinkA=0, shrinkB=2))

    ax.text(0, 6.95, '周囲 360° が 1 枚の環状画像に写る',
            ha='center', va='center', fontsize=10.5, color=NAVY, fontweight='bold')
    ax.text(0, 6.25, 'カタディオプトリック構成（可動部なし・カメラ 1 台）',
            ha='center', va='center', fontsize=8.4, color=GRAY)
    return fig


# ------------------------------------------------ 2. 環状 -> パノラマ -> 検出
def synth_annulus(n=520):
    """説明用の合成環状画像（床＝低彩度、物体＝高彩度）。"""
    yy, xx = np.mgrid[0:n, 0:n]
    cx = cy = n / 2
    r = np.hypot(xx - cx, yy - cy)
    th = (np.degrees(np.arctan2(-(yy - cy), xx - cx))) % 360
    r_in, r_out = n * 0.14, n * 0.47

    img = np.ones((n, n, 3))
    ann = (r >= r_in) & (r <= r_out)
    # 床: ほぼ無彩色、外周ほどわずかに暗い
    shade = 0.86 - 0.18 * (r - r_in) / (r_out - r_in)
    for c in range(3):
        img[..., c] = np.where(ann, shade, img[..., c])

    # 物体（凸面鏡では基部が内側 = 近い側に写る）
    objs = [(25, 16, 0.42, 0.68, (0.90, 0.44, 0.32)),
            (110, 10, 0.30, 0.60, (0.16, 0.62, 0.56)),
            (250, 22, 0.55, 0.85, (0.29, 0.36, 0.78)),
            (315, 8, 0.22, 0.48, (0.91, 0.77, 0.41))]
    for c_deg, w, f0, f1, col in objs:
        d = np.abs((th - c_deg + 180) % 360 - 180)
        band = (d <= w / 2) & ann & (r >= r_in + f0 * (r_out - r_in)) \
            & (r <= r_in + f1 * (r_out - r_in))
        for c in range(3):
            img[..., c] = np.where(band, col[c], img[..., c])

    # 中心のカメラ映り込み（死角）
    img[r < r_in] = 0.22
    img[r > r_out] = 1.0
    return img, (r_in, r_out), objs


def fig_unwrap():
    fig = plt.figure(figsize=(3.9, 4.3))
    gs = fig.add_gridspec(2, 1, height_ratios=[1.75, 1.0], hspace=0.30)

    img, (r_in, r_out), objs = synth_annulus()
    n = img.shape[0]

    ax0 = fig.add_subplot(gs[0])
    ax0.imshow(img, extent=[0, n, 0, n])
    ax0.add_patch(Circle((n / 2, n / 2), r_out, fill=False, ec=NAVY, lw=1.4, ls='-'))
    ax0.add_patch(Circle((n / 2, n / 2), r_in, fill=False, ec=AMBER, lw=1.4))
    for k in range(0, 72, 2):
        a = np.radians(k * 5)
        ax0.plot([n / 2 + r_in * np.cos(a), n / 2 + r_out * np.cos(a)],
                 [n / 2 + r_in * np.sin(a), n / 2 + r_out * np.sin(a)],
                 color='#FFFFFF', lw=0.45, alpha=0.55)
    m = r_out * 1.06
    ax0.set_xlim(n / 2 - m, n / 2 + m)
    ax0.set_ylim(n / 2 - m, n / 2 + m)
    ax0.axis('off')
    ax0.set_title('① 環状画像（5° 刻み 72 方向）', fontsize=9, color=NAVY, pad=4)
    ax0.text(n / 2, n / 2, '死角', ha='center', va='center',
             fontsize=7.0, color='#FFFFFF')

    # パノラマ（横=方位角 / 縦=半径）
    ax1 = fig.add_subplot(gs[1])
    W, H = 360, 100
    pano = np.ones((H, W, 3))
    shade = 0.86 - 0.18 * (np.arange(H) / H)
    pano[:] = shade[:, None, None]
    bearings = np.arange(W) - 180.0          # 列 -> 方位角 [-180, 180)  ROS 準拠
    for c_deg, w, f0, f1, col in objs:
        c = ((c_deg + 180) % 360) - 180
        cols = np.abs((bearings - c + 180) % 360 - 180) <= w / 2
        rows = slice(int(f0 * H), int(f1 * H))
        pano[rows, cols] = col
    ax1.imshow(pano, extent=[-180, 180, 1, 0], aspect='auto')
    ax1.set_ylim(1, 0)

    # 検出された境界（外向き走査で床から最初に外れた半径）
    bx, by = [], []
    for c_deg, w, f0, f1, col in objs:
        for d in np.arange(-w / 2, w / 2 + 0.1, 5):
            bx.append(((c_deg + d + 180) % 360) - 180)
            by.append(f0)
    ax1.scatter(bx, by, s=13, color=ORANGE, ec='#FFFFFF', lw=0.7, zorder=5)
    ax1.set_xticks([-180, -90, 0, 90, 180])
    ax1.set_xlabel('方位角 [deg]（ROS 準拠: x 前方・CCW 正）',
                   fontsize=7.5, color=NAVY, labelpad=1)
    ax1.set_yticks([0.06, 0.94])
    ax1.set_yticklabels(['近', '遠'], fontsize=7)
    ax1.tick_params(labelsize=7, colors=NAVY, length=2)
    for s in ax1.spines.values():
        s.set_color(GRAY)
    ax1.set_title('② パノラマ展開 → 彩度・明度が床から外れた半径を検出',
                  fontsize=9, color=NAVY, pad=4)

    fig.text(0.5, -0.035, 'しきい値は床の頑健な σ（MAD）比 — 彩度 3.0σ / 明度 6.0σ',
             ha='center', fontsize=7.6, color=GRAY)
    return fig


# ---------------------------------------------------------------- 3. 出力
def fig_scan():
    fig = plt.figure(figsize=(4.6, 3.7))
    ax = fig.add_subplot(111, projection='polar')
    ax.set_theta_zero_location('N')
    ax.set_theta_direction(1)

    rmin, rmax = 4.1, 11.1
    ang = np.radians(np.arange(0, 360, 5))

    # 測距範囲のドーナツ
    ax.bar(0, rmax - rmin, width=2 * np.pi, bottom=rmin,
           color=TEAL, alpha=0.09, edgecolor='none')
    for k in ang:
        ax.plot([k, k], [rmin, rmax], color=GRAY, lw=0.4, alpha=0.35)

    # 検出（無いレイは range_max 超過 = 描かない）
    det = {25: 6.2, 30: 6.0, 35: 6.4, 110: 8.6, 115: 8.9,
           250: 5.2, 255: 5.0, 260: 5.4, 265: 5.9, 315: 9.8}
    for deg, d in det.items():
        a = np.radians(deg)
        ax.plot([a, a], [rmin, d], color=ORANGE, lw=2.0, solid_capstyle='round')
        ax.plot([a], [d], marker='o', ms=4.4, color=ORANGE,
                mec='#FFFFFF', mew=0.8, zorder=6)

    ax.text(0, 0, '死角\n< 4.1 cm', ha='center', va='center',
            fontsize=7.0, color=GRAY)

    ax.set_rlim(0, 12.4)
    ax.set_rticks([4.1, 7.5, 11.1])
    ax.set_yticklabels(['2.2cm', '12', '23.3cm'], fontsize=6.8, color=NAVY)
    ax.set_rlabel_position(192)
    ax.set_xticks(np.radians([0, 90, 180, 270]))
    ax.set_xticklabels(['前 0°', '+90°', '180°', '−90°'], fontsize=8, color=NAVY)
    ax.tick_params(pad=0.5)
    ax.grid(color=GRAY, lw=0.5, alpha=0.5)
    ax.spines['polar'].set_color(GRAY)
    ax.set_title('③ 出力: sensor_msgs/LaserScan（72 点・同時刻）',
                 fontsize=9, color=NAVY, pad=10)
    fig.text(0.5, 0.015, '回転式 LiDAR と違い 1 フレームのスナップショット → 走査歪みなし',
             ha='center', fontsize=7.6, color=GRAY)
    return fig


if __name__ == '__main__':
    os.makedirs(OUT, exist_ok=True)
    save(fig_hw(), 'hw.png')
    save(fig_unwrap(), 'unwrap.png')
    save(fig_scan(), 'scan.png')
