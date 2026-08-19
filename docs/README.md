# docs — 概要スライド

このセンサを 1 枚で説明するためのスライドの**生成スクリプト**。**簡易版**（文字が
大きく、数値は最小限）と**詳細版**（実測値・処理の内訳まで載せたもの）の 2 種類。

**生成物（pptx / pdf / png / figs\*.png）はリポジトリに置いていない。** すべて下の
コマンドで作り直せるうえ、鏡や較正を変えるたびに古くなるため。実際、最初の凸面鏡の
ころの版が「測れる距離 4〜11 cm」のまま残っていた。

| ファイル | 内容 |
|---|---|
| `make_simple.py` | 簡易版。図とスライドを一括生成 |
| `make_figs.py` | 詳細版の図（`figs/hw.png` 構成図、`figs/unwrap.png` 環状→パノラマ、`figs/scan.png` 出力） |
| `make_slide.py` | 詳細版のスライド。`make_figs.py` の後に実行する |

## 作り直す

```bash
cd docs
python3 make_simple.py            # figs/*_simple.png + toio_omni_range_simple.pptx
python3 make_figs.py && python3 make_slide.py   # 詳細版
```

必要なもの: `matplotlib`、`python-pptx`、日本語フォント。

## 数値を直す

スライドには測距範囲・レート・処理時間が**べた書き**されている。再較正したら
リポジトリ直下の README の「現在の構成」と突き合わせて、以下を直すこと。

| 場所 | 内容 |
|---|---|
| `make_slide.py` の `specs` | 測距範囲・Hz・1 スキャンの処理時間・バッテリ駆動時間 |
| `make_simple.py` の `tiles` | 測れる距離 |
| `make_simple.py` / `make_figs.py` の `set_yticklabels` | 出力図の半径目盛 |
