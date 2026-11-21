#!/usr/bin/env python3
"""点击地图，输出原图像素、像素值和翻转 Y 后的世界坐标。

依赖：Python 3、NumPy、带 GUI 支持的 OpenCV（cv2），以及桌面显示环境。
在项目根目录运行，例如：
    python3 scripts/map_point_picker.py images/rmuc2025.png
    python3 scripts/map_point_picker.py images/inside_map.png --resolution 0.02
    python3 scripts/map_point_picker.py map.png --max-width 1000 --max-height 700

--resolution 单位为米/像素，默认 0.05；普通图片无法自动提供实际地图比例。
原图 (u, v) 以左上角为原点，u 向右、v 向下，索引从 0 开始。
世界坐标以左下角像素为原点，X 向右、Y 向上：
    x = u * resolution
    y = (height - 1 - v) * resolution
该约定与项目 test(start, end) 的输入一致，不包含原点平移和地图旋转。

左键：添加带编号的点，终端输出原图坐标（不受显示缩放影响）。
C：清除标记。Q / Esc / 关闭窗口：退出。不修改或保存原图片。
支持灰度、BGR、BGRA 图片；颜色值按 OpenCV 的 BGR(A) 顺序输出。
非 8 位图片仅在显示时归一化，终端始终输出原始像素值。
"""

import argparse
import math
from pathlib import Path
import sys

import cv2
import numpy as np


def positive_resolution(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError("分辨率必须是有限的正数（米/像素）")
    return number


def positive_size(value):
    number = int(value)
    if number <= 0:
        raise argparse.ArgumentTypeError("显示尺寸必须为正整数")
    return number


def display_size(width, height, max_width, max_height):
    scale = min(1.0, max_width / width, max_height / height)
    return max(1, round(width * scale)), max(1, round(height * scale))


def original_pixel(x, y, original_size, shown_size):
    """将实际显示尺寸中的位置映射回原图；窗口外返回 None。"""
    width, height = original_size
    shown_width, shown_height = shown_size
    if not (0 <= x < shown_width and 0 <= y < shown_height):
        return None
    return x * width // shown_width, y * height // shown_height


def world_coordinates(u, v, height, resolution):
    return u * resolution, (height - 1 - v) * resolution


def display_image(raw):
    """生成独立的 8 位 BGR 显示副本，保留原始像素供查询。"""
    image = raw
    if raw.ndim == 3 and raw.shape[2] == 4:
        image = raw[:, :, :3]
    if image.dtype != np.uint8:
        values = image.astype(np.float64)
        finite = np.isfinite(values)
        converted = np.zeros(values.shape, dtype=np.uint8)
        if finite.any():
            low, high = values[finite].min(), values[finite].max()
            if high > low:
                converted[finite] = np.clip(
                    (values[finite] - low) / (high - low) * 255, 0, 255
                ).astype(np.uint8)
            elif high > 0:
                converted[finite] = 255
        image = converted
    if image.ndim == 2:
        return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    return image.copy()


class PointPicker:
    WINDOW = "Map point picker"

    def __init__(self, raw, resolution, max_width, max_height):
        self.raw = raw
        self.resolution = resolution
        self.height, self.width = raw.shape[:2]
        self.shown_size = display_size(
            self.width, self.height, max_width, max_height
        )
        image = display_image(raw)
        self.background = cv2.resize(
            image, self.shown_size, interpolation=cv2.INTER_NEAREST
        )
        self.points = []

    def render(self):
        canvas = self.background.copy()
        shown_width, shown_height = self.shown_size
        for index, (u, v, click_x, click_y) in enumerate(self.points, start=1):
            # 保留实际点击位置，避免缩小图片中的标记二次量化偏移。
            cv2.drawMarker(canvas, (click_x, click_y), (0, 0, 255),
                           cv2.MARKER_CROSS, 13, 1)
            label = str(index)
            text_width = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0][0]
            position = (max(0, min(click_x + 8, shown_width - text_width)),
                        max(12, min(click_y - 8, shown_height - 3)))
            cv2.putText(canvas, label, position, cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(canvas, label, position, cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 255, 255), 1, cv2.LINE_AA)
        title = self.WINDOW + " | Left click: add | C: clear | Q/Esc: quit"
        if self.points:
            u, v = self.points[-1][:2]
            x, y = world_coordinates(u, v, self.height, self.resolution)
            title = f"{self.WINDOW} | #{len(self.points)} pixel=({u},{v}) world=({x:.6f},{y:.6f}) m"
        cv2.imshow(self.WINDOW, canvas)
        cv2.setWindowTitle(self.WINDOW, title)

    def on_mouse(self, event, x, y, flags, userdata):
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        pixel = original_pixel(x, y, (self.width, self.height), self.shown_size)
        if pixel is None:
            return
        u, v = pixel
        wx, wy = world_coordinates(u, v, self.height, self.resolution)
        value = self.raw[v, u].tolist()
        self.points.append((u, v, x, y))
        print(f"#{len(self.points)} pixel(u,v)=({u}, {v}) value={value} "
              f"world(x,y)=({wx:.9f}, {wy:.9f}) m", flush=True)
        self.render()

    def run(self):
        try:
            # 固定显示内容尺寸，保证回调坐标与 shown_size 一致。
            cv2.namedWindow(self.WINDOW, cv2.WINDOW_AUTOSIZE)
            self.render()
            cv2.setMouseCallback(self.WINDOW, self.on_mouse)
            while True:
                # 先处理GUI事件，让窗口实际完成显示。
                key = cv2.waitKey(30) & 0xFF
                if key in (27, ord("q"), ord("Q")):
                    break
                # GTK3不支持WND_PROP_VISIBLE，窗口打开时也会返回-1。
                # 使用图像区域尺寸判断关闭，兼容关闭后抛异常的后端。
                try:
                    _, _, width, height = cv2.getWindowImageRect(self.WINDOW)
                except cv2.error:
                    break
                if width <= 0 or height <= 0:
                    break
                if key in (ord("c"), ord("C")):
                    self.points.clear()
                    print("已清除标记。", flush=True)
                    self.render()
        finally:
            cv2.destroyAllWindows()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("image", type=Path, help="图片路径，相对于当前运行目录")
    parser.add_argument("--resolution", type=positive_resolution, default=0.05,
                        help="地图分辨率，米/像素（默认：0.05）")
    parser.add_argument("--max-width", type=positive_size, default=1200,
                        help="显示内容最大宽度，像素（默认：1200）")
    parser.add_argument("--max-height", type=positive_size, default=800,
                        help="显示内容最大高度，像素（默认：800）")
    args = parser.parse_args(argv)
    if not args.image.is_file():
        parser.error(f"图片不存在或不是文件：{args.image}")
    raw = cv2.imread(str(args.image), cv2.IMREAD_UNCHANGED)
    if raw is None:
        parser.error(f"无法读取图片：{args.image}")
    if raw.ndim != 2 and not (raw.ndim == 3 and raw.shape[2] in (3, 4)):
        parser.error("仅支持灰度、BGR 或 BGRA 图片")
    height, width = raw.shape[:2]
    channels = 1 if raw.ndim == 2 else raw.shape[2]
    order = {1: "gray", 3: "BGR", 4: "BGRA"}[channels]
    print(f"图片：{args.image}\n宽={width} 高={height} 通道={channels} "
          f"类型={raw.dtype} 像素顺序={order}\n分辨率={args.resolution} 米/像素\n"
          "world=(u*r, (height-1-v)*r)；左键添加，C 清除，Q/Esc 退出。", flush=True)
    try:
        PointPicker(raw, args.resolution, args.max_width, args.max_height).run()
    except cv2.error as error:
        print(f"OpenCV 窗口错误（需要可用的桌面显示环境）：{error}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        return 0
    return 0


if __name__ == "__main__":
    sys.exit(main())
