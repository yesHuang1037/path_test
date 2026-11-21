"""运行：python3 -B -m unittest discover -s test -p 'test_map_point_picker.py'。"""

import contextlib
import importlib.util
import io
from pathlib import Path
import unittest
from unittest.mock import patch

import cv2
import numpy as np


spec = importlib.util.spec_from_file_location(
    "map_point_picker",
    Path(__file__).resolve().parents[1] / "scripts" / "map_point_picker.py",
)
picker_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(picker_module)


class WindowLoopTest(unittest.TestCase):
    def run_window(self, keys, rectangles):
        picker = picker_module.PointPicker(np.zeros((20, 30), np.uint8), 0.05, 1200, 800)
        picker.points.append((1, 1, 1, 1))
        with contextlib.ExitStack() as stack:
            for name in ("namedWindow", "setMouseCallback", "destroyAllWindows"):
                stack.enter_context(patch.object(cv2, name))
            render = stack.enter_context(patch.object(picker, "render"))
            wait = stack.enter_context(patch.object(cv2, "waitKey", side_effect=keys))
            rect = stack.enter_context(patch.object(cv2, "getWindowImageRect", side_effect=rectangles))
            # GTK的可见属性返回-1，事件循环仍必须运行。
            visible = stack.enter_context(patch.object(cv2, "getWindowProperty", return_value=-1))
            stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
            picker.run()
            visible.assert_not_called()
        return picker, wait.call_count, rect.call_count, render.call_count

    def test_gtk_window_survives_until_quit(self):
        _, waits, rects, _ = self.run_window([-1, -1, ord("q")], [(0, 0, 30, 20)] * 2)
        self.assertEqual((waits, rects), (3, 2))

    def test_escape(self):
        _, waits, rects, _ = self.run_window([27], [])
        self.assertEqual((waits, rects), (1, 0))

    def test_clear_then_close(self):
        picker, waits, _, renders = self.run_window(
            [ord("c"), -1], [(0, 0, 30, 20), (-1, -1, -1, -1)]
        )
        self.assertEqual(picker.points, [])
        self.assertEqual((waits, renders), (2, 2))

    def test_closed_window_error(self):
        _, waits, rects, _ = self.run_window([-1], [cv2.error("window closed")])
        self.assertEqual((waits, rects), (1, 1))


if __name__ == "__main__":
    unittest.main()
