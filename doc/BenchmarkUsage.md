# 自动测试与手动演示：简明指南

## 1. 编译

从项目根目录执行（需要 C++17 编译器、CMake 3.8+、OpenCV、Eigen3、OSQP、OsqpEigen）：

```bash
cmake -S . -B build -DBUILD_TESTING=ON
cmake --build build -j2
ctest --test-dir build --output-on-failure
```

Python 自动测试和报告只使用标准库；原始轨迹窗口回放另需 `cv2`、NumPy 和桌面显示环境。

## 2. 自动测试

```bash
python3 scripts/run_benchmark.py
```

默认先运行动力学示例、两个指定速度后端、6 组固定点，再运行 1000 组随机普通规划，
总共 1009 例，全程不打开窗口。种子按照原 TDT 的 `time(nullptr)` 方式选择并保存。

```bash
# 快速测试或明确指定种子
python3 scripts/run_benchmark.py --count 100 --seed 42

# 只测试随机普通规划；结果目录必须尚不存在
python3 scripts/run_benchmark.py --suite random --count 1000 --output-dir benchmark-results/my-run

# 更换地图，只需指定图片和分辨率，不需要配置文件或修改源码
python3 scripts/run_benchmark.py --map images/inside_map.png --resolution 0.05 --count 1000
```

地图尺寸自动读取。随机点从所有非零自由像素中均匀抽取，起终点不同，允许跨连通区域；
取自由栅格中心，不沿用写死的 28×15 米范围。分辨率默认 0.05 米/像素，普通图片无法推断真实比例。
显式传入 `--map` 时默认只运行随机规划，不把旧地图固定示例用于新地图。
需要旧固定示例时可显式指定 `--suite existing`，越界会在开始执行前报错。

可用参数还包括 `--executable build/play`、`--max-iterations 6`、`--time-limit 10`、
`--max-speed 3`、`--max-acc 1`。速度和加速度只用于普通规划时间分配，不是曲线硬上限；
指定速度示例沿用自己的 8、2 时间分配参数。历史模式不允许覆盖这些参数。

## 3. 查看报告

终端会显示新结果目录，默认在 `benchmark-results/<UTC时间戳>/`。

| 文件 | 用途 |
| --- | --- |
| `summary.md` | 直接阅读通过率、失败原因及各阶段耗时 |
| `summary.csv` | 按测试类型／后端统计通过率，普通规划额外分固定与随机 |
| `timings.csv` | 各阶段的次数、累计、平均、P95、最大耗时；另分成功与失败 |
| `failures.csv` | 失败原因、后端详情和外层限制标志 |
| `results.csv` | 每例结果、失败原因、总耗时、阶段耗时、碰撞数量与 OSQP 状态 |
| `cases.csv` | 起终点清单，包含内部坐标与按原项目约定翻转后的外部坐标 |
| `inputs.json` | 自动生成的完整输入、种子与参数快照，复现时直接读取，不需要手工编辑 |
| `cases/<编号>.json` | 当次前端路径、候选轨迹、区域及动力学采样值 |
| `paths.csv`、`corridors.csv` | 轨迹和区域的合并表 |
| `manifest.json` | 地图／输入／程序校验值、依赖信息、运行状态及整批耗时 |
| `map.png`、`run.log` | 原地图字节副本、完整终端输出 |

功能与单次总耗时单位均为 **ms**，由 C++ 单调时钟记录。总耗时从规划初始化至验证，
不含读图、绘图、等待按键和结果写盘；各阶段之外的少量编排开销也包含在总耗时中。
底层算法自身的内部诊断开销仍包含在对应调用内。整批墙钟耗时由 Python 另行记录，包含进程和报告开销。
未执行阶段为空，不作为零值参与平均；P95 使用排序后的最近秩。

`all` 表示全部示例，`ordinary_all` 只包括固定与随机普通规划，`random` 是随机普通规划。
整体通过率包含前端无路样例；`frontend_pass_rate_pct` 是普通规划前端成功率，
`backend_conditional_pass_rate_pct` 是进入后端后的最终通过率。
`backend_solve_success_rate_pct` 是后端返回 `success=true` 的比例，**包含碰撞检查**，不是单独的 QP 收敛率。

主要失败原因：

- `frontend_failure`：前端未到达终点，后端未执行。
- `iteration_limit`：外层轮数耗尽，仍有碰撞。
- `time_limit`：一轮完成后发现外层累计超时，仍有碰撞。
- `backend_failure`：后端失败，查看 `backend_failure` 详情与 `osqp_axis/status/error`。
- `sampling_failure`、`input_invalid`、`validation_failure`：采样、输入或最终验证未通过。

两个外层限制可能同时达到，分别查看 `iteration_limit_reached` 和 `time_limit_reached`。
`iterations` 是已完成采样检查的外层轮数，不是 OSQP 内部迭代数。
`collision_segments` 是最后完成检查时碰撞的多项式分段数，`collision_lines` 是对返回轨迹复查的碰撞连线数。
后续求解失败时可能保留前一轮轨迹。`osqp_status=-1` 表示没有可用状态，轴 0/1 为 X/Y，
最后一个轴的状态不代表整条曲线通过。OSQP 1.x 状态 7/8 为内部迭代上限／内部超时。
当前依赖中的内层 0.1 秒 setter 问题没有修复。

## 4. 手动演示与任意历史复现

原来的启动方式继续有效，默认地图路径不再依赖当前运行目录：

```bash
cd build
./play
```

任意按键进入下一例，Q／Esc 或关闭窗口退出。可以通过命令行减少数量或换图：

```bash
./build/play --count 10
./build/play --map images/inside_map.png --resolution 0.05 --count 10
```

**重新执行历史输入**（打开演示窗口；目录可以是任意一次历史运行）：

```bash
python3 scripts/run_benchmark.py --rerun benchmark-results/my-run
python3 scripts/run_benchmark.py --rerun benchmark-results/my-run --case-id 25
python3 scripts/run_benchmark.py --rerun benchmark-results/my-run --from-case 25 --failed-only
```

读取历史地图、参数和精确起终点；种子也保留在新记录中。重跑生成新目录，不覆盖原记录。
加 `--headless` 可无窗口重跑。默认完整测试中编号 1 是动力学示例，2～3 是速度示例，
4～9 是固定普通规划，10 起是随机规划；只运行随机测试时编号从 1 开始。

**直接查看当次保存的轨迹**（不执行算法，显示原始耗时）：

```bash
python3 scripts/run_benchmark.py --replay benchmark-results/my-run --case-id 25
python3 scripts/run_benchmark.py --replay benchmark-results/my-run --failed-only
```

同种子和生成参数可以重新生成相同输入，但不同机器／标准库的随机分布实现不保证一致，
所以历史重跑优先读取保存的精确输入。不同运行的耗时会波动，触及时间限制时最终结果也可能变化；
直接回放保存轨迹可准确查看原始结果。程序版本变化会提示，地图或输入快照校验失败则拒绝复现。

注意：所有轨迹和复现输入使用内部 **X向右、Y向下、单位米** 坐标。
`cases.csv` 的 `world_*` 沿用 `(H-1)*resolution-y_internal` 翻转约定；随机点位于栅格中心，
底行可能显示半格负值，不应通过旧版整数翻转函数再次转换。历史模式直接读取内部坐标。

## 5. 中断与常见问题

- Ctrl+C 会终止子进程并保存已完成样例的报告，运行状态为 `interrupted`。
- 手动退出状态为 `stopped`，不能当作完整批量结果。
- 基准正常执行完成即返回 0，存在规划失败不影响该退出码；配置／程序异常或记录不完整返回非零。
- 报告可以重新生成：`python3 scripts/run_benchmark.py --summarize benchmark-results/my-run`。
- 出现“可执行文件不存在”时先编译；相对图片路径相对于启动目录，建议从项目根目录运行脚本。
- 无桌面环境使用自动测试或 `--rerun ... --headless`；窗口演示需要可用的桌面显示环境。
