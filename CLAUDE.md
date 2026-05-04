# CLAUDE.md — RTHM 工作目录

> 本目录由用户手动创建，**不是 MOST 官方提供的示例**。用于开发"实时混合试验装置"（Real-Time Hybrid Model，RTHM）。
> 大量文件（`wecSimInputFile.m`、`SModel_*.slx`、`hydroData/`、`mostData/`、`hydroDataMaker/`、`userDefinedFunctions.m` 等）是从 `MOST/Examples/VolturnUS/` 复制过来的，**目前正处于裁剪和调试中**，与 VolturnUS 不再完全等价。

项目级背景（缩放因子 λ=50、7 桨执行器、UDP 链路、Phase 1–4 路线图等）见仓库根目录 `D:\MOST\CLAUDE.md` 第 2 部分。本文件只记录 RTHM 目录内的本地状态和踩过的坑。

---

## 目录现状

**Level 1 假信号测试可运行，三个物理合理性 case 已跑过**（A0 静止+常风、A1 surge sine、A2 pitch sine）。结果暴露了"F_aero 输出语义"和"判据信号选择"两个原本写错的认知，详见下方"Level 1 三 case 验证结果"小节。

| 文件 | 状态 | 说明 |
|---|---|---|
| `wecSimInputFile.m` | 复制自 VolturnUS，**未使用** | 干净路 B 不再调用它；保留作历史参考 |
| `SModel_RTHM.slx` | **可运行** | 顶层只剩 World Frame + Mechanism Configuration + Solver Configuration + Bushing Joint + `Wind turbine` 子系统 + 6 路位姿输入 |
| `SModel_VolturnUS.slx` | 备份 | 原始 VolturnUS Simulink 模型副本，不要动 |
| `initializeRTHM.m` | **干净路 B 已落地，可运行** | 不再 `run('wecSimInputFile')`；脚本里直接定义 simu / wind / windTurbine 三个对象。详见下方"initializeRTHM.m 实际结构" |
| `MOST开发.md` | 用户笔记 | 记录"假信号 → 全数值回放 → UDP loopback"三级测试方案 |
| `STM32/` | 用户新增 | UDP 测试模型 + 测试脚本 |

---

## Level 1 假信号测试当前的搭建方案

目标：跑通 `windTurbine` 子系统，验证它能用虚拟 6-DOF 位姿计算气动载荷。

```
World Frame ──┐
              ├──► Bushing Joint (B)
Mechanism Config ──┤
              ├──► Wind turbine (w.r.f)  via Bushing Joint (F)
Solver Config ───┘  (这两个块各只有一个 PS 端口，T 接到物理网络任意位置即可)

Sine Wave ──► PS Converter (二阶导) ──► px (surge)
Constant 0 ──► PS Converter (二阶导) ──► py / pz / qx / qy / qz
```

### 顶层三块底座（不要再用 Global Reference Frame）

WEC-Sim 自带的 `Global Reference Frame` 子系统**不能直接复用**——它内部的 `Rigid Transform` mask 引用 `waves(1)` / `body(1)` 的字段做可视化 offset，RTHM 没有这两个对象，编译会报 `TranslationCartesianOffset ... 索引超出数组范围`。

正确做法：从 Simscape 库里拖三个**独立**的块到顶层：
- `World Frame`：Simscape > Multibody > Frames and Transforms
- `Mechanism Configuration`：Simscape > Multibody > Utilities，双击设 `Uniform Gravity = [0 0 -9.80665]`
- `Solver Configuration`：Simscape > Utilities

后两个块各自只有一个 PS 端口，把它们 T 接到 World Frame → Bushing Joint 那条物理线上即可（Simscape 自动加分支）。

### 关键确认：`w.r.f` 是 Simscape Multibody frame 端口

进入 `windTurbine` 子系统能看到 `w.r.f` 是六边形 frame 连接器（编号 1）。所以**必须**用 Simscape multibody（如 Bushing Joint）来喂位姿，**不能**用 Bus / Mux 信号直接接进去。

### 子系统内部已经算好的载荷输出
进 `windTurbine` 子系统，能看到这些已经存在的载荷信号源——**不需要自己加传感器**：

| 块名 | 输出 | 物理含义 |
|---|---|---|
| `TowerBaseLoad` | `fc`, `tc` | 塔基处的 6-DOF 反力 / 反力矩 |
| `TowerTopLoad and deltaYaw` | `ft`, `tt` | **塔顶处**的 6-DOF 反力 / 反力矩（含塔的惯性、重力、陀螺反力） |
| `Aerodynamics + Control` | `F_aero` | **per-blade**、**叶片根坐标系**下的 6-DOF 气动载荷（不是 hub 总气动） |

`Outputs` 子系统（紫色边框）通过一个 17 端口 Mux 把 14 路信号合并成一个 [Nt × 49] 的矩阵输出到 `windTurbine1_out` 变量，无 label。**列序在 `initializeRTHM.m` section 6 的注释里有完整对照表**，必须按该映射取列。

### **F_aero 与 TowerTopLoad 的关系——RTHM 实现的核心区分**

> ⚠️ 之前误以为 "MOST 的 F_aero 是轮毂中心总气动 6-DOF 载荷"——**错的**。源码 `Aerodynamics + Control / Goto_F_aero` 的 propagated signal 是 `F_aero_r_bl`（rotor frame, by blade），即**每个叶片在自己根部坐标系下**的 6-DOF 气动载荷。三个叶片的 6-DOF 简单相加在物理上**没有意义**（坐标系不重合），数量级也明显不对（A0 静止 8 m/s 常风下，sum 出来的 M_y ≈ 1.1×10⁸ N·m，应该 ≈ 0）。

正确理解：

- **`F_aero[i]`（i=1..3）**：第 i 片叶片根部 6-DOF 气动载荷。每个叶片单独看是合理的（典型 IEA15MW @ 8 m/s flap 弯矩约 35–45 MN·m），但三个叶片的根部坐标系互成 120°，向量直接相加是错的。MOST 没有暴露 hub-frame 的总气动信号。
- **`ft / tt`（TowerTopLoad，cols 17:22）**：塔顶 6-DOF 反力，**包含**气动载荷传到塔顶的部分 + 塔上部结构（机舱+轮毂+叶片）的惯性 + 重力 + 陀螺反力。
- 两者通过测量补偿联系（Han et al. 2025 Eq.2–5）：实物试验中 6 轴力传感器读到 ft/tt，扣除上部结构的惯性力 + 重力（在塔顶坐标系下随平台姿态而变）才能反推出近似的 hub 气动。

**RTHM 的 UDP 输出策略**：发给 STM32 的应该是 **`TowerTopLoad - 上部结构惯性补偿 - 重力补偿`** 在 hub-fixed 坐标系下重投影后的 6-DOF 向量，**不是** sum of per-blade `F_aero`，也不是 raw `TowerTopLoad`。补偿系数（上部结构总质量、CG 位置、惯量张量）从 `windTurbine.tower / nacelle / hub / blade` 属性算。

### TowerTopLoad 的静态量级（A0 8 m/s 常风、平台静止）

| 分量 | 实测均值 | 主导项 | 物理解释 |
|---|---|---|---|
| ttFx | +1.44 MN | 气动推力 | F_thrust ≈ 0.5 ρ A V² Cₜ |
| ttFy | +600 N | ≈ 0 | 无侧向气动 |
| ttFz | -18.8 MN | 上部结构重力 m·g | RNA + 部分塔身 ~1900 t |
| ttMx | +11.4 MN·m | 气动转矩 | 与 LUT 给出的 Mₐₑᵣₒ 一致 |
| ttMy | -127.5 MN·m | **重力 × CG 偏移**主导，非气动 | m·g × overhang(~6.7 m) |
| ttMz | -1.2 MN·m | 偏航气动+少量重力分量 | |

**关键提醒**：`ttMy` 在静态下被重力偏置主导，不是气动响应。要看气动 pitching moment，**必须做 (case − A0) 减法或者解析扣除 m·g·sin(qy)·d 项**。

---

## 必须配置的细节（漏掉就跑不出正确结果）

### 1. PS Converter 必须开启二阶导
Bushing Joint 的 6 个 DOF 在 motion-driven 模式下，Simscape 内部需要 [位置, 一阶导, 二阶导]。每个 `Simulink-PS Converter` 双击 → **Input Handling** 选项卡：
- **Filtering and derivatives**: `Filter input, derivatives calculated`
- **Input filtering order**: `Second-order filtering`
- 时间常数取 ≪ 信号周期，例如 0.01 s

不配的话 hub 速度/加速度恒为 0，气动方向只随位置变，结果错。

替代方案：在 Simulink 端就把 [位置, 速度, 加速度] 三路都准备好，PS Converter 设成 `Provide input derivative(s)`。

### 2. Sine Wave 参数（针对 VolturnUS surge 测试）
不要用阶跃或不带限白噪声——MOST LUT 内部需要 hub 速度，瞬时无穷大速度会让气动载荷炸掉。推荐：
- **幅值**: 5 m（参考 VolturnUS 在 Hs=4 m 下的 surge 响应）
- **频率**: 0.08 Hz（≈12.5 s 周期，接近 surge 自然周期）
- **Phase**: 0
- **Sample time**: 0（连续）

频率范围参考：surge/sway/heave 0.05–0.15 Hz，roll/pitch/yaw 0.03–0.1 Hz；roll/pitch/yaw 输入要用 **rad**（MOST 内部统一用 rad）。

### 3. windTurbine 模块期望的位姿语义
模块内部已经做了 `Offset plane → Offset Z → Tower Height → Yaw Joint → Twr2shaft+Tilt → Overhang` 的链式变换。所以 `w.r.f` 端要喂的是**平台参考点**（≈ SWL 处的 platform origin）的 6-DOF 位姿，**不是 hub 中心**。Bushing Joint 的 F 帧位置语义要和原 VolturnUS 模型里 `body(1)` 的参考点保持一致。

### 4. Bushing Joint 的 Force/Torque 必须设为 Automatically Computed

每个 motion-from-input 的 primitive 必须配一个 auto-computed 反力/反力矩，否则 Simscape 编译期会报 "fewer joint primitive degrees of freedom with automatically computed force or torque (1) than with motion from inputs (7)"。

操作：双击 `Bushing Joint`，对 6 个 primitive（Px / Py / Pz + 3 个旋转 primitive）逐一展开 **Actuation**：
- **Motion**: `Provided by Input` ✓
- **Force / Torque**: **`Automatically Computed`**（默认是 `None`，必须改）

旋转 primitive 的字段实际叫 **Torque**，不要漏。改完后块右侧会**多出 6 个 PS 输出端口**（每个 primitive 的反力/反力矩），不用就接 `PS Terminator`。**留着**会更好——后续做"全数值平台 vs RTHM 假信号"对比验证时，这些反力直接就是平台对风机的反作用。

### 为什么 motion-input 计数是 7 而不是 6

Wind turbine 子系统内部的 `Yaw Joint` 也是 motion-driven primitive——即使 `windTurbine.YawControlFlag = 0`（无主动偏航控制），`DeltaYaw` 输入仍以常值 0 接入，仍然算一个 motion-input DOF。所以全网络合计 6（Bushing）+ 1（Yaw）= 7 motion-input DOF。Generator Joint 是 torque-driven（Gen Torque 输入），不计入 motion-input。

---

## initializeRTHM.m 实际结构（干净路 B，已落地）

脚本不再 `run('wecSimInputFile')`，所有对象在脚本内直接构造。完整流程分 5 段：

1. **simu**：实例化 `simulationClass`，设置 `simMechanicsFile / solver / dt / endTime / rampTime / gravity / rho / b2b`，最后调用 `simu.setup()` 填充 `simu.time / maxIt / cicTime` 等 mask 引用字段。**`simu.setup()` 不能省**，否则 Simulink mask 找不到时间相关参数会编译失败。
2. **wind**：实例化 `windClass`，设置 `WindDataFile`（用 `assert(isfile(...))` 提前校验存在），调用 `wind.ComputeWindInput()`。
3. **windTurbine**：实例化 `windTurbineClass('IEA15MW')`，设置参数后调用 `setNumber → loadTurbineData → importControl → importAeroLoadsTable → CreateBEMstruct`（顺序与 `initializeWecSim.m` L217–231 一致，但只针对单台 LUT 模式）。
4. **Variant 控制变量**：`ControlChoice1 / sv_t1_control0/1`、`AeroLoadsChoice1 / sv_t1_AeroLoads0/1`，外加 `visON=0 / sv_visualizationON / sv_visualizationOFF`。这些必须是 base workspace 变量，编译期 Simulink 用它们做 Variant 选择。
5. **跑仿真**：`warning('off',...)` 一组 + `set_param(0,'ErrorIfLoadNewModel','off')` + `sim(simu.simMechanicsFile)`。

### 易踩的几个坑

- **`simu.rho = 1025`** 是水密度（即使没 body 也保持），气动用的空气密度在 `windTurbine.BEMdata.rho_air = 1.225`，**别混淆**——把 `simu.rho` 改成 1.225 不会改气动结果，但会让其它引用 `simu.rho` 的代码静默错算。
- **`visON` / `sv_visualizationON/OFF`** 这 3 行即使顶层用了三块底座（不再有 Global Reference Frame）也建议保留——`Wind turbine` 子系统深层有可能间接引用，留着无害，删了再炸更费时。
- **`simu.checkInputs()`** 和 `waves(iW).checkInputs()` **没有调用**——前者目前看不需要，后者根本没 waves。如果以后报"未初始化"类错误再补。
- **`simu.loadSimMechModel(...)`** 是原 `initializeWecSim.m` 用来设 mask 的——干净路里靠 `sim()` 自动加载模型 + base workspace 变量已够，**不必调用**，调用反而会在缺 body block 时报错。

---

## 已踩过的坑（按顺序）

模型从光秃秃到能跑通，按时间顺序遇到过这些错，每条都可能再次遇到——快速对照表：

| 报错信息（关键句） | 根因 | 解法 |
|---|---|---|
| `每个物理网络必须且只能连接到一个 Solver Configuration 模块` | 顶层缺 Solver Configuration | 加 `Solver Configuration`（Simscape > Utilities） |
| `变体模块 ... waveVis ... sv_visualizationON 必须为 ... Simulink.VariantExpression` | 缺 viz 变量 | 脚本里加 `visON=0 / sv_visualizationON/OFF` 三行 |
| `计算 ... Rigid Transform ... TranslationCartesianOffset ... 索引超出数组范围` | 用了 Global Reference Frame，其内部 mask 引用 `waves(1)` / `body(1)` | 不要用 Global Reference Frame；改成三块独立底座 |
| `fewer joint primitive degrees of freedom with automatically computed force or torque (1) than with motion from inputs (7)` | Bushing Joint 的 6 个 primitive Force/Torque 都是 None | 全部改为 `Automatically Computed` |

## Level 1 三 case 验证结果（已完成）

跑过 3 个 case 都用常风 8 m/s + IEA15MW + LUT 模式，sim 时长 60 s，rampTime = 5 s。判据信号统一用 `TowerTopLoad`（cols 17:22）。

| Case | 平台位姿 | 关键判据 | 实测 | 结论 |
|---|---|---|---|---|
| **A0** 静止+常风 | 6 路全 0 | ttFx ≈ 1 MN，ttMy ≈ 0 | ttFx = +1.44 MN ✓<br>**ttMy = -127 MN·m**（重力偏置） | ttMy 静态被重力主导，**不能直接当气动判据** |
| **A1** surge sine | px = 5 sin(2π·0.08·t)，余 0 | corr(v_surge, ttFx) < 0 强相关 | corr = **−0.17** | 方向对、相关性弱（结构惯性反力部分抵消气动响应） |
| **A2** pitch sine | qy = 3°·sin(2π·0.05·t)，余 0 | corr(qy, ttMy) < 0 | corr = **+0.08** | **看不出气动响应**——pitch 让塔顶坐标系旋转，重力投影变化（~MN·m 级）盖过气动小扰动 |

**根因和教训**：

1. **F_aero 列 sum 不等于 hub 气动**——上一节已修正。
2. **A0 的 ttFx ≈ 1.44 MN 比理论估算（~1 MN）偏大约 40%**：可能 LUT 的 Cₜ 在 8 m/s 偏保守，或 IEA15MW LUT 在该工况标定与 OpenFAST 略偏；不影响功能性结论。
3. **A2 失败的真正原因不是模型错**——TowerTopLoad 表达在 *塔顶坐标系*，平台 pitch 时坐标系自己旋转，`m·g·sin(qy)` 投影变化（~1 MN·m 级）远大于气动 pitching moment 扰动（~kN·m 级）。
4. **Level 1 物理合理性判据要做** `(case − A0)` **基线相减**，或者把 ttLoad 先转回 *塔顶 inertial 坐标系*（手算 ⊕ 反向 R(qx,qy,qz)）再做气动判据。

## 下一步调试顺序

1. **重做 A1/A2 判据**：把 `ttLoad − ttLoad_A0_mean` 当作"动态响应"，再算相关性。预期 A1 corr → 强负、A2 corr → 强负。
2. **开始 hub-frame 气动重建**：写一个工具函数 `hubAeroFromTT(ttLoad, motion, params)`，输入 ttLoad + 平台 6-DOF 位姿/速度/加速度 + 上部结构质量/惯量参数，输出 hub-fixed 坐标系下的近似气动 6-DOF。这是 Han et al. 2025 Eq.2–5 的实现，也是 RTHM 通过 UDP 发出去的目标量的来源。
3. **基线 LUT 单步耗时 benchmark**：用 `tic/toc` 包 `sim(...)`，3 次取均值，除以步数。CLAUDE.md TODO Phase 3 要求 < 10 ms/step。
4. **PS Converter 滤波时间常数敏感性扫描**：0.001 / 0.01 / 0.1 s，看对 ttLoad 的影响。
5. **进入 Level 2**：跑原版 `SModel_VolturnUS.slx` 全数值，存 `body(1)` 6-DOF 时序，回放给 RTHM 模型，逐点对比 ttLoad / hub-aero。
6. **进入 Level 3**：UDP loopback（详见 `MOST开发.md`）。
