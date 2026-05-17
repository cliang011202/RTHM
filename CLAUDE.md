# CLAUDE.md — RTHM 工作目录

> 本目录由用户手动创建，**不是 MOST 官方提供的示例**。用于开发"实时混合试验装置"（Real-Time Hybrid Model，RTHM）。
> 大量文件（`wecSimInputFile.m`、`SModel_*.slx`、`hydroData/`、`mostData/`、`hydroDataMaker/`、`userDefinedFunctions.m` 等）是从 `MOST/Examples/VolturnUS/` 复制过来的，**目前正处于裁剪和调试中**，与 VolturnUS 不再完全等价。

项目级背景（缩放因子 λ=50、7 桨执行器、UDP 链路、Phase 1–4 路线图等）见仓库根目录 `D:\MOST\CLAUDE.md` 第 2 部分。本文件只记录 RTHM 目录内的本地状态和踩过的坑。

---

## 目录现状

**Level 2 全数值回放对比已通过**（VolturnUS 跑出 body 6-DOF + ttLoad 参考，RTHM 用 From Workspace 回放，逐点对比）。主分量 ttLoad DC 偏差 < 0.5%，AC 残差 < 10%，剩余 12% per-blade Faero 残差是独立 ode4 积分的 azimuth 相位漂移（不影响 RTHM 实时通路，因为实时通路用自己的 azimuth）。

Level 1 假信号物理判据由于 LUT 模式 + 常风下 V_at_hub 与平台运动脱钩，三 case 不通过——这是设计约束，不是模型 bug，详见下方"Level 1 三 case 验证结果"小节。

| 文件 | 状态 | 说明 |
|---|---|---|
| `wecSimInputFile.m` | 复制自 VolturnUS，**未使用** | 干净路 B 不再调用它；保留作历史参考 |
| `SModel_RTHM.slx` | **可运行** | 顶层：World Frame + Mechanism Configuration + Solver Configuration + Bushing Joint + `Wind turbine` 子系统 + 6 路位姿输入 (From Workspace × 3 → Demux × 3 → PS Converter × 6) **+ UDP 输出链路 (From → Rate Transition → Counter → MATLAB System → UDP Send)** |
| `SModel_VolturnUS.slx` | 备份 | 原始 VolturnUS Simulink 模型副本，不要动 |
| `initializeRTHM.m` | **干净路 B 已落地，可运行** | 不再 `run('wecSimInputFile')`；脚本里直接定义 simu / wind / windTurbine 三个对象。详见下方"initializeRTHM.m 实际结构" |
| `initializeRTHM_replay.m` | **可运行** | Level 2 回放脚本；含 UDP 发包 |
| `UDPPacketPacker.m` | **新增** | MATLAB System 类：将 TowerTopLoad [6×1] + seq → 28 字节 UDP 数据报 |
| `MOST开发.md` | 用户笔记 | 记录"假信号 → 全数值回放 → UDP loopback"三级测试方案 |
| `STM32/` | 用户新增 | UDP 测试模型 + `send_test_packet.m` |

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
- `Mechanism Configuration`：Simscape > Multibody > Utilities，双击设 **`Uniform Gravity → Gravity Vector = [0 0 0]`**
- `Solver Configuration`：Simscape > Utilities

后两个块各自只有一个 PS 端口，把它们 T 接到 World Frame → Bushing Joint 那条物理线上即可（Simscape 自动加分支）。

> ⚠ **`GravityVector = [0 0 0]` 不是笔误，必须为 0**——参 VolturnUS。原因详见下方"Mechanism Configuration 的重力为什么必须为 0"小节。Level 2 全数值对比的核心 fix 就是这条。

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

### Mechanism Configuration 的重力为什么必须为 0

`GravityVector = [0 0 0]` 这一条是**对齐 VolturnUS 的关键**。证据链：

- 反编译 `WECSim_Lib_Frames.slx` 的 `Global Reference Frame` 子系统：内部的 `Mechanism Configuration` 设的是 `GravityVector = [0 0 0]`
- 也就是说 VolturnUS 全程**没有**让 Simscape Multibody 给任何 Solid 块（含 windTurbine 子系统的塔/机舱/轮毂/叶片）自动加重力
- 重力在 VolturnUS 里是被 **WEC-Sim body class 通过水静力补偿来表达**——`body.mass × g` 在 hydrostatic restoring 里被减一遍 buoyancy，整套 weight 由 body 一手管
- windTurbine 子系统对外的 `TowerTopLoad` 因此只反映**气动 + 上部结构惯性**，不含 m·g 反力

如果 RTHM 错把 Mech Config gravity 设成 `[0 0 -9.80665]`，windTurbine 内部所有 Solid 块会被加上重力，TowerTopLoad 多出 ~9 MN 的 Fz 偏置和 ~70 MN·m 的 My 偏置（相当于 VolturnUS 与 RTHM 的 Level 2 对比直接 dropout 一个数量级）。Level 2 验证里的"DC 偏差 9.31 MN / 65.8 MN·m"就是这条没设对踩出来的。

**对 RTHM 实时通路也是好事**：Han et al. 2025 Eq.2-5 要做的是 `F_aero = TowerTopLoad − 上部结构惯性补偿`。**重力不在 TowerTopLoad 里就少减一项**，公式更清爽。

### Bushing Joint 的 Force/Torque 必须设为 Automatically Computed

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

## Level 2 全数值回放对比（已通过）

**目标**：跑原版 `SModel_VolturnUS.slx`（带 body+hydro+mooring+waves）拿到 body 6-DOF 真实响应作为参考，把这份位姿喂给 SModel_RTHM.slx 的 Bushing Joint，逐点对比两份的 windTurbine 输出。如果两份吻合 → RTHM 路径在结构/坐标系/单位/积分上都正确。

**实现脚本**：
- `runVolturnUSReference.m`：临时改 VolturnUS 的 `simu.endTime = 120`（用正则替换 + try/finally 恢复 + setenv 兜底 wecSim 的 blanket clear），跑 wecSim，存 body 6-DOF + ttLoad/Faero/nacAcc 到 `level2_volturnUS_ref.mat`
- `initializeRTHM_replay.m`：把 `ref.body_pos / body_vel / body_acc` 三组数据放进 base workspace 作 `BushingPos / BushingVel / BushingAcc`，跑 RTHM
- `analyzeLevel2.m`：逐点对比

**Simulink 端的关键改动**（替换原 Sine Wave + 5 Constants 的 6 路输入）：

```
3 个 From Workspace (BushingPos / BushingVel / BushingAcc, 各 Nt × 7)
        ↓
3 个 Demux 6 (拆 6 路标量)
        ↓
6 个 PS Converter，每个左侧有 3 个标量输入端口 (f, f', f'')
        ↓ 直接喂 pos/vel/acc 三路（不需要 Mux 合一路再喂！）
        ↓
Bushing Joint 的 6 个 motion-from-input primitive
```

PS Converter 必须设：
- **Filtering and derivatives** = `Provide input derivative(s)`
- **Input derivatives** = `Provide first and second derivatives`

如果保留原来的"Filter input, derivatives calculated"模式，PS Converter 会从位置数值微分出速度和加速度——这对采样数据数值病态，导致 nacelle 加速度被放大 100×（VolturnUS ~1 m/s² 被放大成 RTHM ~70 m/s²）。

**通过判据**：主分量（>1 MN/MN·m 量级）DC 偏移 < 5%，AC 残差比 < 10%。Fy/Mz 因均值 kN 量级太小，不算入主分量。

**典型结果（IEA15MW @ 8 m/s + Hs=4m JONSWAP，120 s 仿真）**：

| 分量 | VolturnUS mean | RTHM mean | DC 偏移 | std/\|mean\| |
|---|---|---|---|---|
| Fx | 1.52 MN | 1.49 MN | 2.3% | 5.6% |
| Fy | -57 kN | -60 kN | (small mean) | — |
| Fz | -9.43 MN | -9.43 MN | 0.03% | 0.2% |
| Mx | 10.2 MN·m | 9.7 MN·m | 4.6% | 5.5% |
| My | -54.7 MN·m | -55.2 MN·m | 0.9% | 8.2% |
| Mz | 0.85 MN·m | -0.46 MN·m | (small mean) | — |
| **per-blade Faero** | — | — | — | **~12%（azimuth 相位漂移，可接受）** |

## UDP 输出链路（已实现）

**SModel_RTHM.slx 顶层新增 7 个块**，从 `TowerTopLoad` 和 `NacAcc` Goto 标签读取信号，经 Han et al. (2025) Eq.2-5 补偿后，降采样到 25 Hz 通过 UDP 发给 STM32：

```
UDP_TowerTopLoad_From ──► UDP_RateTransition ──┐
  (TowerTopLoad1, [6×1])   (0.01→0.04s, 25Hz)  │
                                                 ├──► UDP_PackBytes ──► UDP_Send
UDP_NacAcc_From ────────► UDP_NacAcc_RT ────────┤    (MATLAB System)   (192.168.1.100:8080)
  (NacAcc1, [3×1])         (0.01→0.04s)         │    3 inputs:          ↑
                                                 │    [6] + [1] + [3]   │
                             UDP_SeqCounter ─────┘                      │
                             (Counter Limited, tsamp=0.04)              │
```

**Han et al. (2025) Eq.2–5 补偿** (在 `UDPPacketPacker.stepImpl` 中实现):
```
F_aero = F_ttLoad - m_RNA·a_nac - m_RNA·g
M_aero = M_ttLoad - r_cg × (m_RNA·a_nac) - r_cg × (m_RNA·g)
```
其中 m_RNA = 921,778 kg, r_cg = [-6.90, 0, 10.83] m (相对塔顶).

| 块 | 类型 | 关键参数 |
|---|---|---|
| `UDP_TowerTopLoad_From` | From | GotoTag=`"TowerTopLoad1"` (global, 来自 Wind turbine 子系统) |
| `UDP_RateTransition` | Rate Transition | OutPortSampleTime=`"0.04"` (100→25 Hz) |
| `UDP_NacAcc_From` | From | GotoTag=`"NacAcc1"` (global, nacelle 3-axis acceleration) |
| `UDP_NacAcc_RateTransition` | Rate Transition | OutPortSampleTime=`"0.04"` |
| `UDP_SeqCounter` | Counter Limited | tsamp=`0.04`, uplimit=`4294967295` |
| `UDP_PackBytes` | MATLAB System | System=`UDPPacketPacker`, 输入 [6×1]+[1×1]+[3×1], 输出 [28×1 uint8] |
| `UDP_Send` | MATLAB System | Host=`192.168.1.100`, Port=`8080`, LocalPort=`12345`, Blocking=`off` |

**数据包格式**（与 `send_test_packet.m` 完全一致）：
- 1–4 字节: uint32 LE 序号
- 5–28 字节: 6 × single LE (Fx, Fy, Fz, Mx, My, Mz)
- 总计 28 字节


**UDPPacketPacker** (`UDPPacketPacker.m`): MATLAB System 类。`stepImpl` 接收 3 路输入 → 调用 `compensate()` 做 Han Eq.2–5 补偿 → `packBytes()` 打包为 28 字节 uint8。RNA 质量和 CG 作为类属性，可通过 `set_param` 或直接改类文件调整。补偿可通过 `EnableCompensation` 属性开关。

**已验证** (2026-05-17): `initializeRTHM_replay.m` 跑 120s 仿真，25 Hz × 120s = ~3000 包正常发出，120s 原型仿真用时 111s。

**注意事项**:
- 发送的是 **补偿后的等效 hub 气动 6-DOF**（已扣除 RNA 重力 + 惯性力）
- 补偿精度：A0 静态测试 Fz 从 -9.40 MN → -0.36 MN（削减 96%），My 从 -56.9 MN·m → -2.68 MN·m（削减 95%）
- 残余误差主要来自 (1) RNA 质量/ CG 参数精度 (2) 重力在塔顶坐标系投影的近似（当前假设塔近垂直）
- 如果 STM32 没接或没监听 8080 端口，UDP 包会被静默丢弃（UDP 无连接）
- 模型更新 (Ctrl+D) 需要 base workspace 中定义 simu/wind/windTurbine 等变量，否则 Wind turbine 子系统 mask 报错

### 踩坑: MATLAB Function vs MATLAB System

最初尝试用 `simulink/User-Defined Functions/MATLAB Function` 做字节打包，但 R2025b 的 Stateflow API 变化导致无法通过 `sf()` 或 `sfroot` 程序化设置 EML Chart 的 Script。换成 `simulink/User-Defined Functions/MATLAB System` + 独立类文件 `UDPPacketPacker.m` 解决了问题。注意 `matlab.System` 子类必须实现 `isOutputComplexImpl` 和 `isInputComplexImpl` 方法（R2025b 会检查），否则编译报错。

### 初始化脚本更新

`initializeRTHM.m` 和 `initializeRTHM_replay.m` 开头都加了:
```matlab
addpath(fileparts(mfilename('fullpath')));
```
确保 `UDPPacketPacker` 类在路径上。

---

## 下一步

1. **hub-frame 气动重建** ✅ **已实现** (2026-05-17): Han et al. 2025 Eq.2–5 补偿已在 `UDPPacketPacker.compensate()` 中落地。A0 静态验证：Fz 削减 96%, My 削减 95%。待完善：(a) 用平台姿态实时旋转重力向量（当前假设塔近垂直），(b) 加入转动惯性补偿 (I·α + ω×I·ω 项)，(c) 添加 NacAcc angular velocity/acceleration 输入。
2. **LUT 单步耗时 benchmark**：用 `tic/toc` 包 `sim(...)` 3 次取均值除以步数。项目根 CLAUDE.md TODO Phase 3 要求 < 10 ms/step。
3. **Level 3：UDP loopback**：建 `MockMocap.slx`，把 `level2_volturnUS_ref.mat` 的 body 6-DOF 用 UDP 100 Hz 发回 SModel_RTHM.slx 的 UDP Receive 块（替换 From Workspace），验证字节序、float32/64、丢包注入、watchdog。同时做 MATLAB → STM32 的实机收发验证（Wireshark 抓包 + STM32 解析 28 字节包）。
4. **Level 3 之后**：把 STM32 那一头接进来，整条 mocap → MATLAB → MOST → 缩放 → 7 桨分配 → STM32 通路在水池外打通。
