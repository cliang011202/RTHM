## 1 从WEC-Sim中独立MOST

第 1 级：Simulink 内部假信号（最先做）

不走 UDP，直接在 SModel_RTHM.slx 内部用 Signal Generator / From Workspace 给 MOST 风机模块的位姿端口输入。

- 信号选择：用 正弦 或 带限白噪声，不要纯阶跃或不带限的随机数 —— MOST LUT 内部需要 hub
- 信号选择：用 正弦 或 带限白噪声，不要纯阶跃或不带限的随机数 —— MOST LUT 内部需要 hub 的速度，阶跃会产生无穷大瞬时速度，气动载荷瞬间炸掉，掩盖真正的链路问题。
- 频率取浮式平台的典型范围：surge/sway/heave 0.05–0.15 Hz，roll/pitch/yaw 0.03–0.1 Hz；幅值参考 VolturnUS 在 4 m 有义波高下的响应（surge ~5 m，pitch ~3°）。
- 要验证的事： a. windTurbine 模块输入端口的 量纲/坐标系/欧拉角顺序 是否和你打算从动捕送过来的一致 b. 气动载荷输出（F_rx, F_ry, F_rz, M_rx, M_ry, M_rz）随平台位姿变化的方向/正负是否合理 c. MOST LUT 单步耗时（[CLAUDE.md](http://CLAUDE.md) TODO Phase 3 要求 < 10 ms）

第 2 级：回放 WEC-Sim 全数值结果（提高真实度）

跑一次原 SModel_VolturnUS.slx（带 body+hydro+mooring 的全数值版），把 body(1) 的 6-DOF 时序保存到 .mat，再用 From File / From Workspace 喂给独立的 SModel_RTHM.slx。

好处：

- 频谱、相关性都是真实的浮式平台响应
- 同一时刻的气动载荷可以和原版做逐点对比，快速暴露坐标系错位、缩放因子写反、欧拉角约定不一致这类隐蔽 bug
- 等同于一个"参考真值数据集"，后续每次改链路都能回归测试

第 3 级：UDP loopback（验证真实链路）

再开一个 Simulink 模型 MockMocap.slx，从一份保存的 6-DOF 时序里读数据，按 100 Hz 通过 UDP 环回（127.0.0.1）发给 SModel_RTHM.slx。

可以在不接动捕的情况下验证：

- UDP 收发块配置（端口、字节序、float32 vs float64）
- 数据丢失/抖动的处理（设置丢包率注入）
- watchdog 行为（CLAUDE.mdTODO 里有"2 周期未收到包→ESC idle"）
- 动捕→MATLAB→MOST 这一段端到端延迟（ping + 时间戳法）

到第 3 级再把 STM32 那一头的 STM32/UDPtest.slx 接进来，整条 mocap → MATLAB → MOST → 缩放 → 7 桨分配 → STM32 就贯通了，全程都可以在水池外完成。

一些容易翻车的点

- MOST 的 hub 位姿不是平台 COG：windTurbine 的位姿是 hub 中心，需要把动捕给的平台 COG 通过 tower.height + nacelle.Twr2Shft + hub.overhang 投影到 hub。第 1 级假信号阶段就要把这条变换写好。
- 角度单位：MOST 内部多用 rad，动捕 SDK 常给 deg，量纲转换错了会让气动方向反过来但量级看起来"差不多对"，最难查。
- 缩放因子位置：建议把 ×λ / ÷λ³ 等所有缩放都放在两个独立 Subsystem 里（MotionScaler、LoadScaler），不要散落在各处，方便单元验证。
- Solver 选择：独立 RTHM 模型里没有 Simscape 状态了，可以从 ode4 进一步切到纯离散步（Fixed-step discrete），单步会更快、抖动更小。