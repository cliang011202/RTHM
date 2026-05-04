%% analyzeLevel2.m - Level 2 第 3 步：逐点对比 VolturnUS 全数值 vs RTHM 回放
%
% 思路：两份运行使用同一份 turbsim 风场 + 同一份 windTurbine 配置 + 同一份 body 6-DOF
% 时序，只是产生这份位姿的方式不同：
%   - VolturnUS: hydro + waves + mooring 解出来的真实平台动力学响应
%   - RTHM:     直接从 .mat 文件回放上述位姿到 Bushing Joint
%
% 期望：两份的 ttLoad / per-blade Faero 应当**逐点接近**（残差 ≈ 数值噪声，
% 因为 windTurbine 子系统、aero LUT、控制器都完全一样）。
% 残差大 → 提示 RTHM 路径上有坐标系/单位/Mux 解码错位等问题。

clear; close all;

%% --- 1. 加载两份数据 ---
assert(isfile('level2_volturnUS_ref.mat'),  '先跑 runVolturnUSReference.m');
assert(isfile('level2_RTHM_replay.mat'),    '先跑 initializeRTHM_replay.m');
ref = load('level2_volturnUS_ref.mat');
rep = load('level2_RTHM_replay.mat');

%% --- 2. 时间轴对齐 ---
% 两份的 dt 应当一致，但样本数可能差 1（取决于 Simulink 是否落最后一个点）
nMin = min(numel(ref.t), numel(rep.t));
t    = ref.t(1:nMin);
assert(max(abs(ref.t(1:nMin) - rep.t(1:nMin))) < 1e-6, '时间轴对不上');

ttRef = ref.ttLoad(1:nMin, :);
ttRep = rep.ttLoad_RTHM(1:nMin, :);

f1Ref = ref.faero1(1:nMin, :);     f1Rep = rep.faero1_RTHM(1:nMin, :);
f2Ref = ref.faero2(1:nMin, :);     f2Rep = rep.faero2_RTHM(1:nMin, :);
f3Ref = ref.faero3(1:nMin, :);     f3Rep = rep.faero3_RTHM(1:nMin, :);

%% --- 3. 残差统计（稳态段） ---
mask = t > ref.simuMeta.rampTime + 5;       % 跳过 ramp
fprintf('========== ttLoad 逐点对比（稳态段） ==========\n');
fprintf('  分量    VolturnUS mean      RTHM mean         残差 std        相对残差\n');
ttLbl = {'Fx','Fy','Fz','Mx','My','Mz'};
for k = 1:6
    a    = ttRef(mask, k);
    b    = ttRep(mask, k);
    res  = b - a;
    rel  = std(res) / max(std(a), 1);
    fprintf('  %2s  %+13.3e   %+13.3e   %12.3e   %5.2f%%\n', ...
            ttLbl{k}, mean(a), mean(b), std(res), 100*rel);
end

fprintf('\n========== per-blade F_aero 残差 (Frobenius) ==========\n');
fprintf('  blade   ||residual||_F / ||ref||_F\n');
for k = 1:3
    rk = eval(sprintf('f%dRef', k));
    pk = eval(sprintf('f%dRep', k));
    rel = norm(pk(mask,:) - rk(mask,:), 'fro') / max(norm(rk(mask,:), 'fro'), 1);
    fprintf('  blade %d   %.4f%%\n', k, 100*rel);
end

%% --- 4. 通过判据 ---
% 经验阈值：稳态段 ttLoad 各分量相对残差 < 5% 算通过
% （windTurbine 子系统几乎完全一致，残差应接近浮点噪声）
maxRelTT = 0;
for k = 1:6
    a   = ttRef(mask, k);
    b   = ttRep(mask, k);
    rel = std(b - a) / max(std(a), 1);
    maxRelTT = max(maxRelTT, rel);
end
fprintf('\n========== 总评 ==========\n');
fprintf('  ttLoad 最大相对残差 = %.2f%%\n', 100*maxRelTT);
if maxRelTT < 0.05
    fprintf('  ✓ 通过：RTHM 路径与全数值一致\n');
elseif maxRelTT < 0.20
    fprintf('  ⚠ 部分通过：残差 5-20%%，可能源于 PS Converter 滤波时延 / 数值积分顺序差异\n');
else
    fprintf('  ✗ 未通过：残差 > 20%%，RTHM 路径有坐标系或量纲错位\n');
end

%% --- 5. 画图 ---
figure('Name','Level 2: ttLoad VolturnUS vs RTHM replay', 'Position', [100 100 1100 700]);
labels = {'F_x [N]','F_y [N]','F_z [N]','M_x [N·m]','M_y [N·m]','M_z [N·m]'};
for k = 1:6
    subplot(3,2,k);
    plot(t, ttRef(:,k), 'b', 'LineWidth', 1.2); hold on;
    plot(t, ttRep(:,k), 'r--', 'LineWidth', 1.0); grid on;
    ylabel(labels{k});
    if k == 1, legend('VolturnUS','RTHM replay','Location','best'); end
    if k>=5, xlabel('t [s]'); end
end
sgtitle('TowerTopLoad: VolturnUS (蓝) vs RTHM replay (红虚线)');

% 残差子图
figure('Name','Level 2: ttLoad 残差', 'Position', [200 200 1100 700]);
for k = 1:6
    subplot(3,2,k);
    plot(t, ttRep(:,k) - ttRef(:,k)); grid on;
    ylabel(['Δ' labels{k}]);
    if k>=5, xlabel('t [s]'); end
end
sgtitle('RTHM − VolturnUS 残差');
