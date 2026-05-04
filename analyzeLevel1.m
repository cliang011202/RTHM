%% analyzeLevel1.m - Level 1 三 case 基线相减判据
%
% 前置：分别跑 initializeRTHM.m 三次（isCase = 'A0' / 'A1' / 'A2'，
% Simulink Sine Wave 配置对应改），生成
%   level1_A0.mat   (静止 + 常风 8 m/s)
%   level1_A1.mat   (surge sine, px = 5 * sin(2π·0.08·t))
%   level1_A2.mat   (pitch sine, qy = deg2rad(3) * sin(2π·0.05·t))
%
% 本脚本：从 A0 取稳态 ttLoad 基线，从 A1/A2 减掉，再算与运动输入的相关性。
% 对 A2，额外做一次 dttMy ~ qy 的线性最小二乘拟合，扣掉重力投影的 m·g·h_cg·qy
% 主导项，剩下才是真正的气动俯仰响应。

clear; close all;

%% --- 1. 加载三个 case ---
files = {'level1_A0.mat', 'level1_A1.mat', 'level1_A2.mat'};
for k = 1:3
    assert(isfile(files{k}), '%s 不存在；先跑对应 case 的 initializeRTHM.m', files{k});
end
A0 = load(files{1});
A1 = load(files{2});
A2 = load(files{3});

t = A0.t;
assert(isequal(t, A1.t) && isequal(t, A2.t), '三个 case 时间轴不一致');

%% --- 2. A0 稳态基线 ---
mask0 = t > A0.simuMeta.rampTime + 5;
tt0   = mean(A0.ttLoad(mask0,:), 1);    % 1×6 baseline
ttLbl = {'Fx','Fy','Fz','Mx','My','Mz'};

fprintf('========== A0 稳态 ttLoad 基线 ==========\n');
for k = 1:6
    fprintf('  tt0_%s = %+.3e\n', ttLbl{k}, tt0(k));
end

%% --- 3. A1: surge sine 减基线 + dttFx 与 v_surge 相关性 ---
% 假信号解析重构（与 Simulink Sine Wave block 参数严格一致）
A1_amp_px  = 5;                 % m
A1_freq_px = 0.08;              % Hz
px_A1 = A1_amp_px * sin(2*pi*A1_freq_px * t);
vx_A1 = A1_amp_px * 2*pi*A1_freq_px * cos(2*pi*A1_freq_px * t);   % 解析速度

dttLoad_A1 = A1.ttLoad - tt0;          % 全 6 列减基线
dttFx_A1   = dttLoad_A1(:,1);

mask = t > A1.simuMeta.rampTime + 5;

cc1   = corrcoef(vx_A1(mask), dttFx_A1(mask));
p1    = polyfit(vx_A1(mask),  dttFx_A1(mask), 1);   % dttFx ≈ p1(1)·vx + p1(2)
% 线性气动估计参考值
rho_air = 1.225;
D       = 240;        % IEA15MW rotor diameter
A_rotor = pi * D^2/4;
V0      = 8;
dFdV_th = rho_air * A_rotor * V0;                    % ≈ 4.4e5 N/(m/s)（一阶估计）

fprintf('\n========== A1: surge sine ==========\n');
fprintf('  v_surge 幅值      = %.2f m/s\n', max(abs(vx_A1)));
fprintf('  dttFx std         = %.2e N\n', std(dttFx_A1(mask)));
fprintf('  corr(v_surge, dttFx) = %+.3f   (期望 < 0：顺风走 -> 推力减小)\n', cc1(1,2));
fprintf('  拟合斜率 dttFx/dv = %+.2e N/(m/s)\n', p1(1));
fprintf('  线性气动估计       ≈ −%.1e N/(m/s)（顺风走视风速降，符号应为负）\n', dFdV_th);

%% --- 4. A2: pitch sine 减基线 + 扣重力投影后看气动 ---
A2_amp_qy_deg = 3;              % deg
A2_freq_qy    = 0.05;           % Hz
qy_A2 = deg2rad(A2_amp_qy_deg) * sin(2*pi*A2_freq_qy * t);

dttLoad_A2 = A2.ttLoad - tt0;
dttMy_A2   = dttLoad_A2(:,5);

mask = t > A2.simuMeta.rampTime + 5;

% (a) 不扣重力投影直接相关
cc2_raw = corrcoef(qy_A2(mask), dttMy_A2(mask));

% (b) 扣线性重力投影：拟合 dttMy = a·qy + 残差，残差视为气动响应
%     物理上 a ≈ -m_above·g·h_cg（重力悬臂）
b       = qy_A2(mask) \ dttMy_A2(mask);             % LS 斜率 (N·m/rad)
dttMy_aero_A2 = dttMy_A2 - b * qy_A2;
cc2     = corrcoef(qy_A2(mask), dttMy_aero_A2(mask));

% 用 b 反推 m_above·g·h_cg
% 已知 A0 ttFz ≈ -m·g (上部结构总重)
m_g = abs(tt0(3));      % N
h_cg_est = -b / m_g;    % m

fprintf('\n========== A2: pitch sine ==========\n');
fprintf('  qy 幅值                 = %.2f deg = %.4f rad\n', A2_amp_qy_deg, deg2rad(A2_amp_qy_deg));
fprintf('  dttMy std (raw)         = %.2e N·m\n', std(dttMy_A2(mask)));
fprintf('  dttMy std (aero only)   = %.2e N·m\n', std(dttMy_aero_A2(mask)));
fprintf('  拟合重力斜率 b          = %+.3e N·m/rad\n', b);
fprintf('  反推 h_cg               = %+.2f m   (上部结构 CG 距塔顶高度)\n', h_cg_est);
fprintf('  corr(qy, dttMy)         = %+.3f   (扣 A0 基线，未扣重力投影)\n', cc2_raw(1,2));
fprintf('  corr(qy, dttMy_aero)    = %+.3f   (扣基线+扣重力投影后)\n', cc2(1,2));
fprintf('  注：扣完重力投影后还能看到 qy↔aero 反相，才是 OK\n');

%% --- 5. 画图 ---
figure('Name','Level 1 baseline-subtracted analysis', 'Position', [100 100 1100 700]);

% A1 ttFx 全时序 + 基线
subplot(2,2,1);
plot(t, A1.ttLoad(:,1)/1e6, 'b'); hold on;
yline(tt0(1)/1e6, 'r--', 'LineWidth', 1.5);
ylabel('ttFx [MN]'); xlabel('t [s]');
title('A1: ttFx 与 A0 基线'); grid on; legend('A1','A0 mean','Location','best');

% A1 dttFx vs v_surge
subplot(2,2,2);
yyaxis left;  plot(t, dttFx_A1/1e3); ylabel('dttFx [kN]');
yyaxis right; plot(t, vx_A1, '-');   ylabel('v_{surge} [m/s]');
xlabel('t [s]');
title(sprintf('A1: corr(v_{surge}, dttFx) = %+.3f  期望<0', cc1(1,2)));
grid on;

% A2 ttMy 全时序 + 基线
subplot(2,2,3);
plot(t, A2.ttLoad(:,5)/1e6, 'b'); hold on;
yline(tt0(5)/1e6, 'r--', 'LineWidth', 1.5);
ylabel('ttMy [MN·m]'); xlabel('t [s]');
title('A2: ttMy 与 A0 基线'); grid on; legend('A2','A0 mean','Location','best');

% A2 dttMy_aero vs qy
subplot(2,2,4);
yyaxis left;  plot(t, dttMy_aero_A2/1e6); ylabel('dttMy_{aero} [MN·m]');
yyaxis right; plot(t, rad2deg(qy_A2), '-'); ylabel('qy [deg]');
xlabel('t [s]');
title(sprintf('A2: corr(qy, dttMy_{aero}) = %+.3f  期望<0', cc2(1,2)));
grid on;

%% --- 6. 总结 ---
fprintf('\n========== 总结 ==========\n');
verdict = @(c, thr) ternary(c < thr, '✓ 通过', '✗ 未通过');
fprintf('  A1 (surge):              corr = %+.3f   %s   (阈值 < -0.5)\n', ...
        cc1(1,2),  verdict(cc1(1,2), -0.5));
fprintf('  A2 (pitch, raw):         corr = %+.3f   %s   (阈值 < -0.3)\n', ...
        cc2_raw(1,2), verdict(cc2_raw(1,2), -0.3));
fprintf('  A2 (pitch, aero only):   corr = %+.3f   %s   (阈值 < -0.5)\n', ...
        cc2(1,2), verdict(cc2(1,2), -0.5));
fprintf('\n');

% 帮手函数：MATLAB 没原生 ternary
function out = ternary(cond, a, b)
    if cond, out = a; else, out = b; end
end
