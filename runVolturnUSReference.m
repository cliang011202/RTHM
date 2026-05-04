%% runVolturnUSReference.m - Level 2 第 1 步：提取 VolturnUS 全数值参考
%
% 跑原版 SModel_VolturnUS.slx（带 body + hydro + mooring + waves + 风机的完整模型），
% 把 body(1) 的 6-DOF 时序 + 塔顶载荷 + per-blade 气动参考存到 level2_volturnUS_ref.mat。
%
% 这个 .mat 之后会用两次：
%   - body_pos (6-DOF) 喂给 SModel_RTHM.slx 的 Bushing Joint motion 输入（回放）
%   - ttLoad_ref / faero*_ref 作为 RTHM 输出的逐点对比基准
%
% 注意：原版 endTime = 1000 s 实跑要 30+ 分钟。这里压到 120 s 做快速验证。
%
% 前置依赖：
%   D:\MOST\MOST\Examples\VolturnUS\hydroData\VolturnUS\hydro.h5  ✓ 已存在
%   D:\MOST\MOST\Examples\VolturnUS\mostData\turbSim\WIND_8mps.mat ✓ 应该存在
%   simu.simMechanicsFile = 'SModel_VolturnUS.slx'

clear; close all; bdclose('all');

origDir    = pwd;
volturnDir = 'D:\MOST\MOST\Examples\VolturnUS';
rthmDir    = 'D:\MOST\MOST\Examples\RTHM';

assert(isfolder(volturnDir), 'VolturnUS 目录不存在: %s', volturnDir);

cd(volturnDir);
fprintf('cwd → %s\n', pwd);

%% --- 1. 跑 VolturnUS 输入文件，定义 simu/waves/body/wind/windTurbine/constraint/mooring ---
run(fullfile(volturnDir, 'wecSimInputFile.m'));

% 时长压缩，避免一跑 30 min
simu.endTime  = 120;        % 跑 120 s（原版 1000）
simu.rampTime = 20;         % 保持原值，给 hydro 时间稳定
fprintf('VolturnUS sim time overridden: endTime = %.0f s, rampTime = %.0f s\n', ...
        simu.endTime, simu.rampTime);

%% --- 2. 跑 wecSim ---
% wecSim 会调 initializeWecSim → sim → stopWecSim → postProcessWecSim
% 完成后 base workspace 里有 output 这个 responseClass 实例
fprintf('\n>>> 正在跑 VolturnUS（建议关其它仿真窗口）...\n');
tStart = tic;
wecSim;
fprintf('VolturnUS run wall time: %.1f s\n', toc(tStart));

assert(exist('output', 'var') == 1, 'wecSim 跑完后没找到 output；postProcess 可能失败');

%% --- 3. 抽取 body(1) 6-DOF + windTurbine(1) 关键时序 ---
ref = struct();
ref.t           = output.bodies(1).time;
ref.body_pos    = output.bodies(1).position;       % [Nt × 6] [px py pz qx qy qz]
ref.body_vel    = output.bodies(1).velocity;       % [Nt × 6]
ref.body_acc    = output.bodies(1).acceleration;   % [Nt × 6]

ref.ttLoad      = output.windTurbine(1).towerTopLoad;       % [Nt × 6] @ tower top
ref.tbLoad      = output.windTurbine(1).towerBaseLoad;      % [Nt × 6] @ tower base
ref.windV       = output.windTurbine(1).windSpeed;          % [Nt × 3]
ref.rotorSpeed  = output.windTurbine(1).rotorSpeed;         % [Nt × 1] RPM
ref.bladePitch  = output.windTurbine(1).bladePitch;         % [Nt × 1]
ref.faero1      = output.windTurbine(1).blade1AeroLoad;     % [Nt × 6] per-blade
ref.faero2      = output.windTurbine(1).blade2AeroLoad;
ref.faero3      = output.windTurbine(1).blade3AeroLoad;

ref.simuMeta    = struct('rampTime', simu.rampTime, ...
                         'endTime',  simu.endTime,  ...
                         'dt',       simu.dt);
ref.note        = sprintf('Recorded from %s on %s. body initial displacement = [%g %g %g].', ...
                          'SModel_VolturnUS.slx', datestr(now), ...
                          body(1).initial.displacement);

%% --- 4. 存到 RTHM 目录 ---
saveFile = fullfile(rthmDir, 'level2_volturnUS_ref.mat');
save(saveFile, '-struct', 'ref');
fprintf('\n参考数据存到: %s\n', saveFile);
fprintf('  时间跨度: 0 - %.0f s, %d 个样本 (dt = %.3f s)\n', ...
        ref.t(end), numel(ref.t), mean(diff(ref.t)));
fprintf('  body_pos 末态: [%+.2f %+.2f %+.2f] m  [%+.2f %+.2f %+.2f] rad\n', ref.body_pos(end,:));
fprintf('  ttLoad   末态: [%+.2e %+.2e %+.2e] N  [%+.2e %+.2e %+.2e] N·m\n', ref.ttLoad(end,:));

cd(origDir);
fprintf('cwd → %s\n', pwd);
