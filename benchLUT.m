%% benchLUT.m — MOST LUT 单步耗时 benchmark
% 目标: 验证 aeroLoadsType=0 (LUT) 模式在 RTHM 实时通路中的单步计算耗时
% 判据: 均值 < 10 ms/step (Phase 3 要求), 峰值 < 40 ms (控制周期)
%
% 方法: 跑 3 次短仿真 (各 30 s, dt=0.01 → 3000 步), 取 wall time 均值除以步数

clear; close all; bdclose('all');
addpath(fileparts(mfilename('fullpath')));

N_RUNS   = 3;
SIM_T    = 30;       % 仿真时间 [s], 3000 步足够统计
DT       = 0.01;     % 与 RTHM 实时通路一致

%% --- 1. simu ---
simu = simulationClass();
simu.simMechanicsFile = 'SModel_RTHM.slx';
simu.mode      = 'normal';
simu.explorer  = 'off';          % 关掉 Mechanics Explorer 加速
simu.solver    = 'ode4';
simu.dt        = DT;
simu.startTime = 0;
simu.endTime   = SIM_T;
simu.rampTime  = 5;
simu.gravity   = 9.80665;
simu.rho       = 1025;
simu.b2b       = 0;
simu.setup();

%% --- 2. wind ---
wind = windClass();
wind.ConstantWindFlag = 0;
wind.WindDataFile = fullfile('mostData','turbSim','WIND_8mps.mat');
assert(isfile(wind.WindDataFile), 'TurbSim 风场文件不存在: %s', wind.WindDataFile);
wind.ComputeWindInput();

%% --- 3. windTurbine (IEA15MW, LUT) ---
windSpeed0 = 8;
load(fullfile('mostData','windTurbine','control','SteadyStates_IEA15MW.mat'))

windTurbine(1) = windTurbineClass('IEA15MW');
windTurbine(1).aeroLoadsType  = 0;     % LUT
windTurbine(1).control        = 0;
windTurbine(1).omega0         = interp1(SteadyStates.ROSCO.SS.WINDSPEED, SteadyStates.ROSCO.SS.ROTSPD,    windSpeed0);
windTurbine(1).bladepitch0    = interp1(SteadyStates.ROSCO.SS.WINDSPEED, SteadyStates.ROSCO.SS.BLADEPITCH,windSpeed0);
windTurbine(1).GenTorque0     = interp1(SteadyStates.ROSCO.SS.WINDSPEED, SteadyStates.ROSCO.SS.TORQUE,    windSpeed0);
windTurbine(1).aeroLoadsName  = fullfile('mostData','windTurbine','aeroloads','Aeroloads_IEA15MW.mat');
windTurbine(1).turbineName    = fullfile('mostData','windTurbine','turbine_properties','Properties_IEA15MW.mat');
windTurbine(1).bladeDataName  = fullfile('mostData','windTurbine','turbine_properties','Bladedata_IEA15MW.mat');
windTurbine(1).controlName    = fullfile('mostData','windTurbine','control','Control_IEA15MW.mat');
windTurbine(1).offset_plane   = [0 0];
windTurbine(1).YawControlFlag = 0;

windTurbine(1).setNumber(1);
windTurbine(1).loadTurbineData();
windTurbine(1).importControl();
windTurbine(1).importAeroLoadsTable();
windTurbine(1).CreateBEMstruct(wind.Xdiscr, wind.Ydiscr, wind.Zdiscr);

%% --- 4. Variant ---
ControlChoice1   = windTurbine(1).control;
sv_t1_control0   = Simulink.Variant('ControlChoice1==0');
sv_t1_control1   = Simulink.Variant('ControlChoice1==1');
AeroLoadsChoice1 = windTurbine(1).aeroLoadsType;
sv_t1_AeroLoads0 = Simulink.Variant('AeroLoadsChoice1==0');
sv_t1_AeroLoads1 = Simulink.Variant('AeroLoadsChoice1==1');
visON = 0;
sv_visualizationON  = Simulink.Variant('visON==1');
sv_visualizationOFF = Simulink.Variant('visON==0');

%% --- 5. Benchmark ---
warning('off','Simulink:blocks:TDelayTimeTooSmall');
warning('off','Simulink:blocks:BusSelDupBusCreatorSigNames');
warning('off','Simulink:blocks:DivideByZero');
warning('off','sm:sli:setup:compile:SteadyStateStartNotSupported');
set_param(0, 'ErrorIfLoadNewModel', 'off');

% Provide dummy data for From Workspace blocks (static platform = A0 工况)
% SModel_RTHM.slx 当前用 From Workspace 喂 Bushing 位姿（Level 2 回放配置）
% benchmark 不需要运动——全零静态即可测量纯 LUT 计算耗时
% From Workspace 格式: [time, signal_cols], 至少 2 行
BushingPos = [0,       zeros(1,6);
              SIM_T,   zeros(1,6)];
BushingVel = [0,       zeros(1,6);
              SIM_T,   zeros(1,6)];
BushingAcc = [0,       zeros(1,6);
              SIM_T,   zeros(1,6)];
BushingMotion = BushingPos;  % 兼容旧版单变量配置

[~, modelName] = fileparts(simu.simMechanicsFile);
nSteps = round((SIM_T - simu.startTime) / DT);

wallTimes = zeros(N_RUNS, 1);

fprintf('========== LUT Benchmark: %d runs × %d s (%d steps each) ==========\n', ...
    N_RUNS, SIM_T, nSteps);
fprintf('Solver: %s, dt=%.3f s, aeroLoadsType=%d, control=%d\n', ...
    simu.solver, simu.dt, windTurbine(1).aeroLoadsType, windTurbine(1).control);

for iRun = 1:N_RUNS
    % 每次重新加载模型，避免编译缓存干扰测量
    bdclose('all');
    load_system(modelName);
    set_param(modelName, 'ReturnWorkspaceOutputs', 'on');
    set_param(modelName, 'SaveOutput',             'on');
    set_param(modelName, 'SaveTime',               'on');
    set_param(modelName, 'SaveFormat',             'Dataset');
    set_param(modelName, 'StopTime', num2str(SIM_T));
    set_param(modelName, 'SimMechanicsOpenEditorOnUpdate', 'off');

    tic;
    simOut = sim(modelName);
    wallTimes(iRun) = toc;

    fprintf('  Run %d/%d: wall %.2f s  →  %.1f ms/step  (×%.1f real-time)\n', ...
        iRun, N_RUNS, wallTimes(iRun), ...
        wallTimes(iRun)/nSteps*1000, SIM_T/wallTimes(iRun));
end

%% --- 6. Summary ---
avgWall  = mean(wallTimes);
stdWall  = std(wallTimes);
avgPerStep = avgWall / nSteps * 1000;  % ms
peakPerStep = max(wallTimes) / nSteps * 1000;
speedup = SIM_T / avgWall;

fprintf('\n========== 结果 ==========\n');
fprintf('  平均 wall time:    %.2f ± %.2f s\n', avgWall, stdWall);
fprintf('  平均 per-step:     %.2f ms  (判据: < 10 ms)\n', avgPerStep);
fprintf('  最差 per-step:     %.2f ms  (判据: < 40 ms)\n', peakPerStep);
fprintf('  vs real-time:      ×%.1f  (>1 = 超实时)\n', speedup);

fprintf('\n  RTHM 实时通路预估:\n');
fprintf('    总步数 (60 s @ dt=0.01): 6000\n');
fprintf('    预估 sim 耗时:  %.1f s\n', avgPerStep * 6000 / 1000);
fprintf('    控制周期 40 ms 内可用 sim 步数: %.0f 步\n', 40 / avgPerStep);

if avgPerStep < 10
    fprintf('\n  ✅ 通过: 平均 < 10 ms/step\n');
elseif avgPerStep < 40
    fprintf('\n  ⚠ 警告: 10 < 平均 < 40 ms, 可接受但需关注\n');
else
    fprintf('\n  ❌ 不通过: 平均 > 40 ms/step, LUT 模式不能满足 25 Hz 控制周期\n');
end

%% --- 7. Save ---
save('benchLUT_result.mat', 'wallTimes', 'avgPerStep', 'avgWall', 'speedup', 'nSteps', 'SIM_T');
fprintf('\n结果已保存到 benchLUT_result.mat\n');
