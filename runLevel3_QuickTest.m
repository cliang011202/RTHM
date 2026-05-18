%% runLevel3_QuickTest.m — Level 3 快速验证（异步启动 + 主线程持续发包）
%
% 关键约束：UDP Receive 的 socket 只在 sim 运行时存在。
% 方案：set_param('start') 异步启动仿真 → 主线程循环发包 + 轮询状态

clear; close all; bdclose('all');
addpath(fileparts(mfilename('fullpath')));

%% --- 0. 参数 ---
SIM_T   = 60;           % 仿真时间（秒）
N_TEST  = round(SIM_T / 0.01);  % 6000 步

%% --- 1. 加载数据 + 预打包 ---
refFile = 'level2_volturnUS_ref.mat';
assert(isfile(refFile), '%s 不存在；先跑 runVolturnUSReference.m', refFile);
ref = load(refFile);
assert(length(ref.t) >= N_TEST, '参考数据不够 %d 步', N_TEST);

packer = MocapPacketPacker();
packets = cell(N_TEST, 1);
fprintf('预打包 %d 个 UDP 包 (%.0f KB) ... ', N_TEST, N_TEST*76/1024);
tic;
for k = 1:N_TEST
    packets{k} = packer(ref.body_pos(k,:)', ref.body_vel(k,:)', ref.body_acc(k,:)', k);
end
fprintf('%.1f s\n', toc);

%% --- 2. simu / wind / windTurbine（同 initializeRTHM_replay）---
simu = simulationClass();
simu.simMechanicsFile = 'SModel_RTHM.slx';
simu.mode      = 'normal';
simu.explorer  = 'off';
simu.solver    = 'ode4';
simu.dt        = 0.01;
simu.startTime = 0;
simu.endTime   = SIM_T;
simu.rampTime  = ref.simuMeta.rampTime;
simu.gravity   = 9.80665;
simu.rho       = 1025;
simu.b2b       = 0;
simu.setup();

wind = windClass();
wind.ConstantWindFlag = 0;
wind.WindDataFile = fullfile('mostData','turbSim','WIND_8mps.mat');
wind.ComputeWindInput();

windSpeed0 = 8;
load(fullfile('mostData','windTurbine','control','SteadyStates_IEA15MW.mat'));
windTurbine(1) = windTurbineClass('IEA15MW');
windTurbine(1).aeroLoadsType  = 0;
windTurbine(1).control        = 0;
windTurbine(1).omega0         = interp1(SteadyStates.ROSCO.SS.WINDSPEED, SteadyStates.ROSCO.SS.ROTSPD, windSpeed0);
windTurbine(1).bladepitch0    = interp1(SteadyStates.ROSCO.SS.WINDSPEED, SteadyStates.ROSCO.SS.BLADEPITCH, windSpeed0);
windTurbine(1).GenTorque0     = interp1(SteadyStates.ROSCO.SS.WINDSPEED, SteadyStates.ROSCO.SS.TORQUE, windSpeed0);
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

ControlChoice1   = windTurbine(1).control;
sv_t1_control0   = Simulink.Variant('ControlChoice1==0');
sv_t1_control1   = Simulink.Variant('ControlChoice1==1');
AeroLoadsChoice1 = windTurbine(1).aeroLoadsType;
sv_t1_AeroLoads0 = Simulink.Variant('AeroLoadsChoice1==0');
sv_t1_AeroLoads1 = Simulink.Variant('AeroLoadsChoice1==1');
visON = 0;
sv_visualizationON  = Simulink.Variant('visON==1');
sv_visualizationOFF = Simulink.Variant('visON==0');

warning('off','Simulink:blocks:TDelayTimeTooSmall');
warning('off','Simulink:blocks:BusSelDupBusCreatorSigNames');
warning('off','Simulink:blocks:DivideByZero');
warning('off','sm:sli:setup:compile:SteadyStateStartNotSupported');
set_param(0, 'ErrorIfLoadNewModel', 'off');

%% --- 3. 异步启动仿真 ---
[~, modelName] = fileparts(simu.simMechanicsFile);
load_system(modelName);
set_param(modelName, 'ReturnWorkspaceOutputs', 'on');
set_param(modelName, 'SaveOutput',             'on');
set_param(modelName, 'SaveTime',               'on');
set_param(modelName, 'SaveFormat',             'Dataset');
set_param(modelName, 'StopTime', num2str(SIM_T));

fprintf('\n异步启动仿真...\n');
set_param(modelName, 'SimulationCommand', 'start');
pause(0.5);  % 等 Simulink 初始化完成、UDP socket 建立

%% --- 4. 主线程循环发包 ---
sender = dsp.UDPSender('RemoteIPAddress', '127.0.0.1', 'RemoteIPPort', 10001);
sendIdx = 1;
pollInterval = 0.001;  % 1 ms 轮询间隔（由 OS 调度实际精度）
lastReport = 0;

fprintf('开始发包 (%d 包, poll=%.0f ms)...\n', N_TEST, pollInterval*1000);
tic;
while true
    status = get_param(modelName, 'SimulationStatus');
    if strcmp(status, 'stopped') || strcmp(status, 'terminating')
        break;
    end

    if sendIdx <= N_TEST
        sender(packets{sendIdx});
        sendIdx = sendIdx + 1;
    end

    % 每 1000 包报告
    if sendIdx - lastReport >= 1000
        simTime = get_param(modelName, 'SimulationTime');
        fprintf('  已发送 %d/%d, simTime=%.1f s\n', sendIdx-1, N_TEST, str2double(simTime));
        lastReport = sendIdx;
    end

    pause(pollInterval);
end
elapsed = toc;
release(sender);

fprintf('发包循环结束: 共发送 %d/%d 包, wall %.1f s (%.0f pkt/s)\n', ...
    sendIdx-1, N_TEST, elapsed, (sendIdx-1)/elapsed);

%% --- 5. 获取仿真输出 ---
% sim() 返回的 simOut 只在同步调用时可用；异步模式需从 base workspace 或 To Workspace 取
% SModel_RTHM 有 To Workspace 块 windTurbine1_out
if evalin('base', 'exist(''windTurbine1_out'',''var'')==1')
    out = evalin('base', 'windTurbine1_out');
elseif evalin('base', 'exist(''simOut'',''var'')==1')
    simOut = evalin('base', 'simOut');
    if isa(simOut, 'Simulink.SimulationOutput') && isprop(simOut, 'windTurbine1_out')
        out = simOut.windTurbine1_out;
    else
        error('无法从 simOut 获取 windTurbine1_out');
    end
else
    error('找不到 windTurbine1_out');
end

data = out.signals(1).values;
t_sim = out.time;
ttLoad_L3 = data(:, 17:22);

%% --- 6. 对比 Level 2 ---
l2File = 'level2_RTHM_replay.mat';
if isfile(l2File)
    l2 = load(l2File);
    fprintf('\n=== Level 2 vs Level 3 快速对比 (%.0f s) ===\n', SIM_T);

    tCommon = min(t_sim(end), l2.t(end));
    mask3 = t_sim <= tCommon;
    mask2 = l2.t <= tCommon;

    labels = {'Fx','Fy','Fz','Mx','My','Mz'};
    fprintf('%-4s  %-14s %-14s %-10s %-10s\n', 'DOF', 'L2 mean', 'L3 mean', 'DC diff', 'AC diff');
    fprintf('%s\n', repmat('-',1,60));

    for k = 1:6
        l2m = mean(l2.ttLoad_RTHM(mask2, k));
        l3m = mean(ttLoad_L3(mask3, k));
        l2s = std(l2.ttLoad_RTHM(mask2, k));
        l3s = std(ttLoad_L3(mask3, k));
        dc = abs(l3m - l2m) / max(abs(l2m), 1);
        ac = abs(l3s - l2s) / max(l2s, 1e-6);

        flag = '';
        if dc > 0.05 && abs(l2m) > 1e3, flag = ' ⚠DC'; end
        if ac > 0.15 && l2s > 1e3, flag = [flag ' ⚠AC']; end
        fprintf('%-4s  %+14.3e %+14.3e %9.2f%% %9.2f%%%s\n', ...
            labels{k}, l2m, l3m, dc*100, ac*100, flag);
    end
end

%% --- 7. 画图 ---
figure('Name', sprintf('Level 3 QuickTest — %.0fs', SIM_T));
for k = 1:6
    subplot(3,2,k);
    if isfile(l2File)
        plot(l2.t, l2.ttLoad_RTHM(:,k), 'b-', 'DisplayName', 'L2');
        hold on;
    end
    plot(t_sim, ttLoad_L3(:,k), 'r--', 'DisplayName', 'L3');
    grid on;
    ylabel(sprintf('%s', labels{k}));
    if k==1, legend; end
    if k>=5, xlabel('t [s]'); end
end
sgtitle('Level 3 QuickTest — TowerTopLoad');
