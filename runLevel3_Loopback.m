%% runLevel3_Loopback.m — Level 3 UDP loopback: mocap 数据通过 UDP 喂入 RTHM
%
% 前置：
%   1. 已跑过 modifyModelForLevel3.m (SModel_RTHM.slx 已有 UDP Receive 块)
%   2. level2_volturnUS_ref.mat 存在
%
% 原理：
%   - MATLAB timer 在后台以 100 Hz 发送预打包的 mocap UDP 包
%   - sim() 阻塞期间 timer 持续在独立线程发送
%   - SModel_RTHM 的 UDP Receive 接收后解包 → 喂 Bushing Joint
%   - 仿真结果与 Level 2 参考对比

clear; close all; bdclose('all');
addpath(fileparts(mfilename('fullpath')));

% Ensure RTHM directory is on path (for MocapPacketPacker, etc.)
addpath(fileparts(mfilename('fullpath')));

%% --- 0. 加载参考数据 ---
refFile = 'level2_volturnUS_ref.mat';
assert(isfile(refFile), '%s 不存在；先跑 runVolturnUSReference.m', refFile);
ref = load(refFile);

fprintf('=== Level 3 UDP Loopback ===\n');
fprintf('参考数据: %s, %d 步, dt=%.3f s\n\n', refFile, length(ref.t), ref.simuMeta.dt);

%% --- 1. 预打包所有 mocap 数据包 ---
packer = MocapPacketPacker();
nSteps = length(ref.t);
packets = cell(nSteps, 1);
fprintf('预打包 %d 个 UDP 包 ... ', nSteps);
tic;
for k = 1:nSteps
    packets{k} = packer(ref.body_pos(k,:)', ref.body_vel(k,:)', ref.body_acc(k,:)', k);
end
fprintf('完成 (%.1f s)\n', toc);

%% --- 2. 设置 UDP sender (异步) ---
RX_HOST = '127.0.0.1';
RX_PORT = 10001;

% 以 120 Hz 发送（sim 约 ×1.2 real-time，多 20% margin 防止欠速）
% BusyMode='queue' 确保积压回调排队执行而非丢弃
SEND_HZ = 120;
mocapSender = MocapUdpSender(packets, RX_HOST, RX_PORT, SEND_HZ);
fprintf('UDP 发送器: %s:%d (%d 包 @ %d Hz)\n', RX_HOST, RX_PORT, nSteps, SEND_HZ);

%% --- 3. simu (沿用 Level 2 回放参数) ---
simu = simulationClass();
simu.simMechanicsFile = 'SModel_RTHM.slx';
simu.mode      = 'normal';
simu.explorer  = 'off';           % Level 3 关可视化，提速
simu.solver    = 'ode4';
simu.dt        = ref.simuMeta.dt;
simu.startTime = 0;
simu.endTime   = ref.simuMeta.endTime;
simu.rampTime  = ref.simuMeta.rampTime;
simu.gravity   = 9.80665;
simu.rho       = 1025;
simu.b2b       = 0;
simu.setup();

%% --- 4. wind ---
wind = windClass();
wind.ConstantWindFlag = 0;
wind.WindDataFile = fullfile('mostData','turbSim','WIND_8mps.mat');
assert(isfile(wind.WindDataFile), 'turbsim 风场文件不存在: %s', wind.WindDataFile);
wind.ComputeWindInput();

%% --- 5. windTurbine ---
windSpeed0 = 8;
load(fullfile('mostData','windTurbine','control','SteadyStates_IEA15MW.mat'))

windTurbine(1) = windTurbineClass('IEA15MW');
windTurbine(1).aeroLoadsType  = 0;
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

%% --- 6. Variant ---
ControlChoice1   = windTurbine(1).control;
sv_t1_control0   = Simulink.Variant('ControlChoice1==0');
sv_t1_control1   = Simulink.Variant('ControlChoice1==1');
AeroLoadsChoice1 = windTurbine(1).aeroLoadsType;
sv_t1_AeroLoads0 = Simulink.Variant('AeroLoadsChoice1==0');
sv_t1_AeroLoads1 = Simulink.Variant('AeroLoadsChoice1==1');
visON = 0;
sv_visualizationON  = Simulink.Variant('visON==1');
sv_visualizationOFF = Simulink.Variant('visON==0');

%% --- 7. 跑仿真（timer 异步发送 UDP 包）---
warning('off','Simulink:blocks:TDelayTimeTooSmall');
warning('off','Simulink:blocks:BusSelDupBusCreatorSigNames');
warning('off','Simulink:blocks:DivideByZero');
warning('off','sm:sli:setup:compile:SteadyStateStartNotSupported');
set_param(0, 'ErrorIfLoadNewModel', 'off');

[~, modelName] = fileparts(simu.simMechanicsFile);
load_system(modelName);
set_param(modelName, 'ReturnWorkspaceOutputs', 'on');
set_param(modelName, 'SaveOutput',             'on');
set_param(modelName, 'SaveTime',               'on');
set_param(modelName, 'SaveFormat',             'Dataset');
set_param(modelName, 'StopTime', num2str(simu.endTime));

% 启动异步 UDP 发送
fprintf('\n启动 mocap UDP 发送器...\n');
mocapSender.start();
pause(0.2);  % timer 先发几个包填满接收缓冲

fprintf('开始仿真 (%.0f s)...\n', simu.endTime);
tic;
simOut = sim(modelName);
elapsed = toc;
fprintf('仿真完成: wall %.1f s (×%.1f real-time)\n', elapsed, simu.endTime/elapsed);

% 停止 timer
mocapSender.stop();
fprintf('UDP 发送器已停止: 共发送 %d/%d 包\n', mocapSender.SentCount, nSteps);

%% --- 8. 提取 RTHM 输出 ---
out = simOut.windTurbine1_out;
data = out.signals(1).values;
t    = out.time;

c.NacAcc       = 7:9;
c.TowerTopLoad = 17:22;
c.FaeroBlade1  = 31:36;
c.FaeroBlade2  = 37:42;
c.FaeroBlade3  = 43:48;

ttLoad_L3 = data(:, c.TowerTopLoad);
faero1_L3 = data(:, c.FaeroBlade1);
faero2_L3 = data(:, c.FaeroBlade2);
faero3_L3 = data(:, c.FaeroBlade3);

%% --- 9. 与 Level 2 参考对比 ---
% Level 2 参考数据在初始化 RTHM_replay 时存成了 level2_RTHM_replay.mat
% 如果没跑过 initializeRTHM_replay.m，只存 L3 结果
l2File = 'level2_RTHM_replay.mat';
if isfile(l2File)
    l2 = load(l2File);
    fprintf('\n=== Level 2 vs Level 3 对比 ===\n');

    % 对齐时间向量——取公共时间范围
    tCommon = min(t(end), l2.t(end));
    mask3 = t <= tCommon;
    mask2 = l2.t <= tCommon;

    labels = {'Fx','Fy','Fz','Mx','My','Mz'};
    units  = {'N','N','N','N·m','N·m','N·m'};

    fprintf('%-4s  %-12s %-12s %-12s %-12s\n', 'DOF', 'L2 mean', 'L3 mean', 'DC diff', 'AC diff(σ)');
    fprintf('%s\n', repmat('-', 1, 60));

    allPass = true;
    for k = 1:6
        l2Mean = mean(l2.ttLoad_RTHM(mask2, k));
        l3Mean = mean(ttLoad_L3(mask3, k));
        l2Std  = std(l2.ttLoad_RTHM(mask2, k));
        l3Std  = std(ttLoad_L3(mask3, k));

        dcDiff = abs(l3Mean - l2Mean) / max(abs(l2Mean), 1);
        acDiff = abs(l3Std - l2Std) / max(l2Std, 1e-6);

        flag = '';
        if dcDiff > 0.05 && abs(l2Mean) > 1e3
            flag = ' ⚠ DC';
            allPass = false;
        end
        if acDiff > 0.15 && l2Std > 1e3
            flag = [flag ' ⚠ AC'];
            allPass = false;
        end

        fprintf('%-4s  %+12.3e %+12.3e %11.2f%% %11.2f%%%s\n', ...
            labels{k}, l2Mean, l3Mean, dcDiff*100, acDiff*100, flag);
    end

    if allPass
        fprintf('\n✅ Level 3 vs Level 2: DC < 5%%, AC < 15%% — UDP 通路一致\n');
    else
        fprintf('\n⚠ Level 3 vs Level 2: 存在偏差, 需排查\n');
    end

    % 画对比图
    figure('Name', 'Level 2 vs Level 3: TowerTopLoad');
    for k = 1:6
        subplot(3,2,k);
        plot(l2.t, l2.ttLoad_RTHM(:,k), 'b-', 'DisplayName', 'L2 (From Workspace)');
        hold on;
        plot(t, ttLoad_L3(:,k), 'r--', 'DisplayName', 'L3 (UDP)');
        grid on;
        ylabel(sprintf('%s [%s]', labels{k}, units{k}));
        if k==1, legend('Location','best'); end
        if k>=5, xlabel('t [s]'); end
    end
    sgtitle('Level 2 (From Workspace) vs Level 3 (UDP Loopback) — TowerTopLoad');
end

%% --- 10. 保存 ---
save('level3_loopback.mat', 't', 'ttLoad_L3', 'faero1_L3', 'faero2_L3', 'faero3_L3');
fprintf('\n结果已保存到 level3_loopback.mat\n');
