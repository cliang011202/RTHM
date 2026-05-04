%% initializeRTHM_replay.m - Level 2 第 2 步：用 VolturnUS body 6-DOF 回放 RTHM 模型
%
% 与 initializeRTHM.m 的区别：把 6 路位姿输入从 Sine Wave/Constant 换成 From Workspace
% 读取的 body_pos 时序（来自 level2_volturnUS_ref.mat）。
%
% 前置：
%   1. 已跑过 runVolturnUSReference.m，level2_volturnUS_ref.mat 存在
%   2. 已手改 SModel_RTHM.slx，把原来 6 路 (Sine Wave + 5 Constants → 6 PS Converters)
%      改成: From Workspace (变量名 'BushingMotion') → Demux → 6 PS Converters
%      详见下方"Simulink 端改动"

clear; close all; bdclose('all');

%% --- 0. 加载 VolturnUS 参考数据，准备 From Workspace 输入 ---
refFile = 'level2_volturnUS_ref.mat';
assert(isfile(refFile), '%s 不存在；先跑 runVolturnUSReference.m', refFile);
ref = load(refFile);

% From Workspace 接受的最简格式：[time, signal_cols] 的 [Nt × 7] 矩阵
% 6 列分别是 px py pz qx qy qz（与 Bushing Joint primitive 顺序一致）
BushingMotion = [ref.t, ref.body_pos];
assert(size(BushingMotion,2) == 7, 'BushingMotion 应为 Nt×7（time + 6DOF）');

fprintf('回放数据已准备：BushingMotion size = [%d × %d]，时间跨度 0 - %.0f s\n', ...
        size(BushingMotion,1), size(BushingMotion,2), ref.t(end));

%% --- 1. simu (与 VolturnUS 参考时长一致；其它参数与 initializeRTHM.m 相同) ---
simu = simulationClass();
simu.simMechanicsFile = 'SModel_RTHM.slx';
simu.mode      = 'normal';
simu.explorer  = 'on';
simu.solver    = 'ode4';
simu.dt        = ref.simuMeta.dt;          % 与参考一致才能逐点对比
simu.startTime = 0;
simu.endTime   = ref.simuMeta.endTime;     % 与参考一致
simu.rampTime  = ref.simuMeta.rampTime;
simu.gravity   = 9.80665;
simu.rho       = 1025;
simu.b2b       = 0;
simu.setup();

%% --- 2. wind (沿用 VolturnUS 的 turbsim 风场，确保气动条件一致) ---
% VolturnUS 的 wecSimInputFile 用的是 turbsim 风场，回放时也要用同一份才公平
wind = windClass();
wind.ConstantWindFlag = 0;
% 注意：从 VolturnUS 复制过来的 turbsim 文件应当存在于本目录
wind.WindDataFile = fullfile('mostData','turbSim','WIND_8mps.mat');
assert(isfile(wind.WindDataFile), 'turbsim 风场文件不存在: %s', wind.WindDataFile);
wind.ComputeWindInput();

%% --- 3. windTurbine (与 initializeRTHM.m 相同) ---
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

%% --- 4. Variant 控制变量 ---
ControlChoice1   = windTurbine(1).control;
sv_t1_control0   = Simulink.Variant('ControlChoice1==0');
sv_t1_control1   = Simulink.Variant('ControlChoice1==1');
AeroLoadsChoice1 = windTurbine(1).aeroLoadsType;
sv_t1_AeroLoads0 = Simulink.Variant('AeroLoadsChoice1==0');
sv_t1_AeroLoads1 = Simulink.Variant('AeroLoadsChoice1==1');
visON = 0;
sv_visualizationON  = Simulink.Variant('visON==1');
sv_visualizationOFF = Simulink.Variant('visON==0');

%% --- 5. 跑仿真 ---
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

simOut = sim(modelName);

%% --- 6. 取 RTHM ttLoad / faero ---
out = simOut.windTurbine1_out;
data = out.signals(1).values;
t    = out.time;

c.TowerTopLoad = 17:22;
c.FaeroBlade1  = 31:36;
c.FaeroBlade2  = 37:42;
c.FaeroBlade3  = 43:48;

ttLoad_RTHM = data(:, c.TowerTopLoad);
faero1_RTHM = data(:, c.FaeroBlade1);
faero2_RTHM = data(:, c.FaeroBlade2);
faero3_RTHM = data(:, c.FaeroBlade3);

%% --- 7. 保存供 analyzeLevel2.m 分析 ---
save('level2_RTHM_replay.mat', ...
     't', 'ttLoad_RTHM', 'faero1_RTHM', 'faero2_RTHM', 'faero3_RTHM', 'BushingMotion');
fprintf('\nRTHM 回放结果存到 level2_RTHM_replay.mat\n');
