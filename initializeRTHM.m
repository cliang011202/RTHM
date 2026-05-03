%% initializeRTHM - 独立 RTHM 模型的最小初始化脚本（路线 B 干净路）
% 假信号阶段：SModel_RTHM.slx 只保留 windTurbine 子系统 + Bushing Joint，
% 没有 body / waves / mooring / constraint，因此跳过 initializeWecSim.m 里
% 所有与水动力 / 波浪 / 系泊相关的预处理。
clear; close all; bdclose('all');

%% --- 1. simu (only what SModel_RTHM.slx needs) ---
simu = simulationClass();
simu.simMechanicsFile = 'SModel_RTHM.slx';
simu.mode      = 'normal';
simu.explorer  = 'on';
simu.solver    = 'ode4';
simu.dt        = 0.01;
simu.startTime = 0;
simu.endTime   = 60;
simu.rampTime  = 5;
simu.gravity   = 9.80665;
simu.rho       = 1025;       % simu.rho 是水密度（即使没 body 也保持 WEC-Sim 默认）
                             % 气动用的空气密度在 windTurbine.BEMdata.rho_air
simu.b2b       = 0;

% 等价于 initializeWecSim.m 里 simu.setup()，必须显式调用
% 用来填充 simu.time / simu.maxIt / simu.cicTime 等 Simulink mask 引用的字段
simu.setup();

%% --- 2. wind ---
wind = windClass();
% 2.1非定常风场文件
% wind.ConstantWindFlag = 0;
% wind.WindDataFile = fullfile('mostData','turbSim','WIND_8mps.mat');
% assert(isfile(wind.WindDataFile), ...
%     'TurbSim 风场文件不存在: %s', wind.WindDataFile);
% 2.2定长风
wind.ConstantWindFlag    = 1;
wind.V_time_breakpoints  = [0  simu.endTime];
wind.V_modules           = [8  8];                  % 8 m/s 全程恒定
wind.V_directions        = [1 0 0; 1 0 0];          % +X
wind.V_dt                = 0.1;
wind.ComputeWindInput();

%% --- 3. windTurbine ---
windSpeed0 = 8;
load(fullfile('mostData','windTurbine','control','SteadyStates_IEA15MW.mat'))

windTurbine(1) = windTurbineClass('IEA15MW');
windTurbine(1).aeroLoadsType  = 0;     % 0=LUT, 1=BEM (RTHM 必须用 0 满足实时)
windTurbine(1).control        = 0;     % 0=Baseline, 1=ROSCO
windTurbine(1).omega0         = interp1(SteadyStates.ROSCO.SS.WINDSPEED, SteadyStates.ROSCO.SS.ROTSPD,    windSpeed0);
windTurbine(1).bladepitch0    = interp1(SteadyStates.ROSCO.SS.WINDSPEED, SteadyStates.ROSCO.SS.BLADEPITCH,windSpeed0);
windTurbine(1).GenTorque0     = interp1(SteadyStates.ROSCO.SS.WINDSPEED, SteadyStates.ROSCO.SS.TORQUE,    windSpeed0);
windTurbine(1).aeroLoadsName  = fullfile('mostData','windTurbine','aeroloads','Aeroloads_IEA15MW.mat');
windTurbine(1).turbineName    = fullfile('mostData','windTurbine','turbine_properties','Properties_IEA15MW.mat');
windTurbine(1).bladeDataName  = fullfile('mostData','windTurbine','turbine_properties','Bladedata_IEA15MW.mat');
windTurbine(1).controlName    = fullfile('mostData','windTurbine','control','Control_IEA15MW.mat');
windTurbine(1).offset_plane   = [0 0]; % 第 1 级假信号阶段，hub 在原点正上方
windTurbine(1).YawControlFlag = 0;

% 与 initializeWecSim.m L217–231 等价的 windTurbine 预处理
windTurbine(1).setNumber(1);
windTurbine(1).loadTurbineData();
windTurbine(1).importControl();
if windTurbine(1).aeroLoadsType == 0          % LUT
    windTurbine(1).importAeroLoadsTable();
    windTurbine(1).CreateBEMstruct(wind.Xdiscr, wind.Ydiscr, wind.Zdiscr);
elseif windTurbine(1).aeroLoadsType == 1      % BEM (RTHM 不用)
    windTurbine(1).CreateBEMstruct(wind.Xdiscr, wind.Ydiscr, wind.Zdiscr);
else
    error('windTurbine.aeroLoadsType must be 0 (LUT) or 1 (BEM)')
end

%% --- 4. Variant subsystem 控制变量 ---
% 等价于 initializeWecSim.m L478–491
% windTurbine 子系统内部用 sv_t1_control* / sv_t1_AeroLoads* 切换 Baseline/ROSCO 与 LUT/BEM 实现
% 这几个变量必须在 base workspace，否则 Simulink 编译时找不到 Variant 选择条件
ControlChoice1   = windTurbine(1).control;
sv_t1_control0   = Simulink.Variant('ControlChoice1==0');
sv_t1_control1   = Simulink.Variant('ControlChoice1==1');
AeroLoadsChoice1 = windTurbine(1).aeroLoadsType;
sv_t1_AeroLoads0 = Simulink.Variant('AeroLoadsChoice1==0');
sv_t1_AeroLoads1 = Simulink.Variant('AeroLoadsChoice1==1');

% Global Reference Frame 子系统里的 waveVis 变体块依赖这两个变量
% 等价 initializeWecSim.m L493–500；我们没有 waves，固定为 OFF
visON = 0;
sv_visualizationON  = Simulink.Variant('visON==1');
sv_visualizationOFF = Simulink.Variant('visON==0');

%% --- 5. 抑制无关 warning，跑仿真 ---
warning('off','Simulink:blocks:TDelayTimeTooSmall');
warning('off','Simulink:blocks:BusSelDupBusCreatorSigNames');
warning('off','Simulink:blocks:DivideByZero');
warning('off','sm:sli:setup:compile:SteadyStateStartNotSupported');
set_param(0, 'ErrorIfLoadNewModel', 'off');

% 让 To Workspace 块的输出走 simOut 而不是 base workspace
% set_param 只改内存里加载的模型副本，不写回 .slx；每次跑脚本都会重设
[~, modelName] = fileparts(simu.simMechanicsFile);
load_system(modelName);
set_param(modelName, 'ReturnWorkspaceOutputs', 'on');
set_param(modelName, 'SaveOutput',             'on');
set_param(modelName, 'SaveTime',               'on');
set_param(modelName, 'SaveFormat',             'Dataset');

% 同步 simu.endTime -> 模型 StopTime
% 干净路 B 没走 initializeWecSim 的 setSimMechanicsFile 流程，
% .slx 内硬写的 StopTime（VolturnUS 默认 1000）会盖过 simu.endTime
set_param(modelName, 'StopTime', num2str(simu.endTime));

simOut = sim(modelName);

%% --- 6. 诊断：列出 windTurbine1_out 里所有信号的 label / blockName / size ---
% 兼容两种返回形态：
%   - SimulationOutput 对象（ReturnWorkspaceOutputs=on 时） -> simOut.windTurbine1_out
%   - 仅 base workspace 落点（旧模型设置/异常时）          -> windTurbine1_out
if isa(simOut, 'Simulink.SimulationOutput') && isprop(simOut, 'windTurbine1_out')
    out = simOut.windTurbine1_out;
elseif evalin('base', 'exist(''windTurbine1_out'',''var'')==1')
    out = evalin('base', 'windTurbine1_out');
    warning('initializeRTHM:legacyOutput', ...
        '从 base workspace 取 windTurbine1_out；ReturnWorkspaceOutputs 未生效');
else
    error('initializeRTHM:noOutput', ...
        '没找到 windTurbine1_out。class(simOut)=%s', class(simOut));
end

% Outputs/To Workspace 上游是个 17 端口 Mux，全部 49 列一并打包，无 label。
% 列分配（按 Mux 端口 1..17 依次拼接）反推自 MOST_Lib.slx system_4_660.xml：
%
%   Mux port  内容                  宽度    output 列
%   ---------------------------------------------------
%     1       WindSpeed [Vx Vy Vz]    3      1: 3
%     2       TurbinePower            1      4
%     3       RotorSpeed (RPM)        1      5    单位 RPM，不是 rad/s
%     4       BladePitch              1      6
%     5       NacAccX                 1      7
%     6       NacAccY                 1      8
%     7       NacAccZ                 1      9
%     8       NacXdot                 1      10
%     9       TowerBaseLoad           6      11:16   [Fx Fy Fz Mx My Mz]
%    10       TowerTopLoad            6      17:22   含惯性/重力（用于补偿验证）
%    11       blade1RootLoad          6      23:28
%    12       GenTorque               1      29
%    13       Azimuth                 1      30
%    14       F_aero blade 1 (rotor)  6      31:36
%    15       F_aero blade 2 (rotor)  6      37:42
%    16       F_aero blade 3 (rotor)  6      43:48
%    17       deltaYaw                1      49

data = out.signals(1).values;     % [Nt × 49]
t    = out.time;
assert(size(data,2) == 49, ...
    '列数不是 49（实际 %d）。MOST 版本可能改了 Outputs 内 Mux 顺序，需要重新核对', size(data,2));

c.WindSpeed      = 1:3;
c.TurbinePower   = 4;
c.RotorSpeed     = 5;
c.BladePitch     = 6;
c.NacAcc         = 7:9;
c.NacXdot        = 10;
c.TowerBaseLoad  = 11:16;
c.TowerTopLoad   = 17:22;
c.blade1RootLoad = 23:28;
c.GenTorque      = 29;
c.Azimuth        = 30;
c.FaeroBlade1    = 31:36;
c.FaeroBlade2    = 37:42;
c.FaeroBlade3    = 43:48;
c.deltaYaw       = 49;

% 关键聚合量：F_aero @ hub = 三叶片气动载荷之和（已在 rotor frame 对齐）
F_aero  = data(:,c.FaeroBlade1) + data(:,c.FaeroBlade2) + data(:,c.FaeroBlade3);  % [Nt × 6]
ttLoad  = data(:, c.TowerTopLoad);
windV   = data(:, c.WindSpeed);
rotSpd  = data(:, c.RotorSpeed);

% 末态值快速自检（应得稳态附近）：
fprintf('\n=== 末态值 (t=%.1f s) ===\n', t(end));
fprintf('  Wind        = [%.2f %.2f %.2f] m/s\n', windV(end,:));
fprintf('  RotorSpeed  = %.2f rpm = %.3f rad/s\n', rotSpd(end), rotSpd(end)*2*pi/60);
fprintf('  BladePitch  = %.3f rad = %.2f deg\n', data(end,c.BladePitch), rad2deg(data(end,c.BladePitch)));
fprintf('  F_aero hub  = [%.2e %.2e %.2e] N  [%.2e %.2e %.2e] N·m\n', F_aero(end,:));

%% --- 7. 画图 ---
figure('Name','F_aero @ hub (sum of 3 blades, rotor frame)');
labels = {'F_{rx}','F_{ry}','F_{rz}','M_{rx}','M_{ry}','M_{rz}'};
units  = {'N','N','N','N·m','N·m','N·m'};
for k = 1:6
    subplot(3,2,k); plot(t, F_aero(:,k)); grid on;
    ylabel(sprintf('%s [%s]', labels{k}, units{k}));
    if k>=5, xlabel('t [s]'); end
end