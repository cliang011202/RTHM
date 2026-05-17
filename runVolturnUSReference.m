%% runVolturnUSReference.m - Level 2 第 1 步：提取 VolturnUS 全数值参考
%
% 跑原版 SModel_VolturnUS.slx，存 body 6-DOF + 塔顶载荷 + per-blade 气动到
% level2_volturnUS_ref.mat 供后续 RTHM 回放对比。
%
% 实现要点：
%   1. wecSim.m 第 19 行 'clear'（无参数）会抹掉**所有**普通变量。所以脚本里
%      的路径、备份文件名等都要用 setenv 存到环境变量里，wecSim 跑完后再
%      getenv 回来。环境变量不受 clear 影响。
%   2. wecSim 还会重跑 wecSimInputFile.m，覆盖我们的 simu.endTime override；
%      只能临时改 wecSimInputFile.m 文件本身，跑完恢复。
%   3. 用 try/catch 包住 wecSim，无论是否报错都要恢复文件。
%
% 前置：
%   D:\MOST\MOST\Examples\VolturnUS\hydroData\VolturnUS\hydro.h5      ✓
%   D:\MOST\MOST\Examples\VolturnUS\mostData\turbSim\WIND_8mps.mat    ✓

clear; close all; bdclose('all');

%% --- 0. 配置 ---
volturnDir = 'D:\MOST\MOST\Examples\VolturnUS';
rthmDir    = 'D:\MOST\MOST\Examples\RTHM';
inputFile  = fullfile(volturnDir, 'wecSimInputFile.m');
backupFile = fullfile(volturnDir, 'wecSimInputFile.m.bak');
origDir    = pwd;
TARGET_ENDTIME = 120;            % 单位 s

assert(isfolder(volturnDir),  'VolturnUS 目录不存在: %s',   volturnDir);
assert(isfile(inputFile),     'wecSimInputFile.m 不存在: %s', inputFile);

% 把恢复用到的路径塞到环境变量里（wecSim 的 clear 抹不掉环境变量）
setenv('RTHM_VOLTURN_BACKUP',  backupFile);
setenv('RTHM_VOLTURN_INPUT',   inputFile);
setenv('RTHM_VOLTURN_RTHMDIR', rthmDir);
setenv('RTHM_VOLTURN_ORIGDIR', origDir);

%% --- 1. 备份并临时改 wecSimInputFile.m 的 endTime ---
% 幂等性保护：如果检测到已有 .bak，说明上次运行没正常恢复——
% 此时**当前的 inputFile 是被改过的脏状态**，**.bak 才是真正的原版**。
% 必须先用 .bak 还原，再开始本次流程，否则下一步备份会把脏状态当原版覆盖掉真正的原版。
if isfile(backupFile)
    fprintf('检测到已有备份 %s（上次运行未正常退出），先用它恢复\n', backupFile);
    copyfile(backupFile, inputFile);
    delete(backupFile);
end

origText = fileread(inputFile);
copyfile(inputFile, backupFile);
fprintf('已备份 wecSimInputFile.m → %s\n', backupFile);

modText = regexprep(origText, ...
    '(simu\.endTime\s*=\s*)[\d\.eE+-]+(\s*;)', ...
    sprintf('$1%d$2', TARGET_ENDTIME), 'once');
if strcmp(modText, origText)
    % 没匹配上：要么格式变了，要么 endTime 已经等于 TARGET_ENDTIME
    if contains(origText, sprintf('simu.endTime = %d', TARGET_ENDTIME))
        fprintf('simu.endTime 已经是 %d，无需修改\n', TARGET_ENDTIME);
    else
        delete(backupFile);
        error('正则没匹配 simu.endTime；wecSimInputFile.m 格式可能变了');
    end
else
    fid = fopen(inputFile, 'w');
    fwrite(fid, modText);
    fclose(fid);
    fprintf('已临时改 simu.endTime = %d s\n', TARGET_ENDTIME);
end

cd(volturnDir);
fprintf('cwd → %s\n', pwd);

%% --- 2. 跑 wecSim（warning：里面会 clear 整个 workspace）---
try
    fprintf('\n>>> 正在跑 VolturnUS 全数值仿真...\n');
    wecSim;
    wecSimErrored = false;
catch ME
    warning('wecSim 报错: %s', ME.message);
    disp(getReport(ME, 'extended'));
    wecSimErrored = true;
end

%% --- 3. 恢复 wecSimInputFile.m（无论 wecSim 是否成功）---
% 注意：上面 'clear' 把局部变量抹了，从环境变量取回
backupFile = getenv('RTHM_VOLTURN_BACKUP');
inputFile  = getenv('RTHM_VOLTURN_INPUT');
rthmDir    = getenv('RTHM_VOLTURN_RTHMDIR');
origDir    = getenv('RTHM_VOLTURN_ORIGDIR');

if isfile(backupFile)
    copyfile(backupFile, inputFile);
    delete(backupFile);
    fprintf('\nwecSimInputFile.m 已恢复\n');
else
    warning('备份文件丢失，wecSimInputFile.m 可能没有恢复到原状');
end

cd(origDir);
fprintf('cwd → %s\n', pwd);

%% --- 4. 抽取参考数据并保存 ---
if exist('output', 'var') == 1 && exist('wecSimErrored','var')==1 && ~wecSimErrored
    ref.t           = output.bodies(1).time;
    ref.body_pos    = output.bodies(1).position;
    ref.body_vel    = output.bodies(1).velocity;
    ref.body_acc    = output.bodies(1).acceleration;

    ref.ttLoad      = output.windTurbine(1).towerTopLoad;
    ref.tbLoad      = output.windTurbine(1).towerBaseLoad;
    ref.windV       = output.windTurbine(1).windSpeed;
    ref.rotorSpeed  = output.windTurbine(1).rotorSpeed;
    ref.bladePitch  = output.windTurbine(1).bladePitch;
    ref.nacAcc      = output.windTurbine(1).nacelleAcceleration;  % [Nt × 3]，用于运动一致性自检
    ref.faero1      = output.windTurbine(1).blade1AeroLoad;
    ref.faero2      = output.windTurbine(1).blade2AeroLoad;
    ref.faero3      = output.windTurbine(1).blade3AeroLoad;

    ref.simuMeta    = struct('rampTime', simu.rampTime, ...
                             'endTime',  simu.endTime,  ...
                             'dt',       simu.dt);
    ref.bodyInit    = body(1).initial.displacement;

    saveFile = fullfile(rthmDir, 'level2_volturnUS_ref.mat');
    save(saveFile, '-struct', 'ref');
    fprintf('\n参考数据存到: %s\n', saveFile);
    fprintf('  时间跨度: 0 - %.0f s, %d 个样本 (dt = %.3f s)\n', ...
            ref.t(end), numel(ref.t), mean(diff(ref.t)));
    fprintf('  body_pos 末态: [%+.2f %+.2f %+.2f] m  [%+.2f %+.2f %+.2f] rad\n', ref.body_pos(end,:));
    fprintf('  ttLoad   末态: [%+.2e %+.2e %+.2e] N  [%+.2e %+.2e %+.2e] N·m\n', ref.ttLoad(end,:));
    fprintf('  body initial displacement = [%+.2f %+.2f %+.2f] m\n', ref.bodyInit);
else
    warning('output 不存在或 wecSim 出错，未生成参考数据');
end

%% --- 5. 清理环境变量 ---
setenv('RTHM_VOLTURN_BACKUP',  '');
setenv('RTHM_VOLTURN_INPUT',   '');
setenv('RTHM_VOLTURN_RTHMDIR', '');
setenv('RTHM_VOLTURN_ORIGDIR', '');
