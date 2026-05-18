%% modifyModelForLevel3.m — 程序化修改 SModel_RTHM.slx 加入 UDP Receive
% 替换 3 个 From Workspace 为 UDP Receive + MocapPacketUnpacker
% 保留 3 个 Demux + 6 个 PS Converter + 所有下游连线不变

addpath(fileparts(mfilename('fullpath')));

model = 'SModel_RTHM';
fprintf('=== 修改 %s 为 Level 3 UDP 接收模式 ===\n', model);

% 加载模型（如已在内存中则先关闭）
bdclose('all');
load_system(model);

%% --- 0. 记录目标 Demux 块名（后续连线用）---
% 根据模型探索: Demux→pos, Demux1→vel, Demux2→acc
demuxPos = [model '/Demux'];
demuxVel = [model '/Demux1'];
demuxAcc = [model '/Demux2'];

% 确认存在
assert(strcmp(get_param(demuxPos, 'BlockType'), 'Demux'), 'Demux not found');
assert(strcmp(get_param(demuxVel, 'BlockType'), 'Demux'), 'Demux1 not found');
assert(strcmp(get_param(demuxAcc, 'BlockType'), 'Demux'), 'Demux2 not found');

%% --- 1. 断开 From Workspace → Demux 连线, 删除 From Workspace ---
fwBlocks = {'From Workspace', 'From Workspace1', 'From Workspace2'};
for i = 1:length(fwBlocks)
    fullName = [model '/' fwBlocks{i}];
    lh = get_param(fullName, 'LineHandles');
    if lh.Outport > 0
        delete_line(lh.Outport);
    end
    delete_block(fullName);
    fprintf('  已删除: %s\n', fullName);
end

%% --- 2. 添加 UDP Receive 块 ---
% instrumentlib/UDP Receive: 收发都在 instrumentlib 库里
% 先加载库
load_system('instrumentlib');

udpRecv = [model '/UDP_MocapReceive'];
add_block('instrumentlib/UDP Receive', udpRecv, ...
    'LocalPort', '10001', ...
    'DataType', 'uint8', ...
    'DataSize', '[76 1]', ...
    'SampleTime', '0.01', ...
    'EnableBlockingMode', 'off', ...
    'Position', [50, 40, 170, 100]);
fprintf('  已添加: %s\n', udpRecv);

%% --- 3. 添加 MocapPacketUnpacker (MATLAB System) ---
unpacker = [model '/UDP_MocapUnpack'];
add_block('simulink/User-Defined Functions/MATLAB System', unpacker, ...
    'System', 'MocapPacketUnpacker', ...
    'Position', [300, 80, 430, 230]);
fprintf('  已添加: %s\n', unpacker);

%% --- 4. 连线: UDP Receive → MocapPacketUnpacker ---
add_line(model, 'UDP_MocapReceive/1', 'UDP_MocapUnpack/1', 'autorouting', 'on');
fprintf('  已连线: UDP_MocapReceive → UDP_MocapUnpack\n');

%% --- 5. 连线: Unpacker → Demux (pos/vel/acc 三路) ---
% MocapPacketUnpacker 输出:
%   Out1 = pos[6], Out2 = vel[6], Out3 = acc[6], Out4 = seq[1], Out5 = valid[1]
add_line(model, 'UDP_MocapUnpack/1', 'Demux/1', 'autorouting', 'on');
add_line(model, 'UDP_MocapUnpack/2', 'Demux1/1', 'autorouting', 'on');
add_line(model, 'UDP_MocapUnpack/3', 'Demux2/1', 'autorouting', 'on');
fprintf('  已连线: Unpack(pos/vel/acc) → Demux/Demux1/Demux2\n');

%% --- 6. Terminate 未使用的输出 (seq, valid) ---
term = [model '/UDP_Term'];
add_block('simulink/Sinks/Terminator', term, ...
    'Position', [500, 260, 520, 280]);
add_line(model, 'UDP_MocapUnpack/4', 'UDP_Term/1', 'autorouting', 'on');
fprintf('  已添加 Terminator (seq)\n');

term2 = [model '/UDP_Term2'];
add_block('simulink/Sinks/Terminator', term2, ...
    'Position', [500, 300, 520, 320]);
add_line(model, 'UDP_MocapUnpack/5', 'UDP_Term2/1', 'autorouting', 'on');
fprintf('  已添加 Terminator (valid)\n');

%% --- 7. 添加丢包检测 Scope (监控 seq 连续性) ---
% 暂时用 Terminator; 后续需要可以换成 Scope

%% --- 8. 保存 ---
save_system(model);
fprintf('\n=== 模型已保存: %s ===\n', model);
fprintf('\n新增块:\n');
fprintf('  UDP_MocapReceive  — UDP Receive (:10001, uint8, 76B, 0.01s)\n');
fprintf('  UDP_MocapUnpack   — MocapPacketUnpacker (→ pos/vel/acc)\n');
fprintf('  UDP_Term/Term2    — 未使用输出终止\n');
fprintf('\n保留块 (连线不变):\n');
fprintf('  Demux/Demux1/Demux2 → 6 PS Converters → Bushing Joint\n');
