%% testUdpMocapLoopback.m — 纯 MATLAB UDP loopback 测试
% 在 localhost 上验证 mocap 数据 UDP 收发全路径：
%   MocapPacketPacker → dsp.UDPSender → network → dsp.UDPReceiver → MocapPacketUnpacker
%
% 不需要改 Simulink 模型；验证通过后再改 SModel_RTHM.slx 加入 UDP Receive。

clear; close all;
addpath(fileparts(mfilename('fullpath')));

%% --- 参数 ---
RX_PORT = 10001;
TX_PORT = 10002;
PACKET_SIZE = 76;   % 4 + 6×3×4 bytes

%% --- 加载数据 ---
refFile = 'level2_volturnUS_ref.mat';
assert(isfile(refFile), '%s 不存在；先跑 runVolturnUSReference.m', refFile);
ref = load(refFile);
nSteps = length(ref.t);
fprintf('=== UDP Mocap Loopback 测试 ===\n');
fprintf('数据: %s, %d 步, dt=%.3f s\n\n', refFile, nSteps, ref.simuMeta.dt);

%% --- 1. 预打包所有数据包 ---
packer = MocapPacketPacker();
packets = cell(nSteps, 1);
fprintf('预打包 %d 个数据包 ... ', nSteps);
tic;
for k = 1:nSteps
    packets{k} = packer(ref.body_pos(k,:)', ref.body_vel(k,:)', ref.body_acc(k,:)', k);
end
fprintf('完成 (%.1f s)\n', toc);

%% --- 2. 建立 UDP 收发 ---
rx = dsp.UDPReceiver('LocalIPPort', RX_PORT, 'ReceiveBufferSize', 2^16, ...
    'MaximumMessageLength', PACKET_SIZE, 'MessageDataType', 'uint8');
tx = dsp.UDPSender('RemoteIPAddress', '127.0.0.1', 'RemoteIPPort', RX_PORT, ...
    'LocalIPPortSource', 'Property', 'LocalIPPort', TX_PORT);

% 清空接收缓冲区
while rx() ~= 0; end
fprintf('UDP 收发器已建立: 127.0.0.1:%d → :%d\n', TX_PORT, RX_PORT);

%% --- 3. 逐包收发测试 ---
unpacker  = MocapPacketUnpacker();
numFailed = 0;
maxErr    = zeros(6, 3);
received  = 0;
lost      = 0;

fprintf('\n逐包收发 (前 10 步详细, 之后每 1000 步报告) ...\n');

tStart = tic;
for k = 1:nSteps
    % Send
    tx(packets{k});

    % Receive (非阻塞轮询, 最多等 50 ms)
    pkt = [];
    t0 = tic;
    while isempty(pkt) && toc(t0) < 0.05
        pkt = rx();
    end

    if isempty(pkt)
        lost = lost + 1;
        if lost <= 3
            fprintf('  ⚠ Step %d: 丢包 (无响应)\n', k);
        end
        continue;
    end

    % Unpack
    [pos_out, vel_out, acc_out, seq_out, valid] = unpacker(pkt);

    if ~valid
        numFailed = numFailed + 1;
        fprintf('  ❌ Step %d: 解包失败 (包长=%d)\n', k, numel(pkt));
        continue;
    end

    if seq_out ~= k
        numFailed = numFailed + 1;
        fprintf('  ❌ Step %d: 序号不匹配 (收到=%d)\n', k, seq_out);
        continue;
    end

    err_pos = abs(pos_out - ref.body_pos(k,:)');
    err_vel = abs(vel_out - ref.body_vel(k,:)');
    err_acc = abs(acc_out - ref.body_acc(k,:)');

    maxErr(:,1) = max(maxErr(:,1), err_pos);
    maxErr(:,2) = max(maxErr(:,2), err_vel);
    maxErr(:,3) = max(maxErr(:,3), err_acc);

    if err_pos(1) > 1e-4 || err_vel(1) > 1e-4
        numFailed = numFailed + 1;
    end

    received = received + 1;

    if k <= 10
        fprintf('  Step %d: OK seq=%d, pos(1)=%.4f, err=%.2e\n', k, seq_out, pos_out(1), max(err_pos));
    elseif mod(k, 1000) == 0
        fprintf('  Step %d/%d: received=%d, lost=%d, failed=%d\n', ...
            k, nSteps, received, lost, numFailed);
    end
end
elapsed = toc(tStart);
fprintf('收发完成: %.1f s, %.0f pkt/s\n', elapsed, nSteps/elapsed);

%% --- 4. 结果 ---
fprintf('\n========== 结果 ==========\n');
fprintf('  发送: %d 包\n', nSteps);
fprintf('  收到: %d 包\n', received);
fprintf('  丢包: %d (%.2f%%)\n', lost, 100*lost/nSteps);
fprintf('  失败: %d (%.2f%%)\n', numFailed, 100*numFailed/nSteps);

labels = {'px','py','pz','qx','qy','qz'};
fprintf('\n  分量级最大误差:\n');
fprintf('  %-4s  pos [m|rad]    vel [m/s|rad/s]  acc [m/s²|rad/s²]\n', 'DOF');
for i = 1:6
    fprintf('  %-4s  %+.3e       %+.3e        %+.3e\n', ...
        labels{i}, maxErr(i,1), maxErr(i,2), maxErr(i,3));
end

if numFailed == 0 && lost < 0.01 * nSteps
    fprintf('\n✅ UDP Loopback 测试通过\n');
elseif numFailed > 0
    fprintf('\n❌ UDP Loopback 测试失败\n');
else
    fprintf('\n⚠ UDP Loopback 测试通过但有丢包\n');
end
