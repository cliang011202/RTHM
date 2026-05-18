%% testMocapPackUnpack.m — 纯 MATLAB 验证 mocap UDP 打包/解包往返
% 在改 Simulink 模型之前，先验证 MocapPacketPacker ↔ MocapPacketUnpacker 字节序正确
% 也验证 prototype-scale 值经过 single 精度后精度损失

clear; close all;
addpath(fileparts(mfilename('fullpath')));

%% --- 加载 VolturnUS 参考数据 ---
refFile = 'level2_volturnUS_ref.mat';
assert(isfile(refFile), '%s 不存在；先跑 runVolturnUSReference.m', refFile);
ref = load(refFile);

fprintf('=== Mocap UDP 打包/解包往返测试 ===\n');
fprintf('数据: %s, %d 个时间步\n\n', refFile, length(ref.t));

%% --- 1. 打包/解包往返 ---
packer   = MocapPacketPacker();
unpacker = MocapPacketUnpacker();

nSamples = min(length(ref.t), 5);  % 测前 5 步即可
maxErr = zeros(6, 3);  % [pos, vel, acc] 最大误差

fprintf('%-6s %-8s %12s %12s %12s\n', 'Step', 'Seq', 'PosErr(max)', 'VelErr(max)', 'AccErr(max)');
fprintf('%s\n', repmat('-', 1, 60));

for k = 1:nSamples
    pos_in = ref.body_pos(k, :)';
    vel_in = ref.body_vel(k, :)';
    acc_in = ref.body_acc(k, :)';

    % Pack (use object-as-function syntax, which calls stepImpl via public step())
    pkt = packer(pos_in, vel_in, acc_in, k);
    assert(numel(pkt) == 76, 'Packet size mismatch: %d', numel(pkt));

    % Unpack
    [pos_out, vel_out, acc_out, seq_out, valid] = unpacker(pkt);

    assert(valid, 'Step %d: unpacker returned invalid', k);
    assert(seq_out == k, 'Step %d: seq mismatch (%d ~= %d)', k, seq_out, k);

    err_pos = abs(pos_out - pos_in);
    err_vel = abs(vel_out - vel_in);
    err_acc = abs(acc_out - acc_in);

    maxErr(:,1) = max(maxErr(:,1), err_pos);
    maxErr(:,2) = max(maxErr(:,2), err_vel);
    maxErr(:,3) = max(maxErr(:,3), err_acc);

    fprintf('%-6d %-8d %12.3e %12.3e %12.3e\n', ...
        k, seq_out, max(err_pos), max(err_vel), max(err_acc));
end

%% --- 2. 单精度精度损失评估 ---
% 往返过程: double → single (typecast) → double
% 原型尺度量值大 (pos ~m, vel ~m/s, acc ~m/s²)
% single 有 ~7 位有效数字，足以表示这些量
fprintf('\n=== 单精度精度损失评估 ===\n');

% 用最大量值评估
maxPos = max(abs(ref.body_pos(:)));
maxVel = max(abs(ref.body_vel(:)));
maxAcc = max(abs(ref.body_acc(:)));

epsPos = maxPos * eps('single');
epsVel = maxVel * eps('single');
epsAcc = maxAcc * eps('single');

fprintf('  max |pos| = %.2f m    → single ε ≈ %.3e m\n', maxPos, epsPos);
fprintf('  max |vel| = %.3f m/s  → single ε ≈ %.3e m/s\n', maxVel, epsVel);
fprintf('  max |acc| = %.3f m/s² → single ε ≈ %.3e m/s²\n', maxAcc, epsAcc);

labels = {'px','py','pz','qx','qy','qz'};
fprintf('\n  分量级往返误差 (前 %d 步最大):\n', nSamples);
fprintf('  %-4s  pos [m|rad]    vel [m/s|rad/s]  acc [m/s²|rad/s²]\n', 'DOF');
for i = 1:6
    fprintf('  %-4s  %+.3e       %+.3e        %+.3e\n', ...
        labels{i}, maxErr(i,1), maxErr(i,2), maxErr(i,3));
end

%% --- 3. 判定 ---
tol = 1e-6;  % single 精度往返应 < 1e-6 相对误差
if all(maxErr(:) < 1e-3)
    fprintf('\n✅ 往返测试通过：所有分量误差 < 1e-3\n');
else
    fprintf('\n❌ 往返测试失败：存在分量误差 > 1e-3\n');
end

%% --- 4. 字节序验证 ---
% 手动验证第一个包的前 8 字节
pkt1 = packer(ref.body_pos(1,:)', ref.body_vel(1,:)', ref.body_acc(1,:)', 1);
fprintf('\n=== 字节序验证 (Step 1) ===\n');
fprintf('  seq bytes 1-4:  [%02X %02X %02X %02X]  → uint32 LE = %d\n', ...
    pkt1(1), pkt1(2), pkt1(3), pkt1(4), ...
    typecast(pkt1(1:4), 'uint32'));
fprintf('  pos(1) bytes 5-8: [%02X %02X %02X %02X] → single = %.6f\n', ...
    pkt1(5), pkt1(6), pkt1(7), pkt1(8), ...
    typecast(pkt1(5:8), 'single'));
fprintf('  原始 pos(1) = %.6f\n', ref.body_pos(1,1));
