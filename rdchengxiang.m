% 清除工作区和命令窗口
close all;
clear;
clc;

% 生成C/A码
prn = 3; % 选择卫星PRN号
rows = 5000; % 生成的行数
ca_code_matrix = generate_ca_code(prn, rows); % 生成C/A码矩阵

%发射机位置
Tx = [20133725.8,10697303.2,728029.1];
% 接收机坐标
Rx = [0, 0, 100]; % 接收机坐标(米)

% 目标坐标和速度
Tg = [-1000,-1000,100]; % 目标坐标(米)
v_Tg = [0, 0, 0]; % 目标速度(米/秒)

% 发射机速度
v_Tx = [1392.7068, -2766.6856, 138.3063]; % 发射机速度(米/秒)
% 接收机速度
v_Rx = [0, 0, 0]; % 接收机速度(米/秒,假设静止)

% 其他参数
fc = 1575.42e6; % 载波频率(赫兹)
c = 3e8; % 光速(米/秒)
T = 100; % 模拟时间(秒)

% 计算波长
lambda = c / fc; % 波长

% 生成时间序列
tm = linspace(-T/2,T/2,5000)';

% 计算发射机和目标在每个时间点的位置
target_positions = Tg + v_Tg .* tm;
transmitter_positions = Tx + v_Tx .* tm;

% 初始化径向速度数组
v_radial_Tg_Tx = zeros(size(tm, 1), 1); % 目标与发射机之间的径向速度
v_radial_Tg_Rx = zeros(size(tm, 1), 1); % 目标与接收机之间的径向速度
v_radial_Tx_Rx = zeros(size(tm, 1), 1); % 发射机与接收机之间的径向速度

% 计算径向速度
Rx = [0, 0, 100]; % 接收机位置
v_Rx = [0, 0, 0]; % 接收机速度
for i = 1:size(tm, 1)
    % 目标与发射机
    R_Tg_Tx = target_positions(i, :) - transmitter_positions(i, :);
    norm_R_Tg_Tx = norm(R_Tg_Tx);
    if norm_R_Tg_Tx ~= 0
        hat_R_Tg_Tx = R_Tg_Tx / norm_R_Tg_Tx;
        v_rel_Tg_Tx = v_Tg - v_Tx;
        v_radial_Tg_Tx(i) = dot(v_rel_Tg_Tx, hat_R_Tg_Tx);
    end
    
    % 目标与接收机
    R_Tg_Rx = target_positions(i, :) - Rx;
    norm_R_Tg_Rx = norm(R_Tg_Rx);
    if norm_R_Tg_Rx ~= 0
        hat_R_Tg_Rx = R_Tg_Rx / norm_R_Tg_Rx;
        v_rel_Tg_Rx = v_Tg - v_Rx;
        v_radial_Tg_Rx(i) = dot(v_rel_Tg_Rx, hat_R_Tg_Rx);
    end
    
    % 发射机与接收机的相对位置
    R_Tx_Rx = Tx + v_Tx * tm(i) - Rx - v_Rx * tm(i); % 计算当前时刻的位置
    norm_R_Tx_Rx = norm(R_Tx_Rx);
    if norm_R_Tx_Rx ~= 0
        hat_R_Tx_Rx = R_Tx_Rx / norm_R_Tx_Rx; % 单位向量
        v_rel_Tx_Rx = v_Tx - v_Rx; % 计算相对速度
        v_radial_Tx_Rx(i) = dot(v_rel_Tx_Rx, hat_R_Tx_Rx); % 计算径向速度
    end
end

% 初始化径向加速度数组
a_radial_Tg_Tx = zeros(size(tm, 1), 1); % 目标与发射机之间的径向加速度
a_radial_Tg_Rx = zeros(size(tm, 1), 1); % 目标与接收机之间的径向加速度
a_radial_Tx_Rx = zeros(size(tm, 1), 1); % 发射机与接收机之间的径向加速度

% 计算径向加速度
dt = tm(2) - tm(1); % 假设时间间隔是常数
for i = 2:size(tm, 1)
    a_radial_Tg_Tx(i) = (v_radial_Tg_Tx(i) - v_radial_Tg_Tx(i-1)) / dt;
    a_radial_Tg_Rx(i) = (v_radial_Tg_Rx(i) - v_radial_Tg_Rx(i-1)) / dt;
    a_radial_Tx_Rx(i) = (v_radial_Tx_Rx(i) - v_radial_Tx_Rx(i-1)) / dt;
end

% 计算多普勒频移
delta_f_Tg_Tx = (fc * v_radial_Tg_Tx) / c; % 目标与发射机之间的频移
delta_f_Tg_Rx = (fc * v_radial_Tg_Rx) / c; % 目标与接收机之间的频移
delta_f_Tx_Rx = (fc * v_radial_Tx_Rx) / c; % 发射机与接收机之间的频移

f_ref = delta_f_Tg_Tx + delta_f_Tg_Rx;
f_dir = delta_f_Tx_Rx;

% 初始化时延数组
delay_Tg_Tx = zeros(size(tm, 1), 1); % 目标与发射机之间的时延
delay_Tg_Rx = zeros(size(tm, 1), 1); % 目标与接收机之间的时延
delay_Tx_Rx = zeros(size(tm, 1), 1); % 发射机与接收机之间的时延

for i = 1:size(tm, 1)
    % 目标与发射机的距离
    distance_Tg_Tx = norm(target_positions(i, :) - transmitter_positions(i, :));
    delay_Tg_Tx(i) = distance_Tg_Tx / c;
    
    % 目标与接收机的距离
    distance_Tg_Rx = norm(target_positions(i, :) - Rx);
    delay_Tg_Rx(i) = distance_Tg_Rx / c;
    
    % 发射机与接收机的距离
    distance_Tx_Rx = norm(transmitter_positions(i, :) - Rx);
    delay_Tx_Rx(i) = distance_Tx_Rx / c;
end

tau_dir = delay_Tx_Rx;
tau_ref = delay_Tg_Tx + delay_Tg_Rx;

% 初始化相位数组
phase_Tg_Tx = zeros(size(tm, 1), 1);
phase_Tg_Rx = zeros(size(tm, 1), 1);
phase_Tx_Rx = zeros(size(tm, 1), 1);

for i = 1:size(tm, 1)
    % 计算相位
    phase_Tg_Tx(i) = (2 * pi * norm(target_positions(i, :) - transmitter_positions(i, :))) / lambda;
    phase_Tg_Rx(i) = (2 * pi * norm(target_positions(i, :) - Rx)) / lambda;
    phase_Tx_Rx(i) = (2 * pi * norm(transmitter_positions(i, :) - Rx)) / lambda;
end

phi_dir = phase_Tx_Rx;
phi_ref = phase_Tg_Tx + phase_Tg_Rx;

% 对C/A码进行延迟处理
chirp_rate = 20460000;
code_dir = mod(round(tau_dir * chirp_rate), 20460);
code_ref = mod(round(tau_ref * chirp_rate), 20460);

ca_code_dir = zeros(size(ca_code_matrix));
ca_code_ref = zeros(size(ca_code_matrix));

for i = 1:5000
    ca_code_dir(i, :) = circshift(ca_code_matrix(i, :), [0, mod(code_dir(i), 20460)]);
    ca_code_ref(i, :) = circshift(ca_code_matrix(i, :), [0, mod(code_ref(i), 20460)]);
end

% 检测时延是否正确
[m, n] = size(ca_code_matrix);
cross = zeros(m, n);
for i = 1:m
    cross(i,:) = fft(ca_code_ref(i, :)) .* conj(fft(ca_code_dir(i,:)));
end
cross = ifft(cross,[],2);
figure;
imagesc(abs(cross));
title('直返信号互相关结果');

% 载波时延、多普勒处理
Tp = 0.001;
tk = linspace(0,Tp,20460); % 从-Tp/2到Tp/2，10230个采样点

% 预分配矩阵
numTm = length(tm);
kr = (a_radial_Tg_Rx-a_radial_Tx_Rx)/lambda;
kr0 = kr(2,1);
f0 = 6.875553677493369;
R0 = norm(Tx-Tg) + norm(Tg-Rx) - norm(Tx-Rx);
R_bi = lambda * (kr0/2 * tm.^2 + f0 * tm + R0/lambda);

S_d = ca_code_dir .* exp(-1j * 2 * pi * f_dir.*tau_dir);
S_r = ca_code_ref .* exp(-1j * 2 * pi * f_ref.*tau_ref);

% 计算信号的功率
signal_power = mean(abs(S_r(:)).^2); % 信号的平均功率

% 设定所需的信噪比（dB）
SNR_dB = -60;
% 计算噪声功率
SNR_linear = 10^(SNR_dB / 10); % 转换为线性比例
noise_power = signal_power / SNR_linear; % 噪声功率

% 生成复高斯白噪声
noise = sqrt(noise_power) * (randn(size(S_r)) + 1j * randn(size(S_r))) / sqrt(2);

% 添加噪声到信号
% noisy_signal = S_r + noise;
% rc = fft(noisy_signal,[],2) .* conj(fft(S_d,[],2));
% rc = ifft(rc,[],2);

CF = fft(ca_code_ref,[],2) .* conj(fft(ca_code_dir,[],2));
CF = ifft(CF,[],2);
zhishuxiang = exp(-1j * 2 * pi * R_bi/lambda);
rc = CF .* zhishuxiang + noise;
rc1 = rc(:, 1:250); % 保留前 250 列，删除其余列

figure;
imagesc(abs(rc1));
title('直返信号距离压缩结果');

figure;
plot(abs(rc1(1,:)));
title('距离压缩后剖面图');

%-----------------------------方位向压缩----------------------------------%
ha = exp(-1j * pi * kr .* tm .* 2); % 方位向匹配滤波器
rd1 = ifft(fft(ha, [], 1) .* fft(rc, [], 1), [], 1); % 使用 rc 而不是 rc_correct
rd = fft(rd1); % 进行傅里叶变换

figure;
rd = rd(1:1000, :); % 提取前 1000 行数据
imagesc(abs(rd)); % 显示结果
title('RD');

figure;
plot(abs(rd(:, 1)));
