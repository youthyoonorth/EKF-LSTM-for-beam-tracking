clear; clc;

% ---------- 参数 ----------
DT = 0.142;
BASE_Q = 0.1;
SEQ_LENGTH = 15;

% ---------- 构造假 LSTM（先不训练） ----------
net = buildInnovationLSTM(SEQ_LENGTH);

% ---------- Tracker ----------
tracker = EKFLSTMTracker(DT, BASE_Q, net);

% 初始状态 [x y z vx vy vz]
tracker.reset([0; 0; 50; 1; 0; 0]);

% ---------- 人造运动轨迹 ----------
T = 100;
true_pos = zeros(T,3);
meas = zeros(T,6);

for t = 1:T
    true_pos(t,:) = [t, 0.1*t^2, 50];
    meas(t,:) = [true_pos(t,:) + randn(1,3)*0.5, 1, 0, 0];
end

% ---------- 运行 tracker ----------
az = zeros(T,1);
el = zeros(T,1);

for t = 1:T
    [az(t), el(t)] = tracker.step(meas(t,:));
end

% ---------- 可视化 ----------
figure;
subplot(2,1,1); plot(az); title('Azimuth');
subplot(2,1,2); plot(el); title('Elevation');
