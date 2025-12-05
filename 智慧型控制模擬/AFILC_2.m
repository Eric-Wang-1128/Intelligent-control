% MATLAB Implementation of AFILC Model from the Paper
% This script implements the Adaptive Fuzzy Iterative Learning Control (AFILC)
% with the varying iteration lengths and system constraints described in the paper.

% 清除環境變數
clear; clc; close all;

% 路線劃分（示例）
sections = [
    struct('distance', 5e3, 'speed_limit', 100/3.6, 'gradient',0),  % 區段1
    struct('distance', 10e3, 'speed_limit', 200/3.6, 'gradient',0), % 區段2
    struct('distance', 20e3, 'speed_limit', 300/3.6, 'gradient',0), % 區段3
    struct('distance', 15e3, 'speed_limit', 250/3.6, 'gradient',0), % 區段4
    struct('distance', 22e3, 'speed_limit', 300/3.6, 'gradient',0), % 區段5
];

% 列車參數
mass = 350e3;           % 列車質量，單位：kg
max_acceleration = 0.6; % 最大加速度，單位：m/s^2
max_deceleration = -1.0; % 最大減速度，單位：m/s^2（負值）

% 初始化
dt = 1; % 時間步長，單位：秒
t = 0;
v = 0;
s = 0;
i = 1;
additional_resistance_coeff = 0;
time = [];
speed = [];
position = [];
total_resist = [];
additional_resistance = [];

% 遍歷每個區段
for sec = 1:length(sections)
    sec_distance = sections(sec).distance;
    speed_limit = sections(sec).speed_limit;
    gradient = sections(sec).gradient; % 坡度，百分比表示，如 0.5% = 0.005

    % 計算本區段的起始和結束位置
    sec_start_distance = sum([sections(1:sec-1).distance]);
    sec_end_distance = sum([sections(1:sec).distance]);

    % 更新附加阻力邏輯
    while s < sec_end_distance
        % 更新附加阻力條件
        if sec == 2
    % 區段 2 的附加阻力條件
    if s >= sec_start_distance && s <= sec_start_distance + 0.6 * sec_distance
        additional_resistance_coeff = 0.08; % 第一條件
    else
        additional_resistance_coeff = 0;
    end
elseif sec == 3
    % 區段 3 的附加阻力條件
    if s >= sec_start_distance && s <= sec_start_distance + 0.7 * sec_distance
        additional_resistance_coeff = 0.06; % 第二條件
        fprintf('Applying additional resistance coeff: %f at position s: %f\n', additional_resistance_coeff, s);
    else
        additional_resistance_coeff = 0;
    end
        end

        % 計算阻力
        rolling_resistance_coeff = (2977 + 27500 * (1-sin(i))) / mass; % 滾動阻力係數
        machine_resistance_coeff = (25.17 + 2400 * (1-sin(i))) / mass; % 機械阻力係數
        air_resistance_coeff = (0.3864 + 40 * (1-sin(i))) /mass;    % 空氣阻力係數
        grade_resistance = mass * 9.81 * gradient/100 ;
        rolling_resistance = rolling_resistance_coeff;
        machine_resistance = machine_resistance_coeff * v;
        air_resistance = air_resistance_coeff * v^2;
        total_resistance = (grade_resistance + rolling_resistance + machine_resistance + air_resistance + additional_resistance_coeff) * mass;

        % 計算可用牽引力
        if v < speed_limit
            acceleration = (max_acceleration * mass - total_resistance) / mass;
            acceleration = min(acceleration, max_acceleration);
        else
            acceleration = (-total_resistance) / mass;
        end

        % 計算剩餘距離
        remaining_distance_in_section = sec_end_distance - s;
        remaining_total_distance = sum([sections(sec:end).distance]) - (s - sec_start_distance);

        % 檢查減速需求
        braking_distance = (v^2) / (2 * abs(max_deceleration));
        if remaining_total_distance <= braking_distance
            acceleration = max_deceleration;
        end

        % 更新速度和位置
        v = v + acceleration * dt;
        v = min(v, speed_limit); % 不超過速度限制
        v = max(v, 0);           % 速度不為負
        s = s + v * dt;
        t = t + dt;

        % 保存數據
        time(i) = t;
        speed(i) = v;
        position(i) = s;
        total_resist(i) = total_resistance;
        additional_resistance(i) = additional_resistance_coeff * mass;
        i = i + 1;

        % 列車停止時退出
        if v <= 0 && remaining_distance_in_section <= 0
            break;
        end
    end
end

% 將速度和位置轉換為常用單位
speed_kmh_actual = speed * 3.6;
position_km_actual = position / 1000;

figure;
subplot(3,1,1);
plot (time,additional_resistance,'LineWidth', 2);
xlabel('時間 (秒)');
ylabel('additional_resistance');
%繪製速度-時間圖
subplot(3,1,2);
plot(time, speed_kmh_actual, 'LineWidth', 2);
xlabel('時間 (秒)');
ylabel('速度 (km/h)');
title('實際速度-時間圖');
grid on;
% figure;
% subplot(3,1,1);
% plot(time, total_resist, 'LineWidth', 2);
% xlabel('Time (s)');
% ylabel('Total Resistance (N)');
% title('Total Resistance vs Time');
% grid on;
% 繪製位置-時間圖
subplot(3,1,3);
plot(time, position_km_actual, 'LineWidth', 2);
xlabel('時間 (秒)');
ylabel('位置 (公里)');
title('實際位置-時間圖');
grid on;

% 输出速度和位置数据（可选）
data_actual = table(time', position', speed', 'VariableNames', {'time', 'position', 'speed'});


%======================================================================

% 路线划分（示例）
sections = [
    struct('distance', 5e3, 'speed_limit', 100/3.6),   % 区段1
    struct('distance', 10e3, 'speed_limit', 200/3.6),  % 区段2
    struct('distance', 20e3, 'speed_limit', 300/3.6),  % 区段3
    struct('distance', 15e3, 'speed_limit', 250/3.6),  % 区段4
    struct('distance', 22e3, 'speed_limit', 300/3.6),  % 区段5
];

% 列车参数
max_acceleration = 0.6;  % 最大加速度，单位：m/s^2
max_deceleration = -1.0; % 最大减速度，单位：m/s^2（负值）

% 初始化
dt = 1; % 时间步长，单位：秒
t = 0;
v = 0;
s = 0;
i = 1;
time = [];
speed = [];
position = [];

% 遍历每个区段
for sec = 1:length(sections)
    sec_distance = sections(sec).distance;
    speed_limit = sections(sec).speed_limit;
    
    % 计算本区段的起始和结束位置
    sec_start_distance = sum([sections(1:sec-1).distance]);
    sec_end_distance = sum([sections(1:sec).distance]);
    
    while s < sec_end_distance
        % 计算剩余距离
        remaining_distance_in_section = sec_end_distance - s;
        remaining_total_distance = sum([sections(sec:end).distance]) - (s - sec_start_distance);
        
        % 检查是否需要减速
        braking_distance = (v^2) / (2 * abs(max_deceleration));
        if remaining_total_distance <= braking_distance
            acceleration = max_deceleration;
        elseif v < speed_limit
            % 加速阶段
            acceleration = max_acceleration;
        else
            % 匀速阶段
            acceleration = 0;
            v = speed_limit; % 保持在速度限制
        end
        
        % 更新速度和位置
        v = v + acceleration * dt;
        v = min(v, speed_limit); % 不超过速度限制
        v = max(v, 0);           % 速度不为负
        s = s + v * dt;
        t = t + dt;
        
        % 保存数据
        time(i) = t;
        speed(i) = v;
        position(i) = s;
        force_ideal(i) = mass * acceleration;
        i = i + 1;
        
        % 列车停止时退出循环
        if v <= 0 && remaining_total_distance <= 0
            break;
        end
    end
end

% 将速度和位置转换为常用单位
speed_kmh_ideal = speed * 3.6;
position_km_ideal = position / 1000;

% 绘制速度-时间图
figure;
subplot(2,1,1);
plot(time, speed_kmh_ideal, 'LineWidth', 2);
xlabel('时间 (秒)');
ylabel('速度 (km/h)');
title('忽略阻力的速度-时间图');
grid on;

% 绘制位置-时间图
subplot(2,1,2);
plot(time, position_km_ideal, 'LineWidth', 2);
xlabel('时间 (秒)');
ylabel('位置 (公里)');
title('忽略阻力的位置-时间图');
grid on;

% 输出速度和位置数据（可选）
data_ideal = table(time', position', speed', 'VariableNames', {'time', 'position', 'speed'});
% disp(data_ideal);

%====================================================================================================================================

% Step 1: Initialization of Parameters
beta = 0.4; % Adjusted learning gain to improve convergence
psi = [1; 1];
psi_bar = [0; 1];
Gamma = diag([1e-5, 1e-5, 1e-5, 1e-4, 1e-4, 1e-4, 1e-4]); % Reduced learning rate for better stability
kappa = 1e-4; % Reduced learning rate for rho_hat to improve stability
kv = 1;
total_iterations = 100;
dt = 1;
alpha_max = 1;
% Step 2: Read Ideal and Actual Data from CSV
time = data_ideal.time;
time_length = length(time);
p_desired_position = data_ideal.position;
p_desired_speed = data_ideal.speed;
p_actual_position = data_actual.position;
p_actual_speed = data_actual.speed;
% Desired state vector p(t)
p_desired = [p_desired_position'; p_desired_speed'];

% Step 3: Initial Conditions and Learning Variables
x_hat = zeros(7, time_length);  % Initial estimate of theta_hat (unknown parameters)
rho_hat = zeros(time_length);          % Initial estimate of rho_hat
nu = zeros(total_iterations, time_length); % Control input

% Initialize cost functions
J_k = zeros(1, total_iterations); % Cost function J_k
delta_J_k = zeros(1, total_iterations); % Change in cost function ΔJ_k

% Fuzzy System Parameters
m1 = [-3; -3];
m2 = [1; 1];
m3 = [1; -1];
m4 = [2; 2];

sigma1 = [std(p_actual_position); std(p_actual_speed)];
sigma2 = [std(p_actual_position); std(p_actual_speed)];
sigma3 = [std(p_actual_position); std(p_actual_speed)];
sigma4 = [std(p_actual_position); std(p_actual_speed)];

% Gaussian Membership Function Definition
mu = @(m, sigma, x) exp(-(x - m).^2 ./ (2 * sigma.^2));

% Step 4: Iterative Learning Loop
for k = 1:total_iterations
    % Initialize system state based on the final state of the previous iteration
    if k == 1
        x = [p_actual_position'; p_actual_speed'];  % Use actual initial conditions for the first iteration
    end

    for t = 1:time_length
        % Compute errors
        e(:, t) = p_desired(:, t) - x(:, t);
        phi(t) = psi' * e(:, t);
        if abs(phi(t)) > kv 
            if kv>0
               phi(t) = 0.5;
            else
        phi(t) = -0.5;
            end
            end
        alpha(t) = phi(t) / (cos((pi.*(phi(t)).^2) / ((2 * kv).^2))^2);
        

        % Fuzzy Estimation of f_a(s_k(t))
        mu1 = mu(m1(1) * p_desired(1, t), sigma1(1), x(1, t)) * mu(m1(2) * p_desired(2, t), sigma1(2), x(2, t));
        mu2 = mu(m2(1) * p_desired(1, t), sigma2(1), x(1, t)) * mu(m2(2) * p_desired(2, t), sigma2(2), x(2, t));
        mu3 = mu(m3(1) * p_desired(1, t), sigma3(1), x(1, t)) * mu(m3(2) * p_desired(2, t), sigma3(2), x(2, t));
        mu4 = mu(m4(1) * p_desired(1, t), sigma4(1), x(1, t)) * mu(m4(2) * p_desired(2, t), sigma4(2), x(2, t));
        mu_sum = mu1 + mu2 + mu3 + mu4;

        delta1 = mu1 / mu_sum;
        delta2 = mu2 / mu_sum;
        delta3 = mu3 / mu_sum;
        delta4 = mu4 / mu_sum;

        zeta = [-1, -x(2, t), -(x(2, t))^2, -delta1, -delta2, -delta3, -delta4]';

        for j = 1:7
            x_hat(j, t) = x_hat(j, t) - Gamma(j, j) * zeta(j) * alpha(t);
        end

        rho_hat(t) = rho_hat(t) + kappa * abs(alpha(t));

        f_a_hat(t) = x_hat(4, t) * delta1 + x_hat(5, t) * delta2 + x_hat(6, t) * delta3 + x_hat(7, t) * delta4;
        f_b(t) = x_hat(1, t) + x_hat(2, t) * x(2, t) + x_hat(3, t) * (x(2, t))^2;

        % Control Law
        F(t) = beta * phi(t) + psi_bar' * e(:, t) - x_hat(:, t)' * zeta(:) + rho_hat(t) * sign(alpha(t));

        acceleration_hat(t) = F(t) - f_b(t) - f_a_hat(t);

        % Update System State
        x(2, t) = x(2, t) + acceleration_hat(t) * dt;
        x(1, t) = x(1, t) + x(2, t) * dt;

        % Update cost function J_k(t) (first formula)
        J_k_temp = (kv^2 / pi) * tan((pi * phi(t)^2) / (2 * kv^2)) + ...
                   0.5 * (x_hat(:, t)' * Gamma^-1 * x_hat(:, t)) + ...
                   0.5 * rho_hat(t)^2;
        J_k(k) = J_k(k) + J_k_temp * dt;
    end

    % Compute ΔJ_k (second formula)
    if k > 1
        delta_J_k(k) = J_k(k) - J_k(k - 1);
    end

    mu_sum_store(k,t) = mu_sum;
    % Check ΔJ_k < 0 condition
    if k > 1 && delta_J_k(k) >= 0
      
        break;
    end
    total_speed_error = sqrt(mean(e(2, :).^2));
    error(k) = total_speed_error;
end

% Plot results
figure;
subplot(2,1,1);
plot(1:total_iterations, J_k, '-o');
xlabel('Iteration');
ylabel('Cost Function J_k');
title('Cost Function J_k over Iterations');
grid on;

subplot(2,1,2);
plot(2:total_iterations, delta_J_k(2:end), '-o');
xlabel('Iteration');
ylabel('ΔJ_k');
title('Change in Cost Function ΔJ_k over Iterations');
grid on;

% disp(x_hat)
p_actual_position_limit = x(1,1:1159);
p_actual_speed_limit = x(2,1:1159);
f_b_N = f_b * mass;
F_N = F * mass;
f_a_hat_N = f_a_hat * mass;
%disp(p_actual_speed_limit)
% disp(p_actual_speed_limit)
% Step 5: Plot Results
% figure;
% plot(1:total_iterations, sqrt(mean(e(1, :).^2)), 'b');
% title('Position Tracking Error Over Iterations');
% xlabel('Iteration Number');
% ylabel('RMS Position Tracking Error');
% 
% total_speed_error = sqrt(mean(e(2, :).^2));
% figure;
% plot(1:total_iterations, total_speed_error, 'r');
% title('Speed Tracking Error Over Iterations');
% xlabel('Iteration Number');
% ylabel('RMS Speed Tracking Error');
figure;
plot(1:time_length,F);

figure;
subplot(3,2,1);
plot(1:total_iterations, error, 'r');
title('Speed Tracking Error Over Iterations');
xlabel('Iteration Number');
ylabel('RMS Speed Tracking Error');

subplot(3,2,2);
plot(1:time_length,f_a_hat_N)
title('additonal error');
xlabel('Time Step');
ylabel('牛頓(N)');

subplot(3,2,3);
plot(1:time_length,f_b_N)
title('basemental error');
xlabel('Time Step');
ylabel('牛頓(N)');

subplot(3,2,4);
plot(1:time_length,F_N)
title('Force');
xlabel('Time Step');
ylabel('牛頓(N)');

subplot(3,2,5);
plot(1:time_length, p_actual_position_limit, 'b', 1:time_length, p_desired(1, :), 'r--');
title('Position Tracking');
xlabel('Time Step');
ylabel('position (km)');

subplot(3,2,6);
plot(1:time_length, p_actual_speed_limit, 'b', 1:time_length, p_desired(2, :), 'r--');
title('velocity Tracking');
xlabel('Time Step');
ylabel('velocity (km/h)');
legend('Actual speed', 'Desired speed');

% figure;
% plot(1:time_length, p_actual_position, 'b');

%======================================================================
%證明系統能夠追蹤期望數度軌跡及估計出總體的附加阻力
% 路線劃分（示例）
sections = [
    struct('distance', 5e3, 'speed_limit', 100/3.6, 'gradient',0),   % 區段1
    struct('distance', 10e3, 'speed_limit', 200/3.6, 'gradient',0), % 區段2
    struct('distance', 20e3, 'speed_limit', 300/3.6, 'gradient',0), % 區段3
    struct('distance', 15e3, 'speed_limit', 250/3.6, 'gradient',0), % 區段4
    struct('distance', 22e3, 'speed_limit', 300/3.6, 'gradient',0), % 區段5
];

% 列車參數
mass = 350e3;           % 列車質量，單位：kg
max_acceleration = 0.6; % 最大加速度，單位：m/s^2
max_deceleration = -1.0; % 最大減速度，單位：m/s^2（負值）

% 初始化
dt = 1; % 時間步長，單位：秒
t = 0;
v = 0;
s = 0;
i = 1;
additional_resistance_coeff = 0;
time = [];
speed = [];
position = [];
total_resist = [];
additional_resistance = [];
F_N(end+2000) = 0;
% 遍歷每個區段
for sec = 1:length(sections)
    sec_distance = sections(sec).distance;
    speed_limit = sections(sec).speed_limit;
    gradient = sections(sec).gradient; % 坡度，百分比表示，如 0.5% = 0.005

    % 計算本區段的起始和結束位置
    sec_start_distance = sum([sections(1:sec-1).distance]);
    sec_end_distance = sum([sections(1:sec).distance]);

    % 更新附加阻力邏輯
    while s < sec_end_distance
        % 更新附加阻力條件
        if sec == 2
            if s >= sec_start_distance && s <= sec_start_distance + 0.6 * sec_distance
                additional_resistance_coeff = 0.08; % 第一條件
            else
                additional_resistance_coeff = 0;
            end
        elseif sec == 3
            if s >= sec_start_distance && s <= sec_start_distance + 0.7 * sec_distance
                additional_resistance_coeff = 0.06; % 第二條件
                fprintf('Applying additional resistance coeff: %f at position s: %f\n', additional_resistance_coeff, s);
            else
                additional_resistance_coeff = 0;
            end
        else
            additional_resistance_coeff = 0;
        end

        % 計算阻力
        rolling_resistance_coeff = (2977 + 27500 * (1-sin(i))) / mass; % 滾動阻力係數
        machine_resistance_coeff = (25.17 + 2400 * (1-sin(i))) / mass; % 機械阻力係數
        air_resistance_coeff = (0.3864 + 40 * (1-sin(i))) /mass;    % 空氣阻力係數
        grade_resistance = mass * 9.81 * gradient/100 ;
        rolling_resistance = rolling_resistance_coeff;
        machine_resistance = machine_resistance_coeff * v;
        air_resistance = air_resistance_coeff * v^2;
        total_resistance = (grade_resistance + rolling_resistance + machine_resistance + air_resistance + additional_resistance_coeff) * mass;

        % 計算可用牽引力
        if v < speed_limit
            acceleration = (F_N(i)+max_acceleration * mass - total_resistance) / mass;
            acceleration = min(acceleration, max_acceleration);
        else
            acceleration = (-total_resistance) / mass;
        end

        % 計算剩餘距離
        remaining_distance_in_section = sec_end_distance - s;
        remaining_total_distance = sum([sections(sec:end).distance]) - (s - sec_start_distance);

        % 檢查減速需求
        braking_distance = (v^2) / (2 * abs(max_deceleration));
        if remaining_total_distance <= braking_distance
            acceleration = max_deceleration;
        end

        % 更新速度和位置
        v = v + acceleration * dt;
        v = min(v, speed_limit); % 不超過速度限制
        v = max(v, 0);           % 速度不為負
        s = s + v * dt;
        t = t + dt;

        % 保存數據
        time(i) = t;
        speed_proof(i) = v;
        position_proof(i) = s;
        speed_variant_actual_proof(i) = acceleration;
        force_actual_proof(i) = mass * acceleration;
        total_resist(i) = total_resistance;
        i = i + 1;

        % 列車停止時退出
        if v <= 0 && remaining_distance_in_section <= 0
            break;
        end
    end
end
% 將速度和位置轉換為常用單位
speed_kmh_proof = speed_proof * 3.6;
speed_kmh_proof_limit = speed_kmh_actual(1:1159);
position_km_proof = position_proof / 1000;

size(speed_kmh_proof_limit)
size(speed_kmh_ideal)
size(position_km_proof)
size(position_km_ideal)


%繪製速度-時間圖
figure;
subplot(2,1,1);
plot(1:1159, speed_kmh_proof(1:1159), 1:1159, speed_kmh_ideal(1:1159), 'LineWidth', 2);
xlabel('時間 (秒)');
ylabel('速度 (km/h)');
title('速度-時間圖(修正後)');
legend('fixed speed','ideal speed');
grid on;

%繪製位置-時間圖
subplot(2,1,2);
plot(1:1159, position_km_proof(1:1159), 1:1159, position_km_ideal(1:1159), 'LineWidth', 2);
xlabel('時間 (秒)');
ylabel('位置 (km/h)');
title('位置-時間圖(修正後)');
legend('fixed position','ideal position');
grid on;
