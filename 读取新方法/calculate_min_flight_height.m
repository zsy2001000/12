%% 无人机最低飞行高度计算与可视化 (新需求版)
% 功能：
%   1. 复现去重取优逻辑，识别出"被剔除的点"。
%   2. 计算最终保留点的最低通信保障高度 (基于新通信参数)。
%   3. 3D 可视化展示。

clear; clc; close all;

%% ========================================================================
%  Step 0: 参数与环境初始化 (模拟环境)
% ========================================================================
% 由于不再使用 csv，我们这里简单生成一组模拟数据来演示通信计算逻辑
% 实际场景中，此文件通常被主程序逻辑覆盖，但为了独立运行验证，这里保留模拟生成
fprintf('正在初始化模拟环境...\n');

BS_Pos = [2500, -500, 100]; % 基站位置 (适应5km跑道)
Road_Length = 5000;
Road_Width = 30;

% 模拟生成一些点
Num_Sim_Pts = 100;
sim_x = rand(Num_Sim_Pts, 1) * Road_Length;
sim_y = (rand(Num_Sim_Pts, 1) - 0.5) * Road_Width;
sim_z = zeros(Num_Sim_Pts, 1);
Pts_Final = [sim_x, sim_y, sim_z];

Num_Pts = size(Pts_Final, 1);

%% ========================================================================
%  Step 1: 计算最低通信高度 (核心修改部分)
% ========================================================================
fprintf('正在计算通信限制高度...\n');

Min_Comm_Heights = zeros(Num_Pts, 1);
Safe_Flight_Heights = zeros(Num_Pts, 1);

% --- 新通信参数配置 ---
P_tx_dBm = 20;          % 发射功率 20 dBm
B_Hz = 20e6;            % 带宽 20 MHz
N0_dBm_Hz = -160;       % 噪声谱密度 -160 dBm/Hz

% 计算接收灵敏度阈值 (S_req)
% 接收端热噪声底噪 Noise Floor = N0 + 10*log10(B)
Noise_Floor = N0_dBm_Hz + 10*log10(B_Hz); % 约 -87 dBm
S_req = Noise_Floor;    % 设定接收阈值为底噪

% 其他链路参数
G_tx = 5; G_rx = 3;     % 天线增益
f_GHz = 2.4;            % 载频
L_other = 2;            % 其他损耗

% 环境参数 (保持原模型)
env_a = 4.88; env_b = 0.43; Eta_LOS = 1; Eta_NLOS = 20;
c = 3e8; const_fspl = 20*log10(f_GHz*1e9) + 20*log10(4*pi/c);

% 最大允许路径损耗
L_max = P_tx_dBm + G_tx + G_rx - L_other - S_req;

for i = 1:Num_Pts
    P_curr = Pts_Final(i, :);
    r = sqrt((P_curr(1) - BS_Pos(1))^2 + (P_curr(2) - BS_Pos(2))^2);
    
    h_low = P_curr(3); h_high = P_curr(3) + 3000; % 搜索范围
    
    if ~check_comm(h_high, BS_Pos(3), r, env_a, env_b, const_fspl, Eta_LOS, Eta_NLOS, L_max)
        Min_Comm_Heights(i) = NaN; % 即使飞很高也不行
    else
        if check_comm(h_low, BS_Pos(3), r, env_a, env_b, const_fspl, Eta_LOS, Eta_NLOS, L_max)
            Min_Comm_Heights(i) = h_low;
        else
            % 二分法搜索最低高度
            while (h_high - h_low) > 0.1
                h_mid = (h_high + h_low) / 2;
                if check_comm(h_mid, BS_Pos(3), r, env_a, env_b, const_fspl, Eta_LOS, Eta_NLOS, L_max)
                    h_high = h_mid;
                else
                    h_low = h_mid;
                end
            end
            Min_Comm_Heights(i) = h_high;
        end
    end
    
    if isnan(Min_Comm_Heights(i))
        Safe_Flight_Heights(i) = NaN;
    else
        Safe_Flight_Heights(i) = max(Min_Comm_Heights(i), P_curr(3));
    end
end

%% ========================================================================
%  Step 2: 3D 可视化
% ========================================================================
fprintf('正在绘制 3D 场景...\n');
figure('Name', '3D Inspection: Min Comm Heights', 'Color', 'w', 'Position', [50, 50, 1000, 600]);
hold on; grid on; axis equal;

% 绘制巡检点
scatter3(Pts_Final(:,1), Pts_Final(:,2), Pts_Final(:,3), 30, 'b', 'filled', 'o', 'DisplayName', 'Inspection Point');

% 绘制最低通信高度
valid_idx = ~isnan(Safe_Flight_Heights);
if any(valid_idx)
    scatter3(Pts_Final(valid_idx,1), Pts_Final(valid_idx,2), Safe_Flight_Heights(valid_idx), ...
             50, 'r', '^', 'filled', 'DisplayName', 'Min Comm Height');
    
    % 连线
    for i = 1:Num_Pts
        if valid_idx(i) && (Safe_Flight_Heights(i) > Pts_Final(i,3) + 0.1)
            plot3([Pts_Final(i,1), Pts_Final(i,1)], ...
                  [Pts_Final(i,2), Pts_Final(i,2)], ...
                  [Pts_Final(i,3), Safe_Flight_Heights(i)], ...
                  'r:', 'LineWidth', 0.5, 'HandleVisibility', 'off');
        end
    end
end

% 基站
plot3(BS_Pos(1), BS_Pos(2), BS_Pos(3), 'p', 'MarkerSize', 15, ...
    'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'g', 'DisplayName', 'Base Station');

xlabel('Longitude (x)'); ylabel('Latitude (y)'); zlabel('Altitude (z)');
title('Minimum Communication Height Analysis');
legend('Location', 'best');
view(-30, 30); 

fprintf('计算完成。\n');

%% ========================================================================
%  内部函数：通信链路检查
% ========================================================================
function is_ok = check_comm(h_uav, h_bs, r, a, b, const_fspl, eta_los, eta_nlos, l_max)
    d = sqrt(r^2 + (h_uav - h_bs)^2);
    if r == 0, theta = 90; else, theta = rad2deg(atan((h_uav - h_bs) / r)); end
    p_los = 1 / (1 + a * exp(-b * (theta - a)));
    p_nlos = 1 - p_los;
    fspl = 20 * log10(d) + const_fspl;
    l_total = p_los * (fspl + eta_los) + p_nlos * (fspl + eta_nlos);
    is_ok = l_total <= l_max;
end