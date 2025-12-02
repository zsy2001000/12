%% 无人机最低飞行高度计算与可视化 (全要素展示版)
% 功能：
%   1. 复现去重取优逻辑，识别出"被剔除的点"。
%   2. 计算最终保留点的最低通信保障高度。
%   3. 在 3D 场景中全要素展示：
%      - 原始固定点 (黑方块)、原始补充点 (红空心圆)
%      - 被剔除点 (灰色叉号)
%      - 最终路径 (红线 + 蓝实心点)
%      - 最低通信高度 (红色三角形)

clear; clc; close all;

%% ========================================================================
%  Step 0: 参数与文件检查
% ========================================================================
GIS_File = 'gis.csv';
Fixed_Point_File = 'fixed_inspection_points.csv';

% [已知] 无人机基站位置
BS_Pos = [355530, 2941780, 1220]; 

% 算法参数 (用于复现逻辑)
Num_Subsections = 50; 
Threshold_Coeff = 0.2; 

fprintf('正在初始化...\n');

if ~isfile(GIS_File), error('找不到 %s。', GIS_File); end
if ~isfile(Fixed_Point_File), error('找不到 %s。请先运行 gissimulation_final.m。', Fixed_Point_File); end

%% ========================================================================
%  Step 1: 数据准备与逻辑复现 (为了找出被剔除的点)
% ========================================================================
fprintf('正在复现优化逻辑以识别各类点...\n');

% 1. 读取 GIS 背景
data_road = readtable(GIS_File);
rx = data_road.x; ry = data_road.y; rz = data_road.z;
[rx_sorted, sort_idx] = sort(rx);
ry_sorted = ry(sort_idx); rz_sorted = rz(sort_idx);

% 2. 读取固定点
data_fixed = readtable(Fixed_Point_File);
Pts_Fixed = [data_fixed.x, data_fixed.y, data_fixed.z];
Type_Fixed = ones(size(Pts_Fixed,1), 1) * 1;

% 3. 生成原始补充点 (DIPH)
x_min = min(rx); x_max = max(rx);
x_edges = linspace(x_min, x_max, Num_Subsections + 1);
Pts_Supp = [];

for k = 1:Num_Subsections
    idx = (rx_sorted >= x_edges(k)) & (rx_sorted < x_edges(k+1));
    if ~any(idx), continue; end
    px = rx_sorted(idx); py = ry_sorted(idx); pz = rz_sorted(idx);
    cx = mean(px); cy = mean(py);
    
    idx_up = py > cy;
    if any(idx_up)
        d = sqrt((px(idx_up)-cx).^2 + (py(idx_up)-cy).^2);
        [~, mid] = max(d);
        tmp_idx = find(idx_up); real_id = tmp_idx(mid);
        Pts_Supp = [Pts_Supp; px(real_id), py(real_id), pz(real_id)];
    end
    idx_down = py <= cy;
    if any(idx_down)
        d = sqrt((px(idx_down)-cx).^2 + (py(idx_down)-cy).^2);
        [~, mid] = max(d);
        tmp_idx = find(idx_down); real_id = tmp_idx(mid);
        Pts_Supp = [Pts_Supp; px(real_id), py(real_id), pz(real_id)];
    end
end
Type_Supp = ones(size(Pts_Supp,1), 1) * 2;

% 4. 执行去重逻辑
All_Pts = [Pts_Fixed; Pts_Supp];
All_Types = [Type_Fixed; Type_Supp];
[~, sort_order] = sort(All_Pts(:,1));
All_Pts = All_Pts(sort_order, :);
All_Types = All_Types(sort_order);

diffs = diff(All_Pts(:,1:2));
dists = sqrt(sum(diffs.^2, 2));
Limit_Dist = mean(dists) * Threshold_Coeff;

Final_Path = [];
Discarded_Points = [];

i = 1;
while i <= size(All_Pts, 1)
    Current_P = All_Pts(i, :);
    Current_Type = All_Types(i);
    
    if i == size(All_Pts, 1)
        Final_Path = [Final_Path; Current_P, Current_Type];
        break;
    end
    
    Next_P = All_Pts(i+1, :);
    dist_next = sqrt((Current_P(1)-Next_P(1))^2 + (Current_P(2)-Next_P(2))^2);
    
    if dist_next < Limit_Dist
        if i+2 <= size(All_Pts, 1), Target_P = All_Pts(i+2, :); else, Target_P = Current_P; end
        d1 = norm(Current_P(1:3)-Target_P(1:3));
        d2 = norm(Next_P(1:3)-Target_P(1:3));
        
        if d1 < d2
            Final_Path = [Final_Path; Current_P, Current_Type];
            Discarded_Points = [Discarded_Points; Next_P(1:3)];
            i = i + 2;
        else
            Discarded_Points = [Discarded_Points; Current_P(1:3)];
            i = i + 1;
        end
    else
        Final_Path = [Final_Path; Current_P, Current_Type];
        i = i + 1;
    end
end

% 提取最终点坐标
Pts_Final = Final_Path(:, 1:3);
Num_Pts = size(Pts_Final, 1);

%% ========================================================================
%  Step 2: 计算最低通信高度 (仅针对最终保留点)
% ========================================================================
fprintf('正在计算通信限制高度...\n');

Min_Comm_Heights = zeros(Num_Pts, 1);
Safe_Flight_Heights = zeros(Num_Pts, 1);

% 通信参数
P_tx = 20; G_tx = 5; G_rx = 3; f_GHz = 2.4; S_req = -85; L_other = 2;
env_a = 4.88; env_b = 0.43; Eta_LOS = 1; Eta_NLOS = 20;
c = 3e8; const_fspl = 20*log10(f_GHz*1e9) + 20*log10(4*pi/c);
L_max = P_tx + G_tx + G_rx - L_other - S_req;

for i = 1:Num_Pts
    P_curr = Pts_Final(i, :);
    r = sqrt((P_curr(1) - BS_Pos(1))^2 + (P_curr(2) - BS_Pos(2))^2);
    
    h_low = P_curr(3); h_high = P_curr(3) + 2000;
    
    if ~check_comm(h_high, BS_Pos(3), r, env_a, env_b, const_fspl, Eta_LOS, Eta_NLOS, L_max)
        Min_Comm_Heights(i) = NaN;
    else
        if check_comm(h_low, BS_Pos(3), r, env_a, env_b, const_fspl, Eta_LOS, Eta_NLOS, L_max)
            Min_Comm_Heights(i) = h_low;
        else
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
%  Step 3: 3D 可视化 (全要素展示)
% ========================================================================
fprintf('正在绘制 3D 场景...\n');
figure('Name', '3D Inspection: Full Logic Visualization', 'Color', 'w', 'Position', [50, 50, 1200, 800]);
hold on; grid on; axis equal;

% 1. 地形背景 (淡绿色)
step = 5; 
scatter3(rx_sorted(1:step:end), ry_sorted(1:step:end), rz_sorted(1:step:end), ...
         5, [0.7 0.9 0.7], '.', 'HandleVisibility', 'off');

% 2. [底层] 原始固定点 (黑色方块)
if ~isempty(Pts_Fixed)
    scatter3(Pts_Fixed(:,1), Pts_Fixed(:,2), Pts_Fixed(:,3), ...
             30, 'k', 's', 'DisplayName', 'Original Fixed');
end

% 3. [底层] 原始补充点 (红空心圆)
if ~isempty(Pts_Supp)
    scatter3(Pts_Supp(:,1), Pts_Supp(:,2), Pts_Supp(:,3), ...
             30, 'r', 'o', 'DisplayName', 'Original Supp');
end

% 4. [中间层] 被剔除点 (灰色叉号)
if ~isempty(Discarded_Points)
    scatter3(Discarded_Points(:,1), Discarded_Points(:,2), Discarded_Points(:,3), ...
             60, [0.5 0.5 0.5], 'x', 'LineWidth', 2, 'DisplayName', 'Discarded');
end

% 5. [顶层] 最终巡检路径 (红线 + 蓝实心点)
plot3(Pts_Final(:,1), Pts_Final(:,2), Pts_Final(:,3), 'r-', 'LineWidth', 2, 'DisplayName', 'Final Path');
scatter3(Pts_Final(:,1), Pts_Final(:,2), Pts_Final(:,3), 50, 'b', 'filled', 'o', 'DisplayName', 'Selected Point');

% 6. [顶层] 最低通信保障高度 (红色三角形)
valid_idx = ~isnan(Safe_Flight_Heights);
if any(valid_idx)
    scatter3(Pts_Final(valid_idx,1), Pts_Final(valid_idx,2), Safe_Flight_Heights(valid_idx), ...
             60, 'r', '^', 'filled', 'DisplayName', 'Min Comm Height');
         
    % 红色虚线连接
    for i = 1:Num_Pts
        if valid_idx(i) && (Safe_Flight_Heights(i) > Pts_Final(i,3) + 0.5)
            plot3([Pts_Final(i,1), Pts_Final(i,1)], ...
                  [Pts_Final(i,2), Pts_Final(i,2)], ...
                  [Pts_Final(i,3), Safe_Flight_Heights(i)], ...
                  'r:', 'LineWidth', 1.0, 'HandleVisibility', 'off');
        end
    end
end

% 7. 基站位置
plot3(BS_Pos(1), BS_Pos(2), BS_Pos(3), 'p', 'MarkerSize', 15, ...
    'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'g', 'DisplayName', 'Base Station');

% 视图设置
xlabel('Longitude (x)'); ylabel('Latitude (y)'); zlabel('Altitude (z)');
title('3D Inspection: Logic & Min Comm Heights');
legend('Location', 'northeast');
view(-30, 30); 
rotate3d on;

fprintf('绘图完成。\n');
fprintf('图中元素包含：原始固定点/补充点、灰色剔除点、蓝色保留点、红色路径、红色三角形(最低高度)。\n');

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