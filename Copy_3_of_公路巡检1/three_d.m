%% 公路无人机巡检规划 (DIPH) - 3D 悬停高度可视化版
% 功能：
%   1. 读取 gis.csv 生成路径
%   2. 读取 Li.m 生成的 g_range_data.mat 获取高度范围
%   3. 在 3D 场景中标注巡检点及其上方的"悬停有效高度区间"

clear; clc; close all;

%% 1. 数据准备
fprintf('正在检查数据文件...\n');

% 检查 GIS 数据
if ~isfile('gis.csv')
    error('缺少 gis.csv 文件！请确保该文件在当前目录下。');
end

% 检查高度范围数据 (由 Li.m 生成)
if ~isfile('g_range_data.mat')
    error('缺少 g_range_data.mat 文件！\n请先运行 Li.m 计算高度范围 g。');
else
    load('g_range_data.mat'); % 加载 g_min 和 g_max
    fprintf('已加载高度范围 g: [%.2f m, %.2f m]\n', g_min, g_max);
end

% 读取 GIS
data = readtable('gis.csv');
x = data.x; y = data.y; z = data.z;

% 预处理：排序
[x_sorted, sort_idx] = sort(x);
y_sorted = y(sort_idx);
z_sorted = z(sort_idx);

%% 2. DIPH 算法执行 (生成巡检点)
fprintf('正在计算 3D 巡检路径...\n');

Num_Subsections = 50; 
x_edges = linspace(min(x), max(x), Num_Subsections + 1);

Inspection_Points = []; % [x, y, z]
Center_Line_3D = [];    

for k = 1:Num_Subsections
    x_start = x_edges(k);
    x_end   = x_edges(k+1);
    
    idx_bin = (x_sorted >= x_start) & (x_sorted < x_end);
    pts_x = x_sorted(idx_bin); pts_y = y_sorted(idx_bin); pts_z = z_sorted(idx_bin);
    
    if isempty(pts_x), continue; end
    
    % 中心点
    center_x = mean(pts_x); center_y = mean(pts_y); center_z = mean(pts_z);
    Center_Line_3D = [Center_Line_3D; center_x, center_y, center_z];
    
    % 边线分割与最远点选择
    idx_upper = pts_y > center_y;
    idx_lower = pts_y <= center_y;
    
    % --- Upper Side ---
    if any(idx_upper)
        p_x = pts_x(idx_upper); p_y = pts_y(idx_upper); p_z = pts_z(idx_upper);
        dists = sqrt((p_x - center_x).^2 + (p_y - center_y).^2);
        [~, max_id] = max(dists);
        Inspection_Points = [Inspection_Points; p_x(max_id), p_y(max_id), p_z(max_id)];
    end
    
    % --- Lower Side ---
    if any(idx_lower)
        p_x = pts_x(idx_lower); p_y = pts_y(idx_lower); p_z = pts_z(idx_lower);
        dists = sqrt((p_x - center_x).^2 + (p_y - center_y).^2);
        [~, max_id] = max(dists);
        Inspection_Points = [Inspection_Points; p_x(max_id), p_y(max_id), p_z(max_id)];
    end
end

% 路径排序
[~, sort_ip] = sort(Inspection_Points(:,1));
Inspection_Points = Inspection_Points(sort_ip, :);
[~, sort_cl] = sort(Center_Line_3D(:,1));
Center_Line_3D = Center_Line_3D(sort_cl, :);

%% 3. 3D 可视化 (含悬停高度范围)
fprintf('正在绘制 3D 场景...\n');

figure('Name', '3D Inspection with Hover Range', 'Color', 'w', 'Position', [50, 50, 1200, 800]);
hold on; grid on; axis equal;

% (1) 绘制公路地形
step = 5; 
scatter3(x_sorted(1:step:end), y_sorted(1:step:end), z_sorted(1:step:end), ...
         1, 'g', '.', 'MarkerEdgeAlpha', 0.1, 'DisplayName', 'Highway Terrain');

% (2) 绘制中心线
if ~isempty(Center_Line_3D)
    plot3(Center_Line_3D(:,1), Center_Line_3D(:,2), Center_Line_3D(:,3), ...
          '--', 'Color', [0.3 0.3 0.3], 'LineWidth', 1.5, 'DisplayName', 'Center Line');
end

% (3) 绘制巡检点 (红点, 地面投影)
if ~isempty(Inspection_Points)
    % 路径连线
    plot3(Inspection_Points(:,1), Inspection_Points(:,2), Inspection_Points(:,3), ...
          'r-', 'LineWidth', 1.0, 'DisplayName', 'Path (Ground Projection)');
    % 点标记
    scatter3(Inspection_Points(:,1), Inspection_Points(:,2), Inspection_Points(:,3), ...
             20, 'r', 'o', 'filled', 'DisplayName', 'Inspection Point (Ground)');
end

% (4) 【核心】绘制悬停高度范围 (Hover Range)
% 在每个红点正上方画一条垂直线，范围 [z+g_min, z+g_max]
fprintf('正在生成悬停高度标注 (g_min=%.2f, g_max=%.2f)...\n', g_min, g_max);

if ~isempty(Inspection_Points)
    for i = 1:size(Inspection_Points, 1)
        ix = Inspection_Points(i, 1);
        iy = Inspection_Points(i, 2);
        iz = Inspection_Points(i, 3);
        
        % 计算悬停区间的绝对高度
        h_start = iz + g_min;
        h_end   = iz + g_max;
        
        % 绘制垂直线 (蓝色，代表有效拍摄区间)
        % 仅给第一条线加 DisplayName 以避免图例重复
        if i == 1
            plot3([ix, ix], [iy, iy], [h_start, h_end], ...
                  'b-', 'LineWidth', 2, 'DisplayName', 'Valid Hover Range');
            % 绘制上下界限点 (可选)
            plot3(ix, iy, h_start, 'b_', 'MarkerSize', 5, 'HandleVisibility', 'off');
            plot3(ix, iy, h_end,   'b_', 'MarkerSize', 5, 'HandleVisibility', 'off');
        else
            plot3([ix, ix], [iy, iy], [h_start, h_end], 'b-', 'LineWidth', 2, 'HandleVisibility', 'off');
            plot3(ix, iy, h_start, 'b_', 'MarkerSize', 5, 'HandleVisibility', 'off');
            plot3(ix, iy, h_end,   'b_', 'MarkerSize', 5, 'HandleVisibility', 'off');
        end
        
        % 可选：绘制一条虚线连接地面点和悬停区，增加空间感
        plot3([ix, ix], [iy, iy], [iz, h_start], 'k:', 'LineWidth', 0.5, 'Color', [0.5 0.5 0.5, 0.5], 'HandleVisibility', 'off');
    end
end

% 视图设置
xlabel('Longitude (x)'); ylabel('Latitude (y)'); zlabel('Altitude (z)');
title(['3D Inspection Path with Valid Hover Ranges (g: ' num2str(g_min) 'm - ' num2str(g_max) 'm)']);
legend('show', 'Location', 'northeast');
view(-30, 20); rotate3d on;

fprintf('绘图完成。图中蓝色垂直线段表示在该点上方的有效拍摄高度范围。\n');