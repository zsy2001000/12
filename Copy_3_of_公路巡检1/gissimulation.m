%% 公路无人机巡检规划 (DIPH) - 最终修正版
% 修正说明：
%   1. 已移除所有可能导致报错的引用标记。
%   2. 包含中心线 (Center Line) 绘制 (绿色虚线)。
%   3. 包含路径与中心线的几何交点 (Intersections) 计算 (绿色圆圈)。
%   4. 严格复现文档图2的所有细节，不简化任何流程。

clear; clc; close all;

%% ========================================================================
%  Step 1: 数据读取与预处理
%  说明：读取 gis.csv 并按经度排序
% ========================================================================
fprintf('正在读取 GIS 数据...\n');

if ~isfile('gis.csv')
    error('请确保 gis.csv 文件位于当前文件夹中！');
end

data = readtable('gis.csv');
x = data.x; % Longitude
y = data.y; % Latitude

% 按经度(x)排序，模拟沿公路走向的处理顺序
[x_sorted, sort_idx] = sort(x);
y_sorted = y(sort_idx);

%% ========================================================================
%  Step 2: 算法参数与执行 (DIPH Algorithm)
%  说明：执行文档 2.2 - 2.4 的核心步骤
% ========================================================================
fprintf('正在执行 DIPH 路径规划算法...\n');

% 设定子路段数量 (对应文档图2b的分段密度，设定为50段)
Num_Subsections = 50; 

x_min = min(x);
x_max = max(x);
x_edges = linspace(x_min, x_max, Num_Subsections + 1);

% 初始化存储结构
Results = struct('center', {}, 'upper_pts', {}, 'lower_pts', {}, 'x_range', {});
Inspection_Points = [];  % 存储最终巡检点 [x, y]
Center_Line_Points = []; % 存储所有子路段中心点，用于绘制中心线 [x, y]

for k = 1:Num_Subsections
    x_start = x_edges(k);
    x_end   = x_edges(k+1);
    
    % 获取当前子路段内的所有点
    idx_in_bin = (x_sorted >= x_start) & (x_sorted < x_end);
    pts_x = x_sorted(idx_in_bin);
    pts_y = y_sorted(idx_in_bin);
    
    if isempty(pts_x)
        continue;
    end
    
    % === 2.2 中心点选择 ===
    % 计算当前子路段的经纬度均值作为中心点
    center_point = [mean(pts_x), mean(pts_y)];
    Center_Line_Points = [Center_Line_Points; center_point]; 
    
    % === 2.3 边线勾画 ===
    % 根据点的纬度与中心点纬度的比较，将路段分为上下两部分
    idx_upper = pts_y > center_point(2);
    idx_lower = pts_y <= center_point(2);
    
    upper_pts = [pts_x(idx_upper), pts_y(idx_upper)];
    lower_pts = [pts_x(idx_lower), pts_y(idx_lower)];
    
    % 保存分段数据用于后续绘图验证 (图b)
    Results(k).center = center_point;
    Results(k).upper_pts = upper_pts;
    Results(k).lower_pts = lower_pts;
    Results(k).x_range = [x_start, x_end];
    
    % === 2.4 巡检点选择 ===
    % 在每一侧选择离中心点最远的点作为巡检点
    
    % 上半部分最远点
    if ~isempty(upper_pts)
        dists_up = sqrt((upper_pts(:,1) - center_point(1)).^2 + (upper_pts(:,2) - center_point(2)).^2);
        [~, max_id] = max(dists_up);
        Inspection_Points = [Inspection_Points; upper_pts(max_id, :)];
    end
    
    % 下半部分最远点
    if ~isempty(lower_pts)
        dists_low = sqrt((lower_pts(:,1) - center_point(1)).^2 + (lower_pts(:,2) - center_point(2)).^2);
        [~, max_id] = max(dists_low);
        Inspection_Points = [Inspection_Points; lower_pts(max_id, :)];
    end
end

% 对生成的巡检点按经度排序，形成连续路径
[~, sort_ip] = sort(Inspection_Points(:,1));
Inspection_Points = Inspection_Points(sort_ip, :);

% 确保中心线点集也按经度排序
[~, sort_cl] = sort(Center_Line_Points(:,1));
Center_Line_Points = Center_Line_Points(sort_cl, :);

%% ========================================================================
%  Step 3: 计算路径与中心线的交点
%  说明：计算红色巡检路径与绿色中心线的几何交点
% ========================================================================
fprintf('正在计算路径交点...\n');

Intersections = [];

% 遍历每一段巡检路径线段
for i = 1:size(Inspection_Points, 1)-1
    p1 = Inspection_Points(i, :);
    p2 = Inspection_Points(i+1, :);
    
    % 遍历每一段中心线线段
    for j = 1:size(Center_Line_Points, 1)-1
        c1 = Center_Line_Points(j, :);
        c2 = Center_Line_Points(j+1, :);
        
        % 快速排斥实验 (Bounding Box check)以提高效率
        if min(p1(1), p2(1)) > max(c1(1), c2(1)) || max(p1(1), p2(1)) < min(c1(1), c2(1)), continue; end
        
        % 调用线段相交函数
        [x_int, y_int, is_intersect] = segment_intersect_clean(p1, p2, c1, c2);
        if is_intersect
            Intersections = [Intersections; x_int, y_int];
        end
    end
end

%% ========================================================================
%  Step 4: 绘图验证
%  说明：严格按照文档图2的 (a)(b)(c)(d) 格式绘图
% ========================================================================
fprintf('正在绘制图表...\n');

figure('Name', 'DIPH Method Result', 'Color', 'w', 'Position', [50, 50, 1200, 900]);

% --- 图 (a) 原始 GIS 数据 ---
subplot(2, 2, 1);
scatter(x(1:10:end), y(1:10:end), 1, 'b', '.'); % 降采样显示以防卡顿
title('(a) Data Acquisition (GIS Points)');
xlabel('Longitude'); ylabel('Latitude');
axis equal; grid on;

% --- 图 (b) 分段与中心 (局部放大) ---
subplot(2, 2, 2);
hold on;
zoom_indices = 20:26; % 选取中间具有代表性的几段
for i = zoom_indices
    if i > length(Results), break; end
    r = Results(i);
    % 绘制分割区域背景 (模拟图2b的粉/蓝效果)
    if ~isempty(r.upper_pts), scatter(r.upper_pts(1:5:end,1), r.upper_pts(1:5:end,2), 5, 'm', '.', 'DisplayName', 'Upper Area'); end
    if ~isempty(r.lower_pts), scatter(r.lower_pts(1:5:end,1), r.lower_pts(1:5:end,2), 5, 'b', '.', 'DisplayName', 'Lower Area'); end
    % 绘制中心点
    plot(r.center(1), r.center(2), 'r*', 'MarkerSize', 8, 'LineWidth', 1.5);
    % 绘制子路段分割线
    xline(r.x_range(1), '--k', 'Color', [0.5 0.5 0.5]);
end
title('(b) Sub-sections & Centers (Zoomed)');
xlabel('Longitude'); ylabel('Latitude');
axis tight; grid on;

% --- 图 (c) 巡检点与路径 (全景) ---
subplot(2, 2, 3);
hold on;
% 绘制背景路网
scatter(x(1:20:end), y(1:20:end), 1, 'g', '.', 'MarkerEdgeAlpha', 0.2);
% 1. 绘制中心线 (Center Line) - 绿色虚线
plot(Center_Line_Points(:,1), Center_Line_Points(:,2), 'g--', 'LineWidth', 1.5, 'DisplayName', 'Center Line');
% 2. 绘制巡检路径 (Red Path) - 红色实线
plot(Inspection_Points(:,1), Inspection_Points(:,2), 'r-', 'LineWidth', 1.0, 'DisplayName', 'Inspection Path');
% 3. 绘制巡检点 (Stars)
plot(Inspection_Points(:,1), Inspection_Points(:,2), 'r*', 'MarkerSize', 6);

title('(c) Inspection Points & Center Line');
xlabel('Longitude'); ylabel('Latitude');
axis equal; grid on;

% --- 图 (d) 细节展示 (包含交点) ---
subplot(2, 2, 4);
hold on;
% 确定与图(b)相同的放大范围
zoom_min_x = Results(zoom_indices(1)).x_range(1);
zoom_max_x = Results(zoom_indices(end)).x_range(2);
xlim([zoom_min_x, zoom_max_x]);
% 根据该范围内的数据自动调整Y轴
y_in_range = y(x >= zoom_min_x & x <= zoom_max_x);
ylim([min(y_in_range), max(y_in_range)]);

% 绘制局部背景
in_zoom_mask = (x >= zoom_min_x) & (x <= zoom_max_x);
scatter(x(in_zoom_mask), y(in_zoom_mask), 10, [0.8 1 0.8], '.', 'DisplayName', 'Road Points');

% 1. 绘制局部中心线 (绿色虚线)
in_zoom_cl = (Center_Line_Points(:,1) >= zoom_min_x) & (Center_Line_Points(:,1) <= zoom_max_x);
plot(Center_Line_Points(in_zoom_cl,1), Center_Line_Points(in_zoom_cl,2), 'g--', 'LineWidth', 1.5);

% 2. 绘制局部路径 (红色实线 + 红星)
in_zoom_ip = (Inspection_Points(:,1) >= zoom_min_x) & (Inspection_Points(:,1) <= zoom_max_x);
pts_zoom = Inspection_Points(in_zoom_ip, :);
plot(pts_zoom(:,1), pts_zoom(:,2), 'r-', 'LineWidth', 1.5);
plot(pts_zoom(:,1), pts_zoom(:,2), 'r*', 'MarkerSize', 10, 'LineWidth', 1.5);

% 3. 绘制交点 (Intersections) - 绿色空心圆圈
if ~isempty(Intersections)
    in_zoom_int = (Intersections(:,1) >= zoom_min_x) & (Intersections(:,1) <= zoom_max_x);
    plot(Intersections(in_zoom_int, 1), Intersections(in_zoom_int, 2), 'go', 'MarkerSize', 8, 'LineWidth', 1.5, 'MarkerFaceColor', 'none', 'DisplayName', 'Crossing Point');
end

title('(d) Path Intersections (Zoomed Detail)');
xlabel('Longitude'); ylabel('Latitude');
grid on;

fprintf('绘图完成。\n');


%% ========================================================================
%  辅助函数：线段交点计算
% ========================================================================
function [xi, yi, tf] = segment_intersect_clean(P1, P2, P3, P4)
    % 计算线段 P1-P2 和 P3-P4 的交点
    % 输入: P1, P2 为线段1端点 [x, y]
    %       P3, P4 为线段2端点 [x, y]
    % 输出: [xi, yi] 交点坐标, tf (bool) 是否相交
    
    xi = 0; yi = 0; tf = false;
    
    x1 = P1(1); y1 = P1(2);
    x2 = P2(1); y2 = P2(2);
    x3 = P3(1); y3 = P3(2);
    x4 = P4(1); y4 = P4(2);
    
    % 向量叉乘法计算分母
    denom = (y4-y3)*(x2-x1) - (x4-x3)*(y2-y1);
    
    % 如果分母极小，视为平行
    if abs(denom) < 1e-10
        return; 
    end
    
    ua = ((x4-x3)*(y1-y3) - (y4-y3)*(x1-x3)) / denom;
    ub = ((x2-x1)*(y1-y3) - (y2-y1)*(x1-x3)) / denom;
    
    % 检查交点是否在线段范围内
    if (ua >= 0 && ua <= 1) && (ub >= 0 && ub <= 1)
        xi = x1 + ua*(x2-x1);
        yi = y1 + ua*(y2-y1);
        tf = true;
    end
end