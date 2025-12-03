%% 公路无人机巡检规划 (DIPH) - 最终修正完美版
% 修改说明：
%   1. 恢复了路径与中心线的几何交点计算与绘制 (绿圈)。
%   2. 修正了固定点逻辑：仅在文件缺失时生成，保证"固定住"不乱变。
%   3. 调整了固定点密度：增加了生成数量，确保局部视图有 6-7 个点。
%   4. 严格复现了包含中心线(绿虚线)的完美绘图风格。

clear; clc; close all;

%% ========================================================================
%  Step 0: 全局设置与固定点准备
% ========================================================================
GIS_File = 'gis.csv';
Fixed_Point_File = 'fixed_inspection_points.csv';

% DIPH 算法参数
Num_Subsections = 50; % 算法分段数

% [修正点 1] 调整固定点生成数量
% 为了在局部视图(约7个分段)中看到 6-7 个固定点，我们需要平均每段约 1 个点。
% 总分段 50，所以我们需要约 50 个固定点。
% 原逻辑是 1/3 (约33个)，现在调整为 Num_Subsections (约50个)。
Num_Fixed_Desired = Num_Subsections; 

% [修正点 2] 固定点持久化逻辑
% 只有当文件"不存在"时，才调用生成模块。这样保证了固定点一旦生成就不再变化。
if ~isfile(Fixed_Point_File)
    fprintf('[系统提示] 未检测到固定点文件，正在首次生成...\n');
    generate_fixed_points(GIS_File, Fixed_Point_File, Num_Fixed_Desired);
else
    fprintf('[系统提示] 检测到现有固定点文件，将直接读取 (保持位置不变)。\n');
end

%% ========================================================================
%  Step 1: 读取环境数据
% ========================================================================
fprintf('正在加载地图与固定点数据...\n');

% 1. 读取路网 GIS
data_road = readtable(GIS_File);
x = data_road.x; y = data_road.y; z = data_road.z;
[x_sorted, sort_idx] = sort(x);
y_sorted = y(sort_idx);

% 2. 读取固定巡检点
data_fixed = readtable(Fixed_Point_File);
Fixed_Points = [data_fixed.x, data_fixed.y, data_fixed.z]; 

fprintf('固定巡检点加载完毕: %d 个\n', size(Fixed_Points, 1));

%% ========================================================================
%  Step 2: 运行 DIPH 算法生成"补充巡检点"
% ========================================================================
fprintf('正在运行算法生成补充巡检点...\n');

x_min = min(x); x_max = max(x);
x_edges = linspace(x_min, x_max, Num_Subsections + 1);

Results = struct('center', {}, 'upper_pts', {}, 'lower_pts', {}, 'x_range', {});
Supp_Inspection_Points = []; % [x, y]
Center_Line_Points = [];     % [x, y]

for k = 1:Num_Subsections
    % 2.1 划分子路段
    x_start = x_edges(k); x_end = x_edges(k+1);
    idx_bin = (x_sorted >= x_start) & (x_sorted < x_end);
    pts_x = x_sorted(idx_bin); pts_y = y_sorted(idx_bin);
    
    if isempty(pts_x), continue; end
    
    % 2.2 中心点选择
    center_point = [mean(pts_x), mean(pts_y)];
    Center_Line_Points = [Center_Line_Points; center_point];
    
    % 2.3 边线勾画
    idx_upper = pts_y > center_point(2);
    idx_lower = pts_y <= center_point(2);
    upper_pts = [pts_x(idx_upper), pts_y(idx_upper)];
    lower_pts = [pts_x(idx_lower), pts_y(idx_lower)];
    
    Results(k).center = center_point;
    Results(k).upper_pts = upper_pts;
    Results(k).lower_pts = lower_pts;
    Results(k).x_range = [x_start, x_end];
    
    % 2.4 补充巡检点选择 (只选最远点)
    if ~isempty(upper_pts)
        dists = sqrt((upper_pts(:,1)-center_point(1)).^2 + (upper_pts(:,2)-center_point(2)).^2);
        [~, max_id] = max(dists);
        Supp_Inspection_Points = [Supp_Inspection_Points; upper_pts(max_id, :)];
    end
    if ~isempty(lower_pts)
        dists = sqrt((lower_pts(:,1)-center_point(1)).^2 + (lower_pts(:,2)-center_point(2)).^2);
        [~, max_id] = max(dists);
        Supp_Inspection_Points = [Supp_Inspection_Points; lower_pts(max_id, :)];
    end
end

% 排序
[~, sort_supp] = sort(Supp_Inspection_Points(:,1));
Supp_Inspection_Points = Supp_Inspection_Points(sort_supp, :);

% 确保中心线也排序 (为了连线正确)
[~, sort_cl] = sort(Center_Line_Points(:,1));
Center_Line_Points = Center_Line_Points(sort_cl, :);

fprintf('补充点生成完毕: %d 个\n', size(Supp_Inspection_Points, 1));

%% ========================================================================
%  Step 3: [恢复] 计算路径与中心线的交点 (Intersections)
% ========================================================================
fprintf('正在计算几何交点...\n');
Intersections = [];

for i = 1:size(Supp_Inspection_Points, 1)-1
    p1 = Supp_Inspection_Points(i, :);
    p2 = Supp_Inspection_Points(i+1, :);
    
    for j = 1:size(Center_Line_Points, 1)-1
        c1 = Center_Line_Points(j, :);
        c2 = Center_Line_Points(j+1, :);
        
        % 快速包围盒检测
        if min(p1(1), p2(1)) > max(c1(1), c2(1)) || max(p1(1), p2(1)) < min(c1(1), c2(1)), continue; end
        
        % 计算交点
        [xi, yi, tf] = segment_intersect_simple(p1, p2, c1, c2);
        if tf
            Intersections = [Intersections; xi, yi];
        end
    end
end

%% ========================================================================
%  Step 4: 完美绘图展示
% ========================================================================
fprintf('正在绘制结果图...\n');

figure('Name', 'DIPH: Fixed (Existing) & Supplementary (Algo)', 'Color', 'w', 'Position', [50, 50, 1200, 900]);

% --- 图 (a) 原始 GIS ---
subplot(2, 2, 1);
scatter(x(1:10:end), y(1:10:end), 1, 'b', '.');
title('(a) Raw GIS Data'); axis equal; grid on;

% --- 图 (b) 分段细节 (共存展示) ---
subplot(2, 2, 2); hold on;
zoom_indices = 20:26; 

for i = zoom_indices
    if i > length(Results), break; end
    r = Results(i);
    % 背景点
    if ~isempty(r.upper_pts), scatter(r.upper_pts(1:5:end,1), r.upper_pts(1:5:end,2), 5, 'm', '.', 'HandleVisibility','off'); end
    if ~isempty(r.lower_pts), scatter(r.lower_pts(1:5:end,1), r.lower_pts(1:5:end,2), 5, 'b', '.', 'HandleVisibility','off'); end
    plot(r.center(1), r.center(2), 'r*', 'MarkerSize', 5, 'HandleVisibility','off');
    xline(r.x_range(1), '--k', 'Color', [0.8 0.8 0.8], 'HandleVisibility','off');
end

% 确定放大范围
zoom_min = Results(zoom_indices(1)).x_range(1);
zoom_max = Results(zoom_indices(end)).x_range(2);

% 绘制该范围内的固定点 (黑色方块)
in_zoom_fixed = (Fixed_Points(:,1) >= zoom_min) & (Fixed_Points(:,1) <= zoom_max);
if any(in_zoom_fixed)
    scatter(Fixed_Points(in_zoom_fixed,1), Fixed_Points(in_zoom_fixed,2), ...
        40, 'k', 's', 'filled', 'DisplayName', 'Fixed Points (Existing)');
end

title('(b) Sub-sections & Fixed Points');
xlabel('Longitude'); ylabel('Latitude'); axis tight; grid on; legend('Location','best');

% --- 图 (c) 全局视图 (恢复中心线和交点) ---
subplot(2, 2, 3); hold on;
scatter(x(1:20:end), y(1:20:end), 1, 'g', '.', 'MarkerEdgeAlpha', 0.2, 'HandleVisibility','off');

% 1. [恢复] 中心线 (绿色虚线)
plot(Center_Line_Points(:,1), Center_Line_Points(:,2), 'g--', 'LineWidth', 1.5, 'DisplayName', 'Center Line');

% 2. 补充路径 (红色实线)
plot(Supp_Inspection_Points(:,1), Supp_Inspection_Points(:,2), 'r-', 'LineWidth', 1.0, 'HandleVisibility','off');
scatter(Supp_Inspection_Points(:,1), Supp_Inspection_Points(:,2), 20, 'r', '*', 'DisplayName', 'Supp Points (Algorithm)');

% 3. 固定点 (黑方块)
scatter(Fixed_Points(:,1), Fixed_Points(:,2), 20, 'k', 's', 'filled', 'DisplayName', 'Fixed Points (Existing)');

% 4. [恢复] 几何交点 (绿色圆圈)
if ~isempty(Intersections)
    plot(Intersections(:,1), Intersections(:,2), 'go', 'MarkerSize', 6, 'LineWidth', 1.0, 'MarkerFaceColor', 'none', 'HandleVisibility','off');
end

title('(c) Global View: Fixed + Supp + CenterLine');
xlabel('Longitude'); ylabel('Latitude'); axis equal; grid on; legend('Location','best');

% --- 图 (d) 细节展示 (包含所有元素) ---
subplot(2, 2, 4); hold on;
xlim([zoom_min, zoom_max]);
y_range = y(x >= zoom_min & x <= zoom_max);
ylim([min(y_range), max(y_range)]);

% 背景
in_zoom_bg = (x >= zoom_min) & (x <= zoom_max);
scatter(x(in_zoom_bg), y(in_zoom_bg), 10, [0.8 1 0.8], '.', 'HandleVisibility','off');

% 1. [恢复] 局部中心线
in_zoom_cl = (Center_Line_Points(:,1) >= zoom_min) & (Center_Line_Points(:,1) <= zoom_max);
plot(Center_Line_Points(in_zoom_cl,1), Center_Line_Points(in_zoom_cl,2), 'g--', 'LineWidth', 1.5, 'HandleVisibility','off');

% 2. 局部补充点与路径
in_zoom_supp = (Supp_Inspection_Points(:,1) >= zoom_min) & (Supp_Inspection_Points(:,1) <= zoom_max);
pts_s = Supp_Inspection_Points(in_zoom_supp, :);
plot(pts_s(:,1), pts_s(:,2), 'r-', 'LineWidth', 1.5, 'DisplayName', 'Supp Path');
scatter(pts_s(:,1), pts_s(:,2), 60, 'r', '*', 'LineWidth', 1.5, 'DisplayName', 'Supp Points');

% 3. 局部固定点 (确保密度足够)
if any(in_zoom_fixed)
    scatter(Fixed_Points(in_zoom_fixed,1), Fixed_Points(in_zoom_fixed,2), ...
        50, 'k', 's', 'filled', 'DisplayName', 'Fixed Points');
end

% 4. [恢复] 局部交点
if ~isempty(Intersections)
    in_int = (Intersections(:,1) >= zoom_min) & (Intersections(:,1) <= zoom_max);
    plot(Intersections(in_int,1), Intersections(in_int,2), 'go', 'MarkerSize', 8, 'LineWidth', 1.5, 'DisplayName', 'Crossing Point');
end

title('(d) Detail: Path Crossing & Points');
xlabel('Longitude'); ylabel('Latitude'); grid on; legend('Location','best');

fprintf('完成。若要重新生成固定点，请删除 fixed_inspection_points.csv 文件后重试。\n');

%% ========================================================================
%  辅助函数：线段交点计算
% ========================================================================
function [xi, yi, tf] = segment_intersect_simple(P1, P2, P3, P4)
    xi=0; yi=0; tf=false;
    x1=P1(1); y1=P1(2); x2=P2(1); y2=P2(2);
    x3=P3(1); y3=P3(2); x4=P4(1); y4=P4(2);
    
    denom = (y4-y3)*(x2-x1) - (x4-x3)*(y2-y1);
    if abs(denom)<1e-10, return; end
    
    ua = ((x4-x3)*(y1-y3) - (y4-y3)*(x1-x3))/denom;
    ub = ((x2-x1)*(y1-y3) - (y2-y1)*(x1-x3))/denom;
    
    if ua>=0 && ua<=1 && ub>=0 && ub<=1
        xi = x1 + ua*(x2-x1); yi = y1 + ua*(y2-y1); tf=true;
    end
end