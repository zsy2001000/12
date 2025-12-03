%% 公路无人机巡检路径优化 (Path Optimization - Explicit Visualization)
% 功能：
%   1. 读取固定巡检点(严格边缘)和算法生成的补充巡检点。
%   2. 合并并按路径顺序排序。
%   3. [核心机制] 距离阈值检测：若两点距离 < 平均间距的20%，则判定为冗余。
%      机制：在冲突对中，保留距离下一个目标点更近的那个点。
%   4. 输出最终 CSV。
%   5. [绘图优化] 明确区分：原始固定点、原始补充点、被剔除点、最终保留点。

clear; clc; close all;

%% ========================================================================
%  Step 0: 参数设置
% ========================================================================
GIS_File = 'gis.csv';
Fixed_Point_File = 'fixed_inspection_points.csv';
Output_File = 'final_optimized_path.csv';

% DIPH 算法参数 (用于复现补充点)
Num_Subsections = 50; 

% [参数] 距离阈值系数
% 0.2 表示如果两点间距小于平均间距的 20%，则认为太近，需要合并。
Threshold_Coeff = 0.2; 

%% ========================================================================
%  Step 1: 数据准备
% ========================================================================
fprintf('正在准备数据...\n');

if ~isfile(GIS_File), error('缺少 GIS 文件'); end
data_road = readtable(GIS_File);
x = data_road.x; y = data_road.y; z = data_road.z;

if ~isfile(Fixed_Point_File), error('缺少固定点文件，请先运行生成器'); end
data_fixed = readtable(Fixed_Point_File);
Pts_Fixed = [data_fixed.x, data_fixed.y, data_fixed.z];
Type_Fixed = ones(size(Pts_Fixed,1), 1) * 1; % 1=Fixed

x_min = min(x); x_max = max(x);
x_edges = linspace(x_min, x_max, Num_Subsections + 1);
Pts_Supp = [];
[x_s, sort_idx] = sort(x); y_s = y(sort_idx); z_s = z(sort_idx);

for k = 1:Num_Subsections
    idx = (x_s >= x_edges(k)) & (x_s < x_edges(k+1));
    if ~any(idx), continue; end
    px = x_s(idx); py = y_s(idx); pz = z_s(idx);
    center = [mean(px), mean(py)];
    idx_up = py > center(2);
    if any(idx_up)
        d = sqrt((px(idx_up)-center(1)).^2 + (py(idx_up)-center(2)).^2);
        [~, mid] = max(d);
        tmp_idx = find(idx_up); real_id = tmp_idx(mid);
        Pts_Supp = [Pts_Supp; px(real_id), py(real_id), pz(real_id)];
    end
    idx_down = py <= center(2);
    if any(idx_down)
        d = sqrt((px(idx_down)-center(1)).^2 + (py(idx_down)-center(2)).^2);
        [~, mid] = max(d);
        tmp_idx = find(idx_down); real_id = tmp_idx(mid);
        Pts_Supp = [Pts_Supp; px(real_id), py(real_id), pz(real_id)];
    end
end
Type_Supp = ones(size(Pts_Supp,1), 1) * 2; % 2=Supp

fprintf('原始数据加载完成：固定点 %d 个，补充点 %d 个。\n', size(Pts_Fixed,1), size(Pts_Supp,1));

%% ========================================================================
%  Step 2: 路径合并与优化
% ========================================================================
All_Pts = [Pts_Fixed; Pts_Supp];
All_Types = [Type_Fixed; Type_Supp];

[~, sort_order] = sort(All_Pts(:,1));
All_Pts = All_Pts(sort_order, :);
All_Types = All_Types(sort_order);

diffs = diff(All_Pts(:,1:2));
dists = sqrt(sum(diffs.^2, 2));
Avg_Dist = mean(dists);
Limit_Dist = Avg_Dist * Threshold_Coeff;

fprintf('--- 优化参数 ---\n');
fprintf('平均间距: %.2f 米\n', Avg_Dist);
fprintf('合并阈值: %.2f 米\n', Limit_Dist);

Final_Path = [];
Discarded_Points = [];
Conflict_Pairs = [];

i = 1;
while i <= size(All_Pts, 1)
    Current_P = All_Pts(i, :);
    Current_Type = All_Types(i);
    
    if i == size(All_Pts, 1)
        Final_Path = [Final_Path; Current_P, Current_Type];
        break;
    end
    
    Next_P = All_Pts(i+1, :);
    Next_Type = All_Types(i+1);
    
    dist_next = sqrt((Current_P(1)-Next_P(1))^2 + (Current_P(2)-Next_P(2))^2);
    
    if dist_next < Limit_Dist
        if i+2 <= size(All_Pts, 1)
            Target_P = All_Pts(i+2, :);
        else
            Target_P = Current_P; 
        end
        
        dist_curr_target = norm(Current_P(1:3) - Target_P(1:3));
        dist_next_target = norm(Next_P(1:3) - Target_P(1:3));
        
        Conflict_Pairs = [Conflict_Pairs; Current_P(1:2), Next_P(1:2)];
        
        if dist_curr_target < dist_next_target
            Final_Path = [Final_Path; Current_P, Current_Type];
            Discarded_Points = [Discarded_Points; Next_P];
            i = i + 2; 
        else
            Discarded_Points = [Discarded_Points; Current_P];
            i = i + 1; 
        end
    else
        Final_Path = [Final_Path; Current_P, Current_Type];
        i = i + 1;
    end
end

fprintf('优化完成。保留点数: %d，剔除点数: %d。\n', size(Final_Path,1), size(Discarded_Points,1));

T_out = table(Final_Path(:,1), Final_Path(:,2), Final_Path(:,3), Final_Path(:,4), ...
    'VariableNames', {'x', 'y', 'z', 'type_1fixed_2supp'});
writetable(T_out, Output_File);
fprintf('最终优化路径已保存至: %s\n', Output_File);

%% ========================================================================
%  Step 3: 绘图展示 (逻辑可视化版)
% ========================================================================
fprintf('正在绘制逻辑演示图...\n');
figure('Name', 'Path Optimization Logic', 'Color', 'w', 'Position', [50, 50, 1200, 700]);
hold on; grid on;

% 自动范围
if ~isempty(Discarded_Points)
    focus_x = Discarded_Points(1, 1);
    display_range = 2000; 
    xlim([focus_x - display_range, focus_x + display_range]);
else
    mid_x = mean(x);
    xlim([mid_x - 2000, mid_x + 2000]);
end
xl = xlim;

% Y轴
idx_view = (x >= xl(1)) & (x <= xl(2));
if any(idx_view)
    yl_min = min(y(idx_view)); yl_max = max(y(idx_view));
    ylim([yl_min - 80, yl_max + 80]); 
end

% 1. 背景
scatter(x(idx_view), y(idx_view), 15, [0.6 0.8 0.6], '.', 'HandleVisibility', 'off');

% --- 第一层：绘制原始点（作为底图）---
% 1.1 原始固定点 (黑色方框)
idx_fix = (Pts_Fixed(:,1) >= xl(1)) & (Pts_Fixed(:,1) <= xl(2));
if any(idx_fix)
    scatter(Pts_Fixed(idx_fix,1), Pts_Fixed(idx_fix,2), 80, 'k', 's', 'LineWidth', 1.5, 'DisplayName', 'Original Fixed (Source)');
end

% 1.2 原始补充点 (红色圆圈，为了区分最终选中的点，这里用空心圆)
idx_supp = (Pts_Supp(:,1) >= xl(1)) & (Pts_Supp(:,1) <= xl(2));
if any(idx_supp)
    scatter(Pts_Supp(idx_supp,1), Pts_Supp(idx_supp,2), 80, 'r', 'o', 'LineWidth', 1.5, 'DisplayName', 'Original Supp (Source)');
end

% --- 第二层：绘制优化结果 ---

% 2.1 冲突区域 (黄色连线)
if ~isempty(Conflict_Pairs)
    for k = 1:size(Conflict_Pairs, 1)
        cp = Conflict_Pairs(k, :);
        if cp(1) >= xl(1) && cp(1) <= xl(2)
            line([cp(1), cp(3)], [cp(2), cp(4)], 'Color', [1 0.8 0], 'LineWidth', 3, 'HandleVisibility', 'off');
        end
    end
end

% 2.2 被剔除的点 (灰色叉号)
if ~isempty(Discarded_Points)
    idx_dis = (Discarded_Points(:,1) >= xl(1)) & (Discarded_Points(:,1) <= xl(2));
    if any(idx_dis)
        scatter(Discarded_Points(idx_dis,1), Discarded_Points(idx_dis,2), 120, [0.5 0.5 0.5], 'x', ...
            'LineWidth', 2.5, 'DisplayName', 'Discarded (Redundant)');
    end
end

% 2.3 最终路径 (红色实线)
idx_path = (Final_Path(:,1) >= xl(1)) & (Final_Path(:,1) <= xl(2));
Path_View = Final_Path(idx_path, :);

if ~isempty(Path_View)
    % 绘制红线
    plot(Final_Path(:,1), Final_Path(:,2), 'r-', 'LineWidth', 1.5, 'DisplayName', 'Optimized Path');
    
    % 2.4 最终保留的点 (蓝色实心圆)
    % 这些点覆盖在原始点之上，表明它们是被"选中"的状态
    scatter(Path_View(:,1), Path_View(:,2), 60, 'b', 'filled', 'o', 'DisplayName', 'Selected / Kept Point');
end

title('Path Optimization Logic: Selection & Rejection');
xlabel('Longitude'); ylabel('Latitude');
legend('Location', 'bestoutside');

fprintf('绘图完成。图例说明：\n');
fprintf(' - 黑色方框/红色空心圆：原始候选点\n');
fprintf(' - 灰色叉号：因冲突被剔除\n');
fprintf(' - 蓝色实心点：最终保留并执行巡检的点\n');