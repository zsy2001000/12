%% 公路无人机巡检规划 (DIPH) 仿真实验代码
% 对应文档：《路径规划方法》
% 包含内容：
%   1. 模拟公路 GIS 数据生成 (Section 3.1)
%   2. DIPH 路径规划方法实现 (Section 2)
%      - 2.1 数据采集
%      - 2.2 中心点选择
%      - 2.3 边线勾画
%      - 2.4 巡检点选择
%   3. 巡检目标与参数设定 (Section 3.1)
%      - 20% 目标点筛选
%      - 0~30s 悬停时间
%      - 1~100 重要系数

clear; clc; close all;

%% ========================================================================
%  Step 0: 仿真参数设置 (Simulation Settings)
%  对应文档 Section 3.1
% ========================================================================
fprintf('正在初始化仿真参数...\n');

% 1. 公路几何参数
Road_Length_m = 5000;   % 公路长度：5公里 (5000米)
Road_Width_m  = 30;     % 公路宽度：30米

% 2. 巡检点生成目标
% 文档要求生成 20 到 200 个巡检点。
% 因为每个子路段生成 2 个点（两侧各 1 个），所以我们需要划分 N/2 个子路段。
% 在此设定目标生成点数为 100 个 (即 50 个子路段)，您可以在 20-200 间任意修改。
Target_Num_Points = 100; 
Num_Subsections = round(Target_Num_Points / 2); 

% 3. 经纬度转换参数 (模拟真实GIS坐标)
% 假设位于大致纬度 30 度附近，1度纬度约为 111km，1度经度约为 96km
Scale_Lat = 1 / 111000; 
Scale_Lon = 1 / 96000;
Start_Lat = 29.402; % 参考文档图2(a)的起始坐标
Start_Lon = 106.5;

%% ========================================================================
%  Step 1: 数据采集/生成 (Data Collection)
%  对应文档 Section 2.1 & Section 3.1
%  说明：生成覆盖 5km*30m 区域的密集散点，模拟从 GIS 系统获取的原始数据。
% ========================================================================
fprintf('正在生成模拟GIS数据...\n');

% 为了模拟文档图 2(a) 中的曲线公路，我们生成一条略带弯曲的中心线
t = linspace(0, Road_Length_m, 10000); 
Center_X_m = t; 
% 使用正弦函数添加轻微曲线，模拟真实公路走向
Center_Y_m = 200 * sin(t / 1500); 

% 生成密集的原始采样点 (模拟 GIS 读取到的点云)
% 在中心线两侧 +/- Width/2 范围内随机撒点
Num_Raw_Points = 50000; % 模拟采集了 50000 个原始点
Raw_Data = zeros(Num_Raw_Points, 2); % [Latitude, Longitude]

rng(42); % 设定随机种子以保证结果可复现

for i = 1:Num_Raw_Points
    % 随机选择中心线上的一点位置
    idx = randi(length(t));
    cx = Center_X_m(idx);
    cy = Center_Y_m(idx);
    
    % 在道路宽度范围内产生随机偏移 (垂直于道路走向较为复杂，这里简化为Y轴偏移即可覆盖矩形区域，
    % 为了严谨，我们在法线方向偏移)
    % 计算切向量
    if idx < length(t)
        dx = Center_X_m(idx+1) - Center_X_m(idx);
        dy = Center_Y_m(idx+1) - Center_Y_m(idx);
    else
        dx = Center_X_m(idx) - Center_X_m(idx-1);
        dy = Center_Y_m(idx) - Center_Y_m(idx-1);
    end
    len = sqrt(dx^2 + dy^2);
    ux = -dy / len; % 法向量 X
    uy = dx / len;  % 法向量 Y
    
    % 随机偏移量 (-W/2 到 W/2)
    offset = (rand() - 0.5) * Road_Width_m;
    
    % 计算实际米制坐标
    px = cx + ux * offset;
    py = cy + uy * offset;
    
    % 转换为经纬度 (模拟 GIS 数据格式)
    Raw_Data(i, 1) = Start_Lat + py * Scale_Lat; % Latitude
    Raw_Data(i, 2) = Start_Lon + px * Scale_Lon; % Longitude
end

% 按经度排序，方便后续分段处理 (模拟沿公路走向处理)
[~, sort_idx] = sort(Raw_Data(:, 2));
Raw_Data = Raw_Data(sort_idx, :);

fprintf('GIS数据生成完毕。原始点数：%d\n', Num_Raw_Points);

%% ========================================================================
%  Step 2 & 3 & 4: 路径规划核心算法
%  对应文档 Section 2.2 (中心点), 2.3 (边线勾画), 2.4 (巡检点选择)
% ========================================================================
fprintf('开始执行DIPH路径规划算法...\n');

% 存储最终生成的巡检点
% 结构：[Latitude, Longitude]
Inspection_Points = [];

% 2.2 根据经纬度范围将区域划分为子路段 (Sub-sections)
% 我们根据经度范围均匀划分
Lon_Min = min(Raw_Data(:, 2));
Lon_Max = max(Raw_Data(:, 2));
Lon_Step = (Lon_Max - Lon_Min) / Num_Subsections;

for k = 1:Num_Subsections
    % 当前子路段的经度范围
    Sub_Lon_Start = Lon_Min + (k-1) * Lon_Step;
    Sub_Lon_End   = Lon_Min + k * Lon_Step;
    
    % 获取当前子路段内的所有点
    % indices 为逻辑索引
    indices = (Raw_Data(:, 2) >= Sub_Lon_Start) & (Raw_Data(:, 2) < Sub_Lon_End);
    Sub_Points = Raw_Data(indices, :);
    
    if isempty(Sub_Points)
        continue;
    end
    
    % === Section 2.2: 中心点选择 (Center Point Selection) ===
    % "对经纬度值进行平均，得到每个子路段的中心"
    Center_Point = mean(Sub_Points, 1); % [Lat_mean, Lon_mean]
    
    % === Section 2.3: 边线勾画 (Edge Delineation) ===
    % "根据路段中点的纬度与中心点的纬度的比较，将路段分为两部分"
    % Part 1: 纬度 > 中心点纬度 (Side A)
    % Part 2: 纬度 <= 中心点纬度 (Side B)
    Idx_Side_A = Sub_Points(:, 1) > Center_Point(1);
    Idx_Side_B = Sub_Points(:, 1) <= Center_Point(1);
    
    Points_Side_A = Sub_Points(Idx_Side_A, :);
    Points_Side_B = Sub_Points(Idx_Side_B, :);
    
    % === Section 2.4: 巡检点选择 (Inspection Point Selection) ===
    % "选择离各部分中心点最远的点作为巡逻点"
    
    % 处理 Side A
    if ~isempty(Points_Side_A)
        % 计算 Side A 中每个点到中心点(Center_Point)的欧氏距离
        % 注意：直接用经纬度计算欧氏距离在小范围内近似可行，或者转换为米计算
        % 这里为保持数学逻辑一致性，直接计算向量模长
        dists_A = sqrt((Points_Side_A(:,1) - Center_Point(1)).^2 + ...
                       (Points_Side_A(:,2) - Center_Point(2)).^2);
        [~, max_idx_A] = max(dists_A);
        Selected_A = Points_Side_A(max_idx_A, :);
        Inspection_Points = [Inspection_Points; Selected_A];
    end
    
    % 处理 Side B
    if ~isempty(Points_Side_B)
        dists_B = sqrt((Points_Side_B(:,1) - Center_Point(1)).^2 + ...
                       (Points_Side_B(:,2) - Center_Point(2)).^2);
        [~, max_idx_B] = max(dists_B);
        Selected_B = Points_Side_B(max_idx_B, :);
        Inspection_Points = [Inspection_Points; Selected_B];
    end
end

% 按照文档 Section 2.4 结尾描述：
% "最后将所有选定的巡逻点按公路边缘进行排序"
% 我们简单地按经度排序即可整理出顺序
[~, sort_final] = sort(Inspection_Points(:, 2));
Inspection_Points = Inspection_Points(sort_final, :);

Num_Generated = size(Inspection_Points, 1);
fprintf('巡检点生成完毕。共生成 %d 个巡检点。\n', Num_Generated);

%% ========================================================================
%  Step 3: 巡检目标与属性生成
%  对应文档 Section 3.1
% ========================================================================
fprintf('正在生成巡检目标属性...\n');

% 1. 确定巡检目标 (Inspection Targets)
% "其中 20% 的巡逻点视为无人机的检查目标"
Num_Targets = round(0.2 * Num_Generated);
Target_Indices = randperm(Num_Generated, Num_Targets);

% 创建一个标记数组，1表示是目标，0表示普通巡检点
Is_Target = zeros(Num_Generated, 1);
Is_Target(Target_Indices) = 1;

% 2. 生成悬停时间 (Hovering Time)
% "设为 0 ~ 30 秒，随机等概率生成"
Hover_Times = rand(Num_Generated, 1) * 30;

% 3. 生成重要系数 Mj
% "我们在 1 到 100 之间随机生成检验目标的重要系数"
% 文档虽只提及"检验目标"的系数，但数据结构通常对所有点统一，非目标点可设为0或忽略
Importance_Coeffs = zeros(Num_Generated, 1);
% 仅对目标点生成 1-100 的随机数 (假设为整数或浮点，这里用 randi 生成整数)
Importance_Coeffs(Target_Indices) = randi([1, 100], 1, Num_Targets);

%% ========================================================================
%  Step 4: 结果展示与验证
%  绘制类似文档 图2 的示意图
% ========================================================================
fprintf('正在绘制结果...\n');

figure('Name', 'DIPH Simulation Result', 'Color', 'w', 'Position', [100, 100, 1000, 600]);
hold on; grid on; box on;

% 1. 绘制原始 GIS 数据点 (采样部分点以避免绘图过慢，如绘制每50个点中的1个)
plot(Raw_Data(1:10:end, 2), Raw_Data(1:10:end, 1), '.', 'Color', [0.8, 0.8, 0.8], 'MarkerSize', 4, 'DisplayName', 'Raw GIS Data');

% 2. 绘制生成的巡检点
% 分开绘制普通点和目标点
Idx_Normal = find(Is_Target == 0);
Idx_Tgt    = find(Is_Target == 1);

h_normal = plot(Inspection_Points(Idx_Normal, 2), Inspection_Points(Idx_Normal, 1), 'bo', 'LineWidth', 1.5, 'DisplayName', 'Normal Inspection Points');
h_target = plot(Inspection_Points(Idx_Tgt, 2),    Inspection_Points(Idx_Tgt, 1),    'r*', 'LineWidth', 1.5, 'MarkerSize', 8, 'DisplayName', 'Inspection Targets');

% 3. 连线展示路径顺序 (可选)
plot(Inspection_Points(:, 2), Inspection_Points(:, 1), 'k--', 'Color', [0, 0, 0, 0.3], 'DisplayName', 'Path Sequence');

xlabel('Longitude');
ylabel('Latitude');
title(['DIPH Method Simulation Result (Total Points: ', num2str(Num_Generated), ')']);
legend([h_normal, h_target], 'Location', 'best');
axis equal;

fprintf('仿真实验完成。\n');
fprintf('------------------------------------------------\n');
fprintf('结果统计：\n');
fprintf('1. 公路尺寸模拟：5km x 30m\n');
fprintf('2. 生成巡检点总数：%d (目标范围 20-200)\n', Num_Generated);
fprintf('3. 巡检目标点数量 (20%%)：%d\n', Num_Targets);
fprintf('4. 悬停时间范围：%.2f - %.2f 秒\n', min(Hover_Times), max(Hover_Times));
fprintf('5. 目标重要系数范围：%d - %d\n', min(Importance_Coeffs(Importance_Coeffs>0)), max(Importance_Coeffs));
fprintf('------------------------------------------------\n');

%% 数据结构输出说明
% Inspection_Points: [N x 2] 矩阵，每行为 [Lat, Lon]
% Is_Target:         [N x 1] 向量，1为重点目标，0为普通点
% Hover_Times:       [N x 1] 向量，悬停时间
% Importance_Coeffs: [N x 1] 向量，重要系数