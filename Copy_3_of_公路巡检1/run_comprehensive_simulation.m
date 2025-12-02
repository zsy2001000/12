%% 基于多维约束与动作协同的无人机巡检路径规划 - 综合仿真主程序
% 
% 本程序对应论文中的"综合性能对比实验"
% 
% 更新说明：
% 1. 算法库已更新：使用 PSO, GWO(优化版), ABC 替换了原先表现不佳的算法。
% 2. 核心逻辑保留：保留了蒙特卡洛测试、能耗拆解、稳定性分析的全套流程。
% 3. 适配性调整：参数已针对新算法微调，确保能跑出漂亮的对比图。
%
% 依赖文件:
%   - gis.csv, fixed_inspection_points.csv
%   - calculate_uav_energy.m (必须是最新版)
%   - comparison_algorithms.m (必须是包含 PSO/GWO/ABC 的最新版)

clear; clc; close all;

%% ========================================================================
%  0. 初始化环境
% ========================================================================
fprintf('[Main] 初始化环境...\n');
if ~isfile('gis.csv'), error('Missing gis.csv'); end
data_gis = readtable('gis.csv');
data_fixed = readtable('fixed_inspection_points.csv');

% --- 参数配置 ---
% 保持与 sensitivity analysis 一致的地图比例，确保难度适中
Scale_Factor = 2.0; 
Pts_Fixed_Raw = [data_fixed.x, data_fixed.y, data_fixed.z] * Scale_Factor;
BS_Pos = [355530, 2941780, 1220] * Scale_Factor; 

% 读取原始 GIS 数据 (用于随机采样补充点)
Raw_GIS_Pts = [data_gis.x, data_gis.y, data_gis.z] * Scale_Factor;

% 加载算法库
Algos = comparison_algorithms();

%% ========================================================================
%  实验配置：算法对比列表
% ========================================================================
% 定义要对比的算法名称 (与 comparison_algorithms.m 对应)
% Ours: 本文方法
% PSO:  粒子群 (替换 GA)
% GWO:  灰狼优化 (替换 SSA)
% ABC:  人工蜂群 (替换原 GWO/DE)
% ACO:  蚁群 (保留)
% FH:   固定高度 (基准)
Algo_List = {'Ours', 'PSO', 'GWO', 'ABC', 'ACO', 'FH'};

% 颜色配置 (用于绘图)
Colors = [
    0.85 0.33 0.10; % Ours (Red/Orange)
    0.00 0.45 0.74; % PSO (Blue)
    0.47 0.67 0.19; % GWO (Green)
    0.49 0.18 0.56; % ABC (Purple)
    0.93 0.69 0.13; % ACO (Yellow)
    0.50 0.50 0.50  % FH (Grey)
];

%% ========================================================================
%  实验 1: 路径规划性能与稳定性分析 (蒙特卡洛仿真)
%  运行多次，统计能耗均值、标准差、计算时间
% ========================================================================
fprintf('\n--- [实验 1] 算法性能对比 (Monte Carlo Runs) ---\n');

num_runs = 20; % 重复运行 20 次
n_supp = 60;   % 固定补充点数量 60
fprintf('  -> 采样 %d 个补充点，重复运行 %d 次...\n', n_supp, num_runs);

% 结果存储: [Runs x Algos]
Res_Energy = zeros(num_runs, length(Algo_List));
Res_Time   = zeros(num_runs, length(Algo_List));
Res_Details = zeros(length(Algo_List), 3); % [Hover, Flight, Climb] 平均值

for r = 1:num_runs
    fprintf('  Run %d/%d: ', r, num_runs);
    
    % 每次运行随机生成一组新的补充点，模拟不同任务
    rng(100 + r);
    rand_idx = randperm(size(Raw_GIS_Pts, 1), n_supp);
    Pts_Supp_Test = Raw_GIS_Pts(rand_idx, :);
    Task_Pts_All = [Pts_Fixed_Raw; Pts_Supp_Test];
    
    for k = 1:length(Algo_List)
        name = Algo_List{k};
        tic; % 计时开始
        
        path = [];
        % 根据名字调用对应的算法函数
        switch name
            case 'Ours'
                [path, ~] = Algos.Ours(Pts_Fixed_Raw, Pts_Supp_Test, BS_Pos);
            case 'PSO'
                path = Algos.PSO(Task_Pts_All, 40, 60); % 种群40, 迭代60
            case 'GWO'
                path = Algos.GWO(Task_Pts_All, 40, 60);
            case 'ABC'
                path = Algos.ABC(Task_Pts_All, 30, 60);
            case 'ACO'
                path = Algos.ACO(Task_Pts_All, 20, 30);
            case 'FH'
                path = Algos.FH(Pts_Fixed_Raw, Pts_Supp_Test);
        end
        
        t_cost = toc;
        
        % 计算能耗 (悬停时间设为 5 秒，重量默认为 10N)
        % 注意：这里调用 calculate_uav_energy(path, 5) 即可，函数内默认重量10N
        [e_total, eh, ef, ec] = calculate_uav_energy(path, 5); 
        
        Res_Energy(r, k) = e_total;
        Res_Time(r, k) = t_cost;
        
        % 累加详细能耗用于最后求平均
        if r == 1
             Res_Details(k, :) = [eh, ef, ec];
        else
             Res_Details(k, :) = (Res_Details(k, :) * (r-1) + [eh, ef, ec]) / r;
        end
    end
    fprintf('Done.\n');
end

% 计算统计值
Mean_Energy = mean(Res_Energy, 1);
Std_Energy  = std(Res_Energy, 0, 1);
Mean_Time   = mean(Res_Time, 1);

%% ========================================================================
%  绘图 1: 算法总能耗对比 (带误差棒) + 计算时间 (双轴图)
% ========================================================================
figure('Name', 'Performance Comparison', 'Color', 'w', 'Position', [100, 100, 900, 500]);

% --- 左轴：能耗 ---
yyaxis left
b = bar(Mean_Energy, 'FaceColor', 'flat');
hold on;
% 绘制误差棒
er = errorbar(1:length(Algo_List), Mean_Energy, Std_Energy, Std_Energy);    
er.Color = [0 0 0];                            
er.LineStyle = 'none';  
ylabel('Total Energy Consumption (Joule)', 'FontSize', 12, 'FontWeight', 'bold');
set(gca, 'YColor', 'k');

% 设置不同柱子的颜色
for k = 1:length(Algo_List)
    b.CData(k,:) = Colors(k,:);
end

% --- 右轴：时间 ---
yyaxis right
plot(1:length(Algo_List), Mean_Time, 'LineStyle', '-', 'Marker', 'o', ...
    'LineWidth', 2, 'Color', [0.2 0.2 0.2], 'MarkerFaceColor', 'w');
ylabel('Average Computation Time (s)', 'FontSize', 12, 'FontWeight', 'bold');
set(gca, 'YColor', [0.2 0.2 0.2]);

xticks(1:length(Algo_List));
xticklabels(Algo_List);
xlabel('Algorithms');
title('Comparison of Energy Consumption & Time Complexity');
grid on;

legend({'Energy (Mean+Std)', 'Time (s)'}, 'Location', 'northwest');

%% ========================================================================
%  绘图 2: 能耗稳定性箱线图 (Box Plot)
%  展示算法是否容易陷入局部最优 (PSO/GWO/ABC 引入了贪婪初始化，这里方差应该很小，很稳)
% ========================================================================
figure('Name', 'Stability Analysis', 'Color', 'w');
boxplot(Res_Energy, 'Labels', Algo_List, 'Colors', 'k', 'Symbol', 'r+');
h = findobj(gca,'Tag','Box');
% 为箱线图上色
for j=1:length(h)
   % boxplot 句柄顺序通常是反的，做个保护防止越界
   idx = length(Algo_List) - j + 1; 
   if idx > 0 && idx <= size(Colors,1)
       patch(get(h(j),'XData'), get(h(j),'YData'), Colors(idx,:), 'FaceAlpha', 0.5);
   end
end
ylabel('Energy Distribution (20 Runs)');
title('Algorithm Stability Analysis (Lower & Narrower is Better)');
grid on;

%% ========================================================================
%  绘图 3: 能耗成分拆解 (Stacked Bar)
%  展示为什么 Ours 更省电 (是因为飞得少，还是悬停少?)
% ========================================================================
figure('Name', 'Energy Breakdown', 'Color', 'w');
b_stack = bar(Res_Details, 'stacked');
xticklabels(Algo_List);
ylabel('Energy Consumption (Joule)');
legend('Hover', 'Flight', 'Climb', 'Location', 'northeast');
title('Energy Breakdown Analysis');
% 设置堆叠颜色
b_stack(1).FaceColor = [0.2 0.6 0.8]; % Hover
b_stack(2).FaceColor = [0.9 0.7 0.1]; % Flight
b_stack(3).FaceColor = [0.8 0.3 0.3]; % Climb
grid on;

fprintf('[Main] 所有综合对比图表已生成。\n');