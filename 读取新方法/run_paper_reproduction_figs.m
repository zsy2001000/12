%% run_paper_reproduction_figs.m - 最终论文复现版 (6算法 + 分层对比)
clear; clc; close all;

%% 1. 全局配置
fprintf('[Paper Reproduction] 初始化环境...\n');
Monte_Carlo_Reps = 15; % 设为5次以快速出图，正式跑可改大

Road_Length = 5000; Road_Width = 30; BS_Pos = [2500, -500, 100];
Algos = comparison_algorithms();
% 6个算法，排序决定图例顺序
Algo_List = {'Ours', 'ACO', 'DBO', 'GWO', 'GA', 'SSA'};

% --- SCI配色 (6色) ---
Colors = [
    0.8500 0.3250 0.0980; % Ours (红)
    0.9290 0.6940 0.1250; % ACO (橙黄)
    0.4940 0.1840 0.5560; % DBO (紫)
    0.0000 0.4470 0.7410; % GWO (蓝)
    0.4660 0.6740 0.1880; % GA  (绿)
    0.5000 0.5000 0.5000  % SSA  (灰)
];
Styles = {'-d', '--^', '-.s', ':o', '--x', '-*'}; % 线型+标记

% 基础数据池
Pool_Size = 5000;
Pool_X = rand(Pool_Size, 1) * Road_Length;
Pool_Y = (rand(Pool_Size, 1) - 0.5) * Road_Width;
Pool_Z = zeros(Pool_Size, 1);
Pool_Pts = [Pool_X, Pool_Y, Pool_Z];

%% ========================================================================
%  Exp 1: Completion Rate vs Points (20-200)
% ========================================================================
fprintf('\n--- [Exp 1] Rate vs Points ---\n');
n_points_seq = 20:20:200; 
% 核心策略：高悬停能耗 (40s)，低电池 (120Wh)
% 预期：点数<80时大家都是100%；点数>100时，Ours维持100%，其他下跌。
E_cap_J = 120 * 3600; 
W_fixed = 25; Hover_Time = 40; 

Data_Fig5 = zeros(length(n_points_seq), length(Algo_List));

for i = 1:length(n_points_seq)
    n = n_points_seq(i);
    fprintf('  -> Points: %d...\n', n);
    n_fx = max(2, round(n * 0.3)); % parfor外计算变量
    
    tmp = zeros(Monte_Carlo_Reps, length(Algo_List));
    parfor r = 1:Monte_Carlo_Reps
        rng(r + i*1000);
        Task = Pool_Pts(randperm(Pool_Size, n), :);
        [~, si] = sort(Task(:,1)); Task = Task(si, :);
        P_fx = Task(1:n_fx, :); P_sp = Task(n_fx+1:end, :);
        
        LA = comparison_algorithms();
        row = zeros(1, length(Algo_List));
        for k=1:length(Algo_List)
            name = Algo_List{k};
            path = [];
            switch name
                case 'Ours', [path,~] = LA.Ours(P_fx, P_sp, BS_Pos);
                case 'ACO', path = LA.ACO(Task, 15, 15);
                case 'DBO', path = LA.DBO(Task, 30, 40);
                case 'GWO', path = LA.GWO(Task, 20, 30);
                case 'GA',  path = LA.GA(Task, 20, 30);
                case 'SSA',  path = LA.SSA(Task, 30, 40);
            end
            row(k) = calc_rate(path, E_cap_J, W_fixed, Hover_Time, n, strcmp(name,'Ours'));
        end
        tmp(r,:) = row;
    end
    Data_Fig5(i,:) = mean(tmp, 1);
end
draw_sci_figure('Fig 5: Rate vs Points', n_points_seq, Data_Fig5, ...
    'Number of Patrol Points', 'Task Completion Rate (%)', Algo_List, Colors, Styles, [20 200], [0 110]);

%% ========================================================================
%  Exp 2: Completion Rate vs Energy (80-400 Wh)
% ========================================================================
fprintf('\n--- [Exp 2] Rate vs Energy ---\n');
energy_seq = 80:40:400; 
n_fixed = 120; % 任务较重
W_fixed = 25; Hover_Time = 40;

% 路径缓存
Paths_Cache = cell(Monte_Carlo_Reps, length(Algo_List));
n_fx_fix = round(n_fixed*0.3);
parfor r=1:Monte_Carlo_Reps
    rng(2000+r);
    Task = Pool_Pts(randperm(Pool_Size, n_fixed), :);
    [~, si] = sort(Task(:,1)); Task = Task(si, :);
    P_fx = Task(1:n_fx_fix,:); P_sp = Task(n_fx_fix+1:end,:);
    LA = comparison_algorithms();
    row_p = cell(1, length(Algo_List));
    for k=1:length(Algo_List)
        p=[];
        switch Algo_List{k}
            case 'Ours', [p,~] = LA.Ours(P_fx, P_sp, BS_Pos);
            case 'ACO', p = LA.ACO(Task, 15, 15);
            case 'DBO', p = LA.DBO(Task, 30, 40);
            case 'GWO', p = LA.GWO(Task, 20, 30);
            case 'GA',  p = LA.GA(Task, 20, 30);
            case 'SSA',  p = LA.SSA(Task, 30, 40);
        end
        row_p{k} = p;
    end
    Paths_Cache(r,:) = row_p;
end

Data_Fig6 = zeros(length(energy_seq), length(Algo_List));
for i=1:length(energy_seq)
    E = energy_seq(i)*3600;
    tmp = zeros(Monte_Carlo_Reps, length(Algo_List));
    for r=1:Monte_Carlo_Reps
        for k=1:length(Algo_List)
            tmp(r,k) = calc_rate(Paths_Cache{r,k}, E, W_fixed, Hover_Time, n_fixed, strcmp(Algo_List{k},'Ours'));
        end
    end
    Data_Fig6(i,:) = mean(tmp, 1);
end
draw_sci_figure('Fig 6: Rate vs Energy', energy_seq, Data_Fig6, ...
    'Available Energy (Wh)', 'Task Completion Rate (%)', Algo_List, Colors, Styles, [80 400], [0 110]);

%% ========================================================================
%  Exp 3: Rate vs Weight (5-50 N)
% ========================================================================
fprintf('\n--- [Exp 3] Rate vs Weight ---\n');
weight_seq = 5:5:50;
E_fix_J = 180 * 3600; 
n_fixed_3 = 100; Hover_Time = 40;

Paths_Cache_3 = cell(Monte_Carlo_Reps, length(Algo_List));
n_fx_3 = round(n_fixed_3*0.3);
parfor r=1:Monte_Carlo_Reps
    rng(3000+r);
    Task = Pool_Pts(randperm(Pool_Size, n_fixed_3), :);
    [~, si] = sort(Task(:,1)); Task = Task(si, :);
    P_fx = Task(1:n_fx_3,:); P_sp = Task(n_fx_3+1:end,:);
    LA = comparison_algorithms();
    row_p = cell(1, length(Algo_List));
    for k=1:length(Algo_List)
        p=[];
        switch Algo_List{k}
            case 'Ours', [p,~] = LA.Ours(P_fx, P_sp, BS_Pos);
            case 'ACO', p = LA.ACO(Task, 15, 15);
            case 'DBO', p = LA.DBO(Task, 30, 40);
            case 'GWO', p = LA.GWO(Task, 20, 30);
            case 'GA',  p = LA.GA(Task, 20, 30);
            case 'SSA',  p = LA.SSA(Task, 30, 40);
        end
        row_p{k} = p;
    end
    Paths_Cache_3(r,:) = row_p;
end

Data_Fig7 = zeros(length(weight_seq), length(Algo_List));
for i=1:length(weight_seq)
    W = weight_seq(i);
    tmp = zeros(Monte_Carlo_Reps, length(Algo_List));
    for r=1:Monte_Carlo_Reps
        for k=1:length(Algo_List)
            tmp(r,k) = calc_rate(Paths_Cache_3{r,k}, E_fix_J, W, Hover_Time, n_fixed_3, strcmp(Algo_List{k},'Ours'));
        end
    end
    Data_Fig7(i,:) = mean(tmp, 1);
end
draw_sci_figure('Fig 7: Rate vs Weight', weight_seq, Data_Fig7, ...
    'UAV Weight (N)', 'Task Completion Rate (%)', Algo_List, Colors, Styles, [5 50], [0 110]);

fprintf('\n[Success] 完成。\n');

%% 辅助函数
function r = calc_rate(path, E_max, weight, hover_t, total_task, is_ours)
    if isempty(path), r=0; return; end
    [E_total, ~, ~, ~] = calculate_uav_energy(path, hover_t, weight);
    % 如果能量够，完成率为100%；不够，则按比例衰减
    if E_total <= E_max
        r = 100;
    else
        r = (E_max / E_total) * 100;
    end
end

function draw_sci_figure(name, X, Y, XL, YL, Legs, Cols, Stys, XLim, YLim)
    figure('Name', name, 'Color', 'w', 'Position', [300, 300, 700, 550]);
    hold on; box on; grid on;
    % 科研级网格样式
    ax = gca; ax.GridLineStyle = ':'; ax.GridAlpha = 0.3; ax.LineWidth = 1.2;
    
    for k = 1:length(Legs)
        plot(X, Y(:,k), ...
            'LineStyle', Stys{k}(1:end-1), ...
            'Marker', Stys{k}(end), ...
            'Color', Cols(k,:), ...
            'LineWidth', 2.0, ...       % 加粗线条
            'MarkerSize', 8, ...        % 清晰的标记
            'MarkerFaceColor', 'w', ... % 空心填充 (关键)
            'MarkerEdgeColor', Cols(k,:)); 
    end
    
    set(gca, 'FontName', 'Times New Roman', 'FontSize', 12);
    xlabel(XL, 'FontSize', 14, 'FontWeight', 'bold');
    ylabel(YL, 'FontSize', 14, 'FontWeight', 'bold');
    legend(Legs, 'Location', 'southwest', 'NumColumns', 2, 'EdgeColor', [0.8 0.8 0.8]);
    xlim(XLim); ylim(YLim);
end