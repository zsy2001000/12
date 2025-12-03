%% run_comprehensive_simulation.m - 6算法对比 (攀升能耗显著版)
clear; clc; close all;
fprintf('[Main] 初始化...\n');

Monte_Carlo_Reps = 5; 
Road_Length = 5000; Road_Width = 30; BS_Pos = [2500, -500, 100];
Algos = comparison_algorithms();
Algo_List = {'Ours', 'ACO', 'DBO', 'SSA', 'GA', 'GWO'};
Colors = [
    0.85 0.33 0.10; 0.93 0.69 0.13; 0.00 0.50 0.00;
    0.50 0.50 0.50; 0.49 0.18 0.56; 0.00 0.45 0.74
];

Res_Energy = zeros(Monte_Carlo_Reps, length(Algo_List));
Res_Time = zeros(Monte_Carlo_Reps, length(Algo_List));
Res_Details = zeros(length(Algo_List), 3); % [Hover, Flight, Climb]

for r = 1:Monte_Carlo_Reps
    rng(100+r);
    target_pts = 100;
    Num_Raw = 2000;
    
    % [核心修改 1] 大幅增加地形起伏 (0-1500m)
    % 强迫无人机频繁爬升，增加 E_climb 占比
    raw_z = rand(Num_Raw, 1) * 150; 
    Pool = [rand(Num_Raw,1)*Road_Length, (rand(Num_Raw,1)-0.5)*Road_Width, raw_z];
    
    Task = Pool(randperm(Num_Raw, target_pts), :);
    [~, si] = sort(Task(:,1)); Task = Task(si, :);
    n_fx = round(target_pts*0.3);
    P_fx = Task(1:n_fx, :); P_sp = Task(n_fx+1:end, :);
    
    % [核心修改 2] 调整参数以凸显爬升
    W_curr = 30;  % 重量 40N (重载) -> 爬升做功翻倍
    H_time = 10;  % 悬停 10s (减少悬停占比)
    
    for k = 1:length(Algo_List)
        name = Algo_List{k};
        tic;
        path = [];
        LA = Algos;
        switch name
            case 'Ours', [path,~] = LA.Ours(P_fx, P_sp, BS_Pos);
            case 'ACO',  path = LA.ACO(Task, 15, 15);
            case 'DBO',  path = LA.DBO(Task, 30, 40);
            case 'SSA',  path = LA.SSA(Task, 30, 40);
            case 'GA',   path = LA.GA(Task, 20, 30);
            case 'GWO',  path = LA.GWO(Task, 20, 30);
        end
        Res_Time(r,k) = toc;
        
        % 模拟起飞 (0m -> start_z)
        if ~isempty(path)
            start = path(1,:); start(3)=0;
            path_sim = [start; path];
        else
            path_sim = path;
        end
        
        [e_tot, eh, ef, ec] = calculate_uav_energy(path_sim, H_time, W_curr);
        Res_Energy(r, k) = e_tot;
        
        % 累加详情
        if r==1, Res_Details(k,:) = [eh, ef, ec];
        else, Res_Details(k,:) = (Res_Details(k,:)*(r-1) + [eh, ef, ec])/r; end
    end
end

Mean_E = mean(Res_Energy, 1); Std_E = std(Res_Energy, 0, 1); Mean_T = mean(Res_Time, 1);

% 绘图1: 总能耗
figure('Name', 'Performance', 'Color', 'w');
yyaxis left; b = bar(Mean_E, 'FaceColor', 'flat'); hold on;
errorbar(1:6, Mean_E, Std_E, 'k', 'LineStyle', 'none');
ylabel('Total Energy (J)');
for k=1:6, b.CData(k,:) = Colors(k,:); end
yyaxis right; plot(1:6, Mean_T, '-o', 'LineWidth', 2, 'Color', 'k');
ylabel('Time (s)'); xticklabels(Algo_List);
set(gca, 'FontName', 'Times New Roman', 'FontSize', 12); grid on;

% 绘图2: 能耗拆解 (关键图)
figure('Name', 'Breakdown', 'Color', 'w');
b_st = bar(Res_Details, 'stacked');
xticklabels(Algo_List); 
ylabel('Energy Consumption (J)');
set(gca, 'FontName', 'Times New Roman', 'FontSize', 12); grid on;

% 设置图例和颜色
lg = legend('Hovering', 'Level Flight', 'Climbing', 'Location', 'northeast');
b_st(1).FaceColor = [0.2 0.6 0.8]; % Hover (蓝)
b_st(2).FaceColor = [0.9 0.7 0.1]; % Flight (黄)
b_st(3).FaceColor = [0.8 0.2 0.2]; % Climb (红 - 显眼)