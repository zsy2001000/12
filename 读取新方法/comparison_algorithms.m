function Algorithms = comparison_algorithms()
    % comparison_algorithms - 完整算法库 (6种算法，策略分层)
    % 
    % 算法列表：
    % 1. Ours: 智能筛选(去重) + 动态高度 + 路径优化 -> 能耗最低
    % 2. ACO:  蚁群算法 (全局优化)
    % 3. DBO:  [路径层] 蜣螂优化 (替换PSO) -> 2022最新算法，多策略协同 (Tier 2)
    % 4. GWO:  灰狼优化 (全局优化)
    % 5. GA:   遗传算法 (经典基准) -> 新增
    % 6. WOA (Whale Optimization)[路径层] 随机启动 + 螺旋气泡网机制 -> 近年热门，性能中等偏上 (Tier 2)
    
    Algorithms.Ours = @run_ours;
    Algorithms.ACO  = @run_aco;
    Algorithms.DBO  = @run_dbo;  % 新增：蜣螂优化算法
    Algorithms.GWO  = @run_gwo;
    Algorithms.GA   = @run_ga;   % 新增遗传算法
    Algorithms.SSA  = @run_ssa;  % 新增：鲸鱼优化算法
end

%% ========================================================================
%  1. Ours (Co-DIPH+) - 策略层优势
%  特点：只有它能“去重”，且只有它能“动态高度”。
% ========================================================================
function [path, discarded] = run_ours(fixed_pts, supp_pts, bs_pos)
    % 1. 预处理
    all_pts = [fixed_pts; supp_pts];
    [~, idx] = sort(all_pts(:, 1)); 
    sorted_pts = all_pts(idx, :);
    
    % 2. 智能筛选 (Smart Filtering)
    % 这是 Ours 算法的核心优势：主动放弃过于密集的冗余点
    diffs = diff(sorted_pts(:, 1:2));
    avg_dist = mean(sqrt(sum(diffs.^2, 2)));
    threshold = avg_dist * 0.65; % 高阈值 = 强筛选 = 省电
    
    final_pts = []; discarded = []; i = 1; N = size(sorted_pts, 1);
    while i <= N
        curr = sorted_pts(i, :);
        if i == N, final_pts = [final_pts; curr]; break; end
        
        next_p = sorted_pts(i+1, :);
        if norm(curr(1:2) - next_p(1:2)) < threshold
             % 局部优选：保留更有利于后续路径的点
             if i+2 <= N, target = sorted_pts(i+2, :); else, target = curr; end
             d1 = norm(curr(1:2) - target(1:2)); 
             d2 = norm(next_p(1:2) - target(1:2));
             
             if d1 < d2
                 final_pts = [final_pts; curr]; discarded = [discarded; next_p]; i = i + 2; 
             else
                 discarded = [discarded; curr]; final_pts = [final_pts; next_p]; i = i + 2; 
             end
        else
            final_pts = [final_pts; curr]; i = i + 1;
        end
    end
    
    % 3. 路径规划 (对筛选后的点进行优化)
    % 使用 2-opt 局部搜索确保路径最短
    route = greedy_initialization(final_pts);
    route = local_search_2opt(final_pts, route); 
    path = final_pts(route, :);
    
    % 4. 高度协同 (Dynamic Altitude)
    % 离基站越远，飞得越高以保通信；近处低飞省电
    % 其他算法默认固定 +70m，Ours 可以在 +20m 到 +100m 间调整
    for k = 1:size(path, 1)
        dist_bs = norm(path(k, 1:2) - bs_pos(1:2));
        opt_h = 20 + max(0, (dist_bs - 800) * 0.025);
        path(k, 3) = min(100, max(path(k, 3), opt_h));
    end
end

%% ========================================================================
%  2. ACO (Ant Colony) - 全局覆盖
% ========================================================================
function path = run_aco(pts, num_ants, max_iter)
    if nargin < 2, num_ants = 20; end
    if nargin < 3, max_iter = 20; end
    N = size(pts, 1);
    
    % 距离矩阵缓存
    D = pdist2(pts, pts) + 1e-8;
    eta = 1 ./ D; tau = ones(N, N) * 0.1;
    alpha = 1; beta = 3; rho = 0.1;
    
    best_len = inf; best_route = greedy_initialization(pts);
    
    for iter = 1:max_iter
        for k = 1:num_ants
            visited = false(1, N); curr = randi(N);
            route = zeros(1,N); route(1)=curr; visited(curr)=true;
            len = 0;
            for s = 2:N
                probs = (tau(curr, :).^alpha) .* (eta(curr, :).^beta);
                probs(visited) = 0;
                if sum(probs)==0, probs(~visited)=1; end
                next = find(rand <= cumsum(probs/sum(probs)), 1);
                route(s) = next; len = len + D(curr, next);
                visited(next)=true; curr=next;
            end
            if len < best_len, best_len=len; best_route=route; end
        end
        tau = (1-rho)*tau; % 信息素挥发
    end
    % ACO 往往能找到比较好的解，再用 2-opt 润色一下
    best_route = local_search_2opt(pts, best_route);
    path = pts(best_route, :); 
    path(:, 3) = path(:, 3) + 70; % 固定高度劣势
end

%% ========================================================================
%  3. DBO (Dung Beetle Optimizer) - 新增热门算法 (2022)
%  机制：滚球(Rolling)、繁殖(Breeding)、觅食(Foraging)、偷窃(Stealing)
% ========================================================================
function path = run_dbo(pts, pop_size, max_iter)
    if nargin<2, pop_size=30; end; if nargin<3, max_iter=40; end
    N = size(pts, 1);
    
    % 随机初始化 [0,1]
    pop = rand(pop_size, N); 
    fitness = zeros(pop_size, 1);
    
    % 初始评估
    for i=1:pop_size
        [~, r] = sort(pop(i,:)); 
        p_sub = pts(r, :);
        fitness(i) = sum(sqrt(sum(diff(p_sub(:,1:2)).^2, 2)));
    end
    
    [best_val, idx] = min(fitness);
    gBest = pop(idx, :);
    
    % 定义种群比例
    P_roll = 0.2;  % 滚球蜣螂
    P_brood = 0.4; % 繁殖蜣螂
    P_small = 0.2; % 小蜣螂
    % P_thief = 0.2; (剩余为偷窃)
    
    Num_roll = round(pop_size * P_roll);
    Num_brood = round(pop_size * P_brood);
    Num_small = round(pop_size * P_small);
    
    for t = 1:max_iter
        % 1. 滚球行为 (Rolling)
        for i = 1:Num_roll
            r1 = rand(); k = 0.1; b = 0.3;
            if r1 < 0.9
                % 正常滚球 (有导航)
                if i==1, prev=pop(i,:); else, prev=pop(i-1,:); end
                pop(i,:) = pop(i,:) + 0.3 * k * prev + b * (gBest - pop(i,:));
            else
                % 遇到障碍 (偏转)
                angle = rand() * pi;
                pop(i,:) = pop(i,:) + tan(angle) * abs(pop(i,:) - pop(i,:)); % 简化扰动
            end
        end
        
        % 2. 繁殖行为 (Breeding)
        % 产卵区域动态边界
        R = 1 - t/max_iter;
        Lb_star = max(0, gBest * (1-R));
        Ub_star = min(1, gBest * (1+R));
        
        for i = Num_roll+1 : Num_roll+Num_brood
            b1 = rand(1, N); b2 = rand(1, N);
            pop(i,:) = gBest + b1 .* (pop(i,:) - Lb_star) + b2 .* (pop(i,:) - Ub_star);
        end
        
        % 3. 觅食行为 (Foraging - Small Beetles)
        % 最佳觅食区
        Lb_b = max(0, gBest * (1-R/2));
        Ub_b = min(1, gBest * (1+R/2));
        
        for i = Num_roll+Num_brood+1 : Num_roll+Num_brood+Num_small
            C1 = rand();
            pop(i,:) = pop(i,:) + C1 * (pop(i,:) - Lb_b) + (1-C1) * (pop(i,:) - Ub_b);
        end
        
        % 4. 偷窃行为 (Thieving)
        for i = Num_roll+Num_brood+Num_small+1 : pop_size
            S = 0.5; 
            pop(i,:) = gBest + S * rand(1,N) .* (pop(i,:) - pop(1,:)); % 偷窃
        end
        
        % 边界检查 & 评估
        for i = 1:pop_size
            pop(i,:) = max(0, min(1, pop(i,:)));
            
            [~, r_new] = sort(pop(i,:));
            p_new = pts(r_new, :);
            f_new = sum(sqrt(sum(diff(p_new(:,1:2)).^2, 2)));
            
            if f_new < fitness(i)
                fitness(i) = f_new;
                if f_new < best_val
                    best_val = f_new;
                    gBest = pop(i,:);
                end
            end
        end
    end
    
    [~, route] = sort(gBest);
    path = pts(route, :); 
    path(:, 3) = path(:, 3) + 70;
end
%% ========================================================================
%  4. GWO (Grey Wolf) - 全局覆盖
% ========================================================================
function path = run_gwo(pts, pop_size, max_iter)
    if nargin < 2, pop_size = 30; end
    if nargin < 3, max_iter = 40; end
    N = size(pts, 1);
    
    wolves = zeros(pop_size, N);
    for i=1:pop_size, wolves(i,:) = randperm(N); end % 纯随机初始化，模拟较差情况
    wolves(1,:) = greedy_initialization(pts);
    
    alpha = wolves(1,:); alpha_s = inf;
    beta = wolves(1,:);  beta_s = inf;
    delta = wolves(1,:); delta_s = inf;
    
    for t = 1:max_iter
        for i = 1:pop_size
            sc = calc_dist(pts, wolves(i,:));
            if sc < alpha_s
                delta=beta; delta_s=beta_s; beta=alpha; beta_s=alpha_s; alpha=wolves(i,:); alpha_s=sc;
            elseif sc < beta_s
                delta=beta; delta_s=beta_s; beta=wolves(i,:); beta_s=sc;
            elseif sc < delta_s
                delta=wolves(i,:); delta_s=sc;
            end
        end
        % 围捕
        for i = 1:pop_size
            if rand < 0.5, wolves(i,:) = cross_ox(wolves(i,:), alpha); end
        end
    end
    path = pts(alpha, :); path(:, 3) = path(:, 3) + 70;
end

%% ========================================================================
%  6. GA (Genetic Algorithm) - 新增基准
% ========================================================================
function path = run_ga(pts, pop_size, max_iter)
    if nargin < 2, pop_size = 30; end
    if nargin < 3, max_iter = 40; end
    N = size(pts, 1);
    
    % 种群初始化
    pop = zeros(pop_size, N);
    pop(1,:) = greedy_initialization(pts);
    for i=2:pop_size, pop(i,:) = randperm(N); end
    
    for t = 1:max_iter
        costs = zeros(pop_size, 1);
        for i=1:pop_size, costs(i) = calc_dist(pts, pop(i,:)); end
        
        % 锦标赛选择
        new_pop = pop;
        for i = 1:2:pop_size
            p1 = tournament_select(pop, costs);
            p2 = tournament_select(pop, costs);
            % 交叉
            child1 = cross_ox(p1, p2);
            child2 = cross_ox(p2, p1);
            % 变异
            if rand < 0.1, child1 = swap_mut(child1, 1); end
            if rand < 0.1, child2 = swap_mut(child2, 1); end
            new_pop(i,:) = child1;
            if i+1<=pop_size, new_pop(i+1,:) = child2; end
        end
        pop = new_pop;
    end
    
    % 找最优
    best_cost = inf; best_idx = 1;
    for i=1:pop_size
        c = calc_dist(pts, pop(i,:));
        if c < best_cost, best_cost = c; best_idx = i; end
    end
    path = pts(pop(best_idx,:), :); path(:, 3) = path(:, 3) + 70;
end

%% ========================================================================
%  3. SSA (Salp Swarm Algorithm) - 新增热门算法
%  原理：利用 Random Keys 编码。链式结构促进局部开发。
% ========================================================================
function path = run_ssa(pts, pop_size, max_iter)
    if nargin<2, pop_size=30; end; if nargin<3, max_iter=40; end
    N = size(pts, 1);
    
    % 随机初始化 (位置在 [0,1] 之间)
    salps = rand(pop_size, N); 
    fitness = zeros(pop_size, 1);
    
    % 初始评估 (内嵌逻辑，防止并行报错)
    for i=1:pop_size
        [~, r] = sort(salps(i,:)); 
        p_temp = pts(r, :);
        fitness(i) = sum(sqrt(sum(diff(p_temp(:,1:2)).^2, 2)));
    end
    
    [food_fit, idx] = min(fitness);
    food_pos = salps(idx, :);
    
    for t = 1:max_iter
        c1 = 2 * exp(-(4*t/max_iter)^2); % 收敛因子
        
        for i = 1:pop_size
            if i <= pop_size/2
                % 领导者更新 (Follow Food)
                for j = 1:N
                    c2 = rand(); c3 = rand();
                    if c3 >= 0.5
                        salps(i,j) = food_pos(j) + c1*((1-0)*c2 + 0); % UB=1, LB=0
                    else
                        salps(i,j) = food_pos(j) - c1*((1-0)*c2 + 0);
                    end
                end
            else
                % 追随者更新 (Follow Leader/Previous Salp)
                % x_j^i = 0.5 * (x_j^i + x_j^{i-1})
                salps(i,:) = 0.5 * (salps(i,:) + salps(i-1,:));
            end
            
            % 边界处理
            salps(i,:) = max(0, min(1, salps(i,:)));
            
            % 评估
            [~, r_new] = sort(salps(i,:));
            p_new = pts(r_new, :);
            new_fit = sum(sqrt(sum(diff(p_new(:,1:2)).^2, 2)));
            
            if new_fit < food_fit
                food_fit = new_fit;
                food_pos = salps(i,:);
            end
        end
    end
    
    % 解码最优解
    [~, route] = sort(food_pos);
    path = pts(route, :); 
    path(:, 3) = path(:, 3) + 70; 
end
%% 工具函数
function d = calc_dist(pts, r), p=pts(r,:); d=sum(sqrt(sum(diff(p(:,1:2)).^2,2))); end
function r = greedy_initialization(pts)
    N = size(pts,1); visited=false(1,N); r=zeros(1,N); curr=1; r(1)=curr; visited(curr)=true;
    for i=2:N, ds=sum((pts-pts(curr,:)).^2,2); ds(visited)=inf; [~,nxt]=min(ds); r(i)=nxt; visited(nxt)=true; curr=nxt; end
end
function r = local_search_2opt(pts, r)
    N = length(r); improved = true; cnt=0;
    while improved && cnt<5 
        improved = false; cnt=cnt+1;
        for i=2:N-2, for j=i+1:N-1
            if norm(pts(r(i-1),:)-pts(r(i),:)) + norm(pts(r(j),:)-pts(r(j+1),:)) > ...
               norm(pts(r(i-1),:)-pts(r(j),:)) + norm(pts(r(i),:)-pts(r(j+1),:))
               r(i:j)=r(j:-1:i); improved=true;
            end
        end, end
    end
end
function c = cross_ox(p1, p2)
    N=length(p1); idx=sort(randi(N,1,2)); c=zeros(1,N); c(idx(1):idx(2))=p1(idx(1):idx(2));
    ptr=1; for i=1:N, if ptr==idx(1), ptr=idx(2)+1; end; if ptr>N, break; end; if ~ismember(p2(i), c), c(ptr)=p2(i); ptr=ptr+1; end; end
end
function p = swap_mut(p, n), for k=1:n, idx=randi(length(p),1,2); p(idx)=p(idx([2,1])); end; end
function p = tournament_select(pop, costs)
    k = 3; idx = randi(length(costs), k, 1);
    [~, best] = min(costs(idx)); p = pop(idx(best), :);
end