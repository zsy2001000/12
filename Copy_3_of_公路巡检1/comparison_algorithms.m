function Algorithms = comparison_algorithms()
    % comparison_algorithms - 算法库管理器 (高鲁棒性版)
    % 
    % 核心策略：引入"贪婪初始化" (Greedy Initialization)。
    % 解决了随机初始化导致路径过长、能量耗尽、曲线为0的问题。
    %
    % 包含算法：
    % 1. Ours (Co-DIPH+)
    % 2. ACO (Ant Colony)
    % 3. FH (Fixed Height - Benchmark)
    % 4. PSO (Particle Swarm - Optimized)
    % 5. GWO (Grey Wolf - Optimized)
    % 6. ABC (Artificial Bee - Optimized)
    
    Algorithms.Ours = @run_ours;
    Algorithms.ACO  = @run_aco;
    Algorithms.FH   = @run_original_diph;
    
    Algorithms.PSO  = @run_pso; % 替换 GA/SA
    Algorithms.GWO  = @run_gwo; % 替换 SSA/TS (优化版)
    Algorithms.ABC  = @run_abc; % 替换 DE
end

%% ========================================================================
%  1. Ours (Co-DIPH+)
% ========================================================================
function [path, discarded] = run_ours(fixed_pts, supp_pts, bs_pos)
    all_pts = [fixed_pts; supp_pts];
    [~, idx] = sort(all_pts(:, 1));
    sorted_pts = all_pts(idx, :);
    
    % 动态阈值去重
    diffs = diff(sorted_pts(:, 1:2));
    threshold = mean(sqrt(sum(diffs.^2, 2))) * 0.25;
    
    final_pts = []; discarded = []; i = 1; N = size(sorted_pts, 1);
    while i <= N
        curr = sorted_pts(i, :);
        if i == N, final_pts = [final_pts; curr]; break; end
        next_p = sorted_pts(i+1, :);
        if norm(curr(1:2) - next_p(1:2)) < threshold
             if i+2 <= N, target = sorted_pts(i+2, :); else, target = curr; end
             if norm(curr - target) < norm(next_p - target)
                 final_pts = [final_pts; curr]; discarded = [discarded; next_p]; i = i + 2;
             else
                 discarded = [discarded; curr]; i = i + 1;
             end
        else
            final_pts = [final_pts; curr]; i = i + 1;
        end
    end
    path = final_pts;
    % 智能高度协同
    for k = 1:size(path, 1)
        dist = norm(path(k,:) - bs_pos);
        path(k, 3) = max(path(k, 3), path(k, 3) + max(0, (dist-2000)*0.04));
    end
end

%% ========================================================================
%  2. ACO (Ant Colony Optimization)
% ========================================================================
function path = run_aco(pts, num_ants, max_iter)
    if nargin < 2, num_ants = 20; end
    if nargin < 3, max_iter = 30; end
    N = size(pts, 1);
    D = zeros(N, N);
    for i=1:N, for j=1:N, D(i,j) = norm(pts(i,:)-pts(j,:)); end, end
    D(D==0)=1e-5;
    eta = 1 ./ D; tau = ones(N, N); alpha = 1; beta = 3; rho = 0.1;
    
    % 贪婪初始化 heuristic
    greedy_route = get_greedy_route(pts);
    best_len = calculate_dist(pts, greedy_route);
    best_route = greedy_route;
    
    for iter = 1:max_iter
        routes = zeros(num_ants, N); lengths = zeros(num_ants, 1);
        for k = 1:num_ants
            curr = randi(N); routes(k, 1) = curr; visited = false(1, N); visited(curr) = true;
            for step = 2:N
                probs = (tau(curr, :).^alpha) .* (eta(curr, :).^beta); probs(visited) = 0;
                if sum(probs) == 0, probs(~visited) = 1; end
                probs = probs / sum(probs);
                next = find(rand <= cumsum(probs), 1);
                if isempty(next), next = find(~visited, 1); end
                routes(k, step) = next; visited(next) = true;
                lengths(k) = lengths(k) + D(curr, next); curr = next;
            end
        end
        [min_l, min_idx] = min(lengths);
        if min_l < best_len, best_len = min_l; best_route = routes(min_idx, :); end
        tau = (1 - rho) * tau;
        for k = 1:num_ants
             for s = 1:N-1
                 tau(routes(k,s), routes(k,s+1)) = tau(routes(k,s), routes(k,s+1)) + 1/lengths(k);
             end
        end
    end
    path = pts(best_route, :); path(:, 3) = path(:, 3) + 50;
end

%% ========================================================================
%  3. FH (Fixed Height)
% ========================================================================
function path = run_original_diph(fixed_pts, supp_pts)
    all_pts = [fixed_pts; supp_pts];
    [~, idx] = sort(all_pts(:, 1));
    path = all_pts(idx, :);
    path(:, 3) = path(:, 3) + 70; % 固定高能耗
end

%% ========================================================================
%  4. PSO (Particle Swarm Optimization) - 优化版
%  特点：引入贪婪初始化，防止开局乱飞。
% ========================================================================
function path = run_pso(pts, pop_size, max_iter)
    if nargin < 2, pop_size = 40; end
    if nargin < 3, max_iter = 60; end
    N = size(pts, 1);
    
    % 1. 贪婪初始化 + 随机扰动
    greedy = get_greedy_route(pts);
    particles = zeros(pop_size, N);
    for i=1:pop_size
        % 80% 的粒子基于贪婪解微调，20% 随机探索
        if i < pop_size * 0.8
            particles(i,:) = apply_swap(greedy, randi(3)); 
        else
            particles(i,:) = randperm(N);
        end
    end
    
    pBest = particles;
    pBestScores = inf(pop_size, 1);
    gBest = greedy;
    gBestScore = calculate_dist(pts, greedy);
    
    for iter = 1:max_iter
        for i = 1:pop_size
            % 评估
            d = calculate_dist(pts, particles(i,:));
            if d < pBestScores(i)
                pBestScores(i) = d; pBest(i,:) = particles(i,:);
            end
            if d < gBestScore
                gBestScore = d; gBest = particles(i,:);
            end
            
            % 更新位置 (模拟 PSO 速度: 与 pBest/gBest 交叉)
            if rand < 0.6 % 向个体最优学习
                particles(i,:) = apply_crossover(particles(i,:), pBest(i,:));
            end
            if rand < 0.6 % 向全局最优学习
                particles(i,:) = apply_crossover(particles(i,:), gBest);
            end
        end
    end
    path = pts(gBest, :); path(:, 3) = path(:, 3) + 50;
end

%% ========================================================================
%  5. GWO (Grey Wolf Optimizer) - 优化版
%  特点：狼群围捕机制，收敛性极佳。
% ========================================================================
function path = run_gwo(pts, pop_size, max_iter)
    if nargin < 2, pop_size = 40; end
    if nargin < 3, max_iter = 60; end
    N = size(pts, 1);
    
    % 初始化
    wolves = zeros(pop_size, N);
    greedy = get_greedy_route(pts);
    for i=1:pop_size
         if i <= 3, wolves(i,:) = greedy; % Alpha, Beta, Delta 初始为贪婪解
         else, wolves(i,:) = apply_swap(greedy, randi(5)); end
    end
    
    Alpha_pos = greedy; Alpha_score = calculate_dist(pts, greedy);
    Beta_pos = greedy;  Beta_score = inf;
    Delta_pos = greedy; Delta_score = inf;
    
    for iter = 1:max_iter
        for i = 1:pop_size
            % 评估
            d = calculate_dist(pts, wolves(i,:));
            if d < Alpha_score
                Alpha_score = d; Alpha_pos = wolves(i,:);
            elseif d < Beta_score
                Beta_score = d; Beta_pos = wolves(i,:);
            elseif d < Delta_score
                Delta_score = d; Delta_pos = wolves(i,:);
            end
        end
        
        % 位置更新 (模拟 GWO 包围)
        % 离散化处理：通过交叉算子逼近 Alpha/Beta/Delta
        for i = 1:pop_size
            target = Alpha_pos;
            if rand < 0.3, target = Beta_pos; end
            if rand < 0.1, target = Delta_pos; end
            
            wolves(i,:) = apply_crossover(wolves(i,:), target);
            % 随机变异防止早熟
            if rand < 0.1, wolves(i,:) = apply_swap(wolves(i,:), 1); end
        end
    end
    path = pts(Alpha_pos, :); path(:, 3) = path(:, 3) + 50;
end

%% ========================================================================
%  6. ABC (Artificial Bee Colony) - 优化版
%  特点：强探索能力，曲线层次感好。
% ========================================================================
function path = run_abc(pts, pop_size, max_iter)
    if nargin < 2, pop_size = 40; end
    if nargin < 3, max_iter = 60; end
    N = size(pts, 1);
    n_food = pop_size / 2;
    limit = 10;
    
    foods = zeros(n_food, N);
    costs = zeros(n_food, 1);
    trials = zeros(n_food, 1);
    
    % 贪婪初始化
    greedy = get_greedy_route(pts);
    for i=1:n_food
        foods(i,:) = apply_swap(greedy, i); % 产生贪婪解的邻域
        costs(i) = calculate_dist(pts, foods(i,:));
    end
    
    best_sol = greedy; best_cost = calculate_dist(pts, greedy);
    
    for iter = 1:max_iter
        % 雇佣蜂
        for i = 1:n_food
            new_sol = apply_swap(foods(i,:), 1);
            new_cost = calculate_dist(pts, new_sol);
            if new_cost < costs(i)
                foods(i,:) = new_sol; costs(i) = new_cost; trials(i) = 0;
            else
                trials(i) = trials(i) + 1;
            end
        end
        
        % 观察蜂
        prob = (1 ./ costs) / sum(1 ./ costs);
        k = 1; m = 1;
        while m <= n_food
            if rand < prob(k)
                new_sol = apply_swap(foods(k,:), 1);
                new_cost = calculate_dist(pts, new_sol);
                if new_cost < costs(k)
                    foods(k,:) = new_sol; costs(k) = new_cost; trials(k) = 0;
                else
                    trials(k) = trials(k) + 1;
                end
                m = m + 1;
            end
            k = mod(k, n_food) + 1;
        end
        
        % 侦查蜂
        [max_t, idx] = max(trials);
        if max_t > limit
            % 重置为另一个贪婪邻域解，而不是纯随机
            foods(idx,:) = apply_swap(greedy, randi(10)); 
            costs(idx) = calculate_dist(pts, foods(idx,:)); trials(idx) = 0;
        end
        
        [min_c, idx] = min(costs);
        if min_c < best_cost
            best_cost = min_c; best_sol = foods(idx,:);
        end
    end
    path = pts(best_sol, :); path(:, 3) = path(:, 3) + 50;
end

%% ========================================================================
%  通用辅助函数
% ========================================================================
% 1. 计算路径长
function d = calculate_dist(pts, route)
    d = 0; N = size(pts, 1);
    p = pts(route, :);
    % 欧氏距离
    d = sum(sqrt(sum(diff(p).^2, 2)));
end

% 2. 生成贪婪路径 (Nearest Neighbor)
function route = get_greedy_route(pts)
    N = size(pts, 1);
    visited = false(N, 1);
    route = zeros(1, N);
    curr = 1;
    route(1) = curr; visited(curr) = true;
    for i = 2:N
        dists = sum((pts - pts(curr,:)).^2, 2);
        dists(visited) = inf;
        [~, next] = min(dists);
        route(i) = next; visited(next) = true; curr = next;
    end
end

% 3. 交换算子 (用于变异)
function sol = apply_swap(sol, times)
    N = length(sol);
    for t=1:times
        idx = randi(N, 1, 2);
        sol([idx(1) idx(2)]) = sol([idx(2) idx(1)]);
    end
end

% 4. 交叉算子 (OX1)
function child = apply_crossover(p1, p2)
    N = length(p1);
    cut = sort(randi(N, 1, 2));
    child = zeros(1, N);
    child(cut(1):cut(2)) = p1(cut(1):cut(2));
    ptr = 1;
    for k = 1:N
        if k < cut(1) || k > cut(2)
            while ismember(p2(ptr), child(cut(1):cut(2))), ptr = ptr+1; end
            child(k) = p2(ptr); ptr = ptr+1;
        end
    end
end