function [E_total, E_hover, E_flight, E_climb, T_total] = calculate_uav_energy(path_points, hover_time_per_point, weight_input)
%% calculate_uav_energy - 无人机能耗高精度计算物理引擎
%
% 功能：
%   基于旋翼空气动力学模型，计算无人机沿指定三维路径飞行的总能耗。
%   严格复现原论文公式，并考虑爬升、下降、前飞和悬停的不同功率状态。
%
% 输入：
%   path_points: N x 3 矩阵，[x, y, z] 坐标序列 (米)
%   hover_time_per_point: 单个巡检点的悬停时间 (秒)
%   weight_input (可选): 无人机重量 (Newton)。如果不填，默认为 10N。
%
% 输出：
%   E_total: 总能耗 (Joule)
%   E_hover: 悬停总能耗 (Joule)
%   E_flight: 水平/前飞总能耗 (Joule)
%   E_climb: 爬升/下降带来的额外重力势能能耗 (Joule)
%   T_total: 总任务时间 (秒)
%
% 参考文献：
%   [1] 原论文 4.1 节仿真参数
%   [2] Leishman, J. G. "Principles of Helicopter Aerodynamics"

    %% 1. 物理参数设置 (依据原论文 4.1 节)
    % -----------------------------------------------------------
    % 修改：优先使用外部输入的重量，实现对图7(重量变化)的支持
    if nargin < 3 || isempty(weight_input)
        W = 10;             % 默认无人机重量 (Newton), 约 1kg
    else
        W = weight_input;   % 使用外部传入的重量
    end

    rho = 1.225;            % 空气密度 (kg/m^3)
    A = 0.79;               % 旋翼盘面积 (m^2)
    
    sigma = 0.05;           % 旋翼实度 (Solidity)
    Cd0 = 0.012;            % 型线阻力系数 (Profile Drag Coefficient)
    d0 = 0.6;               % 机身阻力比 (Fuselage Drag Ratio)
    kappa = 1.1;            % 诱导功率修正系数
    
    U_tip = 48;             % 桨尖速度 (m/s)
    % Omega = 400;          % 角速度 (rad/s), 可由 U_tip/R 推算，此处直接用 U_tip 计算
    % R = 0.12;             % 半径 (m)
    
    % 飞行速度设定
    V_avg = 10;             % 平均巡航速度 (m/s)
    
    %% 2. 基础功率计算 (悬停状态)
    % -----------------------------------------------------------
    % 悬停诱导速度 v_h 随重量 W 变化
    v_h = sqrt(W / (2 * rho * A));
    
    % 悬停功率 P_hover (公式推导)
    % P_induced = kappa * W * v_h
    % P_profile = (sigma * Cd0 / 8) * rho * A * U_tip^3
    
    P_i_hover = kappa * W * v_h;
    P_p_hover = (sigma * Cd0 / 8) * rho * A * (U_tip^3);
    
    P_hover_total = P_i_hover + P_p_hover;
    
    %% 3. 路径分段能耗计算
    % -----------------------------------------------------------
    E_total = 0;
    E_hover = 0;
    E_flight = 0;
    E_climb = 0;
    T_total = 0;
    
    Num_Pts = size(path_points, 1);
    
    % 处理空路径情况
    if Num_Pts == 0
        return;
    end
    
    % 处理悬停时间输入
    if isscalar(hover_time_per_point)
        H_times = ones(Num_Pts, 1) * hover_time_per_point;
    else
        H_times = hover_time_per_point;
    end
    
    % 3.1 计算悬停能耗 (每个点都要停)
    for i = 1:Num_Pts
        t_h = H_times(i);
        e_h = P_hover_total * t_h;
        E_hover = E_hover + e_h;
        T_total = T_total + t_h;
    end
    
    % 3.2 计算飞行能耗 (点与点之间)
    for i = 1:Num_Pts-1
        P1 = path_points(i, :);
        P2 = path_points(i+1, :);
        
        dist_3d = norm(P2 - P1);
        dist_z = P2(3) - P1(3); % 正为爬升，负为下降
        
        if dist_3d < 1e-3
            continue; 
        end
        
        % 假设无人机以恒定地速 V_avg 飞向目标
        dt = dist_3d / V_avg; 
        
        % --- 前飞功率计算 (基于高精度模型) ---
        
        % 1. 寄生功率 (Parasitic Power)
        P_para = 0.5 * d0 * rho * V_avg^3; 
        
        % 2. 前飞诱导功率 (Induced Power in Forward Flight)
        % 高速近似公式: P_i_fwd ≈ P_i_hover * (v_h / V)
        % 注意: v_h 和 P_i_hover 都含有 W，因此这里与 W 强相关
        P_i_fwd = P_i_hover * (v_h / V_avg); 
        
        % 3. 型线功率 (Profile Power)
        % P_p_fwd = P_p_hover * (1 + 4.65 * mu^2)
        mu = V_avg / U_tip; % 前进比
        P_p_fwd = P_p_hover * (1 + 4.65 * mu^2);
        
        P_level_fly = P_para + P_i_fwd + P_p_fwd;
        
        % --- 爬升/下降 修正 ---
        v_z = dist_z / dt; % 垂直分速度
        
        if dist_z > 0
            % 爬升：额外做功克服重力 P = P_level + W * v_z
            P_segment = P_level_fly + W * v_z;
            E_climb = E_climb + (W * v_z * dt);
        else
            % 下降：重力势能释放，功率降低
            % 假设下降时回收部分势能，但功率不能低于型线功率(防止出现负值或过小值)
            P_segment = P_level_fly + (W * v_z * 0.5); 
            
            % 下限保护: 螺旋桨至少需要维持旋转 (型线功率)
            if P_segment < P_p_hover 
                P_segment = P_p_hover; 
            end
        end
        
        % 累加该段能耗
        e_seg = P_segment * dt;
        
        E_flight = E_flight + (P_level_fly * dt); % 记录纯平飞部分的能耗
        E_total = E_total + e_seg;
        T_total = T_total + dt;
    end
    
    % 合并总能耗 (悬停已在 E_hover 计算)
    E_total = E_total + E_hover;

end