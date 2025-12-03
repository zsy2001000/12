function generate_fixed_points(gis_filename, output_filename, num_fixed_desired)
%% 模块：生成固定巡检点 (严格边缘 + 手动指定版)
% 逻辑修正：
%   1. [关键] 废弃"前30%"策略，改为"严格边缘提取"。
%      只选取距离中心线最远距离 99% 以上的点，确保点绝对在公路最外沿，绝不进入路面。
%   2. 保留手动指定点逻辑，并将其吸附到最新的严格边缘点上。
%   3. 维持双侧平衡随机补齐。

    fprintf('[模块启动] 正在生成严格位于边缘的固定巡检点...\n');

    % 1. 读取数据
    if ~isfile(gis_filename)
        error('未找到 GIS 文件: %s', gis_filename);
    end
    data = readtable(gis_filename);
    x = data.x; y = data.y; z = data.z;

    % =====================================================================
    % 手动指定关键点 (您图中的蓝色标记区域)
    % =====================================================================
    Manual_Inputs = [
        3.54325e5, 2.94065e6;  % 左下
        3.54375e5, 2.94066e6;  % 下侧
        3.54470e5, 2.94090e6;  % 右上1
        3.54490e5, 2.94100e6;  % 右上2
        3.54420e5, 2.94073e6;  % 中间
    ];

    % 2. 构建"严格边缘"候选池
    [x_sorted, sort_idx] = sort(x);
    y_sorted = y(sort_idx);
    z_sorted = z(sort_idx);
    
    Upper_Pool = []; 
    Lower_Pool = []; 
    
    % 使用极高密度切片，确保能捕捉到每一处的边缘
    Fine_Sections = 300; 
    x_edges = linspace(min(x), max(x), Fine_Sections + 1);
    
    for k = 1:Fine_Sections
        idx = (x_sorted >= x_edges(k)) & (x_sorted < x_edges(k+1));
        if ~any(idx), continue; end
        
        sub_x = x_sorted(idx); sub_y = y_sorted(idx); sub_z = z_sorted(idx);
        cy = mean(sub_y);
        
        % --- 上边缘严格筛选 ---
        idx_up = sub_y > cy;
        if any(idx_up)
            dists = sqrt((sub_x(idx_up)-mean(sub_x)).^2 + (sub_y(idx_up)-cy).^2);
            max_d = max(dists);
            
            % [核心修改] 阈值从 0.3 改为 0.99 (甚至 0.995)
            % 只有距离中心最远距离达到 99% 的点才被视为"边缘"
            edge_mask = dists >= (max_d * 0.99); 
            
            tmp_x = sub_x(idx_up); tmp_y = sub_y(idx_up); tmp_z = sub_z(idx_up);
            Upper_Pool = [Upper_Pool; tmp_x(edge_mask), tmp_y(edge_mask), tmp_z(edge_mask)];
        end
        
        % --- 下边缘严格筛选 ---
        idx_down = sub_y <= cy;
        if any(idx_down)
            dists = sqrt((sub_x(idx_down)-mean(sub_x)).^2 + (sub_y(idx_down)-cy).^2);
            max_d = max(dists);
            
            % [核心修改] 同样严格限制下边缘
            edge_mask = dists >= (max_d * 0.99);
            
            tmp_x = sub_x(idx_down); tmp_y = sub_y(idx_down); tmp_z = sub_z(idx_down);
            Lower_Pool = [Lower_Pool; tmp_x(edge_mask), tmp_y(edge_mask), tmp_z(edge_mask)];
        end
    end
    
    All_Candidates = [Upper_Pool; Lower_Pool];
    
    % 3. 处理手动指定点 (吸附到新的严格边缘)
    Final_Fixed_Points = [];
    
    if ~isempty(Manual_Inputs)
        for i = 1:size(Manual_Inputs, 1)
            mx = Manual_Inputs(i, 1);
            my = Manual_Inputs(i, 2);
            % 寻找最近的严格边缘点
            dists = sqrt((All_Candidates(:,1) - mx).^2 + (All_Candidates(:,2) - my).^2);
            [~, min_idx] = min(dists);
            Final_Fixed_Points = [Final_Fixed_Points; All_Candidates(min_idx, :)];
        end
    end
    
    % 4. 随机补齐 (双侧平衡)
    Num_Existing = size(Final_Fixed_Points, 1);
    Num_To_Generate = num_fixed_desired - Num_Existing;
    
    if Num_To_Generate > 0
        Num_Upper_Target = ceil(Num_To_Generate / 2);
        Num_Lower_Target = Num_To_Generate - Num_Upper_Target;
        
        % 从上侧严格边缘池抽取
        if size(Upper_Pool, 1) < Num_Upper_Target
            Selected_Upper = Upper_Pool;
        else
            rand_idx = randperm(size(Upper_Pool, 1), Num_Upper_Target);
            Selected_Upper = Upper_Pool(rand_idx, :);
        end
        
        % 从下侧严格边缘池抽取
        if size(Lower_Pool, 1) < Num_Lower_Target
            Selected_Lower = Lower_Pool;
        else
            rand_idx = randperm(size(Lower_Pool, 1), Num_Lower_Target);
            Selected_Lower = Lower_Pool(rand_idx, :);
        end
        
        Final_Fixed_Points = [Final_Fixed_Points; Selected_Upper; Selected_Lower];
    end
    
    % 按经度排序
    [~, sort_fix] = sort(Final_Fixed_Points(:,1));
    Final_Fixed_Points = Final_Fixed_Points(sort_fix, :);
    
    % 5. 保存
    T = table(Final_Fixed_Points(:,1), Final_Fixed_Points(:,2), Final_Fixed_Points(:,3), ...
              'VariableNames', {'x', 'y', 'z'});
    writetable(T, output_filename);
    
    fprintf('[模块完成] 固定点生成完毕 (手动:%d, 补齐后总计:%d)。\n', Num_Existing, size(Final_Fixed_Points, 1));
    fprintf('           位置: 已严格锁定在公路最外边缘。\n');
end