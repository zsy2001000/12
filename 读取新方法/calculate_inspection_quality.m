function [focal_length, quality_score, is_feasible] = calculate_inspection_quality(flight_height, target_size, required_pixels)
%% calculate_inspection_quality - 视觉-动作协同模型 (专利核心)
%
% 功能：
%   基于针孔相机模型和专利中的动作设定公式，计算在特定飞行高度下，
%   为了看清目标所需的焦距，并评估成像质量。
%
% 输入：
%   flight_height:   无人机相对目标的垂直高度 (米)
%   target_size:     目标在地面的特征尺寸 (米)，例如裂缝宽度 0.002m
%   required_pixels: 目标在图像中至少需要占据的像素数 (例如 5 px)
%
% 输出：
%   focal_length:  所需的相机焦距 (毫米)
%   quality_score: 成像质量评分 (0-1)，1为最佳
%   is_feasible:   布尔值，该动作是否在相机硬件能力范围内
%
% 参考文献：
%   [1] 专利《一种融合目标巡检动作设定的无人机航迹规划方法》公式(2)-(3)

    %% 1. 相机硬件参数 (假设为某典型工业无人机相机，如 Zenmuse H20)
    % -----------------------------------------------------------
    Sensor_Width_mm = 6.4;      % 传感器宽度 (1/2.3英寸CMOS)
    Image_Width_px = 4000;      % 图片宽度 (像素)
    Pixel_Size_mm = Sensor_Width_mm / Image_Width_px; % 单个像素物理尺寸
    
    F_min = 4.5;    % 最小焦距 (mm, 广角)
    F_max = 135;    % 最大焦距 (mm, 长焦/变焦)
    
    %% 2. 计算所需 GSD (地面采样距离)
    % -----------------------------------------------------------
    % GSD = 目标物理尺寸 / 所需像素数
    % 例如：裂缝宽 2mm，需要 5个像素覆盖 -> GSD = 0.4 mm/pixel
    Required_GSD_m = target_size / required_pixels;
    
    %% 3. 计算所需焦距 (Focal Length)
    % -----------------------------------------------------------
    % 公式：GSD = (Pixel_Size * Height) / Focal_Length
    % 变换得：Focal_Length = (Pixel_Size * Height) / GSD
    
    % 注意单位统一：Pixel_Size(mm), Height(m), GSD(m) -> f(mm)
    % f(mm) = (Pixel_Size(mm) * Height(m) * 1000(mm/m)) / (GSD(m) * 1000(mm/m)) ?
    % 推导：
    % h / f = GSD / px_size (相似三角形)
    % f = (px_size * h) / GSD
    
    % 将高度转为 mm 计算
    H_mm = flight_height * 1000;
    
    % 计算理论焦距 f_calc (mm)
    f_calc = (Pixel_Size_mm * H_mm) / (Required_GSD_m * 1000); 
    
    focal_length = f_calc;
    
    %% 4. 可行性与质量评估
    % -----------------------------------------------------------
    is_feasible = true;
    quality_score = 1.0;
    
    if f_calc > F_max
        % 情况 1: 需要的焦距超过了相机最大焦距 (飞太高了，看不清)
        focal_length = F_max; % 只能用到最大变焦
        
        % 重新计算实际 GSD
        Actual_GSD_m = (Pixel_Size_mm * H_mm) / (F_max * 1000);
        
        % 质量评分下降
        % Score = Required_GSD / Actual_GSD
        % 如果实际 GSD 是 2cm，要求 1cm，则分数为 0.5
        quality_score = Required_GSD_m / Actual_GSD_m;
        
        % 如果评分太低 (例如 < 0.5)，则视为不可行任务
        if quality_score < 0.5
            is_feasible = false;
        end
        
    elseif f_calc < F_min
        % 情况 2: 需要的焦距小于最小焦距 (飞太低了，视野太小)
        % 虽然可以看清，但可能视野覆盖不足，这里简化为质量仍为1
        focal_length = F_min;
        is_feasible = true; 
    else
        % 情况 3: 焦距在范围内，完美匹配
        quality_score = 1.0;
    end

end