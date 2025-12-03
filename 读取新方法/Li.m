% =========================================================================
%  Li.m - g (距离/高度) 范围计算器 & 数据导出
%  功能: 计算 g 范围并保存为 'g_range_data.mat' 供 three_d.m 调用
% =========================================================================

clear; clc;

% 1. 定义输入提示
prompt = {
    '请输入 a (容忍误差, e.g. 0.1):', ...
    '请输入 c (经度):', ...
    '请输入 d (传感器长 mm):', ...
    '请输入 e (传感器宽 mm):', ...
    '请输入 f_min (最小焦距 mm):', ...
    '请输入 f_max (最大焦距 mm):'
};

dlgtitle = 'g (高度) 范围计算输入';
dims = [1 50];
definput = {'10', '36', '24', '0.1', '24', '70'}; % 默认值

% 2. 弹出输入框
answer = inputdlg(prompt, dlgtitle, dims, definput);

if isempty(answer)
    msgbox('已取消输入', '提示');
    return;
end

try
    % 3. 获取输入值
    a = str2double(answer{1});
    c = str2double(answer{2});
    d = str2double(answer{3});
    e = str2double(answer{4});
    f_min = str2double(answer{5});
    f_max = str2double(answer{6});
    
    % 数据校验
    if any([a, c, d, f_min, f_max] <= 0), error('数值必须为正！'); end
    if e < 0 || e >= 10, error(' e 必须在 0-10 之间！'); end
    if f_min > f_max, error('f_min 不能大于 f_max！'); end

    % 4. 核心计算
    b = min(c, d); % 有效限制边
    
    % 计算 g (距离/高度) 范围
    g_min = (a * f_min) / (b * (1 + e));
    g_max = (a * f_max) / (b * (1 - e));
    
    % 5. 【关键步骤】保存数据供 three_d.m 使用
    save('g_range_data.mat', 'g_min', 'g_max');
    
    % 6. 结果反馈
    fprintf('计算完成: g_min = %.4f, g_max = %.4f\n', g_min, g_max);
    fprintf('数据已保存至: g_range_data.mat\n');
    
    msg_str = sprintf(['计算成功！\n\n', ...
                       '高度范围 g: [%.4f, %.4f]\n\n', ...
                       '数据已自动保存。\n', ...
                       '请现在运行 three_d.m 查看可视化结果。'], ...
                       g_min, g_max);
    msgbox(msg_str, '完成');
    
catch ME
    errordlg(ME.message, '计算错误');
end