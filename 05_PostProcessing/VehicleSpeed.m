% 颜色定义
rr = [1 0.7 0.7];
rb = [0.62 0.66 1];
step_length = 1;
trackwidth = 2.5;

% 加载数据
% Track = simVFtzto(step_length);
load('Track.mat');
load('control0704.mat');  res_ctrl = res;
load('passive0704.mat');  res_pass = res;

% 创建图形
zoomplus=4;
cm_to_points = 28.3465*zoomplus; % 厘米到磅的转换因子
fig_width_cm = 7.7;
fig_width_points = fig_width_cm * cm_to_points;
fig_height_points = fig_width_points *0.5; % 高度设为宽度的1.25倍，可调整
font_size = 7.5*zoomplus; % 6号字 = 7.5磅
line_width = 2;
fig = figure('Units', 'points', 'Position', [300, 300, fig_width_points, fig_height_points], 'Color', 'w', 'Renderer', 'painters');


% 绘制数据（线宽设置为2）
plot(Track.S(1:length(res_pass.X(1,:))), res_ctrl.X(3,:), 'r-.', 'Linewidth', 4); hold on;
plot(Track.S(1:length(res_pass.X(1,:))), res_pass.X(3,:), 'b', 'Linewidth', 4);

% 设置坐标轴
xlabel('赛车行驶距离(m)','Position', [150,5,0], 'FontName', '宋体', 'FontSize', 18, 'FontWeight', 'bold');
ylabel('车速(m/s)', 'FontName', '宋体', 'FontSize', font_size, 'FontWeight', 'bold');
set(gca, 'FontName', '宋体', 'FontSize', font_size, 'FontWeight', 'bold', ...
    'LineWidth', 4, 'XAxisLocation', 'origin', 'YAxisLocation', 'origin');

% 设置数据范围和刻度
daspect([5, 1, 1])
xlim([0, 330]);
ylim([10,34]);
% xticks([1, 8, 30]);
% yticks([5, 1, 1]);
box off

% 明确设置刻度标签的字体
set(gca, 'FontName', '宋体', 'FontSize', font_size);

% 创建图例
legendLine1 = line([0 0.001], [0 0], 'Color', 'b', 'LineStyle', '-', 'Linewidth', 4);
legendLine2 = line([0 0.001], [0 0], 'Color', 'r', 'LineStyle', '-.', 'Linewidth', 4);
h1 = legend([legendLine1, legendLine2], {'被动悬架', '主动外倾'}, 'Location', 'northwest', ...
    'FontName', '宋体', 'FontSize', font_size, 'FontWeight', 'bold');
set(h1, 'Box', 'off');
set(gca, 'Layer', 'top');