rr=[1 0.7 0.7];
rb=[0.62 0.66 1];
step_length = 1;
trackwidth=2.5;
% Track = simVFtzto(step_length);
load('Track.mat');
load('control0704.mat');  res_ctrl = res;
load('passive0704.mat');  res_pass = res;

% 创建图形
zoomplus=4;
cm_to_points = 28.3465*zoomplus; % 厘米到磅的转换因子
fig_width_cm = 7.7;
fig_width_points = fig_width_cm * cm_to_points;
fig_height_points = fig_width_points *0.9; % 高度设为宽度的1.25倍，可调整
font_size = 7.5*zoomplus; % 6号字 = 7.5磅
line_width = 2;
fig = figure('Units', 'points', 'Position', [300, 300, fig_width_points, fig_height_points], 'Color', 'w', 'Renderer', 'painters');

% 定义子图位置（增加左右边距）
subplot_width = 0.8;  % 子图宽度（原0.8，减小以增加边距）
subplot_height = 0.18;
gap = 0.04;
subplot_positions = [
    0.15, 0.8, subplot_width, subplot_height;                % 子图1
    0.15, 0.8 - subplot_height - gap, subplot_width, subplot_height;  % 子图2
    0.15, 0.8 - 2*(subplot_height + gap), subplot_width, subplot_height;  % 子图3
    0.15, 0.8 - 3*(subplot_height + gap), subplot_width, subplot_height];  % 子图4

% 固定X轴范围
x_range = [0, 330];
p_y= [-35,800,0];
p_y2= [-35,300,0];
% 定义Y轴范围和刻度，避免终点和刻度重合
y_range = [-200,1800];
y_ticks = [-0, 800, 1600]; 
y_range2 = [-200,800];
y_ticks2 = [-0, 300, 600]; 
% 子图1
ax1 = axes('Position', subplot_positions(1,:));
plot(Track.S(1:length(res_pass.U(1,:))), res_ctrl.Fy(1,:), 'r-.', 'Linewidth', 4); hold on;
plot(Track.S(1:length(res_pass.U(1,:))), res_pass.Fy(1,:), 'b', 'Linewidth', 4);
ylabel('左前(N)','Position', p_y2, 'FontName', 'SimSun', 'FontSize', font_size);
set(ax1, 'Box', 'off', 'LineWidth', 1.2, 'FontSize', font_size, 'FontWeight', 'bold', ...
    'TickDir', 'out', 'FontName', 'Times New Roman', 'XTickLabel', []);
xlim(ax1, x_range);
ylim(ax1, y_range2);
set(ax1, 'YTick', y_ticks2);
% 明确设置刻度标签的字体
set(gca, 'FontName', '宋体', 'FontSize', font_size, 'LineWidth', 4);
% 子图2
ax2 = axes('Position', subplot_positions(2,:));
plot(Track.S(1:length(res_pass.U(1,:))), res_ctrl.Fy(2,:), 'r-.', 'Linewidth', 4); hold on;
plot(Track.S(1:length(res_pass.U(1,:))), res_pass.Fy(2,:), 'b', 'Linewidth', 4);
ylabel('右前(N)','Position', p_y, 'FontName', 'SimSun', 'FontSize', font_size);
set(ax2, 'Box', 'off', 'Linewidth', 1.2, 'FontSize', font_size, 'FontWeight', 'bold', ...
    'TickDir', 'out', 'FontName', 'Times New Roman', 'XTickLabel', []);
xlim(ax2, x_range);
ylim(ax2, y_range);
set(ax2, 'YTick', y_ticks);
% 明确设置刻度标签的字体
set(gca, 'FontName', '宋体', 'FontSize', font_size, 'LineWidth', 4);
% 子图3
ax3 = axes('Position', subplot_positions(3,:));
plot(Track.S(1:length(res_pass.U(1,:))), res_ctrl.Fy(3,:), 'r-.', 'Linewidth', 4); hold on;
plot(Track.S(1:length(res_pass.U(1,:))), res_pass.Fy(3,:), 'b', 'Linewidth', 4);
ylabel('左后(N)','Position', p_y2, 'FontName', 'SimSun', 'FontSize', font_size);
set(ax3, 'Box', 'off', 'Linewidth', 1.2, 'FontSize', font_size, 'FontWeight', 'bold', ...
    'TickDir', 'out', 'FontName', 'Times New Roman', 'XTickLabel', []);
xlim(ax3, x_range);
ylim(ax3, y_range2);
set(ax3, 'YTick', y_ticks2);
% 明确设置刻度标签的字体
set(gca, 'FontName', '宋体', 'FontSize', font_size, 'LineWidth', 4);
% 子图4
ax4 = axes('Position', subplot_positions(4,:));
plot(Track.S(1:length(res_pass.U(1,:))), res_ctrl.Fy(4,:), 'r-.', 'Linewidth',4); hold on;
plot(Track.S(1:length(res_pass.U(1,:))), res_pass.Fy(4,:), 'b', 'Linewidth', 4);
xlabel('赛车行驶距离 (m)', 'FontName', 'SimSun', 'FontSize', font_size);
ylabel('右后(N)','Position', p_y, 'FontName', 'SimSun', 'FontSize', font_size);
set(ax4, 'Box', 'off', 'Linewidth', 1.2, 'FontSize', font_size, 'FontWeight', 'bold', ...
    'TickDir', 'out', 'FontName', 'Times New Roman');
xlim(ax4, x_range);
ylim(ax4, y_range);
set(ax4, 'YTick', y_ticks);

% 共享图例（调整位置以避免覆盖）
% 共享图例（调整位置以避免覆盖）
legendLine1 = line([0 0.001], [0 0], 'Color', 'b', 'LineStyle', '-','Linewidth',4);
legendLine2 = line([0 0.001], [0 0], 'Color', 'r', 'LineStyle', '-.','Linewidth',4);
legend([legendLine1, legendLine2], {'被动悬架','主动外倾'}, 'Location', 'northoutside', 'FontSize', font_size, ...
       'FontName', 'SimSun', 'Box', 'off', 'Orientation', 'horizontal');

  % 添加图例
% h1=legend([legendLine1, legendLine2], {'测试数据','有限元模型'}, 'FontName', 'SimSun', 'Location', 'northwest','FontSize',18);
% set(h1,'Box','off');
set(gca,'FontWeight','bold'); 
set(gca,'FontSize',18);
set(gca,'Layer','top');
% 明确设置刻度标签的字体
set(gca, 'FontName', '宋体', 'FontSize', font_size, 'LineWidth', 4);
% 统一数字字体
set(findall(gcf, 'Type', 'text'), 'FontName', 'SimSun');
set(gcf, 'Color', 'w', 'InvertHardcopy', 'off');
print(fig, '-dpng', '-r600', 'fy_four_wheel_77mm.png');