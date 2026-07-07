% 颜色定义
rr = [1 0.7 0.7];
rb = [0.62 0.66 1];
step_length = 1;
trackwidth = 2.5;

% 加载数据

load('Track.mat');
load('control0704.mat');  res_ctrl = res;
load('passive0704.mat');  res_pass = res;
load("lag004.mat")
% 创建图形
zoomplus = 4;
cm_to_points = 28.3465 * zoomplus; % 厘米到磅的转换因子
fig_width_cm = 7.7; % 保持原画布宽度
fig_width_points = fig_width_cm * cm_to_points;
fig_height_points = fig_width_points * 0.9; % 画布高度比例为1.1
font_size = 7.5 * zoomplus; % 字体大小
line_width = 4;

% 数据准备
wight = res_ctrl.U(6,:) / 9.8; % 纵向加速度（暂时保留，未使用）
x_data = res_ctrl.U(7,:) / 9.8; % 侧向加速度
y_data1 = res_ctrl.U(28,:) * 180 / pi; % 左前轮外倾角
y_data2 = res_ctrl.U(29,:) * 180 / pi; % 右前轮外倾角
y_data3 = res_ctrl.U(30,:) * 180 / pi; % 左后轮外倾角
y_data4 = res_ctrl.U(31,:) * 180 / pi; % 右后轮外倾角

% ====== 新增：统一四个拉格朗日乘子的颜色刻度范围 ======
all_lag = [lag.clam_fl(:); lag.clam_fr(:); lag.clam_rl(:); lag.clam_rr(:)];
clim_global = [min(all_lag), max(all_lag)];

% 创建图形窗口
fig = figure('Units', 'points', 'Position', [300, 300, fig_width_points, fig_height_points], ...
    'Color', 'w', 'Renderer', 'painters');

% 子图1：左前轮外倾角 (res.U(28,:))
subplot(2, 2, 1);
scatter(x_data, y_data1, 50, lag.clam_fl, 'filled', 'DisplayName', '圈速仿真结果', ...
    'MarkerFaceAlpha', 1, 'MarkerEdgeColor', 'none');
set(gca, 'FontName', '宋体', 'FontSize', font_size, 'FontWeight', 'bold', ...
    'LineWidth', line_width, 'XAxisLocation', 'origin', 'YAxisLocation', 'origin', ...
    'XTickLabelRotation', 0, 'TickDir', 'out', 'CLim', clim_global);
xticks = get(gca, 'XTick');
xticks_labels = arrayfun(@num2str, xticks, 'UniformOutput', false);
xticks_labels(xticks == 0) = {''};
set(gca, 'XTick', xticks, 'XTickLabel', {});
for i = 1:length(xticks)
    if ~strcmp(xticks_labels{i}, '')
        text(xticks(i), -0.05, xticks_labels{i}, ...
            'HorizontalAlignment', 'center', 'VerticalAlignment', 'top', ...
            'FontName', '宋体', 'FontSize', font_size, 'FontWeight', 'bold');
    end
end
box off;

% 子图2：右前轮外倾角 (res.U(29,:))
subplot(2, 2, 2);
scatter(x_data, y_data2, 50, lag.clam_fr, 'filled', 'DisplayName', '圈速仿真结果', ...
    'MarkerFaceAlpha', 1, 'MarkerEdgeColor', 'none');
set(gca, 'FontName', '宋体', 'FontSize', font_size, 'FontWeight', 'bold', ...
    'LineWidth', line_width, 'XAxisLocation', 'origin', 'YAxisLocation', 'origin', ...
    'XTickLabelRotation', 0, 'TickDir', 'out', 'CLim', clim_global);
box off;

% 子图3：左后轮外倾角 (res.U(30,:))
subplot(2, 2, 3);
scatter(x_data, y_data3, 50, lag.clam_rl, 'filled', 'DisplayName', '圈速仿真结果', ...
    'MarkerFaceAlpha', 1, 'MarkerEdgeColor', 'none');
set(gca, 'FontName', '宋体', 'FontSize', font_size, 'FontWeight', 'bold', ...
    'LineWidth', line_width, 'XAxisLocation', 'origin', 'YAxisLocation', 'origin', ...
    'XTickLabelRotation', 0, 'TickDir', 'out', 'CLim', clim_global);
xticks = get(gca, 'XTick');
xticks_labels = arrayfun(@num2str, xticks, 'UniformOutput', false);
xticks_labels(xticks == 0) = {''};
set(gca, 'XTick', xticks, 'XTickLabel', {});
for i = 1:length(xticks)
    if ~strcmp(xticks_labels{i}, '')
        text(xticks(i), -0.05, xticks_labels{i}, ...
            'HorizontalAlignment', 'center', 'VerticalAlignment', 'top', ...
            'FontName', '宋体', 'FontSize', font_size, 'FontWeight', 'bold');
    end
end
box off;

% 子图4：右后轮外倾角 (res.U(31,:))
subplot(2, 2, 4);
scatter(x_data, y_data4, 50, lag.clam_rr, 'filled', 'DisplayName', '圈速仿真结果', ...
    'MarkerFaceAlpha', 1, 'MarkerEdgeColor', 'none');
set(gca, 'FontName', '宋体', 'FontSize', font_size, 'FontWeight', 'bold', ...
    'LineWidth', line_width, 'XAxisLocation', 'origin', 'YAxisLocation', 'origin', ...
    'XTickLabelRotation', 0, 'TickDir', 'out', 'CLim', clim_global);
box off;

% 同步纵轴刻度
linkaxes([subplot(2, 2, 1), subplot(2, 2, 2), subplot(2, 2, 3), subplot(2, 2, 4)], 'y');

% ====== 色带彩色 ======
colormap(fig, 'jet');

c = colorbar(subplot(2, 2, 4), 'Location', 'eastoutside'); % 附加到右下子图
c.Limits = clim_global; % 使用全局拉格朗日乘子的最大最小值
c.LineWidth = line_width;
c.FontName = '宋体';
c.FontSize = font_size;
c.Label.String = '拉格朗日乘子';   % 单位未知，如果有单位请告诉我补上，比如 '拉格朗日乘子 (N)'
c.Label.FontName = '宋体';
c.Label.FontSize = font_size;
c.Label.FontWeight = 'bold';
set(c, 'Position', [0.80 0.15 0.03 0.7]);

% ====== 调整子图位置：缩小每个子图尺寸，加大彼此间距和边距 ======
set(subplot(2, 2, 1), 'Position', [0.14 0.58 0.30 0.33]); % 左上子图（左侧留位置给共享ylabel）
set(subplot(2, 2, 2), 'Position', [0.48 0.58 0.30 0.33]); % 右上子图
set(subplot(2, 2, 3), 'Position', [0.14 0.16 0.30 0.33]); % 左下子图（底部留位置给共享xlabel）
set(subplot(2, 2, 4), 'Position', [0.48 0.16 0.30 0.33]); % 右下子图

% ===== 分图标签，置于各子图正下方 =====
axs = findobj(fig, 'Type', 'axes');
pos = cell2mat(get(axs, 'Position'));
[~, idx] = sortrows([-pos(:,2), pos(:,1)]);
axs = axs(idx);

labels = {'左前轮', '右前轮', '左后轮', '右后轮'};
for k = 1:4
    t = text(axs(k), 0, 0, labels{k}, 'FontName', '宋体', ...
        'FontSize', font_size, 'FontWeight', 'bold', ...
        'HorizontalAlignment', 'center', 'VerticalAlignment', 'top');
    set(t, 'Units', 'normalized', 'Position', [0.5, -0.13, 0]);
end

% ====== 整张图共享的坐标轴单位标签 ======
ax_shared = axes(fig, 'Position', [0 0 1 1], 'Visible', 'off');
uistack(ax_shared, 'bottom');

text(ax_shared, 0.03, 0.5, '外倾角 (deg)', ...
    'FontName', '宋体', 'FontSize', font_size, 'FontWeight', 'bold', ...
    'Rotation', 90, 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');

text(ax_shared, 0.5, 0.02, '侧向加速度 (g)', ...
    'FontName', '宋体', 'FontSize', font_size, 'FontWeight', 'bold', ...
    'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');