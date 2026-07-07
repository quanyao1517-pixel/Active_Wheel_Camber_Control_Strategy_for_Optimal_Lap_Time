%% 加权最小二乘参数辨识：1–3阶 × 区分/不区分加减速（共6种模型）× 四轮
% 权重 = |拉格朗日乘子|（每轮取对应字段 clam_fl/fr/rl/rr）。
% 模型形式（默认不含纵向一次项，与选型口径一致）：
%   不区分： camber = a + sum_{j=1..p} cj0 * x^j
%   区分  ： camber = a + sum_{j=1..p} (cj0 + cj1*sign(ax)) * x^j
%   x = 侧向加速度 / max|侧向加速度|（归一化，带符号）。
%   2次/区分 即式(24)：a=常数, c10=d0, c11=d1, c20=e0, c21=e1。
% 若最终式(24)保留纵向一次项 b*ax，将 include_b 置 true（所有模型统一加 b 列）。
% 指标口径：全部采用加权拟合（WLS，权重=|lambda|），评价指标为加权R²。
% 实现：模型对参数线性 -> 闭式解（OLS: X\z；WLS: (sqrt(w).*X)\(sqrt(w).*z)），无需初值。

clear; clc; close all;

%% 0. 选项
include_b   = false;                    % 式(24)是否含 b*ax 一次项
plot_p      = 2;  plot_dist = true;     % 绘图所用模型（默认=式24：2次/区分）
save_fig    = true;

%% 1. 数据
load('control0704.mat'); res_ctrl = res;
load('lag004.mat');

g = 9.8;
ax = res_ctrl.U(6,:).' / g;             % 纵向加速度 (g)
ay = res_ctrl.U(7,:).' / g;             % 侧向加速度 (g)
ax(ax==0) = eps;  ay(ay==0) = eps;
x  = ay / max(abs(ay));                 % 归一化侧向加速度

wheel_rows  = [28 29 30 31];
wheel_names = {'左前FL','右前FR','左后RL','右后RR'};
wheel_tags  = {'fl','fr','rl','rr'};
wfield      = {'clam_fl','clam_fr','clam_rl','clam_rr'};   % 权重字段与轮一一对应
degrees = [1 2 3];  dists = [false true];

csv = {};   % 参数导出：wheel, degree, distinguish, wR2, param, value
Rz  = cell(1,4);  Rb = cell(1,4);   % 存储各轮外倾角与绘图模型WLS参数（供四轮汇总图）
Rw2 = zeros(1,4);                   % 生产模型（plot_p/plot_dist）各轮加权R²

%% 2. 四轮 × 6模型：WLS 参数辨识（附 OLS 的普通R²）
for wi = 1:4
    z = res_ctrl.U(wheel_rows(wi),:).' * 180/pi;    % 外倾角 (deg)
    n = numel(z);

    lam = lag.(wfield{wi})(:);
    w   = abs(lam);                                  % 权重 = |lambda|
    if sum(w)==0, w = ones(n,1); end
    sw  = sqrt(w);

    zbw  = sum(w.*z)/sum(w);
    wSST = sum(w.*(z - zbw).^2);

    fprintf('\n================ %s (n=%d, 权重=|lambda|, include_b=%d) ================\n', ...
            wheel_names{wi}, n, include_b);

    for p = degrees
        for dist = dists
            [X, names] = build_X(x, ax, p, dist, include_b);
            m = size(X,2);

            beta_wls = (sw.*X) \ (sw.*z);            % WLS 参数
            wR2 = 1 - sum(w.*(z - X*beta_wls).^2)/wSST;

            tg = '不区分'; if dist, tg = '区分'; end
            fprintf('  [%d次/%s]  k=%d  加权R2=%.4f\n', p, tg, m, wR2);
            for j = 1:m
                fprintf('      %-4s = %+ .6f\n', names{j}, beta_wls(j));
                csv(end+1,:) = {wheel_tags{wi}, p, double(dist), wR2, names{j}, beta_wls(j)}; %#ok<AGROW>
            end
        end
    end

    %% 3. 绘图（沿用原排版风格；模型 = plot_p 次 / plot_dist）
    [Xp, ~]  = build_X(x, ax, plot_p, plot_dist, include_b);
    beta_p   = (sw.*Xp) \ (sw.*z);
    Rw2(wi)  = 1 - sum(w.*(z - Xp*beta_p).^2)/wSST;   % 写入自动生成函数的注释

    zoomplus = 4;
    cmpt = 28.3465*zoomplus; figW = 7.7*cmpt; figH = figW*0.5;
    fs = 7.5*zoomplus; lw = 2;
    figure('Units','points','Position',[300,300,figW,figH],'Color','w','Renderer','painters');

    lat_grid = linspace(min(ay), max(ay), 100).';
    xg = lat_grid / max(abs(ay));
    ax_pos = mean(ax(ax>0)); if isempty(ax_pos)||isnan(ax_pos), ax_pos =  0.5; end
    ax_neg = mean(ax(ax<0)); if isempty(ax_neg)||isnan(ax_neg), ax_neg = -0.5; end
    [Xg_pos,~] = build_X(xg, ax_pos*ones(size(xg)), plot_p, plot_dist, include_b);
    [Xg_neg,~] = build_X(xg, ax_neg*ones(size(xg)), plot_p, plot_dist, include_b);
    camber_pos = Xg_pos * beta_p;
    camber_neg = Xg_neg * beta_p;

    plot(lat_grid, camber_pos, 'b-',  'LineWidth', lw); hold on;
    plot(lat_grid, camber_neg, 'r-.', 'LineWidth', lw);
    pi_ = ax>0; ni_ = ax<0;
    scatter(ay(pi_), z(pi_), 50, 'b', 'filled', 'MarkerFaceAlpha', 0.5);
    scatter(ay(ni_), z(ni_), 50, 'r', 'filled', 'MarkerFaceAlpha', 0.5);

    set(gca,'FontName','宋体','FontSize',fs,'FontWeight','bold','LineWidth',lw, ...
        'XAxisLocation','origin','YAxisLocation','origin','XTickLabelRotation',0,'TickDir','out');
    xlabel('侧向加速度 (g)','FontName','宋体','FontSize',fs,'FontWeight','bold');
    ylabel('外倾角 (deg)','FontName','宋体','FontSize',fs,'FontWeight','bold');
    xlim([min(ay), max(ay)]); ylim([min(z)-0.1, max(z)+0.1]);

    xt = linspace(min(ay), max(ay), 2);
    xtl = arrayfun(@(v) sprintf('%.1f',v), xt, 'UniformOutput', false); xtl(xt==0) = {''};
    set(gca,'XTick',xt,'XTickLabel',xtl);
    yt = linspace(min(z)-0.5, max(z)+0.5, 5);
    ytl = arrayfun(@(v) sprintf('%.1f',v), yt, 'UniformOutput', false); ytl(yt==0) = {''};
    set(gca,'YTick',yt,'YTickLabel',ytl);
    box off;

    l1 = line([0 0.001],[0 0],'Color','b','LineStyle','-', 'LineWidth',lw);
    l2 = line([0 0.001],[0 0],'Color','r','LineStyle','-.','LineWidth',lw);
    h1 = legend([l1,l2], {'纵向加速度 > 0','纵向加速度 < 0'}, ...
        'Location','northwest','FontName','宋体','FontSize',fs,'FontWeight','bold');
    set(h1,'Box','off'); set(gca,'Layer','top');

    if save_fig
        saveas(gcf, sprintf('camber_fit_wls_%dorder_%s_%s.png', plot_p, tern(plot_dist,'dist','nodist'), wheel_tags{wi}));
    end

    Rz{wi} = z;  Rb{wi} = beta_p;   % 供四轮汇总图
end

%% 3b. 四轮拟合效果汇总图（2×2，同一模型：plot_p 次 / plot_dist）
zoomplus = 4;  cmpt = 28.3465*zoomplus;
figW = 16*cmpt;  figH = figW*0.62;          % 双栏宽 16 cm
fs = 7.5*zoomplus;  lw = 2;
figure('Units','points','Position',[100,100,figW,figH],'Color','w','Renderer','painters');

lat_grid = linspace(min(ay), max(ay), 100).';
xg = lat_grid / max(abs(ay));
ax_pos = mean(ax(ax>0)); if isempty(ax_pos)||isnan(ax_pos), ax_pos =  0.5; end
ax_neg = mean(ax(ax<0)); if isempty(ax_neg)||isnan(ax_neg), ax_neg = -0.5; end
[Xg_pos,~] = build_X(xg, ax_pos*ones(size(xg)), plot_p, plot_dist, include_b);
[Xg_neg,~] = build_X(xg, ax_neg*ones(size(xg)), plot_p, plot_dist, include_b);

panel = {'(a) 左前轮','(b) 右前轮','(c) 左后轮','(d) 右后轮'};
pi_ = ax>0;  ni_ = ax<0;
for wi = 1:4
    subplot(2,2,wi);
    z = Rz{wi};  bp = Rb{wi};
    plot(lat_grid, Xg_pos*bp, 'b-',  'LineWidth', lw); hold on;
    plot(lat_grid, Xg_neg*bp, 'r-.', 'LineWidth', lw);
    scatter(ay(pi_), z(pi_), 25, 'b', 'filled', 'MarkerFaceAlpha', 0.5);
    scatter(ay(ni_), z(ni_), 25, 'r', 'filled', 'MarkerFaceAlpha', 0.5);

    set(gca,'FontName','宋体','FontSize',fs,'FontWeight','bold','LineWidth',lw, ...
        'XAxisLocation','origin','YAxisLocation','origin','TickDir','out');
    xlim([min(ay), max(ay)]);  ylim([min(z)-0.1, max(z)+0.1]);
    xt = linspace(min(ay), max(ay), 2);
    xtl = arrayfun(@(v) sprintf('%.1f',v), xt, 'UniformOutput', false); xtl(xt==0) = {''};
    set(gca,'XTick',xt,'XTickLabel',xtl);
    yt = linspace(min(z)-0.5, max(z)+0.5, 5);
    ytl = arrayfun(@(v) sprintf('%.1f',v), yt, 'UniformOutput', false); ytl(yt==0) = {''};
    set(gca,'YTick',yt,'YTickLabel',ytl);
    box off;  set(gca,'Layer','top');
    title(panel{wi}, 'FontName','宋体','FontSize',fs,'FontWeight','bold');
    if wi >= 3,       xlabel('侧向加速度 (g)','FontName','宋体','FontSize',fs,'FontWeight','bold'); end
    if mod(wi,2)==1,  ylabel('外倾角 (deg)','FontName','宋体','FontSize',fs,'FontWeight','bold'); end
    if wi == 1
        l1 = line([0 0.001],[0 0],'Color','b','LineStyle','-', 'LineWidth',lw);
        l2 = line([0 0.001],[0 0],'Color','r','LineStyle','-.','LineWidth',lw);
        h  = legend([l1,l2], {'纵向加速度 > 0','纵向加速度 < 0'}, 'Location','northwest', ...
             'FontName','宋体','FontSize',fs*0.8,'FontWeight','bold');
        set(h,'Box','off');
    end
end
if save_fig
    saveas(gcf, sprintf('camber_fit_wls_%dorder_%s_all4.png', plot_p, tern(plot_dist,'dist','nodist')));
end

%% 4. 导出全部 WLS 参数
T = cell2table(csv, 'VariableNames', {'wheel','degree','distinguish','wR2','param','value'});
writetable(T, 'wls_fit_params.csv');
fprintf('\n已导出 wls_fit_params.csv（全部6模型×四轮的WLS参数与加权R²）\n');
fprintf('注：2次/区分 即式(24)，参数对应 a=a, c10=d0, c11=d1, c20=e0, c21=e1%s。\n', ...
        tern(include_b, '（含 b）', ''));



%% ===== 局部函数 =====
function s_out = eqn_str(b, pnames, branch)
    % branch=false: 打印含 sign(ax) 的通式; branch=+1/-1: 合并为该分支的数值系数
    if ~islogical(branch) && branch ~= 0
        % 合并 cj0 + branch*cj1
        terms = {}; vals = [];
        for i = 1:numel(pnames)
            nm = pnames{i};
            if nm(1)=='c' && nm(end)=='1'
                continue;                          % 并入基值，跳过
            elseif nm(1)=='c' && nm(end)=='0'
                j  = str2double(nm(2:end-1));
                i1 = find(strcmp(pnames, sprintf('c%d1', j)), 1);
                v  = b(i);  if ~isempty(i1), v = v + branch*b(i1); end
                terms{end+1} = pow_str(j);  vals(end+1) = v; %#ok<AGROW>
            else
                terms{end+1} = var_str(nm); vals(end+1) = b(i); %#ok<AGROW>
            end
        end
    else
        terms = cellfun(@var_str, pnames, 'UniformOutput', false);
        vals  = b(:).';
    end
    s_out = '';
    for i = 1:numel(vals)
        if isempty(terms{i}), piece = sprintf('%+.4f', vals(i));
        else,                 piece = sprintf('%+.4f%s', vals(i), terms{i});
        end
        s_out = [s_out, piece, ' ']; %#ok<AGROW>
    end
    s_out = strtrim(s_out);
    if s_out(1)=='+', s_out = s_out(2:end); end
end

function t = var_str(nm)
    switch nm
        case 'a', t = '';
        case 'b', t = '·ax';
        otherwise
            j = str2double(nm(2:end-1));
            if nm(end)=='0', t = pow_str(j);
            else,            t = ['·sign(ax)', pow_str(j)];
            end
    end
end

function t = pow_str(j)
    if j==1, t = '·x'; else, t = sprintf('·x^%d', j); end
end
function [X, names] = build_X(x, ax, p, dist, include_b)
    s = sign(ax);
    X = ones(numel(x),1);  names = {'a'};
    if include_b
        X = [X, ax];  names{end+1} = 'b';
    end
    for j = 1:p
        X = [X, x.^j];            names{end+1} = sprintf('c%d0', j);
        if dist
            X = [X, s.*(x.^j)];   names{end+1} = sprintf('c%d1', j);
        end
    end
end

function out = tern(c, a, b)
    if c, out = a; else, out = b; end
end