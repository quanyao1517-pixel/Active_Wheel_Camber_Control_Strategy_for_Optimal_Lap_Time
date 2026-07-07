%% 拉格朗日乘子 λ–ε 扫描验算：002/008 相对 004 的最大相对误差
load("lag004.mat"); lag004 = lag;
load("lag008.mat"); lag008 = lag;
load("lag002.mat"); lag002 = lag;

wheels = {'clam_fl','clam_fr','clam_rl','clam_rr'};
names  = {'左前','右前','左后','右后'};

fprintf('%-6s | %-22s | %-22s\n','车轮','002 vs 004 (按max/按范围)','008 vs 004 (按max/按范围)');
for i = 1:4
    ref  = full(lag004.(wheels{i}));  ref = ref(:);
    l2   = full(lag002.(wheels{i}));  l2  = l2(:)*2;
    l8   = full(lag008.(wheels{i}));  l8  = l8(:)/2;

    scale_max = max(abs(ref));            % 口径1：|λ004| 最大值
    scale_rng = max(ref) - min(ref);      % 口径2：λ004 的范围

    e2_max = max(abs(l2 - ref)) / scale_max * 100;
    e2_rng = max(abs(l2 - ref)) / scale_rng * 100;
    e8_max = max(abs(l8 - ref)) / scale_max * 100;
    e8_rng = max(abs(l8 - ref)) / scale_rng * 100;

    fprintf('%-6s | %6.2f%% / %6.2f%%      | %6.2f%% / %6.2f%%\n', ...
            names{i}, e2_max, e2_rng, e8_max, e8_rng);
end

