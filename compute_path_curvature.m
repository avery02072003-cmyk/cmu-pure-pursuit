% compute_path_curvature.m — 由路徑點座標與航向角估算曲率 kappa = d(phi)/ds
% 抽出自 my_multi_path.m 的 compute_kappa，邏輯不變

function kappa = compute_path_curvature(x, y, phi, params)
    ds = hypot(gradient(x), gradient(y));
    ds(ds<1e-6) = 1e-6;
    kappa = gradient(unwrap(phi)) ./ ds;
    kmax = tan(deg2rad(35)) / params.L1;
    kappa = max(min(kappa, kmax), -kmax);
    win = 9;
    kernel = ones(1,win)/win;
    kappa = conv([kappa(1)*ones(1,(win-1)/2), kappa(:)', kappa(end)*ones(1,(win-1)/2)], kernel, 'valid');
    kappa = kappa(:);
end
