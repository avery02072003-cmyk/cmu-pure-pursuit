% ==============================================================================
% 檔案名稱: STEP4_ETS2_Animation_MultiView.m
% 版本: V4.5 (Fix Trailer Orientation & Legend Pos)
% 功能: 
%   1. [修復] 修正 3D 拖車模型前後接反的問題 (重新計算局部座標幾何)。
%   2. [優化] 將 3D 主視圖的圖例移至右上角，避免遮擋左上角的 OSD 資訊。
%   3. [繼承] 保留 V4.4 的所有功能 (防呆讀取、參數連動、增強顯示)。
% ==============================================================================

%% 1. 系統初始化
clear; close all; clc;

% --- 設定 ---
PLAYBACK_SPEED = 1.0;   
TARGET_FPS = 60;        

try
    if ~exist('vehicle_params.mat','file') || ~exist('reference_path.mat','file') || ~exist('simulation_results.mat','file')
        error('Missing Data. Please run STEP 1, 2, 3 first. / 缺少數據，請先執行 STEP 1, 2, 3。');
    end
    
    load('vehicle_params.mat', 'params');
    load('reference_path.mat', 'refpath');
    load('simulation_results.mat', 'results');
    
    hist = results.hist;
    simData.time = results.ts;
    simData.X0 = hist.x0; simData.Y0 = hist.y0; simData.Theta0 = hist.theta0;
    simData.X1 = hist.x1; simData.Y1 = hist.y1; simData.Theta1 = hist.theta1;
    simData.v_cmd = hist.v_cmd; 
    
    if isfield(hist, 'acc'), simData.acc = hist.acc;
    else, simData.acc = [0; diff(hist.v_cmd)./0.02]; end
    
    if isfield(hist, 'omega_cmd'), simData.w_cmd = hist.omega_cmd;
    else, simData.w_cmd = [0; diff(hist.theta0) ./ 0.05]; end
    
    if isfield(hist, 'err_y')
        simData.ye = hist.err_y; 
    else
        warning('歷史資料中缺少 err_y，將顯示為 0。請重新執行 STEP 3 以獲得完整數據。');
        simData.ye = zeros(length(simData.time), 1);
    end
    
    if isfield(hist, 'Hx'), simData.XH = hist.Hx; simData.YH = hist.Hy;
    else, simData.XH = simData.X0 - params.M1 * cos(simData.Theta0); simData.YH = simData.Y0 - params.M1 * sin(simData.Theta0); end
    N = length(simData.time);

catch ME
    error(['Error: ' ME.message]);
end

max_speed_kmh = max(simData.v_cmd) * 3.6;
max_acc_g = max(abs(simData.acc)) / 9.81;

if isfield(params, 'tractor_length'), lenT = params.tractor_length; else, lenT = params.L1 + 1.5; end
if isfield(params, 'trailer_length'), lenH = params.trailer_length; else, lenH = params.L2 + 2.0; end
widthT = params.tractor_width;
widthH = params.trailer_width;
totalLen = lenT + lenH - 0.5; 

str_dim = sprintf('Specs: Tractor[L=%.1f, W=%.1f] Trailer[L=%.1f, W=%.1f] Total≈%.1fm', ...
    lenT, widthT, lenH, widthH, totalLen);

strTractorC = 'Tractor Center (車頭中心-黃)'; 
strTrailerC = 'Trailer Center (拖車中心-橘)';
strHitch    = 'Hitch Point (連接點-白)';
strTracL    = 'Front Left (前左-青)';
strTracR    = 'Front Right (前右-洋紅)';
strTrlL     = 'Rear Left (後左-綠虛)';
strTrlR     = 'Rear Right (後右-紅虛)';

%% 2. 視窗佈局設定
scr = get(0, 'ScreenSize');
W = scr(3); H = scr(4);

% [Figure 1] 3D 主視窗
fig3D = figure('Name', 'Figure 1: 3D Simulation View', ...
               'Color', [0.1 0.1 0.12], ... 
               'Position', [10, H*0.2, W*0.55, H*0.7], ...
               'Renderer', 'opengl', ...
               'GraphicsSmoothing', 'on', ...
               'NumberTitle', 'off', 'MenuBar', 'figure');

ax3D = axes('Parent', fig3D, 'Color', 'none', ...
            'XColor', 'none', 'YColor', 'none', 'ZColor', 'none'); 
hold(ax3D, 'on'); axis(ax3D, 'equal');
camproj(ax3D, 'perspective'); grid(ax3D, 'off'); 
ax3D.CameraViewAngleMode = 'manual'; ax3D.CameraViewAngle = 40; 

light('Position', [-100 -100 200], 'Style', 'local', 'Color', [1.0 1.0 0.95], 'Parent', ax3D);
light('Position', [100 100 50], 'Style', 'local', 'Color', [0.6 0.6 0.7], 'Parent', ax3D);   
light('Position', [0 -50 10], 'Style', 'local', 'Color', [0.5 0.5 0.5], 'Parent', ax3D); 
lighting(ax3D, 'gouraud'); 

title(ax3D, ['3D Pursuit View | ' str_dim], 'Color', 'w', 'FontSize', 11, 'Interpreter', 'none');

% [Figure 2] 地圖系統
figMap = figure('Name', 'Figure 2: Navigation Maps', ...
                'Color', [0.1 0.1 0.12], ...
                'Position', [W*0.57, H*0.5, W*0.42, H*0.4], ...
                'NumberTitle', 'off', 'MenuBar', 'figure');
tMap = tiledlayout(figMap, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

axTactical = nexttile(tMap);
hold(axTactical, 'on'); grid(axTactical, 'on'); axis(axTactical, 'equal');
title(axTactical, 'Local Map / 局部戰術圖', 'Color', 'w');
set(axTactical, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'GridColor', [0.3 0.3 0.3]); 

axGlobal = nexttile(tMap);
hold(axGlobal, 'on'); grid(axGlobal, 'on'); axis(axGlobal, 'equal');
title(axGlobal, 'Global Map / 全域賽道圖', 'Color', 'w');
set(axGlobal, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'GridColor', [0.2 0.2 0.2]);

draw2DTrack_Optimized(axGlobal, refpath.x, refpath.y, refpath.wL, refpath.wR);
annotateTrackWidth_Smart(axGlobal, refpath.x, refpath.y, refpath.wL, refpath.wR); 

% [Figure 3] 數據儀表板
figData = figure('Name', 'Figure 3: Telemetry', ...
                 'Color', [0.1 0.1 0.12], ...
                 'Position', [W*0.57, 50, W*0.42, H*0.4], ...
                 'NumberTitle', 'off', 'MenuBar', 'figure');
tData = tiledlayout(figData, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

axErr = nexttile(tData);
title(axErr, 'Lateral Error [m]', 'Color', 'w'); grid(axErr, 'on'); hold(axErr, 'on');
set(axErr, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'YLim', [-0.5 0.5]);

axArt = nexttile(tData);
title(axArt, 'Articulation Angle [deg]', 'Color', 'w'); grid(axArt, 'on'); hold(axArt, 'on');
set(axArt, 'Color', 'k', 'XColor', 'w', 'YColor', 'w');

axSteer = nexttile(tData);
title(axSteer, 'Steering Angle [deg]', 'Color', 'w'); grid(axSteer, 'on'); hold(axSteer, 'on');
set(axSteer, 'Color', 'k', 'XColor', 'w', 'YColor', 'w');


%% 3. 場景物件初始化

% --- 3D 物件 ---
hGround = patch(ax3D, ...
    [-400 400 400 -400], [-400 -400 400 400], [-0.2 -0.2 -0.2 -0.2], ...
    [0.1 0.1 0.1], 'EdgeColor', 'none', 'FaceAlpha', 1.0, 'HandleVisibility', 'off');
hGroundGrp = hgtransform('Parent', ax3D); set(hGround, 'Parent', hGroundGrp);

draw3DTrack(ax3D, refpath.x, refpath.y, refpath.wL, refpath.wR);

h3D_TraceP0 = animatedline(ax3D, 'Color', [1 1 0], 'LineWidth', 2, 'DisplayName', strTractorC);     
h3D_TraceH  = animatedline(ax3D, 'Color', [1 1 1], 'LineWidth', 1.5, 'DisplayName', strHitch);   
h3D_TraceP1 = animatedline(ax3D, 'Color', [1 0.5 0], 'LineWidth', 2, 'DisplayName', strTrailerC);     

h3D_TraceFL = animatedline(ax3D, 'Color', 'c', 'LineWidth', 1, 'LineStyle', '-', 'DisplayName', strTracL);
h3D_TraceFR = animatedline(ax3D, 'Color', 'm', 'LineWidth', 1, 'LineStyle', '-', 'DisplayName', strTracR);
h3D_TraceRL = animatedline(ax3D, 'Color', 'g', 'LineWidth', 1, 'LineStyle', '--', 'DisplayName', strTrlL);
h3D_TraceRR = animatedline(ax3D, 'Color', 'r', 'LineWidth', 1, 'LineStyle', '--', 'DisplayName', strTrlR);

% [修改] 將圖例移至右上角，避免擋住 OSD
legend(ax3D, 'TextColor', 'w', 'Color', [0.1 0.1 0.1], 'EdgeColor', 'w', 'Location', 'northeast');

hTextSpeed3D = text(ax3D, 0, 0, 5, '0.0 km/h', 'Color', 'y', 'FontSize', 14, ...
                    'FontWeight', 'bold', 'HorizontalAlignment', 'center');

h3D_Tractor = hgtransform('Parent', ax3D);
h3D_Trailer = hgtransform('Parent', ax3D);
[hWheels_Front, hWheels_Rear, hWheels_Trailer] = buildV33Truck(h3D_Tractor, h3D_Trailer, params);

% --- 2D 物件 ---
mapAxes = [axTactical, axGlobal];
h2D_Tractor = gobjects(2,1); h2D_Trailer = gobjects(2,1);
hMap_TraceP0 = gobjects(2,1); hMap_TraceH = gobjects(2,1); hMap_TraceP1 = gobjects(2,1);
hMap_TraceFL = gobjects(2,1); hMap_TraceFR = gobjects(2,1);
hMap_TraceRL = gobjects(2,1); hMap_TraceRR = gobjects(2,1);

for i = 1:2
    ax = mapAxes(i);
    
    if i == 1
        hMap_TraceP0(i) = animatedline(ax, 'Color', [1 1 0], 'LineWidth', 1.5, 'DisplayName', strTractorC);
        hMap_TraceH(i)  = animatedline(ax, 'Color', [1 1 1], 'LineWidth', 1.0, 'DisplayName', strHitch);
        hMap_TraceP1(i) = animatedline(ax, 'Color', [1 0.5 0], 'LineWidth', 1.5, 'DisplayName', strTrailerC);
        
        hMap_TraceFL(i) = animatedline(ax, 'Color', 'c', 'LineWidth', 1, 'LineStyle', '-', 'DisplayName', strTracL);
        hMap_TraceFR(i) = animatedline(ax, 'Color', 'm', 'LineWidth', 1, 'LineStyle', '-', 'DisplayName', strTracR);
        hMap_TraceRL(i) = animatedline(ax, 'Color', 'g', 'LineWidth', 1, 'LineStyle', '--', 'DisplayName', strTrlL);
        hMap_TraceRR(i) = animatedline(ax, 'Color', 'r', 'LineWidth', 1, 'LineStyle', '--', 'DisplayName', strTrlR);
        
        legend(ax, 'TextColor', 'w', 'Color', [0.1 0.1 0.1], 'Location', 'bestoutside');
        hTextSpeed2D = text(ax, 0, 0, '0.0 m/s', 'Color', 'y', 'FontSize', 12, ...
                            'FontWeight', 'bold', 'BackgroundColor', [0 0 0 0.6]);
    end
    
    h2D_Tractor(i) = hgtransform('Parent', ax);
    h2D_Trailer(i) = hgtransform('Parent', ax);
    
    L1 = lenT; W1 = widthT;
    L2 = params.L2; W2 = widthH;
    RearOverhang = 3.5; FrontOverhang = 1.6;
    
    cabLen = 1.8; offset = (params.L1/2 - 1.0) - cabLen/2;
    patch('XData', [offset, offset+cabLen, offset+cabLen, offset], ...
          'YData', [-W1/2, -W1/2, W1/2, W1/2], ...
          'FaceColor', [0.2 0.7 0.3], 'EdgeColor', 'w', 'Parent', h2D_Tractor(i), 'HandleVisibility', 'off');
      
    patch('XData', [L2+FrontOverhang, L2+FrontOverhang, -RearOverhang, -RearOverhang], ...
          'YData', [-W2/2, W2/2, W2/2, -W2/2], ...
          'FaceColor', [0.85 0.5 0.1], 'EdgeColor', 'w', 'Parent', h2D_Trailer(i), 'HandleVisibility', 'off');
      
    plot(ax, 0, 0, 'wo', 'MarkerSize', 4, 'MarkerFaceColor', 'w', 'Parent', h2D_Tractor(i), 'HandleVisibility', 'off'); 

    if i==1
        draw2DTrack_Optimized(ax, refpath.x, refpath.y, refpath.wL, refpath.wR); 
    end
end

hLineErr = animatedline(axErr, 'Color', 'c', 'LineWidth', 1.5);
yline(axErr, 0, 'w:', 'LineWidth', 1);
hTextErr = text(axErr, 0, 0, 'Init', 'Color', 'w', 'FontSize', 12, 'FontWeight', 'bold', ...
                'BackgroundColor', [0 0 0 0.6], 'EdgeColor', 'c', 'Margin', 5);
hLineArt = animatedline(axArt, 'Color', 'y', 'LineWidth', 1.5);
hLineSteer = animatedline(axSteer, 'Color', 'm', 'LineWidth', 1.5);


%% 4. 動畫迴圈
fprintf('Starting V4.5 Animation (Fix Trailer & Legend)...\n');

dt_sim = simData.time(2) - simData.time(1);
tStart = tic;
VIEW_RADIUS = 45; 
W1_half = params.tractor_width / 2; W2_half = params.trailer_width / 2;
RearOverhang = 3.5; 

strOSD = {'Time: 0.0s', 'Speed: 0.0 km/h', 'Acc: 0.00 m/s^2', sprintf('Max Spd: %.1f', max_speed_kmh)};
hOSD = text(ax3D, 0.02, 0.9, strOSD, 'Units', 'normalized', 'Color', 'g', 'FontSize', 14, ...
            'FontName', 'Consolas', 'BackgroundColor', [0 0 0 0.5], 'EdgeColor', 'w');

while ishandle(fig3D) && ishandle(figMap)
    tCurrent = toc(tStart) * PLAYBACK_SPEED;
    if tCurrent > simData.time(end), break; end
    k = round(tCurrent / dt_sim) + 1;
    if k > N, k = N; end
    
    x0 = simData.X0(k); y0 = simData.Y0(k); th0 = simData.Theta0(k);
    xh = simData.XH(k); yh = simData.YH(k);
    x1 = simData.X1(k); y1 = simData.Y1(k); th1 = simData.Theta1(k);
    v_curr = simData.v_cmd(k); w_curr = simData.w_cmd(k);
    current_ye = simData.ye(k);
    acc_curr = simData.acc(k);
    
    steer_angle = 0; if abs(v_curr) > 0.1, steer_angle = atan(params.L1 * w_curr / v_curr); end
    steer_angle = max(-0.78, min(0.78, steer_angle));
    
    set(h3D_Tractor, 'Matrix', makehgtform('translate', [x0, y0, 0.5], 'zrotate', th0));
    set(h3D_Trailer, 'Matrix', makehgtform('translate', [x1, y1, 0.5], 'zrotate', th1));
    set(hGroundGrp, 'Matrix', makehgtform('translate', [x0, y0, 0])); 
    
    for w = 1:length(hWheels_Front)
        originPos = hWheels_Front(w).UserData.Origin;
        M_steer = makehgtform('translate', originPos) * makehgtform('zrotate', steer_angle);
        set(hWheels_Front(w), 'Matrix', M_steer);
    end
    
    addpoints(h3D_TraceP0, x0, y0, 0.05); addpoints(h3D_TraceH,  xh, yh, 0.05); addpoints(h3D_TraceP1, x1, y1, 0.05);
    
    FrontPos = params.L1 + 1.0; 
    FL_x = x0 + FrontPos*cos(th0) - W1_half*sin(th0); FL_y = y0 + FrontPos*sin(th0) + W1_half*cos(th0);
    FR_x = x0 + FrontPos*cos(th0) + W1_half*sin(th0); FR_y = y0 + FrontPos*sin(th0) - W1_half*cos(th0);
    RL_x = x1 - RearOverhang*cos(th1) - W2_half*sin(th1); RL_y = y1 - RearOverhang*sin(th1) + W2_half*cos(th1);
    RR_x = x1 - RearOverhang*cos(th1) + W2_half*sin(th1); RR_y = y1 - RearOverhang*sin(th1) - W2_half*cos(th1);
    
    addpoints(h3D_TraceFL, FL_x, FL_y, 0.02); addpoints(h3D_TraceFR, FR_x, FR_y, 0.02);
    addpoints(h3D_TraceRL, RL_x, RL_y, 0.02); addpoints(h3D_TraceRR, RR_x, RR_y, 0.02);
    
    xlim(ax3D, [x0 - VIEW_RADIUS, x0 + VIEW_RADIUS]); ylim(ax3D, [y0 - VIEW_RADIUS, y0 + VIEW_RADIUS]);
    camDist = 40; camH = 22; offsetAngle = deg2rad(25); 
    campos(ax3D, [x0 - camDist * cos(th0+offsetAngle), y0 - camDist * sin(th0+offsetAngle), camH]);
    camtarget(ax3D, [x0 + 5*cos(th0), y0 + 5*sin(th0), 1.0]);
    
    set(hTextSpeed3D, 'Position', [x0, y0, 6], 'String', sprintf('%.1f km/h', v_curr*3.6));
    
    for i = 1:2
        set(h2D_Tractor(i), 'Matrix', makehgtform('translate', [x0, y0, 0], 'zrotate', th0));
        set(h2D_Trailer(i), 'Matrix', makehgtform('translate', [x1, y1, 0], 'zrotate', th1));
        
        if i == 1 
            addpoints(hMap_TraceP0(i), x0, y0); addpoints(hMap_TraceH(i),  xh, yh); addpoints(hMap_TraceP1(i), x1, y1);
            addpoints(hMap_TraceFL(i), FL_x, FL_y); addpoints(hMap_TraceFR(i), FR_x, FR_y);
            addpoints(hMap_TraceRL(i), RL_x, RL_y); addpoints(hMap_TraceRR(i), RR_x, RR_y);
            
            xlim(axTactical, [x0-50, x0+50]); ylim(axTactical, [y0-50, y0+50]);
            set(hTextSpeed2D, 'Position', [x0-45, y0+45, 0], 'String', sprintf('SPD: %.2f m/s', v_curr));
        end
    end
    
    addpoints(hLineErr, tCurrent, current_ye);
    addpoints(hLineArt, tCurrent, rad2deg(th0-th1));
    addpoints(hLineSteer, tCurrent, rad2deg(steer_angle));
    
    txt_pos_y = current_ye + 0.1 * sign(current_ye); 
    if abs(current_ye) < 0.05, txt_pos_y = 0.1; end 
    set(hTextErr, 'Position', [tCurrent, txt_pos_y, 0], 'String', sprintf('Err: %.4f m', current_ye));
    
    strOSD = {
        sprintf('Time: %.1f s', tCurrent), ...
        sprintf('Speed: %.1f km/h', v_curr*3.6), ...
        sprintf('Acc: %+.2f m/s^2', acc_curr), ...
        sprintf('Max Spd: %.1f km/h', max_speed_kmh)
    };
    set(hOSD, 'String', strOSD);
    
    if tCurrent > 10
        x_min = tCurrent - 10; x_max = tCurrent;
        xlim(axErr, [x_min, x_max]); xlim(axArt, [x_min, x_max]); xlim(axSteer, [x_min, x_max]);
        set(hTextErr, 'Position', [tCurrent - 2, txt_pos_y, 0]);
    end
    
    drawnow limitrate;
end

ax3D.CameraViewAngleMode = 'auto'; ax3D.CameraTargetMode = 'auto'; ax3D.CameraPositionMode = 'auto';
zoom(fig3D, 'on'); pan(fig3D, 'on'); rotate3d(fig3D, 'on');
zoom(figMap, 'on'); pan(figMap, 'on'); zoom(figData, 'on'); pan(figData, 'on');


%% =========================================================================
%  輔助函數
% =========================================================================

function draw2DTrack_Optimized(ax, x, y, wL, wR)
    diff_x = diff(x); diff_y = diff(y);
    dist = [0; cumsum(sqrt(diff_x.^2 + diff_y.^2))];
    
    visual_stride_dist = 0.5; 
    idx_vis = [1]; last_dist = 0;
    for k = 2:length(dist)
        if dist(k) - last_dist >= visual_stride_dist
            idx_vis = [idx_vis; k]; last_dist = dist(k);
        end
    end
    if idx_vis(end) ~= length(x), idx_vis = [idx_vis; length(x)]; end
    
    x_vis = x(idx_vis); y_vis = y(idx_vis);
    wL_vis = wL(idx_vis); wR_vis = wR(idx_vis);
    
    dx = gradient(x_vis); dy = gradient(y_vis); len = sqrt(dx.^2 + dy.^2);
    nx = -dy ./ len; ny = dx ./ len;
    
    X_L = x_vis + nx .* wL_vis; Y_L = y_vis + ny .* wL_vis;
    X_R = x_vis - nx .* wR_vis; Y_R = y_vis - ny .* wR_vis;
    
    dist_sq = diff(x_vis).^2 + diff(y_vis).^2;
    jump_idx = find(dist_sq > 100);
    if ~isempty(jump_idx)
        X_L(jump_idx+1) = NaN; Y_L(jump_idx+1) = NaN; 
        X_R(jump_idx+1) = NaN; Y_R(jump_idx+1) = NaN;
    end
    
    plot(ax, X_L, Y_L, '-', 'LineWidth', 0.5, 'Color', [0.6 0.6 0.6 0.6], 'HandleVisibility', 'off');
    plot(ax, X_R, Y_R, '-', 'LineWidth', 0.5, 'Color', [0.6 0.6 0.6 0.6], 'HandleVisibility', 'off');
end

function annotateTrackWidth_Smart(ax, x, y, wL, wR)
    diff_x = diff(x); diff_y = diff(y);
    dist = [0; cumsum(sqrt(diff_x.^2 + diff_y.^2))];
    
    MIN_LABEL_SPACING = 30.0; 
    MAX_LABEL_GAP = 150.0;    
    WIDTH_CHANGE_THRESHOLD = 0.5; 
    
    last_text_dist = -MAX_LABEL_GAP; 
    last_displayed_w = -1; 
    
    for k = 1:length(dist)
        dist_since_last = dist(k) - last_text_dist;
        if dist_since_last < MIN_LABEL_SPACING, continue; end
        
        current_w = wL(k) + wR(k);
        is_significant_change = abs(current_w - last_displayed_w) > WIDTH_CHANGE_THRESHOLD;
        is_long_gap = dist_since_last >= MAX_LABEL_GAP;
        
        if k == 1 || is_significant_change || is_long_gap
            text(ax, x(k)+2, y(k), sprintf('W=%.1fm', current_w), ...
                 'Color', 'y', 'FontSize', 8, 'Clipping', 'on', ...
                 'HorizontalAlignment', 'left');
            last_text_dist = dist(k);
            last_displayed_w = current_w;
        end
    end
end

function draw3DTrack(ax, x, y, wL, wR)
    step = 5; 
    x = x(1:step:end); y = y(1:step:end); wL = wL(1:step:end); wR = wR(1:step:end);
    dx = gradient(x); dy = gradient(y); len = sqrt(dx.^2 + dy.^2);
    nx = -dy ./ len; ny = dx ./ len; 
    dist_sq = diff(x).^2 + diff(y).^2;
    jump_idx = find(dist_sq > 100); 
    if ~isempty(jump_idx), x(jump_idx+1) = NaN; y(jump_idx+1) = NaN; end
    X_L = x + nx .* wL; Y_L = y + ny .* wL; X_R = x - nx .* wR; Y_R = y - ny .* wR;
    Z_level = -0.04;
    X_strip = [X_L'; X_R']; Y_strip = [Y_L'; Y_R']; Z_strip = ones(size(X_strip)) * Z_level;
    surface(X_strip, Y_strip, Z_strip, 'Parent', ax, 'FaceColor', [0.3 0.3 0.35], 'EdgeColor', 'none', ... 
            'AmbientStrength', 0.6, 'DiffuseStrength', 0.8, 'HandleVisibility', 'off');
end

function [hFront, hRear, hTrailer] = buildV33Truck(hTractorGrp, hTrailerGrp, params)
    colCab = [0.2 0.7 0.3]; colBox = [0.85 0.5 0.1]; 
    colChas = [0.2 0.2 0.2]; colTire = [0.35 0.35 0.35]; colRim = [0.8 0.8 0.8];
    colGlass = [0.2 0.3 0.5]; colBumper = [0.1 0.1 0.1];
    
    if isfield(params, 'tractor_length'), L1 = params.tractor_length; else, L1 = params.L1+1.5; end
    if isfield(params, 'trailer_length'), L2 = params.trailer_length; else, L2 = params.L2+2.0; end
    W1 = params.tractor_width; WheelBase = params.L1; 
    
    chassisLen = WheelBase + 1.5; 
    createBox(hTractorGrp, [chassisLen, W1*0.4, 0.4], [WheelBase/2, 0, 0.4], colChas);
    cabLen = 1.6; cabH = 2.4; cabX = WheelBase; cabZ = 0.4 + 0.2 + cabH/2 + 0.15; 
    createBox(hTractorGrp, [cabLen, W1, cabH], [cabX, 0, cabZ], colCab);
    createBox(hTractorGrp, [0.05, W1-0.2, 1.2], [cabX+cabLen/2, 0, cabZ+0.3], colGlass); 
    createBox(hTractorGrp, [0.2, W1, 0.4], [cabX+cabLen/2, 0, 0.5], colBumper); 
    
    wheelR = 0.52; wheelW = 0.32;
    hFront = gobjects(0); hRear = gobjects(0);
    posF1 = [WheelBase, W1/2-0.1, wheelR]; posF2 = [WheelBase, -W1/2+0.1, wheelR];
    hFront(1) = createCorrectWheel(hTractorGrp, posF1, wheelR, wheelW, colTire, colRim);
    hFront(2) = createCorrectWheel(hTractorGrp, posF2, wheelR, wheelW, colTire, colRim);
    hFront(1).UserData.Origin = posF1; hFront(2).UserData.Origin = posF2;
    hRear(1) = createCorrectWheel(hTractorGrp, [0, W1/2-0.1, wheelR], wheelR, wheelW, colTire, colRim);
    hRear(2) = createCorrectWheel(hTractorGrp, [0, -W1/2+0.1, wheelR], wheelR, wheelW, colTire, colRim);
    M1 = params.M1;
    createCylinder(hTractorGrp, 0.45, 0.15, [-M1, 0, 0.6], [0.1 0.1 0.1]);

    W2 = params.trailer_width;
    Overhang_Front = 1.6; Overhang_Rear = 3.5; 
    Z_Base = 0.85; FrameH = 0.2; 
    
    % [修正] 重新計算貨櫃包圍盒與車輪位置 (相對於後軸中心)
    TrailerActualLength = L2 + Overhang_Front + Overhang_Rear;
    BoxCenter_X = (L2 + Overhang_Front - Overhang_Rear) / 2;
    
    createBox(hTrailerGrp, [TrailerActualLength, W2-0.6, FrameH], [BoxCenter_X, 0, Z_Base + FrameH/2], colChas);
    BoxH = 2.8; BoxZ_Center = Z_Base + FrameH + BoxH/2;
    createBox(hTrailerGrp, [TrailerActualLength, W2, BoxH], [BoxCenter_X, 0, BoxZ_Center], colBox);
    createCylinder(hTrailerGrp, 0.15, 0.25, [L2, 0, 0.6], [0.15 0.15 0.15]);
    hTrailer = gobjects(0);
    axle_spacing = 1.1; 
    for i = -1:1 
        wx = i * axle_spacing; 
        % 車輪位置修正為相對於後軸中心 (X=0)
        createCorrectWheel(hTrailerGrp, [wx, W2/2-0.1, wheelR], wheelR, wheelW, colTire, colRim); 
        createCorrectWheel(hTrailerGrp, [wx, -W2/2+0.1, wheelR], wheelR, wheelW, colTire, colRim);
    end
end

function hGrp = createCorrectWheel(parent, pos, r, w, colTire, colRim)
    hGrp = hgtransform('Parent', parent);
    set(hGrp, 'Matrix', makehgtform('translate', pos));
    [C_X, C_Y, C_Z] = cylinder(r, 24); X = C_X; Y = (C_Z - 0.5) * w; Z = C_Y;
    surf(X, Y, Z, 'Parent', hGrp, 'FaceColor', colTire, 'EdgeColor', 'none', 'SpecularStrength', 0.5);
    fill3(X(1,:), Y(1,:), Z(1,:), colRim, 'Parent', hGrp, 'EdgeColor', 'none', 'SpecularStrength', 0.6);
    fill3(X(2,:), Y(2,:), Z(2,:), colRim, 'Parent', hGrp, 'EdgeColor', 'none', 'SpecularStrength', 0.6);
end

function createBox(parent, dims, pos, color)
    [v, f] = getBoxVerts(); v = v .* dims; v = v + pos;
    patch('Vertices', v, 'Faces', f, 'FaceColor', color, 'EdgeColor', 'none', 'Parent', parent, 'FaceLighting', 'gouraud');
end

function createCylinder(parent, r, h, pos, color)
    [X, Y, Z] = cylinder(r, 16); Z = Z * h;
    surf(X + pos(1), Y + pos(2), Z + pos(3), 'Parent', parent, 'FaceColor', color, 'EdgeColor', 'none');
end

function [v, f] = getBoxVerts()
    v = [-0.5 -0.5 -0.5; 0.5 -0.5 -0.5; 0.5 0.5 -0.5; -0.5 0.5 -0.5; ...
         -0.5 -0.5  0.5; 0.5 -0.5  0.5; 0.5 0.5  0.5; -0.5 0.5  0.5];
    f = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
end