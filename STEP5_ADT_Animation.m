% =========================================================================
% STEP5_ADT_Animation.m
% 用 Automated Driving Toolbox 的 drivingScenario 畫路面（比自己手刻的灰色
% 色塊路面更好看，有車道線），車輛模型跟跟拍相機則直接沿用
% STEP4_ETS2_Animation_MultiView.m（實驗室\論文1\matlab\beta1-5 那一版）
% 的手刻卡車模型與相機邏輯——不用 ADT 內建 vehicle actor 的空盒子網格。
%
% 設計取捨：ADT 的 drivingScenario 在這裡「只」負責畫路面（road()），沒有
% 註冊任何 vehicle actor，也沒有呼叫 advance()/updatePlots()。車輛姿態
% 完全來自 simulation_results.mat 的 hist.x0/y0/yaw0（拖車頭後軸）與
% hist.x1/y1/yaw1（貨櫃後軸），透過 hgtransform 直接驅動——這跟
% STEP4_ETS2 是同一套做法，好處是拖車頭/貨櫃的幾何原點都定義在後軸上
% （buildV33Truck 裡車輪、車廂座標都相對後軸算），不需要像 ADT vehicle
% actor 那樣還要換算成車身幾何中心。
% =========================================================================

clear; clc; close all;

load('simulation_results.mat', 'results');
hist   = results.hist;
params = results.params;
refpath = results.refpath;
Nsim = numel(hist.x0);

% -------------------------------------------------------------------------
% 母路徑點數太密（30000點）直接餵給 road() 會很重，用弧長等距抽稀到約 2m
% 間距，純粹是路面網格的視覺密度，跟控制/模擬完全無關。
% -------------------------------------------------------------------------
s_full = [0; cumsum(hypot(diff(refpath.x), diff(refpath.y)))];
s_target = (0:2.0:s_full(end))';
road_x = interp1(s_full, refpath.x, s_target);
road_y = interp1(s_full, refpath.y, s_target);

scenario = drivingScenario;
road_width = params.lane_width * 3;   % 涵蓋候選路徑扇形（±1.5個車道寬）
road(scenario, [road_x, road_y, zeros(size(road_x))], road_width, ...
    'Lanes', lanespec(3, 'Width', params.lane_width));

plot(scenario);
fig3D = gcf;
fig3D.Name = 'STEP5: 3D Pursuit View (ADT road + STEP4_ETS2 truck model)';
fig3D.Color = 'w';
fig3D.Position = [80 80 1000 750];

ax3D = gca;
hold(ax3D, 'on'); axis(ax3D, 'equal');
ax3D.Color = 'w'; ax3D.XColor = 'none'; ax3D.YColor = 'none'; ax3D.ZColor = 'none';
camproj(ax3D, 'perspective'); grid(ax3D, 'off');
ax3D.CameraViewAngleMode = 'manual'; ax3D.CameraViewAngle = 40;

% 不用自訂光源/gouraud 陰影——那會讓路面跟車身出現漸層明暗，畫面看起來
% 比較「有氣氛」但道路本身的顏色/邊界反而不夠清楚。改用 MATLAB 預設的
% 平面上色（flat shading，沒有材質光影），顏色本身就是實際顏色，不會
% 因為光源角度被加深或洗白。
lighting(ax3D, 'none');

h3D_TraceP0 = animatedline(ax3D, 'Color', [0.9 0.7 0], 'LineWidth', 2, 'DisplayName', 'Tractor Center');
h3D_TraceH  = animatedline(ax3D, 'Color', [0.3 0.3 0.3], 'LineWidth', 1.5, 'DisplayName', 'Hitch Point');
h3D_TraceP1 = animatedline(ax3D, 'Color', [0.9 0.4 0], 'LineWidth', 2, 'DisplayName', 'Trailer Center');
% 只列這三條軌跡線，不然 ADT road() 畫的路面/車道線圖層也會被自動抓進圖例
% （顯示成 data1/data2/data3 這種沒有意義的名字）。
legend(ax3D, [h3D_TraceP0, h3D_TraceH, h3D_TraceP1], ...
    'TextColor', 'k', 'Color', 'w', 'EdgeColor', 'k', 'Location', 'northeast');

hTextSpeed3D = text(ax3D, 0, 0, 5, '0.0 km/h', 'Color', [0.8 0.2 0], 'FontSize', 14, ...
                    'FontWeight', 'bold', 'HorizontalAlignment', 'center');
strOSD = {'Time: 0.0s', 'Speed: 0.0 km/h'};
hOSD = text(ax3D, 0.02, 0.9, strOSD, 'Units', 'normalized', 'Color', 'k', 'FontSize', 14, ...
            'FontName', 'Consolas', 'BackgroundColor', [1 1 1 0.7], 'EdgeColor', 'k');

h3D_Tractor = hgtransform('Parent', ax3D);
h3D_Trailer = hgtransform('Parent', ax3D);
[hWheels_Front, ~, ~] = buildV33Truck(h3D_Tractor, h3D_Trailer, params);

% -------------------------------------------------------------------------
% 跟拍相機（照抄 STEP4_ETS2_Animation_MultiView.m 的公式）：相機架在拖車頭
% 後方 camDist、高度 camH，偏一個 offsetAngle 讓視角不會死板地正後方；
% 目標點設在拖車頭前方 5m，讓視野自然帶到前方路況。每一幀都重新設一次
% campos/camtarget，所以播放中無法用滑鼠拖曳旋轉視角（曾經試過改成
% 固定相機讓使用者自由旋轉，但調不出滿意的畫面，先維持這個鎖定跟拍版本）。
% -------------------------------------------------------------------------
camDist = 40; camH = 22; offsetAngle = deg2rad(25);
VIEW_RADIUS = 45;

playback_speed = 1.0;   % 1.0 = 正常速度，跟 STEP4 同一套慣例

for k = 1:Nsim
    x0 = hist.x0(k); y0 = hist.y0(k); th0 = hist.yaw0(k);
    x1 = hist.x1(k); y1 = hist.y1(k); th1 = hist.yaw1(k);
    xh = hist.xh(k); yh = hist.yh(k);
    v_curr = hist.v(k);

    set(h3D_Tractor, 'Matrix', makehgtform('translate', [x0, y0, 0.5], 'zrotate', th0));
    set(h3D_Trailer, 'Matrix', makehgtform('translate', [x1, y1, 0.5], 'zrotate', th1));

    steer_angle = 0;
    if abs(v_curr) > 0.1
        steer_angle = hist.delta(k);
    end
    for w = 1:numel(hWheels_Front)
        originPos = hWheels_Front(w).UserData.Origin;
        M_steer = makehgtform('translate', originPos) * makehgtform('zrotate', steer_angle);
        set(hWheels_Front(w), 'Matrix', M_steer);
    end

    addpoints(h3D_TraceP0, x0, y0, 0.05);
    addpoints(h3D_TraceH,  xh, yh, 0.05);
    addpoints(h3D_TraceP1, x1, y1, 0.05);

    xlim(ax3D, [x0 - VIEW_RADIUS, x0 + VIEW_RADIUS]);
    ylim(ax3D, [y0 - VIEW_RADIUS, y0 + VIEW_RADIUS]);
    campos(ax3D, [x0 - camDist * cos(th0 + offsetAngle), y0 - camDist * sin(th0 + offsetAngle), camH]);
    camtarget(ax3D, [x0 + 5 * cos(th0), y0 + 5 * sin(th0), 1.0]);
    set(hTextSpeed3D, 'Position', [x0, y0, 6], 'String', sprintf('%.1f km/h', v_curr * 3.6));
    set(hOSD, 'String', {sprintf('Time: %.1f s', results.ts(k)), sprintf('Speed: %.1f km/h', v_curr * 3.6)});

    drawnow limitrate;
    pause(params.Ts / playback_speed);
end

fprintf('動畫播放完畢，共 %d 幀。\n', Nsim);

% =========================================================================
%  車輛 3D 模型輔助函式（照搬 STEP4_ETS2_Animation_MultiView.m，未改動幾何
%  邏輯——只有這幾個函式，路面/地圖/儀表板相關的函式沒有搬過來）
% =========================================================================

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
