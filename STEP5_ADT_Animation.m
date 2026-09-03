% =========================================================================
% STEP5_ADT_Animation.m
% 用 Automated Driving Toolbox 的 drivingScenario 播放 main_pure_pursuit_sim.m
% 算好的聯結車（tractor-trailer）模擬結果，取代 STEP4 自己畫的 2D 圖形動畫。
%
% 這一版只做「合成跑道 + 既有模擬結果」的視覺化，還沒接真實地圖（OSM）。
% 拖車頭與貨櫃的姿態完全來自 simulation_results.mat 裡逐步記錄的
% hist.x0/y0/yaw0（拖車頭後軸）與 hist.x1/y1/yaw1（貨櫃後軸）——也就是說，
% 這裡沒有用 drivingScenario 自己的 trajectory()/advance() 物理積分，
% 而是每一幀手動把兩顆 vehicle actor 的 Position/Yaw 設成我們自己模擬算出
% 來的值，drivingScenario 純粹當「渲染引擎」用，確保動畫呈現的鉸接角行為
% 跟 main_pure_pursuit_sim.m 的運動學模型完全一致（不會被 toolbox 自己的
% 路徑擬合悄悄改掉貨櫃甩尾的樣子）。
% =========================================================================

clear; clc; close all;

load('simulation_results.mat', 'results');
hist   = results.hist;
params = results.params;
refpath = results.refpath;
Nsim = numel(hist.x0);

% -------------------------------------------------------------------------
% Vehicle actor 的 Position 是「車身幾何中心」，不是我們模擬狀態用的後軸。
% 换算：車身沿 heading 方向從後軸往前 overhang、往後 rear_overhang，
% 中心相對後軸的偏移量 = (軸距 + 前懸 - 後懸) / 2。
% 拖車頭：offset = (L1 + tractor_front_overhang - tractor_rear_overhang)/2
% 貨櫃  ：offset = (L2 + trailer_front_overhang - trailer_rear_overhang)/2
%         （貨櫃的「前懸」量的是鉸接點到貨櫃前緣，L2 是鉸接點到貨櫃後軸，
%          兩者相加才是後軸到貨櫃前緣的距離，公式跟拖車頭同一套邏輯）
% -------------------------------------------------------------------------
tractor_center_offset = (params.L1 + params.tractor_front_overhang - params.tractor_rear_overhang) / 2;
trailer_center_offset = (params.L2 + params.trailer_front_overhang - params.trailer_rear_overhang) / 2;

tractor_cx = hist.x0 + tractor_center_offset * cos(hist.yaw0);
tractor_cy = hist.y0 + tractor_center_offset * sin(hist.yaw0);
trailer_cx = hist.x1 + trailer_center_offset * cos(hist.yaw1);
trailer_cy = hist.y1 + trailer_center_offset * sin(hist.yaw1);

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

tractor = vehicle(scenario, 'ClassID', 2, ...
    'Length', params.tractor_length, 'Width', params.tractor_width, 'Height', 3.0, ...
    'Position', [tractor_cx(1), tractor_cy(1), 0], ...
    'Yaw', rad2deg(hist.yaw0(1)), 'Name', 'Tractor');

trailer = vehicle(scenario, 'ClassID', 2, ...
    'Length', params.trailer_length, 'Width', params.trailer_width, 'Height', 3.0, ...
    'Position', [trailer_cx(1), trailer_cy(1), 0], ...
    'Yaw', rad2deg(hist.yaw1(1)), 'Name', 'Trailer');

% -------------------------------------------------------------------------
% 兩個視角同時開：鳥瞰整條跑道 + chasePlot 跟拍拖車頭（3D 網格車輛模型）。
% chasePlot 會自己接管所在 figure 的版面配置，跟 subplot 併排會互相打架，
% 所以兩個視角各開一個獨立視窗，而不是塞進同一個 figure 的 subplot。
% -------------------------------------------------------------------------
fig1 = figure('Name', '鳥瞰視角（Bird''s-Eye）', 'Position', [80 100 650 650]);
plot(scenario);
title('鳥瞰視角（Bird''s-Eye）');

fig2 = figure('Name', '跟拍視角（Chase View，拖車頭）', 'Position', [760 100 650 650]);
ax2 = axes('Parent', fig2);
chasePlot(tractor, 'Centerline', 'on', 'Meshes', 'on', 'ViewHeight', 6, 'ViewPitch', -15, 'Parent', ax2);

playback_speed = 1.0;   % 1.0 = 正常速度，跟 STEP4 同一套慣例

for k = 1:Nsim
    tractor.Position = [tractor_cx(k), tractor_cy(k), 0];
    tractor.Yaw = rad2deg(hist.yaw0(k));
    trailer.Position = [trailer_cx(k), trailer_cy(k), 0];
    trailer.Yaw = rad2deg(hist.yaw1(k));

    updatePlots(scenario);
    drawnow limitrate;
    pause(params.Ts / playback_speed);
end

fprintf('動畫播放完畢，共 %d 幀。\n', Nsim);
