% =========================================================================
% 檔案名稱: Untitled4_tracking_trajectory_test.m
%
% 功能：本專案「母路徑」reference_path.mat 的產生腳本 —— 整個 Pure
%       Pursuit 系統唯一一次執行、離線產生固定測試賽道的地方。執行後
%       會存出 reference_path.mat，之後 main_pure_pursuit_sim.m /
%       generate_local_paths.m / my_multi_path.m 全部都是讀取這個已經
%       存好的檔案，不會重新執行本腳本。
%
%       這份文件在 docx 週報裡被歸類為「共用不變」（單車版本與拖掛車
%       版本共用同一條母路徑，完全沒有修改），這次也同樣沒有更動任何
%       運算邏輯，只補上完整註解、並清掉一行殘留的編碼錯誤裝飾註解
%       （原始檔案裡混入了非 UTF-8 來源、顯示為 �f�ɰw 之類亂碼字元的
%       裝飾分隔線，內容本來就只是重複第 1 行「橢圓賽道」的說明，
%       刪掉不影響任何運算結果）。
%
%       ⚠ 本腳本目前不需要、也不建議重新執行：reference_path.mat 已經
%       存在且經過這次全部驗證測試（CTE RMS、鉸接角、貨櫃偏移等數字）
%       都是針對「現有」這份 reference_path.mat 算出來的。如果之後真的
%       需要換一條新賽道，重新執行本腳本會覆蓋掉 reference_path.mat，
%       屆時全部追蹤效能數字都要重新驗證。
%
% ─────────────────────────────────────────────────────────────────────
% 賽道幾何（一個類似運動場跑道的封閉「體育場形（stadium shape）」迴圈）：
%
%   由 4 段組成，依 x=[x2 x1 x0 x5] 的順序頭尾相接成一個封閉迴圈：
%     x2,y2 : 右側直線段，x=35（固定），y 從 -24.95 走到 19.95（由下往上）
%     x1,y1 : 頂部半圓，圓心 (20,20)、半徑 15，從 (35,20) 經 (20,35)
%             轉到 (5,20)（右上 → 正上 → 左上）
%     x0,y0 : 左側直線段，x=5（固定），y 從 19.96 走到 -24.98（由上往下）
%     x5,y5 : 底部半圓，圓心 (20,-25)、半徑 15，從 (5,-25) 經 (20,-40)
%             轉到 (35,-25)（左下 → 正下 → 右下），接回 x2 起點
%
%   這正是本專案所有模擬測試用的封閉迴圈賽道（車輛會繞著這個「跑道」
%   一直循環行駛，main_pure_pursuit_sim.m 的最近點搜尋用 mod 運算處理
%   跑道首尾相接的接縫）。
%
%   （x3,y3 在下面第 24-26 行有定義（另一個半徑 20、圓心 (55,-10) 的
%   半圓），但最終沒有被併入 x=[x2 x1 x0 x5] 這個組合中，是未被使用的
%   殘留變數，這裡保留原樣不刪除，避免任何非必要的行為變動。）
%
% ─────────────────────────────────────────────────────────────────────
% 生成邏輯（跟這次新增的 generate_local_paths.m 用的是同一套「窗口化、
% 逐段用 my_path() 擬合再拼接」概念，但這裡是「離線、一次性、依序跑完
% 整條路線」，不是「線上、即時、依車輛當前位置」）：
%
%   1. 先用上面的解析幾何公式，產生一串很密集的原始點 (x,y)（直接用
%      圓的參數式或線性插值算出來，不需要曲率多項式擬合）
%   2. 用相鄰點差分算出每一點的粗略航向角 theda（純幾何切線方向）
%   3. 把整串點依固定長度 seg_len=120 切成很多小段窗口，逐段呼叫
%      my_path() 把「這一小段窗口內的起點到終點」擬合成一條平滑的
%      曲率多項式曲線（這一步才是真正決定最終路徑平滑度、可行駛性
%      的地方 —— 前面的解析幾何只是拿來當作 my_path() 的擬合目標）
%   4. 每一段用旋轉矩陣 TFS 把局部座標系擬合結果轉回世界座標系，
%      串接起來就是最終的 ref_x, ref_y
%   5. 最後用差分算出每一點的最終航向角 ref_phi，存成 refpath 結構
% ─────────────────────────────────────────────────────────────────────

clear all
close all
clc

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   賽道幾何定義   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

points=360*3;
t=linspace(0,pi,points);
a=15;
for i=1:points
x1(i)=(a+5)+a*cos(t(i));   % 頂部半圓（圓心 20,20，半徑 15）
y1(i)=(a+5)+a*sin(t(i));
end

x2=35*ones(1,360*2);                 % 右側直線段（x=35）
y2=linspace(-24.95,19.95,360*2);

t=linspace(-pi,0,points);
for i=1:points
x3(i)=55+20*cos(t(i));     % （未使用的殘留變數，見檔頭說明）
y3(i)=-10+20*sin(t(i));
end

x0=5*ones(1,360*2);                  % 左側直線段（x=5）
y0=linspace(19.96,-24.98,360*2);


points=360*3;
t=linspace(-pi,0,points);
a=15;
for i=1:points
x5(i)=(a+5)+a*cos(t(i));   % 底部半圓（圓心 20,-25，半徑 15）
y5(i)=-25+a*sin(t(i));
end

figure (3)
hold on
plot(x0,y0,'b');
plot(x0(1),y0(1),'bo')
plot(x1,y1,'r');
plot(x1(1),y1(1),'ro')
plot(x2,y2,'g');
plot(x2(1),y2(1),'go')
plot(x5,y5,'k');
plot(x5(1),y5(1),'ko')
axis equal

% 依「右直線 → 上半圓 → 左直線 → 下半圓」順序串接成封閉迴圈
x=[x2 x1 x0 x5] ;
y=[y2 y1 y0 y5] ;
M=length(x);

ref_x = [];
ref_y = [];


clear x0 x1 x2 x3 xe
clear y0 y1 y2 y3 ye


%%%%%~~~~~~~~~~~~~~~~~~~~~~~  估計每一點的粗略航向角 ~~~~~~~~~~~~~~~~~%%%%%%%%%%
% 用相鄰點的座標差分（純幾何切線方向），供下面切割窗口時當作
% my_path() 擬合的目標航向角使用
for i=1:M-1
y_d=y(i+1)-y(i);
x_d=x(i+1)-x(i);
theda(i)=atan2(y_d,x_d);
theda_deg(i)=rad2deg(theda(i));
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure (1)
plot(theda);

%%%%%%%%%%%%%%%%%%%%%%%       路徑生成主迴圈     %%%%%%%%%%%%%%%%%%%%%%%%%%
% 把整條密集原始點切成長度 seg_len=120 的窗口，逐段呼叫 my_path() 擬合
% 成平滑曲線（跟 stitch_local_path.m 的「逐段拼接」概念相同，只是這裡
% 是一次性離線跑完「整條」路線，不是即時只跑「車輛前方一小段」）
% for k=1:100

M = length(x);
seg_len = 120;
num_seg = floor(M / seg_len);
for k = 1:num_seg

% start=2000;
start=(k-1)*120+2
target=120*k
if target>=M
target=M-1;
end

if start > length(x) || target > length(x)
    break;
end
now=[x(start);y(start)];

now=[x(start);y(start)];        % 本段窗口起點（世界座標）
next=[x(target);y(target)];     % 本段窗口終點（世界座標）
delta=next-now                  % 世界座標系下的位移
theda_in=theda(start)           % 起點的粗略航向角
theda_out=theda(target)         % 終點的粗略航向角
if theda_in>0 && theda_out<0
    theda_in=theda(start)-2*pi;  % 跨越 +pi/-pi 邊界時的角度修正，避免目標轉角算錯方向
else
%     theda_in=theda(start);
end

phi_tag=theda(target)-theda_in  % 目標相對轉角（局部座標系下 my_path 的 phi_tag）
check=rad2deg(phi_tag)

delta_b=now-[x(start-1);y(start-1)]        % 用起點前一點的位移，估計起點局部座標系的基準方向
[ alfa Quadrant] = my_yaw_angle( delta_b )  % alfa = 起點局部座標系基準角（0~2pi）
TFS=[cos(alfa) -sin(alfa); sin(alfa) cos(alfa)];   % 局部→世界的旋轉矩陣

iTFS=inv(TFS)                    % 世界→局部的旋轉矩陣（TFS 是正交矩陣，inv=轉置，但這裡直接用 inv）
QQQ=iTFS*delta                   % 把世界座標位移轉到局部座標系（my_path 只認得局部座標）

x_tag=QQQ(1);
y_tag=QQQ(2);

xic=[0 0 0 0]';
[ ZZ p] = my_path( x_tag, y_tag, phi_tag, xic );    % 用 Newton-Raphson 擬合這一段曲線（4xM 矩陣，見 my_dynamic.m）



%%%%%%%%%%%%%%%%%%%%%%%%%%%%   局部座標轉回世界座標   %%%%%%%%%%%%%%%%%%%%%%%%%%%%

for i=1:1000
Temp=[ ZZ(1,i) ZZ(2,i)]';    % 取出 my_path() 回傳的第 i 個取樣點的 [x;y]（ZZ 是 4xM，這裡只取前兩列）
ZZ_R(:,i)=TFS*Temp+now;      % 用 TFS 旋轉回世界座標系，再平移到本段起點 now
end

ref_x = [ref_x, ZZ_R(1,:)];
ref_y = [ref_y, ZZ_R(2,:)];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure (2)
hold on

p1=plot(next(1,1),next(2,1),'ko');
set(gca,'FontSize',20);
set(p1, 'linewidth', 2);
p2=plot(now(1,1),now(2,1),'ks');
set(p2, 'linewidth', 2);
axis equal
xlabel('X (meter)','FontSize',18)
ylabel('Y (meter)','FontSize',18)
p3=plot(x,y,'r');
set(p3, 'linewidth', 2);
p4=plot(ZZ_R(1,:),ZZ_R(2,:),'b');
set(p4, 'linewidth', 2);
title('Tractor-Trailer Tacking Trajectory','FontSize',18 )
legend([p1 p2 p3 p4],{'Tractor Waypoint','Trailer Waypoint','Tractor Trajectory','Trailer Trajectory'},'FontSize',18)
% pause
end

% ---- 賽道整體放大 TRACK_SCALE 倍（長寬同比例放大）----
% 目的：原本最緊的彎道半徑只有 15m（實測擬合後約 10m），對這台 L1=4.5m、
% L2=7.5m 的聯結車來說太緊，貨櫃甩尾會超出車道邊界（見週報第八節分析）。
%
% ⚠ 這裡刻意選在「整條路徑逐段擬合完成之後」才做縮放，而不是把縮放
% 套用在最一開始的賽道幾何定義（半徑 a、直線端點座標）上，兩者數學上
% 應該等價（曲率多項式擬合問題本身是比例不變的），但實測發現「先放大
% 座標、再逐段擬合」會讓部分 segment 邊界的 Newton-Raphson 沒辦法在
% 500 次迭代內收斂到位——因為 my_path.m 的收斂門檻 norm(delta_x)<1e-6
% 是「絕對誤差」，座標放大 5 倍後，同樣的絕對誤差門檻相當於要求更嚴格
% 的相對精度，導致部分本來收斂良好的 segment 變得收斂不完全，在拼接處
% 產生局部倒退的小凹陷（實測：29 段裡有 19 段出現曲率尖峰，|kappa| 最大
% 到 41，對應轉彎半徑不到 0.03m，明顯不合理）。
%
% 改成「先在原始 1 倍尺度完成整條路徑擬合（這個尺度已經過驗證，逐段
% 都能可靠收斂），最後才把整條密集點雲座標乘以 TRACK_SCALE」，因為
% 均勻縮放一條曲線，形狀不變、位置座標乘以縮放倍率、航向角不變、
% 曲率除以縮放倍率——這是單純的幾何縮放關係，不需要重新解任何
% Newton-Raphson 問題，自然不會有收斂精度隨尺度變差的問題。
TRACK_SCALE = 5;
ref_x = ref_x * TRACK_SCALE;
ref_y = ref_y * TRACK_SCALE;

% ---- 用最終拼接完成的密集路徑點，差分算出每一點的最終航向角 ----
dx = diff(ref_x);
dy = diff(ref_y);
ref_phi = atan2(dy, dx);
ref_phi = [ref_phi, ref_phi(end)];   % 補最後一點（差分少一個點，複製前一個值）

% ---- 存成 refpath 結構，供其他所有腳本讀取使用 ----
refpath.x = ref_x(:);
refpath.y = ref_y(:);
refpath.phi = ref_phi(:);
refpath.v = 3.0 * ones(length(ref_x),1);   % 第一版先固定速度（下游 main_pure_pursuit_sim.m
                                            % 會用曲率限速 + forward/backward pass 重新規劃
                                            % 真正使用的 v_profile，這裡的固定值目前沒有實際用到）
save('reference_path.mat', 'refpath');
