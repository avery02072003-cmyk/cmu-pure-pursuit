% =========================================================================
% 檔案名稱: my_dynamic.m
%
% 功能：my_path.m 用 Newton-Raphson 解出收斂的曲率多項式係數 p=[b,c,d,s_f]
%       之後，本函式負責把「這組係數代表的完整路徑」用前向歐拉積分
%       （Forward Euler）展開成 M=1000 個密集取樣點，這才是真正拿去畫圖、
%       拿去給下游（my_multi_path.m / stitch_local_path.m）當作候選路徑
%       幾何形狀的資料。
%
%       my_endpoint.m 只算「終點」一個狀態（給 Newton-Raphson 檢查收斂用），
%       本函式則是把「起點到終點之間的每一步」都算出來（給實際使用路徑
%       的下游模組用）—— 兩者算的是同一條曲線，只是一個只看頭尾、
%       一個看全程。
%
% 積分的微分方程（狀態向量 x=[x座標; y座標; 航向角theta; 曲率kappa]）：
%       dx/ds     = cos(theta(s))
%       dy/ds     = sin(theta(s))
%       dtheta/ds = kappa(s) = b*s + c*s^2 + d*s^3        （曲率的定義）
%       dkappa/ds = b + 2*c*s + 3*d*s^2                    （kappa(s) 對 s 的導數）
%
%       前兩式是「沿切線方向前進」的標準運動學積分；後兩式則是直接把
%       kappa(s)、kappa'(s) 的解析公式當成 ODE 右手邊，用同一套歐拉積分
%       跟 x,y 一起往前推進（好處是整條路徑上任一點的曲率值也一併算出來，
%       不用另外呼叫多項式公式）。
%
%       用前向歐拉積分（x(k+1) = x(k) + f(x(k))*Ds）而不是解析解，原因跟
%       my_fk_gk.m 相同：cos(theta(s))、sin(theta(s)) 沒有初等函數解析積分，
%       只能數值方法。這裡用最簡單的歐拉法而非 Simpson 法，因為
%       M=1000 取樣點已經夠密，累積誤差可忽略，且逐步遞推同時能存下
%       每一步的完整狀態（辛普森法只求「單一個」定積分結果，做不到這件事）。
%
% ⚠ 回傳格式很重要（曾經是本專案一個真實的 bug 來源，這裡特別說明）：
%       輸出 output_args 是 4xM 矩陣 —— 「列」是 [x;y;theta;kappa] 四個
%       狀態分量，「欄」才是沿弧長的 M 個取樣點。也就是說：
%           output_args(1, :) = 整條路徑的 x 座標序列
%           output_args(2, :) = 整條路徑的 y 座標序列
%           output_args(:, i) = 第 i 個取樣點的完整狀態 [x;y;theta;kappa]
%       如果誤把它當成 Mx4（列是取樣點、欄是狀態分量）來用，例如寫成
%       seg(:,1) 想取「x 欄」，實際上會取到「第 1 個取樣點的四維狀態」，
%       導致路徑退化成只有頭幾個點、其餘資訊被誤讀 —— 這正是這次修正
%       my_multi_path.m 時發現的 transpose（轉置）錯誤的根源。任何新的
%       呼叫端（例如 stitch_local_path.m）在使用 my_path() 的回傳值時，
%       務必先用 seg=seg' 轉置成 Mx4，再用 seg(:,1)/(:,2)/(:,3) 取
%       x/y/theta 欄，詳見 stitch_local_path.m 的用法與註解。
%
% 輸入：
%   p   : 曲率多項式係數 [b, c, d, s_f]（Newton-Raphson 收斂後的解）
%   xic : 4x1 初始狀態 [x0; y0; theta0; kappa0]（在 my_path.m 的呼叫情境下，
%         每一段路徑固定從局部座標系原點、局部朝向 0、曲率 0 開始，
%         即 xic=[0,0,0,0]，這樣才能讓多段路徑用旋轉平移拼接時，
%         銜接處的曲率保持連續）
%
% 輸出：
%   output_args : 4xM 矩陣（M=1000），見上方「回傳格式」說明
% =========================================================================

function [ output_args ] = my_dynamic( p, xic )
M=1000; % 用 1000 個取樣點展開整段路徑（歐拉積分步數）
b=p(1);
c=p(2);
d=p(3);
s_f=p(4);
x=zeros(4,M);
x(:,1)=xic;              % 第 1 欄 = 初始狀態
S=linspace(0,s_f,M);     % 對應每個取樣點的弧長座標
Ds=s_f/M;                % 每步的弧長增量（歐拉積分步長）
for k=1:M-1
s=S(k);
x(1,k+1)=x(1,k)+cos(x(3,k))*Ds;                       % x(s+Ds) = x(s) + cos(theta(s))*Ds
x(2,k+1)=x(2,k)+sin(x(3,k))*Ds;                       % y(s+Ds) = y(s) + sin(theta(s))*Ds
x(3,k+1)=x(3,k)+(b*s+c*s^2+d*s^3)*Ds;                 % theta(s+Ds) = theta(s) + kappa(s)*Ds
x(4,k+1)=x(4,k)+(b+2*c*s+3*d*s^2)*Ds;                 % kappa(s+Ds) = kappa(s) + kappa'(s)*Ds
end
output_args=x;
end
