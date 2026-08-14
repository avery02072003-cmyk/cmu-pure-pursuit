% =========================================================================
% 檔案名稱: my_path.m
%
% 功能：路徑生成引擎的「頂層入口」。給定一個局部座標系下的目標終點姿態
%       (x_tag, y_tag, phi_tag)，用 Newton-Raphson 迭代解出一組三次曲率
%       多項式係數 p=[b,c,d,s_f]，使得車輛從局部原點（朝向 0、曲率 0）
%       出發、沿著 kappa(s)=b*s+c*s^2+d*s^3 這條曲率規律走一段弧長 s_f 後，
%       終點狀態剛好等於 (x_tag, y_tag, phi_tag, 0)（终點曲率固定要求為 0，
%       這樣多段路徑首尾相接時彎道曲率才會連續、不會有轉向瞬間跳變）。
%
%       這個演算法是「參數化軌跡生成（Parametric Trajectory Generation）」
%       的經典做法：與其直接對 (x,y) 做曲線擬合，不如反過來假設「曲率
%       是弧長的多項式」，因為曲率直接對應方向盤轉角（阿克曼轉向幾何：
%       delta = atan(kappa*L)），用曲率當自由變數解出來的路徑，天生就是
%       車輛「轉得動」的路徑，不會出現真實車輛做不到的急轉彎。
%
% ⚠ 本次修改重要說明（回答「跟原本的 my_path 有甚麼區別」）：
%       my_path.m／my_endpoint.m／My_Jacobian.m／my_Fkn_Gkn.m／my_fk_gk.m／
%       my_dynamic.m 這一整組「數學求解引擎」本身在這次修改中【完全沒有
%       更動】—— 求解邏輯、收斂公式、Newton-Raphson 迭代方式都跟原本
%       一模一樣。
%
%       真正被修正的是「呼叫端怎麼用它的輸出」：my_dynamic.m 回傳的是
%       4xM（列＝狀態分量、欄＝取樣點）矩陣，但舊版 my_multi_path.m 誤把
%       它當成 Mx4 在用（seg(:,1) 誤以為是 x 欄，實際上是「第 1 個取樣點
%       的四維狀態」），導致每段路徑只生出約 4 個幾乎重疊的點、路徑退化
%       成雜訊。這個轉置錯誤已經在 stitch_local_path.m 修正（呼叫
%       my_path() 後先 seg=seg' 轉置再取用），my_path() 本身完全沒變。
%
%       至於「即時生成」則是全新架構層面的改動，跟 my_path() 求解一段
%       曲線的邏輯無關：以前是整趟路線（可能上百公尺）在模擬「開始前」
%       一次性呼叫 my_path() 逐段拼出全部候選路徑；現在改成
%       generate_local_paths.m 在模擬「過程中」每隔 0.5 秒（T_replan 步）
%       用車輛「當下位置」為錨點，只對前方一小段窗口（local_horizon_m，
%       預設 35m）呼叫 my_path()，重新生成一批新的候選路徑。my_path() 本身
%       完全不知道、也不需要知道自己是被整條路線呼叫還是被局部窗口呼叫，
%       它永遠只是「給我起點終點姿態，我解一段曲線給你」的無狀態函式，
%       這正是「模組化」的意義：上層架構怎麼改，都不需要動到這個底層
%       數學引擎。
%
% Newton-Raphson 迭代流程：
%   1. 目標狀態 x_dest = [x_tag; y_tag; phi_tag; 0]（終點曲率固定要求為 0）
%   2. 初始猜測：p = [0, 0, 0, arc]，arc = sqrt(x_tag^2+y_tag^2)（直線距離
%      當作弧長初始猜測，曲率係數 b,c,d 初始猜測為 0，即先猜一條直線）
%   3. 每次迭代：
%        a. 用目前的 p 算出 my_endpoint(p,k) 的終點狀態 x_end
%        b. 殘差 delta_x = x_dest - x_end；若 norm(delta_x) < 1e-6 視為
%           收斂，提早跳出迴圈
%        c. 用 My_Jacobian(4,k,p) 算出 4x4 Jacobian 矩陣 J_M
%        d. 用 Moore-Penrose 虛擬反矩陣 pinv(J_M)（比 inv() 更穩健，
%           即使 J_M 接近奇異也不會直接報錯，而是給出最小平方意義下的解）
%           算出修正量 delta_p = pinv(J_M) * delta_x
%        e. p = p + delta_p，進入下一輪迭代
%   4. 最多迭代 500 次（防止無法收斂時無限迴圈）；若迭代中途 p 出現 NaN
%      或數值爆炸（|p|>1e6），視為此段路徑幾何不可行，直接丟出例外
%      （呼叫端 stitch_local_path.m / my_multi_path.m 用 try/catch 接住，
%      跳過這一段不可行的路徑）。
%   5. 收斂後，用 my_dynamic(p, xic) 把整條路徑用 1000 個點展開回傳。
%
% 輸入：
%   x_tag, y_tag, phi_tag : 目標終點在「局部座標系」下的位置與航向角
%                            （局部座標系定義：原點＝路徑起點，x 軸方向＝
%                            起點航向，由呼叫端負責把世界座標系旋轉平移
%                            轉換成這個局部座標系，再把結果轉換回去，
%                            見 stitch_local_path.m）
%   xic                    : 4x1 初始狀態 [x0;y0;theta0;kappa0]，在目前
%                            所有呼叫情境下固定傳入 [0,0,0,0]
%
% 輸出：
%   Output : my_dynamic() 展開的 4xM 完整路徑軌跡（4xM，非 Mx4，見上方說明）
%   ceff   : 收斂後的曲率多項式係數 p=[b,c,d,s_f]
% =========================================================================

function [ Output  ceff] = my_path( x_tag, y_tag, phi_tag, xic )

x_dest=[x_tag y_tag phi_tag 0]';   % 目標終點狀態，終點曲率固定要求為 0（銜接處曲率連續）
arc=sqrt(x_tag^2+y_tag^2);         % 直線距離，當作弧長 s_f 的初始猜測

N=4;  % Jacobian 矩陣階數（對應 4 個未知係數 b,c,d,s_f）
k=20; % 辛普森積分的取樣點數（僅用於求解迭代過程，跟 my_dynamic 的 1000 點展開無關）
p=[0 0 0 arc];                     % 初始猜測：曲率係數全為 0（先猜一條直線），弧長猜 arc
x_end = my_endpoint(p, k);

for i=1:500
    delta_x = x_dest - x_end;      % 終點狀態殘差

    if norm(delta_x) < 1e-6
        break;   % 已收斂，提早結束
    end
    if any(isnan(p)) || any(abs(p) > 1e6)
        error('my_path: Newton-Raphson 發散，此段路徑不可行');
    end

    J_M  = My_Jacobian( N, k, p );     % 4x4 Jacobian：終點狀態對 [b,c,d,s_f] 的偏導數
    iJ_M = pinv(J_M);                  % Moore-Penrose 虛擬反矩陣（比 inv 更穩健）
    temp = iJ_M * delta_x;             % Newton-Raphson 修正量
    delta_p = temp';
    p = p + delta_p;                   % 更新係數猜測
    x_end = my_endpoint(p, k);         % 用新係數重算終點狀態，準備下一輪比較
end

delta_x=x_dest-x_end;
Output= my_dynamic( p, xic );      % 收斂後，展開成 1000 點的完整路徑軌跡（4xM）
ceff=p;
end
