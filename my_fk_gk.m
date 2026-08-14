% =========================================================================
% 檔案名稱: my_fk_gk.m
%
% 功能：計算「三次曲率多項式路徑」在弧長 s 處的瞬時航向角 theta(s)，
%       並回傳 cos(theta(s))、sin(theta(s))。這是整個路徑生成引擎最底層
%       的被積函數（積分核），供 my_Fkn_Gkn.m 做數值積分時逐點呼叫。
%
% 數學背景（跟 my_path.m / My_Jacobian.m / my_endpoint.m 共用同一套模型）：
%   本引擎把「路徑上每一點的曲率」表示成弧長 s 的三次多項式：
%
%       kappa(s) = b*s + c*s^2 + d*s^3        （注意：沒有常數項，kappa(0)=0）
%
%   其中 (b, c, d) 就是外部傳進來的係數向量 p_ceff 的前三個元素
%   （p_ceff = [b, c, d, s_f]，s_f 是這一段路徑的總弧長，也是待解未知數之一）。
%
%   航向角是曲率對弧長的積分（這是「曲率」的定義：kappa = dtheta/ds）：
%
%       theta(s) = ∫[0,s] kappa(sigma) dsigma
%                = (1/2)*b*s^2 + (1/3)*c*s^3 + (1/4)*d*s^4
%
%   這個積分有解析解（多項式逐項積分），不需要數值方法，所以本函式
%   直接用公式算出 theta(s)，這也是本函式存在的意義：把「求 theta(s)」
%   這件事獨立出來，讓 my_Fkn_Gkn.m 可以在數值積分（Simpson's rule）
%   的每一個取樣點上重複呼叫。
%
%   有了 theta(s)，車輛在路徑上位置的變化率就是標準的「單位切向量」：
%       dx/ds = cos(theta(s))
%       dy/ds = sin(theta(s))
%   把這兩個值對 s 積分（在 my_Fkn_Gkn.m 裡用 Simpson's rule 做），
%   就能算出路徑上任一點的 (x, y) 座標 —— 這就是本函式回傳
%   cos(theta)、sin(theta) 而不是直接回傳 (x, y) 的原因：本函式只負責
%   「被積函數」這一步，真正的積分交給呼叫端做。
%
% 輸入：
%   p_ceff : 曲率多項式係數向量 [b, c, d, s_f]（s_f 在本函式中其實用不到，
%            只是沿用同一個係數向量格式方便呼叫）
%   s_now  : 要計算的弧長位置 s（純量）
%
% 輸出：
%   output_args_a : cos(theta(s))
%   output_args_b : sin(theta(s))
% =========================================================================

function [ output_args_a output_args_b ] = my_fk_gk( p_ceff, s_now )
p=p_ceff;
s=s_now;

% theta(s) = (1/2)*b*s^2 + (1/3)*c*s^3 + (1/4)*d*s^4
% —— kappa(s)=b*s+c*s^2+d*s^3 逐項積分後的解析解（積分常數為 0，因為 kappa(0)=0 且
%    我們定義每一段路徑局部座標系的起始航向角 theta(0)=0）
theda=(1/2)*p(1)*s^2+(1/3)*p(2)*s^3+(1/4)*p(3)*s^4;

output_args_a = cos(theda);   % dx/ds，供積分得到 x(s)
output_args_b = sin(theda);   % dy/ds，供積分得到 y(s)

end
