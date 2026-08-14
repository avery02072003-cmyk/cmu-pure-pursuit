% =========================================================================
% 檔案名稱: my_endpoint.m
%
% 功能：給定目前這一輪 Newton-Raphson 迭代猜測的曲率多項式係數
%       p_ceff = [b, c, d, s_f]，算出「如果真的照這個係數走一段弧長
%       s_f 的路徑，終點的狀態會是什麼」，也就是 [x_end; y_end; theta_end;
%       kappa_end]。my_path.m 每次迭代都會呼叫本函式，拿算出來的終點狀態
%       跟「真正想要到達的目標狀態」比較，兩者的差距（殘差）就是
%       Newton-Raphson 下一步要修正的量。
%
% 四個終點量各自怎麼來：
%   x_end, y_end : 沒有解析解，要靠數值積分。呼叫 my_Fkn_Gkn(0, sample, p)
%                  取得 F_0 = ∫[0,s_f] cos(theta(s)) ds = x_end，
%                  以及 G_0 = ∫[0,s_f] sin(theta(s)) ds = y_end
%                  （見 my_Fkn_Gkn.m、my_fk_gk.m 的詳細推導）。
%   theta_end     : 有解析解（多項式積分），theta(s)=∫[0,s] kappa(sigma)dsigma
%                  = (1/2)b*s^2+(1/3)c*s^3+(1/4)d*s^4，直接代入 s=s_f 即可，
%                  不需要數值積分。
%   kappa_end     : 直接代入 kappa(s)=b*s+c*s^2+d*s^3 求值，也是解析解。
%
% 輸入：
%   p_ceff     : 曲率多項式係數向量 [b, c, d, s_f]
%   sample_num : 數值積分（x_end, y_end）用的辛普森取樣點數
%
% 輸出：
%   output_args : 4x1 終點狀態向量 [x_end; y_end; theta_end; kappa_end]
% =========================================================================

function [ output_args ] = my_endpoint( p_ceff, sample_num  )

p = p_ceff;

s= p(4);          % s_f：這一段路徑的總弧長（Newton-Raphson 也會迭代修正這個值）

sample=sample_num ;

% x_end, y_end：靠 Simpson's rule 數值積分求得（見 my_Fkn_Gkn.m）
[ x_end y_end ] = my_Fkn_Gkn(0, sample, p);

% kappa_end：kappa(s) = b*s + c*s^2 + d*s^3，代入 s=s_f 的解析解
kappa=p(1)*s+p(2)*s^2+p(3)*s^3;

% theta_end：theta(s) = (1/2)b*s^2 + (1/3)c*s^3 + (1/4)d*s^4，代入 s=s_f 的解析解
theda=(1/2)*p(1)*s^2+(1/3)*p(2)*s^3+(1/4)*p(3)*s^4;

output_args=[x_end;y_end;theda;kappa];

end
