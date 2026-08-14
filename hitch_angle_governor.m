% =========================================================================
% 檔案名稱: hitch_angle_governor.m
%
% 功能：這次新增的模組，是鉸接角安全機制的「第二道防線」（即時動態層）。
%       compute_hitch_speed_cap.m 是在「規劃階段」用曲率預估提前降速，
%       但預估終究只是穩態圓周運動的近似公式，實際動態追蹤過程中
%       （例如剛切換候選路徑的瞬間、或路徑曲率變化太快追蹤器來不及
%       反應）鉸接角仍有可能比預估的更大。本函式監控「當下實際量到的
%       鉸接角」，一旦逼近折疊上限 phi_max，就即時介入降速，是整個
%       安全機制裡最後、也最直接的一道防線。
%
% 為什麼只降速、不修改轉向角：
%       修改轉向角需要同時考慮 pure_pursuit_controller.m 的幾何追蹤邏輯
%       （delta_pp、delta_fb 兩項如何互動），若在這裡再疊加一次轉向角
%       修正，容易跟控制器自己的修正邏輯互相干擾、甚至互相抵銷或放大
%       震盪。「降速」則是獨立、正交於轉向邏輯的手段：车辆速度降低後，
%       同樣的曲率下側向動態（含貨櫃甩尾的動態響應）會更緩和，且給了
%       控制器更多反應時間，是風險最低的介入方式。
%
% 降速公式（分段線性、平滑過渡，不是硬切斷）：
%       hitch_now <= warn_th（phi_max * hitch_gov_warn_frac）：完全不介入
%       warn_th < hitch_now < hard_th（phi_max * hitch_gov_hard_frac）：
%           frac = (hitch_now - warn_th) / (hard_th - warn_th)   ∈ [0,1]
%           v_ref_out = v_ref_in * (1 - frac)                    線性降速
%       hitch_now >= hard_th：
%           frac 飽和於 1，v_ref_out 降到 v_profile_min（最低速度，不會降到 0，
%           避免車輛完全停止導致其他公式除以 v=0 產生數值問題）
%
%       用「線性內插＋兩段門檻」而不是單一硬門檻，是為了避免車輛在
%       門檻附近來回穿越時，速度指令跟著劇烈跳動（bang-bang 式的
%       開關控制容易引發震盪）；warn_th 到 hard_th 之間留出的緩衝區，
%       讓降速過程平滑漸進。
%
% 輸入：
%   v_ref_in : 尚未套用鉸接角安全網之前的參考速度（來自速度規劃 v_profile）
%   yaw0     : 拖車頭目前航向角（rad）
%   yaw1     : 貨櫃目前航向角（rad）
%   params   : 需要 phi_max, hitch_gov_warn_frac, hitch_gov_hard_frac, v_profile_min
%
% 輸出：
%   v_ref_out  : 套用安全網之後的參考速度（若未介入則等於 v_ref_in）
%   gov_active : 本步是否有介入降速（true/false），存進 hist 供事後分析
%                「安全網有沒有被真的用到」
%   hitch_now  : 本步正確 wrap 過的即時鉸接角（rad），同樣存進 hist 供分析
% =========================================================================

function [v_ref_out, gov_active, hitch_now] = hitch_angle_governor(v_ref_in, yaw0, yaw1, params)
    d = yaw0 - yaw1;
    hitch_now = abs(atan2(sin(d), cos(d)));   % 正確 wrap 到 (-pi, pi]，即時鉸接角

    warn_th = params.phi_max * params.hitch_gov_warn_frac;   % 開始介入的門檻
    hard_th = params.phi_max * params.hitch_gov_hard_frac;   % 降到最低速的門檻

    if hitch_now <= warn_th
        v_ref_out = v_ref_in;
        gov_active = false;
        return;
    end

    frac = min(1, (hitch_now - warn_th) / max(hard_th - warn_th, 1e-6));   % 介入強度 0~1
    v_ref_out = max(params.v_profile_min, v_ref_in * (1 - frac));          % 線性降速，設下限
    gov_active = true;
end
