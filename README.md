# CMU Pure Pursuit — 論文 Pure Pursuit 實作

本 repo 以 [cmu-path-method](https://github.com/avery02072003-cmyk/cmu-path-method) 為基礎，  
將 CMU 自駕車軌跡規劃論文的核心約束改寫為 **Pure Pursuit 追蹤控制器**。

---

## 論文對應關係

| 論文元素 | 本 repo 實作位置 |
|---|---|
| 側向加速度約束 `a_lat ≤ a_lat_max` | `main_pure_pursuit_sim.m` 曲率限速段 + `pure_pursuit_controller.m` 硬限制 |
| 縱向加速度約束 `a_acc / a_dec` | `main_pure_pursuit_sim.m` forward-backward speed pass |
| 速度規劃 `v_profile` | `main_pure_pursuit_sim.m` curvature-based + forward-backward smoothing |
| 路徑追蹤控制器 | `pure_pursuit_controller.m` |
| 航向誤差修正 | `delta_fb = -Kh * he / v`（heading feedback） |
| 曲率自適應前視距離 `Ld` | `Ld = Ld0 + kv*v - kappa_gain*kappa` |

---

## 核心檔案

| 檔案 | 說明 |
|---|---|
| `main_pure_pursuit_sim.m` | 主模擬：速度規劃 + 追蹤模擬主迴圈 + 結果繪圖 |
| `pure_pursuit_controller.m` | Pure pursuit 幾何 + 弧長搜尋 + 航向回授 + 側向加速度限制 |
| `reference_path.mat` | 參考路徑（含 x, y, phi, v） |

---

## 控制器架構

```
refpath (x, y, phi, kappa, v_profile)
        │
        ▼
  曲率限速 + forward-backward speed pass
        │
        ▼
  pure_pursuit_controller
  ├── 弧長累加搜尋 idx_target（沿路徑前進方向，累加弧長 ≥ Ld）
  ├── delta_pp = atan2(2*L*sin(alpha), Ld)    ← pure pursuit 幾何
  ├── yaw_target = 座標差分估計 idx_target 切線方向
  ├── delta_fb = -Kh * heading_error / v      ← 航向回授
  └── 側向加速度硬限制：|v²·κ| ≤ a_lat_max
        │
        ▼
  bicycle model 模擬（Euler積分）
```

---

## 最終版本效能（commit e1c823f）

| 指標 | 數值 |
|---|---|
| CTE RMS | **0.0276 m** |
| Heading Error RMS | **2.8706 deg** |
| Max \|delta\| | **11.4445 deg** |
| Speed range | **4.5073 ~ 6.0000 m/s** |
| Max \|a_lat\| | **2.0000 m/s²**（恰好在限制邊界） |

---

## 參數設定

```matlab
params.Ts         = 0.05;    % 模擬時間步（s）
params.L          = 2.7;     % 車軸距（m）
params.Ld0        = 2.0;     % 基礎前視距離（m）
params.kv         = 0.3;     % 速度前視補償係數
params.Ld_min     = 1.2;     % 前視距離下限（m）
params.Ld_max     = 8.0;     % 前視距離上限（m）
params.kappa_gain = 6.0;     % 曲率前視縮短係數
params.Kh         = 0.4;     % 航向回授增益
params.v_des      = 6.0;     % 期望巡航速度（m/s）
params.v_min      = 1.0;     % 最低速度（m/s）
params.a_lat_max  = 2.0;     % 側向加速度限制（m/s²）
params.a_acc_max  = 1.0;     % 加速度限制（m/s²）
params.a_dec_max  = 1.5;     % 減速度限制（m/s²）
```

---

## 執行方式

1. 開啟 MATLAB
2. 執行 `main_pure_pursuit_sim.m`
3. 結果自動輸出 4 張圖（路徑追蹤、轉向指令、追蹤誤差、速度與側向加速度）及終端機數值

---

## 相關 Repo

- [cmu-path-method](https://github.com/avery02072003-cmyk/cmu-path-method)：本 repo 的基礎，論文路徑幾何與可行性計算工具集
