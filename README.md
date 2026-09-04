# CMU Pure Pursuit — 論文 Pure Pursuit 實作

本 repo 以 [cmu-path-method](https://github.com/avery02072003-cmyk/cmu-path-method) 為基礎，
將 CMU 自駕車軌跡規劃論文的核心約束改寫為 **Pure Pursuit 追蹤控制器**，追蹤對象是
**半聯結車（tractor-trailer，off-axle hitch 模型）**，並加上即時局部路徑重規劃、
多候選路徑評分、離散車道決策圖等模組。

---

## 論文對應關係

| 論文元素 | 本 repo 實作位置 |
|---|---|
| 側向加速度約束 `a_lat ≤ a_lat_max` | `compute_v_profile.m` 曲率限速 + `pure_pursuit_controller.m` 硬限制 |
| 縱向加速度約束 `a_acc / a_dec` | `main_pure_pursuit_sim.m` / `compute_v_profile.m` forward-backward speed pass |
| 速度規劃 `v_profile` | `compute_v_profile.m`（曲率限速 + forward/backward pass）+ `shape_v_profile_cubic.m`（三次多項式弧長平滑） |
| 路徑追蹤控制器 | `pure_pursuit_controller.m` |
| 航向誤差修正 | `delta_fb = -Kh * he / v`（heading feedback） |
| 曲率自適應前視距離 `Ld` | `Ld = Ld0 + kv*v - kappa_gain*kappa`，上限隨車速動態放寬 |
| 聯結車運動學 | off-axle hitch trailer model（`main_pure_pursuit_sim.m` 車輛狀態更新段） |

---

## 核心檔案

| 檔案 | 說明 |
|---|---|
| `STEP1_VehicleParameters.m` | 車輛/控制器參數設定，輸出 `vehicle_params.mat` |
| `main_pure_pursuit_sim.m` | 主模擬：速度規劃 + 即時追蹤模擬主迴圈 + 結果繪圖，輸出 `simulation_results.mat` |
| `pure_pursuit_controller.m` | Pure pursuit 幾何 + 弧長搜尋 + 航向回授 + 側向加速度限制 + 動態前視上限 |
| `generate_local_paths.m` | 每次 replan 以車輛當下位置為錨點，即時生成 N 條候選局部路徑（視窗大小隨車速/煞車距離縮放） |
| `select_best_path.m` | 依橫向誤差/曲率/鉸接角評分選出最佳候選路徑，含偏出候選範圍時的「導回中心」recovery mode |
| `compute_v_profile.m` | 候選路徑速度剖面（曲率限速 + forward/backward 加減速 pass） |
| `shape_v_profile_cubic.m` | 用三次多項式弧長邊界條件，把速度剖面整形成 v/a 連續的平滑減速曲線 |
| `hitch_angle_governor.m` | 即時鉸接角安全網，接近甩尾極限時強制降速 |
| `generate_decision_branches.m` / `estimate_current_lane_id.m` | 離散車道決策分支圖（直行/變換車道）的分支生成邏輯 |
| `reference_path.mat` | 母參考路徑（含 x, y, phi, v） |
| `vehicle_params.mat` / `simulation_results.mat` | STEP1 / 主模擬的輸出資料，供各動畫腳本讀取 |

### 視覺化／動畫腳本（任選其一或多個，皆讀 `simulation_results.mat`）

| 檔案 | 說明 |
|---|---|
| `STEP3_DecisionGraphDemo.m` | 決策分支圖動畫 demo（腳本排定車道序列，走過所有分支情境） |
| `STEP4_Animation_MultiView.m` | 2D 俯視戰術圖：候選路徑扇形、決策分支圖、車道邊界、貨櫃壓線即時讀數 |
| `STEP4_ETS2_Animation_MultiView.m` | 3D 手刻卡車模型動畫（多視窗：3D 跟拍 + 地圖 + 遙測儀表板） |
| `STEP5_ADT_Animation.m` | Automated Driving Toolbox 路面（`drivingScenario`+`road()`）+ 手刻卡車模型/跟拍相機 |

---

## 控制器架構

```
STEP1_VehicleParameters.m ──▶ vehicle_params.mat
                                     │
reference_path.mat（母路徑）─────────┤
                                     ▼
                     main_pure_pursuit_sim.m
                     ├─ 母路徑曲率估算 + 速度剖面規劃
                     │
                     └─ 主模擬迴圈（每步 Ts=0.05s）
                         ├─ 每 T_replan 步：
                         │    generate_local_paths()  以車輛當下位置為錨點
                         │         │                  即時生成 N 條候選路徑
                         │         ▼
                         │    select_best_path()       依 CTE/曲率/鉸接角評分
                         │                              （偏出候選範圍時導回中心）
                         │
                         └─ 每步：
                              pure_pursuit_controller()  弧長搜尋 + 航向回授
                                   │                     + 側向加速度限制
                                   ▼
                              hitch_angle_governor()      即時鉸接角安全網
                                   │
                                   ▼
                              off-axle hitch 聯結車運動學（Euler 積分）
                                     │
                                     ▼
                           simulation_results.mat（供 STEP3/4/5 動畫讀取）
```

---

## 最終版本效能

| 指標 | v_des = 25 m/s | v_des = 6 m/s（回歸測試） |
|---|---|---|
| CTE RMS | **0.0152 m** | **0.0069 m** |
| Max \|CTE\| | 0.445 m | — |
| Max Hitch Angle | 7.87° | — |
| Max \|delta\| | 5.18° | — |
| 偏出賽道比例 | 0% | 0% |

---

## 參數設定（節錄自 `STEP1_VehicleParameters.m`）

```matlab
% 車輛幾何（tractor-trailer）
params.L1 = 4.5;              % 拖車頭軸距 (m)
params.M1 = 1.0;              % 鉸接點偏置：後軸到鉸接點距離 (m)
params.L2 = 7.5;              % 鉸接點到貨櫃後軸距離 (m)
params.lane_width = 3.5;      % 單一車道寬度 (m)

% 控制器
params.Ts         = 0.05;     % 模擬時間步（s）
params.Ld0        = 2.0;      % 基礎前視距離（m）
params.kv         = 0.3;      % 速度前視補償係數
params.Ld_min     = 1.2;      % 前視距離下限（m）
params.Ld_max     = 8.0;      % 前視距離下限保障（實際上限隨車速動態放寬）
params.kappa_gain = 6.0;      % 曲率前視縮短係數
params.Kh         = 0.4;      % 航向回授增益

% 速度規劃
params.v_des      = 25.0;     % 期望巡航速度（m/s）
params.a_lat_max  = 2.0;      % 側向加速度限制（m/s²）
params.a_acc_max  = 1.0;      % 縱向加速度上限（m/s²）
params.a_dec_max  = 1.5;      % 縱向減速度上限（m/s²）
params.a_lat_margin = 0.85;   % 側向加速度安全邊際
```

---

## 執行方式

1. 開啟 MATLAB
2. 執行 `STEP1_VehicleParameters.m`（產生 `vehicle_params.mat`）
3. 確認 `reference_path.mat` 存在（母路徑資料）
4. 執行 `main_pure_pursuit_sim.m`（跑主模擬，輸出 `simulation_results.mat` + 追蹤結果圖表）
5. 視需要執行任一視覺化腳本觀看動畫：`STEP3_DecisionGraphDemo.m` /
   `STEP4_Animation_MultiView.m` / `STEP4_ETS2_Animation_MultiView.m` /
   `STEP5_ADT_Animation.m`（此項需要 Automated Driving Toolbox）

---

## 相關 Repo

- [cmu-path-method](https://github.com/avery02072003-cmyk/cmu-path-method)：本 repo 的基礎，論文路徑幾何與可行性計算工具集
