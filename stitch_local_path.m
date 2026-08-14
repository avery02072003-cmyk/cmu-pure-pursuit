% stitch_local_path.m — 逐段呼叫 my_path() 連接相鄰 waypoint，組成連續路徑
% 抽出自 my_multi_path.m 的內層迴圈，修正原本的轉置錯誤：
%   my_path()（實際上是 my_dynamic()）回傳的是 4xM 矩陣（列=[x;y;theta;kappa]，欄=沿弧長取樣點），
%   不是 Mx4，之前 seg(:,1)/seg(:,2) 誤把「第1、2個取樣點的狀態向量」當成 x 欄/y 欄，
%   導致每段只貢獻 4 個幾乎重疊的點、路徑退化成雜訊。這裡先轉置 seg 再依 sample_stride 降採樣。

function [path_x, path_y, path_phi] = stitch_local_path(wp_shifted, phi_wp, sample_stride)
    path_x = []; path_y = []; path_phi = [];
    xic = [0, 0, 0, 0];   % 每段都從局部原點、局部朝向 0 開始

    for j = 1:size(wp_shifted,1)-1
        dx_world = wp_shifted(j+1,1) - wp_shifted(j,1);
        dy_world = wp_shifted(j+1,2) - wp_shifted(j,2);

        phi_start = phi_wp(j);

        c = cos(-phi_start); s = sin(-phi_start);
        x_tag = c*dx_world - s*dy_world;
        y_tag = s*dx_world + c*dy_world;

        phi_tag = phi_wp(j+1) - phi_start;
        phi_tag = atan2(sin(phi_tag), cos(phi_tag));

        try
            [seg, ~] = my_path(x_tag, y_tag, phi_tag, xic);

            seg = seg';                          % 修正：4xM -> Mx4 (rows=沿弧長的取樣點)
            seg = seg(1:sample_stride:end, :);    % 降採樣，控制候選路徑點數密度

            c2 = cos(phi_start); s2 = sin(phi_start);
            seg_x_world = c2*seg(:,1) - s2*seg(:,2) + wp_shifted(j,1);
            seg_y_world = s2*seg(:,1) + c2*seg(:,2) + wp_shifted(j,2);
            seg_phi_world = seg(:,3) + phi_start;

            path_x   = [path_x;   seg_x_world];
            path_y   = [path_y;   seg_y_world];
            path_phi = [path_phi; seg_phi_world];

            xic = [0, 0, 0, 0];
        catch
            continue;
        end
    end
end
