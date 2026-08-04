function path_candidates = my_multi_path(gps_waypoints, N_paths, params)
    path_candidates = cell(1, N_paths);
    offsets = linspace(-params.lane_width/2, params.lane_width/2, N_paths);
    for i = 1:N_paths
        wp_shifted = shift_waypoints_lateral(gps_waypoints, offsets(i));
        path_candidates{i} = my_path(wp_shifted);
    end
end

function wp_out = shift_waypoints_lateral(wp, d)
    dx = gradient(wp(:,1));
    dy = gradient(wp(:,2));
    heading = atan2(dy, dx);
    nx = -sin(heading);
    ny = cos(heading);
    wp_out = [wp(:,1) + d*nx, wp(:,2) + d*ny];
end