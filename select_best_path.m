function best_idx = select_best_path(path_candidates, trailer_state, params)
    N = numel(path_candidates);
    scores = inf(1, N);
    for i = 1:N
        p = path_candidates{i};
        cte = compute_cte(p, trailer_state);
        feas = check_hitch_angle(p, trailer_state, params);
        curv = max(abs(p.kappa));
        if feas
            scores(i) = params.w_cte*cte + params.w_kappa*curv;
        end
    end
    [~, best_idx] = min(scores);
end