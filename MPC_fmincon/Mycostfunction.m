function Cost = Mycostfunction(Np, state_follower, G, F, state_leader, U, Q, W, F_q)
    x_follower = zeros(3, Np + 1); % Define a predictive state
    x_follower(:, 1) = state_follower;
    for k = 1: Np % k \in [0, Np - 1]
        x_follower(:, k+1) = follower_dynamics(x_follower(:, k), U(k), G, F);
    end

    Cost_process_x = 0;
    Cost_u = 0;
    x_leader = state_leader;
    for k = 1: Np
        Cost_process_x = Cost_process_x + sqrt((x_follower(:, k) - x_leader(:, k))' * Q * (x_follower(:, k) - x_leader(:, k)));
        Cost_u = Cost_u + sqrt(U(k) * W * U(k));
    end
    Cost_terminal_x = sqrt((x_follower(:, Np + 1) - x_leader(:, Np + 1))' * F_q * (x_follower(:, Np + 1) - x_leader(:, Np + 1)));

    Cost = Cost_process_x + Cost_terminal_x+Cost_u;
end

