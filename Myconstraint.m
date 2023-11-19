function [C, Ceq] = Myconstraint(Np, state_follower, U, G, F, state_leader)
    x_follower = zeros(3, Np + 1); % Define a predictive state
    x_follower(:, 1) = state_follower;
    for k = 1: Np % k \in [0, Np - 1]
        x_follower(:, k+1) = follower_dynamics(x_follower(:, k), U(k), G, F);
    end
    %% Terminal MPC constraint.
    x_leader = state_leader;
    Ceq = x_follower(:, Np+1) - x_leader(:, Np+1);
    C = [];
end