function x_follower = follower_dynamics(state_follower, U, G, F)
    %%% Recording the evolution of follower's state
    x_follower = G * state_follower + F * U;
end