%%% Leader Dynamics
function x_leader = leader_dynamics(x_leader_state, k, step, t0, t_max)
    x_leader = x_leader_state;
    if (k*step<=round((t_max - t0)/4))
    x_leader(3)=1;
    elseif((round((t_max - t0)/4)<k*step&&k*step<=round((t_max - t0)/2)))
    x_leader(3)=0;
    elseif ((round((t_max - t0)/2)<k*step&&k*step<=(t_max - (t_max - t0)/4)))
    x_leader(3)=-0.6;
    elseif (((t_max - (t_max - t0)/4)<k*step&&k*step<=t_max))
    x_leader(3)=0;
    end
    x_leader(2)=x_leader(2)+x_leader(3)*step;
    x_leader(1)=x_leader(1)+x_leader(2)*step+1/2*x_leader(3)*step^2;
end