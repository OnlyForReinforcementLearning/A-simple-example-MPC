clear all; clc; close all;

%%% This is a simple example for MPC

%% Simulation Initialization 
% simulation time initalization
t0 = 0; t_max = 100; t_step = 0.02;
k_max = round((t_max - t0) / t_step);
%% MPC controller Initialization
% Prediction horizon
Np = 10;
% Weighted parameters
Q = eye(3);  % state weights
hat_Q = kron(eye(Np + 1), Q);
W = eye(1); % control input weights
hat_W = kron(Np, W);
%% System Initialization
% the state of the leader
% we assume the leader's state can be obtained during [0, sim_time + predict_time]
state_leader = zeros(3, k_max + Np);
% the state of the follower
D0 = 20;
state_follower = zeros(3, k_max);
state_follower(:, 1) = [-D0; 0; 0];
% System parameters of the follower
tau = 0.5;  
A = [0     1      0;
       0     0      1;
       0     0    -1/tau];

B = [0;   0;    1];
% system error
state_error = zeros(3, k_max);
% The state of control input 
U_optim_recording= zeros(1, k_max);
%% Eular's method
syms t
G = expm(A * t_step);
F = int(expm(A*t), t, 0, t_step)*B;
F = double(F); 
clear t

%% Simulation 
% The evolution of the leader's state 
for k = 2: k_max + Np
    x_leader = state_leader(:, k-1);
    if (k*t_step<=25)
    x_leader(3)=1;
    elseif((25<k*t_step&&k*t_step<=50))
    x_leader(3)=0;
    elseif ((50<k*t_step&&k*t_step<=75))
    x_leader(3)=-0.6;
    elseif ((75<k*t_step&&k*t_step<=100))
    x_leader(3)=0;
    end
    x_leader(2)=x_leader(2)+x_leader(3)*t_step;
    x_leader(1)=x_leader(1)+x_leader(2)*t_step+1/2*x_leader(3)*t_step^2;
    state_leader(:, k) = x_leader;
end

%%%（Using interior-point-convex）LQR Optimize
%%%  Due to the mosek, I have to use the code, and some details can be seen in MOSEK website 
 options = mskoptimset('');
%  options = mskoptimset(options,'MaxTime','16');

% The evolution of the follower's state
for k = 2: k_max
    %Error state
    state_error(:, k - 1) = state_follower(:, k - 1) - state_leader(:, k - 1) + [D0; 0; 0];
    n = size(G, 1);  % array
    p = size(F, 2); %row
    
    M = [eye(n); zeros(Np * n, n)]; 
    C = zeros((Np+1) * n, Np * p);
    temp = eye(n);
    %%%  Based on existing code of Github
    for m = 1: Np
        each_row = m * n + (1:n);                                                
        C(each_row,:) = [temp* F, C(each_row - n,1:end-p)];                              
        temp = G*temp;
        M(each_row,:) = temp;                                                    
    end
    hat_G = M' * hat_Q * M;
    hat_E = M' * hat_Q * C;
    hat_H = C' * hat_Q * C + hat_W;
    
    f = ((state_error(:, k - 1))' * hat_E)';
    %Convert the MPC into QP problem
    %%% Qp-solver 
    [U_k,fval,exitflag,output,lambda] = quadprog(hat_H, f,[],[],[],[],[],[],[], options);
    U_optim_recording(:, k - 1) = U_k(1);
    state_follower(:, k) = G * state_follower(:, k - 1) + F * U_optim_recording(:, k - 1); 
    %%% Show the process
    disp(['k = ', num2str(k), ', elapsed time = ', num2str(toc,'%.2f')]);
end

state_error(:, k) = state_follower(:, k) - state_leader(:, k) + [D0; 0; 0];
t = (1:k_max+Np) * t_step;
t1 = (1:k_max) * t_step;
figure(1)
plot(t, state_leader(1, :), 'b'); hold on;
plot(t1, state_follower(1, :), 'r'); hold on;
xlim([0, 100]);
grid on;
figure(2)
plot(t1, state_error(1, :), 'b'); hold on;
grid on;