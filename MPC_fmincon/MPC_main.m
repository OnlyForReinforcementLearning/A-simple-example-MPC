clear all; clc; close all;

%% Simulation Initialization 
% simulation time initalization
t0 = 0; t_max = 100; t_step = 0.02;
k_max = round((t_max - t0) / t_step);
%% MPC controller Initialization
% Prediction horizon
Np = 10;
%%% Boundary 
Umin = -30;
Umax = 30;
% Weighted parameters
Q = eye(3);  % state weights
W = eye(1); % control input weights
F_q = 0.5 * eye(3); % Final terminal state weights
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
U0 = zeros(Np, 1);
%% Eular's method
syms t
G = expm(A * t_step);
F = int(expm(A*t), t, 0, t_step)*B;
F = double(F); 
clear t

Algorithm = {'interior-point','sqp','active-set','trust-region-reflective'};
Tol = 1e-5;
Iter = 4000;
% optNLP = optimset( 'LargeScale', 'off', 'GradObj','off', 'GradConstr','off',...
%     'DerivativeCheck', 'off', 'Display', 'iter', 'TolX', 1e-9,...
%     'TolFun', 1e-9, 'TolCon', 1e-9, 'MaxFunEvals',5000,...
%     'DiffMinChange',1e-5,'Algorithm','interior-point');
optNLP = optimset('Display','off','TolFun', 1e-5, 'MaxIter', 4000,...
                'LargeScale', 'off', 'RelLineSrchBnd', [], 'RelLineSrchBndDuration', 1);

for k = 2:k_max + Np % k = [1, Np]
    state_leader(:, k) = leader_dynamics(state_leader(:, k - 1), k, t_step, t0, t_max);
end

state_error(:, 1) = state_follower(:, 1) - state_leader(:, 1) + [D0; 0; 0];
Start_total_time = tic; % Recording the total start time
reference = cell(0);
for k = 2:k_max
    Ai = []; b = []; Aeq = []; beq = []; % No linear constraints
    lb = Umin*ones(Np,1); %% Boundary
    ub = Umax*ones(Np,1); 
    reference = state_leader(:, k-1: k-1+Np) - repmat([D0;0;0], 1, Np+1);
    Start_computing = tic; % Recording the start time of computing
    %%% MPC-Solver by using fmincon
    [UTemp,fval,exitflag,output] = fmincon(@(U)Mycostfunction(Np, state_follower(:, k-1), G, F, reference, U, Q, W, F_q),...
    U0, Ai, b, Aeq, beq, lb, ub, [], optNLP);
% %  Based on nonlinear constraint, a smaller state error can be obtained.
% % The weights of cost function can be smaller but cost much time.
%     [UTemp,fval,exitflag,output] = fmincon(@(U)Mycostfunction(Np, state_follower(:, k-1), G, F, reference, U, Q, W, F_q),...
%     U0, Ai, b, Aeq, beq, lb, ub, @(U)Myconstraint(Np, state_follower(:, k-1), U, G, F, reference), optNLP);
    End_computing = toc(Start_computing); % Recording the end time of computing
    %%% Only use first optimal control input
    U_optim_recording(:, k) = UTemp(1);
    state_follower(:, k) = follower_dynamics(state_follower(:, k - 1), U_optim_recording(:, k), G, F);
    state_error(:, k) = state_follower(:, k) - state_leader(:, k) + [D0; 0; 0];
    disp(['k = ', num2str(k), ', computing time = ', num2str(End_computing,'%.2f'), ', state_position_error=', num2str(state_error(1, k))]);
end
End_total_time = toc(Start_total_time); % Recording the total start time
disp(['Total time=' num2str(End_total_time,'%.2f')]);
t = (1:k_max+Np) * t_step;
t1 = (1:k_max) * t_step;
figure(1)
plot(t, state_leader(1, :), 'b'); hold on;
plot(t1, state_follower(1, :), 'r'); hold on;
xlim([0, 100]);
grid on;

figure(2)
plot(t1, state_error(1, :), 'b'); hold on;
xlim([0, 100]);
grid on;