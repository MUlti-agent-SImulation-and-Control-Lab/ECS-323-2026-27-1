function [t, Xhist] = forward_euler_edge_do(X0, T, dt, n, d, D_big, D_t_big, gradf, alpha, beta)
%FORWARD_EULER_EDGE_DO Simulates edge_do dynamics using Forward Euler.
%
% Inputs:
%   X0      - Initial state [x0; v0]
%   T       - Final simulation time
%   dt      - Time step
%   n       - Number of agents
%   d       - Dimension of each agent state
%   D       - Incidence matrix
%   gradf   - Function handle for gradient
%   alpha   - Algorithm parameter
%   beta    - Algorithm parameter
%
% Outputs:
%   t       - Time vector
%   Xhist   - State trajectory
%
% Example:
%   [t,X] = forward_euler_edge_do(X0,20,1e-3,n,d,D,@gradf,alpha,beta);
N = round(T/dt);

t = (0:N)'*dt;

% Each row is one time instant (same as ode45)
Xhist = zeros(N+1,length(X0));

X = X0;
Xhist(1,:) = X';

for k = 1:N

    dX = edge_do( ...
        t(k), ...
        X, ...
        n, ...
        d, ...
        D_big, ...
        D_t_big, ...
        gradf, ...
        alpha, ...
        beta);

    X = X + dt*dX;

    Xhist(k+1,:) = X';

end

end
