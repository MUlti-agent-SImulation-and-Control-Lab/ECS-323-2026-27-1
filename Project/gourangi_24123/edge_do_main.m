function edge_do_main(alpha, beta, filename)
%% Parameters
n = 6;      % Number of agents
d = 1;      % Dimension of each agent state

%% Communication Graph

A = [0 1 1 0 1 1;
    1 0 0 1 0 0;
    1 0 0 0 1 0;
    0 1 0 0 1 0;
    1 0 1 1 0 1;
    1 0 0 0 1 0];

D = incidence_matrix(A, n);

D_big  = kron(D, eye(d));
Dt_big = D_big';

%% Cost Functions
costs = {
    @(x) sin(x);
    @(x) cos(x);
    @(x) exp(0.1*x);
    @(x) (x-4)^4;
    @(x) (x+3)^2;
    @(x) x^2;
};

grads = {
    @(x) cos(x);
    @(x) -sin(x);
    @(x) 0.1*exp(0.1*x);
    @(x) 4*(x-4)^3;
    @(x) 2*(x+3);
    @(x) 2*x;
};

%% Initial Conditions
x0 = [2;
    1;
    0.5;
    -0.5;
    -1;
    -2];

v0 = zeros(size(D_big,1),1);

X0 = [x0;
    v0];

%% Simulation

T  = 10;
dt = 5e-4;

[t, X] = forward_euler_edge_do( ...
    X0, ...
    T, ...
    dt, ...
    n, ...
    d, ...
    D_big, ...
    Dt_big, ...
    @(x) stacked_gradient(x, grads, d, n), ...
    alpha, ...
    beta);

%% Extract States
x = X(:, 1:n*d);
v = X(:, n*d+1:end);

%% Compute quantities
total_grad = zeros(length(t),1);
for k = 1:length(t)
    total_grad(k) = sum(stacked_gradient(x(k,:).', grads, d, n));
end

cost = zeros(length(t),1);
for k = 1:length(t)
    cost(k) = total_cost(costs, x(k,:).', n);
end

%% Plots
fig = figure;

% Fixed figure size (8 cm wide: suitable for a single-column paper)
fig.Units = 'centimeters';
fig.Position = [2 2 5.6 7.2];
fig.Color = 'w';

FS = 8;           % font size
LW = 1.4;         % line width

%% Agent states
nexttile
plot(t, x, 'LineWidth', LW)
ylabel('$x_i$', ...
    'Interpreter', 'latex', ...
    'FontSize', FS)

grid on
box on
set(gca,...
    'FontSize', FS,...
    'TickLabelInterpreter', 'latex',...
    'LineWidth', 0.8)

%% Cost function
nexttile
plot(t, cost, 'LineWidth', LW)
ylabel('$f(x)$', ...
    'Interpreter', 'latex', ...
    'FontSize', FS)

grid on
box on
set(gca,...
    'FontSize', FS,...
    'TickLabelInterpreter', 'latex',...
    'LineWidth', 0.8)

%% Sum of gradients
nexttile
plot(t, total_grad, 'LineWidth', LW)
ylabel('$\sum_{i=1}^{n}\nabla f_i(x_i)$', ...
    'Interpreter', 'latex', ...
    'FontSize', FS)

xlabel('Time (s)', ...
    'Interpreter', 'latex', ...
    'FontSize', FS)

grid on
box on
set(gca,...
    'FontSize', FS,...
    'TickLabelInterpreter', 'latex',...
    'LineWidth', 0.8)

%% Export as vector graphics
exportgraphics(fig, filename, 'ContentType', 'vector');

%% Local Functions

    function g = stacked_gradient(x, grads, d, n)
        %STACKED_GRADIENT Returns the stacked local gradients.
        g = zeros(n*d,1);

        for i = 1:n
            idx = (i-1)*d + 1 : i*d;
            g(idx) = grads{i}(x(idx));
        end

    end

    function f = total_cost(costs, x, n)
        %TOTAL_COST Returns the sum of all local cost functions.
        f = 0;
        for i = 1:n
            f = f + costs{i}(x(i));
        end
    end

end