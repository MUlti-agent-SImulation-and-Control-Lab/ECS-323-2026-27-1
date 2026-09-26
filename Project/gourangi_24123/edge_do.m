function dXdt = edge_do(t, X, n, d, D_big, D_t_big, gradf, alpha, beta)
    % Distributed optimization using new dynamics
    % State vector X = [x; s] 

    % extract states
    x = X(1 : n*d);
    v = X(n*d + 1 : end);

    % dynamics
    dv = (alpha*beta) .* D_big * sign(D_t_big*x);
    dx = -alpha*gradf(x) - ((1/alpha) .* dv) - v;

    dXdt = [dx; dv];
end