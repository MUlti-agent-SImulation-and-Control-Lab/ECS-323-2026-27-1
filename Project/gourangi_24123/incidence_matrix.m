function D = incidence_matrix(A, n)
%INCIDENCE_MATRIX Construct an incidence matrix of an undirected graph.
%
%   D = incidence_matrix(A)
%
%   Input:
%       A - n x n symmetric adjacency matrix of an undirected graph.
%
%   Output:
%       D - n x m incidence matrix, where m is the number of edges.
%
%   Orientation:
%       Every edge is oriented from the lower-indexed vertex to the
%       higher-indexed vertex.

    num_edges = nnz(triu(A,1));
    D = zeros(n, num_edges);

    edge = 1;

    for i = 1:n-1
        for j = i+1:n
            if A(i,j) ~= 0
                % Orient edge i --> j
                D(i,edge) = -1;
                D(j,edge) = 1;
                edge = edge + 1;
            end
        end
    end
end