function [mean_dist, std_dist, max_dist, ix, iy] = dtw3d_oneWay(s, t)
    ns = size(s, 1);
    nt = size(t, 1);
    cost = inf(ns, nt);
    distance_matrix = zeros(ns, nt); % To store pairwise distances for analysis

    % Initialize cost matrix and distance matrix
    cost(1,1) = norm(s(1,:) - t(1,:));
    distance_matrix(1,1) = cost(1,1);
    for i = 2:ns
        distance_matrix(i,1) = norm(s(i,:) - t(1,:));
        cost(i,1) = cost(i-1,1) + distance_matrix(i,1);
    end
    for j = 2:nt
        distance_matrix(1,j) = norm(s(1,:) - t(j,:));
        cost(1,j) = cost(1,j-1) + distance_matrix(1,j);
    end

    % Compute DTW cost matrix and update distance matrix
    for i = 2:ns
        for j = 2:nt
            % Update to ensure minimum cost calculation
            localCost = norm(s(i,:) - t(j,:));
            distance_matrix(i,j) = localCost;
            cost(i,j) = localCost + min([cost(i-1,j), cost(i,j-1), cost(i-1,j-1)]);
        end
    end

    % Backtrack to find the optimal path, preferring to advance in 's'
    i = ns;
    j = nt;
    ix = [ns];
    iy = [nt];
    path_distances = [distance_matrix(ns, nt)];

    % We allow movement in 't' more freely
    while i > 1 && j > 1
        window = 15; % You can adjust the window size
        distance1 = min(min(distance_matrix(min(max([i],1),ns),min(max([j-window:j+window],1),nt))));
        distance2 = min(min(distance_matrix(min(max([i-window:i+window],1),ns),min(max([j],1),nt))));
        distance = min(distance1, distance2);
        path_distances = [distance, path_distances];
        
        if cost(i-1, j) < cost(i, j-1) && cost(i-1, j) < cost(i-1, j-1)
            i = i - 1; % Move up
        elseif cost(i, j-1) < cost(i-1, j-1)
            j = j - 1; % Move left
        else
            i = i - 1; % Move diagonal
            j = j - 1;
        end
        
        ix = [i, ix];
        iy = [j, iy];
    end

    % Ensure to reach the beginning of both sequences
    while i > 1
        i = i - 1;
        ix = [i, ix];
        iy = [1, iy];
        path_distances = [distance_matrix(i, 1), path_distances];
    end
    while j > 1
        j = j - 1;
        iy = [j, iy];
        ix = [1, ix];
        path_distances = [distance_matrix(1, j), path_distances];
    end

    % Compute the mean and standard deviation of the distances along the path
    mean_dist = mean(path_distances);
    std_dist = std(path_distances);
    max_dist = max(path_distances);
end
