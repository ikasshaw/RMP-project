% Isaac Shaw
% Robot Motion Planning
% 7/19/2025

function path = Astar(adj, heuristic, options)

arguments
    adj double
    heuristic double
    options.start_idx (1,1) double = 1
    options.goal_idx (1,1) double = size(adj, 1)
end
start_idx = options.start_idx;
goal_idx = options.goal_idx;

% Input validation
if ~ismatrix(adj)
    error('Adjacency must be a matrix')
elseif size(adj,1) ~= size(adj,2) 
    error('Adjacency matrix must be a square matrix')
end

path = [];
[adj_m, ~] = size(adj);
visited = false(adj_m, 1);
nodeData = {}; % Cell array to hold distances and nodes parent in path

% Algorithm

% Create a structure for each node with:
% Best distance and the parent node this best distance was routed through
for i = 1:adj_m
    nodeData{i} = struct('cost', inf, 'final_cost', inf, 'prev', NaN);
end

nodeData{start_idx}.cost = 0; %
nodeData{start_idx}.final_cost = heuristic(start_idx); %

while any(~visited)
    
    unvisited = find(~visited);
    cur_node_idx = unvisited(1);

    for i=1:numel(unvisited)
        idx = unvisited(i);
        if nodeData{idx}.final_cost < nodeData{cur_node_idx}.final_cost
            cur_node_idx = idx;
        end
    end

    % Get the least cost node idx
    if cur_node_idx == goal_idx
        break
    end
    
    visited(cur_node_idx) = true;
    connections = find(adj(cur_node_idx,:));
    
    for connection=connections
        newCost = nodeData{cur_node_idx}.cost + adj(cur_node_idx, connection);
        if newCost < nodeData{connection}.cost
            nodeData{connection}.prev = cur_node_idx;
            nodeData{connection}.cost = newCost;
            nodeData{connection}.final_cost = nodeData{connection}.cost + heuristic(connection);
        end
    end
end

% Get the path by traversing the tracker from the goal node back to the start
total_length = 0;
current = goal_idx;

% Check that a path was generated
if nodeData{goal_idx}.cost == inf
    error('No path exists between these nodes')
end

while current ~= start_idx
    path(end+1) = current;
    total_length = total_length + adj(current, nodeData{current}.prev);
    current = nodeData{current}.prev;
end

path(end+1) = start_idx;
path = flip(path);

end