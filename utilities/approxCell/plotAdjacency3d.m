function plotAdjacency3d(adj,verts, options)

arguments
    adj (:,:) double
    verts (3, :) double
    options.linewidth = 2.5
    options.color = 'c'
end

disp('Plotting Adjacencies')

%% Get current axes
axs = gca;
hold(axs,'on');

%% Allow the code to work with weighted adjacencies
bin = ~isfinite(adj);
adj(bin) = 0;
bin = adj > 0;
adj(bin) = 1;

%% Plot verts
pltV = plot3(axs,verts(1,:),verts(2,:),verts(3,:),'*b', 'MarkerSize', 3);
drawnow;

%% Plot adjacencies
plotpoints = [];
for i = 1:size(adj,2)

    for j = i:size(adj,2)

        if i == j
            continue
        end

        if adj(i,j) == 1

            pltI = plot3(axs,verts(1,i),verts(2,i),verts(3,i),'r', 'MarkerSize', 20, 'Marker', 'hexagram', 'LineWidth', 3);
            pltJ = plot3(axs,verts(1,j),verts(2,j),verts(3,j),'m', 'MarkerSize', 20, 'Marker', 'hexagram', 'LineWidth', 3);
            plotpoints(:,end+1:end+2) = [verts(:,i), verts(:,j)];
            line = plot3(axs,plotpoints(1, end-1:end), plotpoints(2, end-1:end), plotpoints(3, end-1:end), 'r', 'LineWidth', options.linewidth);
            
            delete(pltI)
            delete(pltJ)
            set(line, 'Color', options.color)
            % plot3(axs,plotpoints(1, i:i+1),plotpoints(2, i:i+1), plotpoints(3, i:i+1), options.color, 'LineWidth', options.linewidth);

        end
        drawnow;
    end
end

% for i=1:2:size(plotpoints, 2)
%     plot3(axs,plotpoints(1, i:i+1),plotpoints(2, i:i+1), plotpoints(3, i:i+1), options.color, 'LineWidth', options.linewidth);
% end
end