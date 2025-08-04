function plotAdjacency(adj,verts, options)

arguments
    adj double
    verts double
    options.linewidth = 2.5
    options.color = 'c'

end

%% Get current axes
axs = gca;
hold(axs,'on');

%% Allow the code to work with weighted adjacencies
bin = ~isfinite(adj);
adj(bin) = 0;
bin = adj > 0;
adj(bin) = 1;

%% Plot verts
pltV = plot(axs,verts(1,:),verts(2,:),'*b');

%% Plot adjacencies
for i = 1:size(adj,1)
    for j = 1:size(adj,2)
        if adj(i,j) == 1
            plot(axs,[verts(1,i),verts(1,j)],[verts(2,i),verts(2,j)], options.color, 'LineWidth', options.linewidth);
        end
    end
end