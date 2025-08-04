% Isaac Shaw
% Robot Motion Planning
% 7/26/2025
% Homework 10 Test Script

%% Test Approximate Cell Decomposition Brute Force
clear all
close all
clc

fig = figure('Name', 'HW10 Brute Force Method');
axs = axes('Parent', fig, 'DataAspectRatio', [1, 1, 1], 'NextPlot', 'add');
axis tight

load('HW10_TestCase_ApproxCell.mat');

brute = true;

[myadj, mywAdj, xy] = approxCellGraph(q_init, q_goal, CB, bounds, 'debug', true, 'brute', brute);

a = isequal(myadj, adj);
b = all(fillmissing((mywAdj - wAdj), 'constant', 0) < 1e-8, 'all');
c = all(xy - verts < 1e-8, 'all');

approxCellGraph_Pass = all([a, b, c])

if brute
    if ~a
        diff = myadj-adj;
        [idx_pos_m, idx_pos_n]= find(diff == 1);
        disp([idx_pos_m idx_pos_n])

        [idx_neg_m, idx_neg_n]= find(diff == -1);
        disp([idx_neg_m idx_neg_n])
    end

    if ~b

        diff = mywAdj-wAdj;
        disp(diff)

        disp(mywAdj)
        disp(wAdj)
    end

    if ~c
        disp(xy)
        disp(verts)
    end
end

plotAdjacency(adj, verts, 'linewidth', 3, 'color', 'r');
plotAdjacency(myadj, xy, 'linewidth', .5, 'color', 'b');

saveas(gcf, [strrep(get(fig, 'Name'),' ', '_'),'.png'])

%% Test Approximate Cell Decomposition Adjacent Only

clear all

load('HW10_TestCase_ApproxCell.mat');

fig = figure('Name', 'HW10 Non-Brute Force Method');
axs = axes('Parent', fig, 'DataAspectRatio', [1, 1, 1], 'NextPlot', 'add');
axis tight

[myadj, mywAdj, xy] = approxCellGraph(q_init, q_goal, CB, bounds, 'debug', true, 'brute', false);

plotAdjacency(adj, verts, 'linewidth', 3, 'color', 'r');
plotAdjacency(myadj, xy, 'linewidth', 1, 'color', 'b');

saveas(gcf, [strrep(get(fig, 'Name'),' ', '_'),'.png'])
