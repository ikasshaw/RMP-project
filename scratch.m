close all
clear all
clc

A = getConvexPolyPoints(5, "radius", 1, 'regular', true)
B = getConvexPolyPoints(5, "radius", 2, 'regular', true)
C = getConvexPolyPoints(5, "radius", 3, 'regular', true)

A = [0 1 1 0; 0 0, 1 1]
B = [.5 1.5 1.5 .5; 0 0, 1 1]
C = [2 3 3 2; 0 0, 1 1]



A = polyshape(A.')
hold on
B = polyshape(B.')
C = polyshape(C.')
% plot(A)
% plot(B)
% plot(C)
drawnow


int = intersect(A, B)

sub = subtract(B, int)

intC = intersect(A, C)

subC = subtract(C, intC)


plot(int)
plot(sub)

plot(intC)
plot(subC)
