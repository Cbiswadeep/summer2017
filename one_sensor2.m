function [ h ] = circle(x,y,r)
%UNTITLED7 Summary of this function goes here
%   Detailed explanation goes here
%function h = circle(x,y,r)

hold on
th = 0:pi/50:2*pi;
xunit = r * cos(th) + x;
yunit = r * sin(th) + y;
h = plot(xunit, yunit);
hold off


end

