function y=approxqfunc( x )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
% y=1/12*exp(-x^2/2) + 1/4*exp(-2/3*x^2);
y=1/12*exp(-x^2/2)+1/4*exp(-2/3*x^2);
end
function rhs=lorenz(t,w)
sigma=10;
rho=28;
b=8/3;
xdot=sigma*(w(2)-w(1));
ydot=rho*w(1)-w(2)-w(1)*w(3);
zdot=w(1)*w(2)-b*w(3);
rhs=[xdot;
    ydot;
    zdot];
