clear all; close all; clc;

% Parameters:
m = 1; % Mass of the pendulum
M = 5; % Mass of the cart
L = 2; % Length of the pendulum
g = -9.81; % Gravititaional constant
d = 1; % (Damping Coefficient)?
u = 0; % Control input

% Simulation:
tspan = 0:0.1:10;
y0 = [0;
      0;
      pi;
      0.5]; % Equibilirium states
[t,x] = ode45(@(t,x)invpen_model(x,m,M,L,g,d,u),tspan,y0);

% Visualization:
for k=1:length(t)
    invpen_plot(x(k,:),m,M,L);
end



