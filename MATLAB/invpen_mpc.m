%% Start

clear all; clc;

%% Inverted Pendulum on a Cart

% Define system parameters and continuous model

M = 5; % mass of the cart
m = 0.2; % mass of the pendulum
b = 0.1; % coefficient of friction for cart
l = 0.3; % length of pendulum center of mass
F = 0.0; % force applied to the cart
I = 0.00; % inertia
x = 0.0; % cart posiotion coordinate
theta = 0.0; % pendulum angle from vertical
g = 9.81; 

%% transfer function method:

% q = (M+m)*(I+m*l^2)-(m*l)^2;
% 
% s = tf('s');
% 
% P_cart = (((I+m*l^2)/q)*s^2 - (m*g*l/q))/(s^4 + (b*(I + m*l^2))*s^3/q - ((M + m)*m*g*l)*s^2/q - b*m*g*l*s/q);
% 
% P_pend = (m*l*s/q)/(s^3 + (b*(I + m*l^2))*s^2/q - ((M + m)*m*g*l)*s/q - b*m*g*l/q);
% 
% sys_tf = [P_cart ; P_pend];
% 
% inputs = {'u'};
% 
% outputs = {'x'; 'phi'};
% 
% set(sys_tf,'InputName',inputs)
% set(sys_tf,'OutputName',outputs)
% 
% sys_tf

%% state-space method:

p = I*(M+m)+M*m*l^2; %denominator for the A and B matrices

A = [0      1              0           0;
     0 -(I+m*l^2)*b/p  (m^2*g*l^2)/p   0;
     0      0              0           1;
     0 -(m*l*b)/p       m*g*l*(M+m)/p  0];

B = [     0;
     (I+m*l^2)/p;
          0;
        m*l/p];

C = [1 0 0 0;
     0 0 1 0];

D = [0;
     0];

states = {'x' 'x_dot' 'phi' 'phi_dot'};
inputs = {'u'};
outputs = {'x'; 'phi'};

sys_ss = ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs);

%% Open-loop impulse responce:

% t=0:0.01:1;
% impulse(sys_tf,t);
% title('Open-Loop Impulse Response')

%% Open-loop step response:

% t = 0:0.05:10;
% u = ones(size(t));
% [y,t] = lsim(sys_tf,u,t);
% plot(t,y)
% title('Open-Loop Step Response')
% axis([0 3 0 50])
% legend('x','phi')
% 
% step_info = lsiminfo(y,t);
% cart_info = step_info(1)
% pend_info = step_info(2)

%% PID Controller Design:

% Kp = 1;
% Ki = 1;
% Kd = 1;
% C = pid(Kp,Ki,Kd);
% T = feedback(P_pend,C);

%% Simulation

% Define time vector and input (force)
t = 0:0.01:10; % simulation time (0 to 10 seconds with 0.01s step)
u = zeros(size(t)); % zero input (no external force)
u(t == 1) = 0.1; % Apply an impulse force of 0.1N at t=1s

% Simulate system response
[~,~,xout] = lsim(sys_ss, u, t);

% Extract states (cart position and pendulum angle)
x_cart = xout(:, 1);
theta_pendulum = xout(:, 3);

%% Visualization

% Set up figure for animation
figure;
hold on;
axis equal;
xlim([-3, 3]);
ylim([-1, 1]);
xlabel('X Position (m)');
ylabel('Y Position (m)');

cart_width = 0.2; % width of the cart for visualization
cart_height = 0.1; % height of the cart for visualization

% Plot the pendulum and cart
for i = 1:length(t)

    cla; % Clear the previous frame

    % Cart position
    cart_x = x_cart(i);
    
    % Pendulum position
    pendulum_x = cart_x + l * sin(theta_pendulum(i));
    pendulum_y = l * cos(theta_pendulum(i));
    
    
    % Plot cart
    rectangle('Position', [cart_x - cart_width/2, 0, cart_width, cart_height], 'FaceColor', 'b');
    
    % Plot pendulum
    plot([cart_x, pendulum_x], [0, pendulum_y], 'r', 'LineWidth', 2);
    plot(pendulum_x, pendulum_y, 'ro', 'MarkerFaceColor', 'r');
    
    % Set axis limits and labels
    xlim([-3, 3]);
    ylim([-1, 1]);
    xlabel('X Position (m)');
    ylabel('Y Position (m)');
    
    pause(0.01); % Pause to create animation effect
end
