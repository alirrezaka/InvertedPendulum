
% A function to plot the simulation of the inverted pendulum on a cart problem.

function invpen_plot(state,m,M,L)
x_pos = state(1); % Position of the cart
th = state(3); % Angle of the pendulum

% Dimensions:
W = 1*sqrt(M/5);  % Cart width
H = .5*sqrt(M/5); % Cart height
wr = .2;          % Wheel radius
mr = .3*sqrt(m);  % Mass radius

% Positions:
y_pos = wr/2+H/2; % cart vertical position
pendx = x_pos + L*sin(th);
pendy = y_pos - L*cos(th);

% Plot:
plot([-10 10],[0 0],'k','LineWidth',2), hold on % Draws a black ('k') horizontal line at y = 0 to represent the ground. The line spans from x = -10 to x = 10. 'LineWidth',2 makes the line thicker. 'hold on' ensures that subsequent objects (cart, wheels, pendulum) are plotted on top of this ground line rather than replacing it.
rectangle('Position',[x_pos-W/2,y_pos-H/2,W,H],'Curvature',0.1,'FaceColor',[0.5 0.5 1],'LineWidth',1.5); % Plot cart. Draws the cart as a rectangle centered at (x, y_p). Width = W, Height = H. (x-W/2, y_p-H/2) ensures the cart is centered at (x, y_p). ''Curvature',0.1' adds slight rounded corners (not a sharp-edged rectangle). ''FaceColor',[.5 0.5 1]' fills the cart with a blue-ish color ([R G B] format). ''LineWidth',1.5' sets the border thickness of the rectangle.
rectangle('Position',[x_pos-0.9*W/2,0,wr,wr],'Curvature',1,'FaceColor',[0 0 0],'LineWidth',1.5); % Plot left wheel. Centered slightly to the left of the cart (x-.9*W/2). ''Curvature',1' makes the rectangle a circle. ''FaceColor',[0 0 0]' fills it with black ([0 0 0]).
rectangle('Position',[x_pos+0.9*W/2-wr,0,wr,wr],'Curvature',1,'FaceColor',[0 0 0],'LineWidth',1.5); % Plot right wheel. Centered slightly to the right (x+.9*W/2-wr).  ''Curvature',1' makes the rectangle a circle. ''FaceColor',[0 0 0]' fills it with black ([0 0 0]).
plot([x_pos pendx],[y_pos pendy],'k','LineWidth',2); % Plot pendulum rod. Draws a black ('k') line from the pivot point (x, y_p) (attached to the cart) to the pendulum mass (pendx, pendy). 'LineWidth',2 makes the pendulum rod thicker.
rectangle('Position',[pendx-mr/2,pendy-mr/2,mr,mr],'Curvature',1,'FaceColor',[1 0.1 0.1],'LineWidth',1.5); % Plot pendulum mass. Represents the pendulum mass (a red circle) positioned at (pendx, pendy). Centered correctly using pendx-mr/2 and pendy-mr/2. ''Curvature',1' ensures the mass is circular. ''FaceColor',[1 0.1 0.1]' fills it with red ([1 0.1 0.1]).

% Configuration of the visualization settings:
axis([-5 5 -2 2.5]); axis equal % Sets the plot limits. X-axis: from -5 to 5 (defines how much horizontal space is visible). Y-axis: from -2 to 2.5 (defines the vertical range). 'axis equal' ensures that the aspect ratio is equal, meaning: 1 unit on the x-axis equals 1 unit on the y-axis and prevents distortion in the visualization.
set(gcf,'Position',[100 100 1000 400]) % Set the Figure Window Size. 'gcf (Get Current Figure)' refers to the current figure window. 'set(gcf, 'Position', [100 100 1000 400])' sets the size and position of the figure window: [100 100 1000 400] → [x y width height], (100, 100): Position of the window (from the bottom-left corner of the screen), 1000x400: Width = 1000 pixels, Height = 400 pixels. This makes the visualization wider, which is useful for animations.
drawnow, hold off % Refresh the Plot and Release Hold. 'drawnow' forces MATLAB to immediately update the figure and ensures smooth animations when looping through frames. 'hold off' stops holding previous plots and the next 'plot' command will clear the figure before drawing new content.





