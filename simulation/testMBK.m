clear; clc; close all;
% Given initial conditions
x_0 = 0.5;     % Initial displacement
v_0 = -3.5;    % Initial velocity

% Constants calculation
k=10;
m= 1.837;
alpha = sqrt(k/m);   % Define your alpha value

% Calculate A and B
A = x_0;
B = v_0 + alpha * A;

% Time range
t = 0:0.1:10;  % Define your desired time range

% Calculate displacement x(t)
x = v_0 * t .* exp(-alpha * t)+x_0;

% Calculate velocity x_dot(t)
x_dot = (A + B * t) .* exp(-alpha * t)+0.5;
% x_dot = (A + B * t) .* exp(-alpha * t) + (v_0 - A);
% Plotting displacement

figure;
hold on;
plot(t, x, 'b', 'LineWidth', 1.5);  % Plot displacement in blue
% plot(t, x_dot, 'r', 'LineWidth', 1.5);  % Plot velocity in red
hold off;

title('Displacement and Velocity vs Time');
xlabel('Time');
ylabel('Displacement / Velocity');
legend('Displacement (x(t))', 'Velocity (x_dot(t))');
grid on;