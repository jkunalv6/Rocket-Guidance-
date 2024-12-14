% Parameters setup
params.gravity = [0; 0; -9.81]; % Gravitational acceleration in m/s^2
params.Cd = 0.5; % Drag coefficient
params.rho = 1.225; % Air density in kg/m^3
params.A = 0.1; % Cross-sectional area in m^2
params.mass = @(t) max(50 - 0.1 * t , 30 ) ; % Mass as a function of time
params.thrust = @(t) [0; 0; 1000 *(t<30)   ]; % Constant thrust force in N
y0 = [0; 0; .1; 0; 0; 0];  
params.wind_speed_range = [-20,20];
params.gust_duration = 2;
params.gust_interval = 5;


% Time span for simulation
tspan = [0, 100];  
% Run the simulation

[t, y] = ode45(@(t, y) rocket_dynamics(t, y, params), tspan, y0);

% Plot the trajectory

subplot(2,2,1)
plot(t, y(:,3), 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Z-Position (m)');
title('Rocket Altitude Over Time');
grid on;
subplot(2,2,2)
plot(t, y(:,6), 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Z-Position (m)');
title('Rocket Velocity Over Time');
grid on;
subplot(2,2,3)
plot(t, y(:,4), 'LineWidth', 2);
xlabel('Time (s)');
ylabel('X-  Speed ');
title('X Over Time');
grid on;
subplot(2,2,4)
plot(t, y(:,5), 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Y_ Wind Speed');
title('Y speed Over Time');
grid on;
figure;
plot3(y(:,1), y(:,2), y(:,3), 'LineWidth', 2);
xlabel('X Position (m)');
ylabel('Y Position (m)');
zlabel('Z Position (m)');
grid on;
title('Rocket Trajectory');
