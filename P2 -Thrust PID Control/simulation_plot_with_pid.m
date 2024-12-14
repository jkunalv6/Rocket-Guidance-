Kp = 0.8 ;
Ki = .3;
Kd = 0.09 ;


target_alt = 1000;

params.gravity = -9.81; % Gravitational acceleration in m/s^2
params.Cd = 0.5; % Drag coefficient
params.rho = 1.225; % Air density in kg/m^3
params.A = 0.1; % Cross-sectional area in m^2
params.mass = @(t) max(50 - 0.1 * t , 30 ) ; % Mass as a function of time
params.thrust = 1000.00  ; % Constant thrust force in N
y0 = [0; 0; 0; 0; 0; 0];  
integral = 0;
prev_error = 0;
prev_derivative = 0;

% Time span for simulation
tspan = [0, 10];  
[t,y] = ode45(@(t,y) rocket_dynamics_with_pid(t,y,params,target_alt,Kp,Ki,Kd,prev_error,integral),tspan, y0);


figure;
plot(t, y(:,3), 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Z-Position (m)');
title('Rocket Altitude Over Time');
grid on;
figure;
plot3(y(:,1), y(:,2), y(:,3), 'LineWidth', 2);
xlabel('X Position (m)');
ylabel('Y Position (m)');
zlabel('Z Position (m)');
grid on;
title('Rocket Trajectory');