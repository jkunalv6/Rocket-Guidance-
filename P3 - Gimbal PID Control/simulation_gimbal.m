Kp = 1.0 ;
Ki = .3;
Kd = 0.09 ;

Kp_pitch = 1.2 ;
Ki_pitch = .3;
Kd_pitch = 0.09 ;

target_alt = 1000;

params.gravity = -9.81; % Gravitational acceleration in m/s^2
params.Cd = 0.5; % Drag coefficient
params.rho = 1.225; % Air density in kg/m^3
params.A = 0.1; % Cross-sectional area in m^2
params.mass = @(t) max(50 - 0.1 * t , 30 ) ; % Mass as a function of time
params.thrust = 1000.00  ; % Constant thrust force in N
y0 = [0;0;1;0;0;0];  
integral = 0;
prev_error = 0;
prev_derivative = 0;
integral_pitch = 0;
prev_error_pitch = 0;
prev_derivative_pitch = 0;
params.wind_speed_range = [-40,40];
params.gust_duration = 2;
params.gust_interval = 5;

% Time span for simulation
tspan = [0, 10];  
[t,y] = ode45(@(t,y)rocket_guidance_gimbal_pitch(t,y,params,target_alt,Kp,Ki,Kd,Kp_pitch,Ki_pitch,Kd_pitch,prev_error,integral,prev_error_pitch,integral_pitch),tspan, y0);


subplot(2,2,1)
plot(t, y(:,3), 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Z-Position (m)');
title('Rocket Altitude Over Time');
grid on;
subplot(2,2,2)
plot(t, y(:,2), 'LineWidth', 2);
xlabel('Time (s)');
ylabel('y-Position (m)');
title('Rocket Y pos Over Time');
grid on;
subplot(2,2,3)
plot(t, y(:,1), 'LineWidth', 2);
xlabel('Time (s)');
ylabel('X-Position (m)');
title('Rocket X pos Over Time');
grid on;
figure;
plot3(y(:,1), y(:,2), y(:,3), 'LineWidth', 2);
xlabel('X Position (m)');
ylabel('Y Position (m)');
zlabel('Z Position (m)');
grid on;
title('Rocket Trajectory');