function dydt = rocket_dynamics(t, y, params)
    % Unpack state variables
    x = y(1:3);  % Position vector [x, y, z]
    v = y(4:6);  % Velocity vector [vx, vy, vz]

    % Compute forces
    m = params.mass(t); % Mass at time t
    F_gravity = m * params.gravity; % Gravitational force vector (3x1)
    F_drag = -0.5 * params.Cd * params.rho * params.A * norm(v) * v; % Drag force (3x1)
    F_thrust = params.thrust(t); % Thrust force (3x1)

    persistent last_gust_time wind_speed 
        if isempty(last_gust_time)
            last_gust_time = 0;
            wind_speed =0 ;
        end 
    if t - last_gust_time >= params.gust_interval 
        wind_speed = params.wind_speed_range(1) + (params.wind_speed_range(2) - params.wind_speed_range(1))* rand ;
        last_gust_time = t;
    elseif t - last_gust_time >= params.gust_duration
        wind_speed = 0 ;
    end 


    ang =  (2*rand* round(rand) -1)*rand ;
    in_ang =(ang - 1);


    F_wind = -params.Cd* norm([v(1) + wind_speed*ang ,v(2) + wind_speed*in_ang, v(3)])* [v(1) + wind_speed*ang ;v(2) + wind_speed*in_ang; 0];

    % Compute net force and acceleration
    F_net = F_thrust + F_drag + F_gravity +F_wind*.1; % Net force (3x1)
    a = F_net / m; % Acceleration (3x1)

    % Compute derivatives
    dxdt = v; % Velocity is the time derivative of position (3x1)
    dvdt = a; % Acceleration is the time derivative of velocity (3x1)



    % Concatenate into a single column vector (6x1)
    dydt = [dxdt; dvdt]; % 6x1 column vector

    if x(3) <= 0 
        dydt = zeros(6,1);
        return ;
    end 

end
