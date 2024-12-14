function dydt= rocket_dynamics(t,y,params,target_alt,Kp,Ki,Kd,Kp_pitch,Ki_pitch,Kd_pitch,prev_error,integral,integx , integy)
    x = y(1:3);
    v = y(4:6);
    disp('');
    persistent last_time 
    if isempty(last_time)
        last_time = 0 ;
    end
    dt = t - last_time ;
   
    last_time = t;

    x1 = 1000 - x(1);
    y1 = 770 - x(2); 
    z1 = target_alt - x(3);
    t_direction = [x1 ; y1 ; z1];

    t_direction = t_direction/norm(t_direction);

    tx1 =atan2(t_direction(2), t_direction(3));
    ty1 = atan2(t_direction(1), t_direction(3));
    
    tx2 = atan2(v(2),v(3));
    ty2 = atan2(v(1),v(3));

    dtx = tx1 -tx2 ;
    dty = ty1 - ty2 ;
    derivx = dtx/dt ;
    derivy = dty/dt;
    min_der_A = -10;
    max_der_A = 10;
    derivx = max(min(derivx,max_der_A),min_der_A);
    derivy = max(min(derivy,max_der_A),min_der_A);

    integx = integx + dtx*dt ;
    integy = integy + dty*dt ;

    tau_x =  dtx*Kp_pitch  ;
    tau_y = dty*Kp_pitch  ;
    

    tau_x = max(min(tau_x, pi/6), -pi/6);
    tau_y = max(min(tau_y, pi/6), -pi/6);
    error = target_alt - x(3);
    max_error = 1000;  % Set a maximum limit for the error
    error = min(max(error, -max_error), max_error);

    derivative = (error- prev_error)/dt;
    min_der = -1000;
    max_der = 1000;
    derivative = max(min(derivative,max_der),min_der);

    integral = integral + error*dt ;
    max_integral = 100;  % Set a maximum limit for the integral term
    min_integral = -100;  % Set a minimum limit for the integral term

    integral = max(min(integral, max_integral), min_integral);

    control_output = error*Kp + integral*Ki + derivative*Kd  ;
    if isinf(control_output)
        control_output = 0;
    else
        control_output = round(control_output);
    end

    th = params.thrust + control_output;
    mass = params.mass(t);
    thrust1 =[0;0 ; th];
    Rx = [1,0,0;
           0,cos(tau_x),-sin(tau_x);
           0,sin(tau_x),cos(tau_x)];

    Ry = [cos(tau_y),0,sin(tau_y);
           0,1,0;
           -sin(tau_y),0,cos(tau_y)];
        
    R = Ry*Rx;
    
    
    F_thrust = R* thrust1;

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
    in_ang =(ang - 10*rand);


    F_wind = -params.Cd* norm([v(1) + wind_speed*ang ,v(2) + wind_speed*in_ang, v(3)])* [v(1) + wind_speed*ang ;v(2) + wind_speed*in_ang; 0];
    F_liftx = 0.5*params.Cl*params.rho*cos(tx1)*params.A1*v(1)^2 ;
    F_lifty = 0.5*params.Cl*params.rho*sin(ty1)*params.A1*v(2)^2 ;
    F_lift = [F_liftx; F_lifty; 0];
   
    F_gravity = [0;0 ; params.gravity*mass];
    F_drag = -0.5*params.Cd*params.A*norm(v)*v*params.rho;
    F_total =  F_thrust + F_drag + F_gravity  ;
    a = F_total/mass;
    dydt = [v;a];
    prev_error = error ;

 

    disp(['Altitutude ', num2str(x(3))]);
    disp(['time: ', num2str(t)]);


end 