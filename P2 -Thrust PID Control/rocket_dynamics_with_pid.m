function dydt= rocket_dynamics_with_pid(t,y,params,target_alt,Kp,Ki,Kd,prev_error,integral)
    x = y(1:3);
    v = y(4:6);
    disp('');
    persistent last_time 
    if isempty(last_time)
        last_time = 0 ;
    end

    dt = t - last_time ;
    last_time = t;
    disp(['dt: ', num2str(dt)]);
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
    disp(['th: ', num2str(th)]);
    disp(['output: ', num2str(control_output)]);
    mass = params.mass(t);
    F_thrust =[0;0 ; th];
    disp(['Thrust: ', num2str(th)]);
    F_gravity = [0;0 ; params.gravity*mass];
    F_drag = -0.5*params.Cd*params.A*norm(v)*v*params.rho;
    F_total =  F_thrust+ F_drag + F_gravity ;
    a = F_total/mass;
    dydt = [v;a];
    prev_error = error ;
 
    disp(['prev error: ', num2str(prev_error)]);
    disp(['Derivative: ', num2str(derivative)]);
    disp(['Altitutude ', num2str(x(3))]);
    disp(['error: ', num2str(error)]);

end 