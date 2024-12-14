function rotated_thrust = rotate_thrust(base_vector, tau_x,tau_y)
    Rx = [1,0,0;
           0,cos(tau_x),-sin(tau_x);
           0,sin(tau_x),cos(tau_x)];

    Ry = [cos(tau_y),0,sin(tau_y);
           0,1,0;
           -sin(tau_y),0,cos(tau_y)];
        
    rotated_thrust = Ry*Rx;
end
