
function attitude_kinematics_321(time_pts, omega_aee, initial_euler_angles)

%     dt = time_pts(2) - time_pts(1);
    dt = 1;
    euler_angles = initial_euler_angles;
%     angles_list = zeros(numel(time_pts) - 1, 4);
%     angles_list(1, 1) = 0;
%     angles_list(1, 2) = euler_angles(1);
%     angles_list(1, 3) = euler_angles(2);
%     angles_list(1, 4) = euler_angles(3);
    
    for m1 = 1:numel(time_pts) % - 1 
         
%         dt = time_pts(m1 + 1) - time_pts(m1);
        omega_aee_t = omega_aee(:, m1);
        euler_angles = rk4_step(euler_angles);
%         angles_list(m1, 1) = time_pts(m1);
%         angles_list(m1, 2) = euler_angles(1);
%         angles_list(m1, 3) = euler_angles(2);
%         angles_list(m1, 4) = euler_angles(3);
        fprintf('Psi: {%.4f}, Theta: {%.4f}, Phi: {%.4f}, Time = {%.2f}\n', euler_angles(1), euler_angles(2), euler_angles(3), time_pts(m1))

        
    end
   
    function x_dot = f_x(x_t)
        
        e_thta = x_t(2)*pi/180;
        e_phi = x_t(3)*pi/180;
       
        x_dot = [-sin(e_thta) 0 1; 
                sin(e_phi)*cos(e_thta) cos(e_phi) 0;
                cos(e_phi)*cos(e_thta) -sin(e_phi) 0] \ omega_aee_t;
        
    end

    function x_t_plus_dt = rk4_step(x_t)
       
        k1 = dt * f_x(x_t);
        k2 = dt * f_x(x_t + 0.5*k1);
        k3 = dt * f_x(x_t + 0.5*k2);
        k4 = dt * f_x(x_t + k3);
        
        x_t_plus_dt = x_t + (1/6)*k1 + (1/3)*k2 + (1/3)*k3 + (1/6)*k4;       
    end

end
