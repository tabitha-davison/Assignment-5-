function modal_anal()
    
    % System parameters
 % define system parameters
    box_params = struct();
    box_params.m = 1;
    box_params.g = 9.81;
    box_params.k_list = [0.25; 0.25; 0.25; 0.25; 0.25; 0.25; 0.25; 0.25];
    box_params.l0_list = [0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25];
    box_params.P_world = [-3, -5, -6, -5, 2, 5, 5, 3;
                          -3, -3, 3, 4, 4, 3, -3, -3];
    box_params.P_box = [-4, -4, -4, -4, 4, 4, 4, 4;
                        -2, -2, 2, 2, 2, 2, -2, -2];

    % box width and height
    box_width = abs(box_params.P_box(1, 3) - box_params.P_box(1, 5));
    box_height = abs(box_params.P_box(2, 1) - box_params.P_box(2, 3));

    % calculating moment of inertia of box 
    box_params.I = (1/12)*(box_height^2 + box_width^2);
    
    t0 = 0;
    
    my_rate_func = @(t_in,V_in) box_rate_func_tabby(t_in,V_in,box_params);

    % wrapper function for solving equilibrium points
    V_dot_func = @(V0) box_rate_func_tabby(t0, V0, box_params);

    % State vector V intial conditions
    x0 = 1;
    y0 = 2;
    theta0 = pi/4;
    vx0 = 0;
    vy0 = 0;
    vtheta0 = 0;
    V = [x0; y0; theta0; vx0; vy0; vtheta0];

    % finding equilibrium points
    Veq = multiNewton(V_dot_func, V);

    J_approx = approximate_jacobian(@(V) box_rate_func_tabby(0, V, box_params), V);

    % wrapper function of the linear approximation
    my_linear_rate = @(t_in,V_in) J_approx*(V_in-Veq);

    Q = J_approx(4:end, 1:3);

    epsilon = 10^-3;
    tspan = [0, 10];
    V0 = Veq + epsilon*V;

    % linear vs nonlinear
    [t_listnonlinear, V_listnonlinear] = ode45(my_rate_func,tspan,V0);
    [t_listlinear, V_listlinear] = ode45(my_linear_rate,tspan,V0);


    % eigenvalues and eigenvector
    [U_mode,omega_n] = eig(Q);
    
    %small numbers
    epsilon = 1;
    V1 = Veq + epsilon*[U_mode(:, 1);0;0;0]; % theta modal shape
    V2 = Veq + epsilon*[U_mode(:, 2);0;0;0]; % x modal shape
    V3 = Veq + epsilon*[U_mode(:, 3);0;0;0]; % y modal shape
    
    x_modal = Veq(1)+epsilon*U_mode(3, 1)*cos(omega_n(1, 1)*t_listlinear);
    y_modal = Veq(2)+epsilon*U_mode(1, 2)*cos(omega_n(2, 2)*t_listlinear);
    theta_modal = Veq(3)+epsilon*U_mode(2, 3)*cos(omega_n(3, 3)*t_listlinear);
    
    % run the integration of nonlinear system
    [tx_nonlinear, Vx_modal] = ode45(my_rate_func, tspan, V2);

    [ty_nonlinear, Vy_modal] = ode45(my_rate_func, tspan, V3);

    [ttheta_nonlinear, Vtheta_modal] = ode45(my_rate_func, tspan, V1);

    figure(1)
    clf
    subplot(3,1,1)
    hold on
    plot(t_listlinear, x_modal, '-r')
    plot(tx_nonlinear, Vx_modal(:,1), '-b')
    title('Predicted Vibration Mode vs. Simulated Nonlinear Behavior')
    xlabel('Time (s)')
    ylabel('X-Position')
    legend('Linear','Nonlinear')
    hold off
    
    subplot(3,1,2)
    hold on
    plot(t_listlinear,y_modal, '-r')
    plot(ty_nonlinear, Vy_modal(:,2), '-b')
    title('Predicted Vibration Mode vs. Simulated Nonlinear Behavior')
    xlabel('Time (s)')
    ylabel('Y-Position')
    legend('Linear','Nonlinear')
    hold off
    
    subplot(3,1,3)
    hold on
    plot(t_listlinear, theta_modal, '-r')
    plot(ttheta_nonlinear, Vtheta_modal(:,3), '-b')
    title('Predicted Vibration Mode vs. Simulated Nonlinear Behavior')
    xlabel('Time (s)')
    ylabel('Theta')
    legend('Linear','Nonlinear')
    hold off

end