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

    x0 = 0;
    y0 = 0;
    theta0 = pi/6;
    vx0 = 5;
    vy0 = 5;
    omega0 = 6;

    V0 = [x0;y0;theta0;vx0;vy0;omega0];

    box_rate_func_tabby(0, V0, box_params)

    J_approx = approximate_jacobian(@(V) box_rate_func_tabby(0, V, box_params), V0);
    Veq = multiNewton(@(V) box_rate_func_tabby(0, V, box_params), V0);
    Q = J_approx(4:6, 1:3);
    tspan = [0, 10];
    
    modal_anal_function(Q, Veq, @(t, V) box_rate_func_tabby(0, V, box_params), tspan)

end


function modal_anal_function(Q, Veq, my_rate_func, t)

    [U_mode,omega_n] = eig(Q);
    
    %small numbers
    epsilon = 1e-5;
    V0 = Veq + epsilon*[U_mode(:, 1);0;0;0];
    tspan = linspace(t(1),1,t(end));
    
    %run the integration of nonlinear system
    [tlist_nonlinear,Vlist_nonlinear] = ode45(my_rate_func,tspan,V0);
    omega_n = diag(omega_n);
    
    x_modal = Veq(1)+epsilon*U_mode(1)*cos(omega_n(1)*tlist_nonlinear);
    y_modal = Veq(2)+epsilon*U_mode(2)*cos(omega_n(2)*tlist_nonlinear);
    theta_modal = Veq(3)+epsilon*U_mode(3)*cos(omega_n(3)*tlist_nonlinear);
    
    figure(1)
    clf
    subplot(3,1,1)
    hold on
    plot(tlist_nonlinear, x_modal, 'or')
    plot( tlist_nonlinear, Vlist_nonlinear(1), 'ob')
    title('Predicted Vibration Mode vs. Simulated Nonlinear Behavior')
    xlabel('Time (t)')
    ylabel('X-Axis')
    legend('Modal','Nonlinear')
    hold off
    
    subplot(3,1,2)
    hold on
    plot(tlist_nonlinear, y_modal, 'or')
    plot(tlist_nonlinear, Vlist_nonlinear(2), 'ob')
    title('Predicted Vibration Mode vs. Simulated Nonlinear Behavior')
    xlabel('Time (t)')
    ylabel('Y-Axis')
    legend('Modal','nonlinear')
    hold off
    
    subplot(3,1,3)
    hold on
    plot(tlist_nonlinear, theta_modal, 'or')
    plot(tlist_nonlinear, Vlist_nonlinear(3), 'ob')
    title('Predicted Vibration Mode vs. Simulated Nonlinear Behavior')
    xlabel('Time (t)')
    ylabel('Theta')
    legend('Modal','nonlinear')
    hold off

end