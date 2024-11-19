clear;
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
omega0 = 0;

V0 = [x0;y0;theta0;vx0;vy0;omega0];
tspan = [0 10];

my_rate_func = @(t_in, V_in) box_rate_func_tabby(t_in, V_in, box_params);

V_eq = multiNewton(@(V) box_rate_func_tabby(0, V, box_params), V0);

J_approx = approximate_jacobian(@(V) box_rate_func_tabby(0, V, box_params), V_eq);

my_linear_rate = @(t_in, V_in) J_approx * (V_in - V_eq);


dx0 = 7; dy0 = 5; dtheta0 = 0.9;
vx0 = 0; vy0 = 0; vtheta0 = 0;
epsilon = 0.1;
V0 = V_eq + epsilon * [dx0; dy0; dtheta0; vx0; vy0; vtheta0];

tspan = [0, 10];

[t_list_nonlinear, Vlist_nonlinear] = ode45(my_rate_func, tspan, V0);
[t_list_linear, Vlist_linear] = ode45(my_linear_rate, tspan, V0);

% Plot results
figure;
subplot(3, 1, 1);
plot(t_list_linear, Vlist_linear(:,1), 'r--', t_list_nonlinear, Vlist_nonlinear(:,1), 'b-');
title('Comparison of x Displacements');
legend('Linear', 'Nonlinear');
xlabel('Time (s)');
ylabel('x Displacement');

subplot(3, 1, 2);
plot(t_list_linear, Vlist_linear(:,2), 'r--', t_list_nonlinear, Vlist_nonlinear(:,2), 'b-');
title('Comparison of y Displacements');
legend('Linear', 'Nonlinear');
xlabel('Time (s)');
ylabel('y Displacement');

subplot(3, 1, 3);
plot(t_list_linear, Vlist_linear(:,3), 'r--', t_list_nonlinear, Vlist_nonlinear(:,3), 'b-');
title('Comparison of θ Displacements');
legend('Linear', 'Nonlinear');
xlabel('Time (s)');
ylabel('θ Displacement');

disp('Simulation complete.');
