% Linearization and Comparison of Nonlinear System
clear;
% System parameters
LW = 10; LH = 1; LG = 3;
m = 0.3; Ic = (1/12)*(LH^2+LW^2);
g = 9.81; k = 20; k_list = 20*[.5*k,.5*k,.5*k,.5*k,.5*k,.5*k,.5*k,.5*k];
l0 = 1.5*LG;
Pbl_box = [-LW;-LH]/2;
Pbr_box = [LW;-LH]/2;
Ptl_box = [-LW;LH]/2;
Ptr_box = [LW;LH]/2;
boundary_pts = [Pbl_box,Pbr_box,Ptr_box,Ptl_box,Pbl_box];
Pbl1_world = Pbl_box + [-LG;-LG];
Pbl2_world = Pbl_box + [LG;-LG];
Pbr1_world = Pbr_box + [0;-l0];
Pbr2_world = Pbr_box + [l0;0];
Ptl1_world = Ptl_box + [-l0; l0];
Ptl2_world = Ptl_box + [-l0; 0];
Ptr1_world = Pbr_box + [l0;l0];
Ptr2_world = Pbr_box + [0;l0];

P_world = [Pbl1_world,Pbl2_world,Pbr1_world,Pbr2_world, Ptl1_world, Ptl2_world, Ptr1_world, Ptr2_world];
P_box = [Pbl_box,Pbl_box,Pbr_box,Pbr_box, Ptl_box, Ptl_box, Ptr_box, Ptr_box];
%define system parameters
box_params = struct();
box_params.m = m;
box_params.I = Ic;
box_params.g = g;
box_params.k_list = k_list;
box_params.l0_list = l0*ones(size(P_world,2));
box_params.P_world = P_world;
box_params.P_box = P_box;
box_params.boundary_pts = boundary_pts;

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
epsilon = 1;
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
