
function non_linear_system()

    LW = 10; LH = 1; LG = 3;
    m = 1; Ic = (1/12)*(LH^2+LW^2);
    g = 1; k = 20; k_list = [.5*k,.5*k,2*k,5*k];
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
    P_world = [Pbl1_world,Pbl2_world,Pbr1_world,Pbr2_world];
    P_box = [Pbl_box,Pbl_box,Pbr_box,Pbr_box];
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

    my_linear_rate = @(t_in,V_in) J_approx*(V_in-V_eq);

    Veq = multiNewton(my_linear_rate,0);

    dx0 = 1.5;
    dy0 = 1;
    dtheta0 = 0.435;
    vx0 = 0;
    vy0 = 0;
    vtheta0 = 0;

    %small number used to scale initial perturbation
    epsilon = 0.000001;
    V0 = Veq + epsilon*[dx0;dy0;dtheta0;vx0;vy0;vtheta0];
    tspan = [0 10];
    h_ref = 0.1;
     
 my_rate_func = @(t_in, V_in) box_rate_func_tabby(t_in, V_in,box_params);
    
%  [tlist_nonlinear,Vlist_nonlinear] = your_integrator(my_rate_func,tspan,V0,...);

 [t_list_nonlinear, Vlist_nonlinear, ~, ~] = fixed_step_integration(my_rate_func, step_func, tspan, V0, h_ref)

  [t_list_linear, Vlist_linear, ~, ~] = fixed_step_integration(my_linear_rate, step_func, tspan, V0, h_ref)

%  [tlist_linear,Vlist_linear] = your_integrator(my_linear_rate,tspan,V0,...);

end