function simulate_box()
    %define system parameters
    box_params = struct();
    box_params.m = 2
    box_params.I = 4
    box_params.k_list = 
    box_params.l0_list = 
    box_params.P_world = 
    box_params.P_box = 
    %load the system parameters into the rate function
    %via an anonymous function
    my_rate_func = @(t_in,V_in) box_rate_func(t_in,V_in,box_params);
    x0 = 0;
    y0 = 0;
    theta0 = 0.78;
    vx0 = 2;
    vy0 = 4;
    omega0 = 2;
    
    V0 = [x0;y0;theta0;vx0;vy0;omega0];
    tspan = [0 10];

    %run the integration
    [tlist,Vlist] = ode45(my_rate_func,tspan,V0)
end