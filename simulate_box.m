function simulate_box()

    LW = 10; LH = 1; LG = 3;
    m = 0.3; Ic = (1/12)*(LH^2+LW^2);
    g = 9.81; k = 20; k_list = 200*[5*k,5*k,5*k,5*k,5*k,5*k,5*k,5*k];
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

    %load the system parameters into the rate function
    %via an anonymous function
    my_rate_func = @(t_in,V_in) box_rate_func_tabby(t_in,V_in,box_params);
    x0 = 0;
    y0 = 0;
    theta0 = pi/6;
    vx0 = 5;
    vy0 = 5;
    omega0 = 0;
    
    V0 = [x0;y0;theta0;vx0;vy0;omega0];
    tspan = [0 10];

    %run the integration
    [tlist,Vlist] = ode45(my_rate_func,tspan,V0);

    num_zigs = 5;
    w = .1;

    for i = 1:(length(Vlist)-1)

        box_corners = compute_rbt_tabby(Vlist(i,1), Vlist(i,2), Vlist(i,3), boundary_pts);

        clf;
        hold on;
        axis equal;
        axis([-15, 15, -15, 15]);

        plot(box_params.P_world(1, :), box_params.P_world(2, :), 'ko', 'MarkerFaceColor', 'k');  % Static anchor points
        plot([box_corners(1, :), box_corners(1, 1)], [box_corners(2, :), box_corners(2, 1)], 'b-', 'LineWidth', 2);

        spring_plot_struct = initialize_spring_plot(num_zigs, w);

        update_spring_plot(spring_plot_struct, box_params.P_world(:, 1), box_corners(:, 1));
        spring_plot_struct = initialize_spring_plot(num_zigs, w);

        update_spring_plot(spring_plot_struct, box_params.P_world(:, 2), box_corners(:, 1));
        spring_plot_struct = initialize_spring_plot(num_zigs, w);

        update_spring_plot(spring_plot_struct, box_params.P_world(:, 3), box_corners(:, 2));
        spring_plot_struct = initialize_spring_plot(num_zigs, w);

        update_spring_plot(spring_plot_struct, box_params.P_world(:, 4), box_corners(:, 2));
        spring_plot_struct = initialize_spring_plot(num_zigs, w);

        update_spring_plot(spring_plot_struct, box_params.P_world(:, 5), box_corners(:, 4));
        spring_plot_struct = initialize_spring_plot(num_zigs, w);

        update_spring_plot(spring_plot_struct, box_params.P_world(:, 6), box_corners(:, 4));
        spring_plot_struct = initialize_spring_plot(num_zigs, w);

        update_spring_plot(spring_plot_struct, box_params.P_world(:, 7), box_corners(:, 3));
        spring_plot_struct = initialize_spring_plot(num_zigs, w);

        update_spring_plot(spring_plot_struct, box_params.P_world(:, 8), box_corners(:, 3));

        drawnow;

    end
end