function box_corners = simulate_box_2()
    % Define system parameters
    % box_params = struct();
    % box_params.m = 1; % mass of the box
    % box_params.I = (1/12) * (1^2 + 10^2); % moment of inertia for box (replace with actual dimensions if different)
    % box_params.g = 1; % gravitational acceleration
    % box_params.k_list = [.5*20, .5*20, 2*20, 5*20]; % spring constants
    % box_params.l0_list = 1.5 * 3 * ones(size(box_params.k_list)); % unstretched spring lengths
    % box_params.P_world = [ [-5; -0.5], [-5; 0.5], [0; -2.25], [2.25; 0] ]; % anchor points in world coordinates
    % box_params.P_box = [ [-5; -0.5], [-5; 0.5], [5; 0.5], [0; 0] ]; % attachment points in box coordinates
    
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
    Ptl1_world = Ptl_box + [-l0; l0];
    Ptr1_world = Pbr_box + [l0;l0];
    
    P_world = [Pbl1_world,Pbl2_world,Pbr1_world,Pbr2_world, Ptl1_world, Ptr1_world];
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
    % Define initial conditions
    x0 = 0; % initial x-position
    y0 = 0; % initial y-position
    theta0 = 0; % initial angle (in radians)
    vx0 = 0; % initial x-velocity
    vy0 = 0; % initial y-velocity
    vtheta0 = 0; % initial angular velocity
    V0 = [x0; y0; theta0; vx0; vy0; vtheta0]; % initial state vector
    
    % Time span for simulation
    tspan = [0, 10]; % define the time interval
    tspan = linspace(tspan(1), tspan(2), 200);
    % Anonymous function handle for the rate function
    my_rate_func = @(t_in, V_in) box_rate_func_tabby(t_in, V_in, box_params);
    
    % Run the integration (replace with your integration method)
    [tlist, Vlist] = ode45(my_rate_func, tspan, V0); % Example with ode45
    
    % Spring plotting initialization
    num_zigs = 5; % Number of zig-zags in the spring
    w = .1; % Width of the spring
    spring_plot_struct = initialize_spring_plot(num_zigs, w);
    
    % Set up the plot
    figure;
    hold on;
    axis equal;
    axis([-10, 10, -7, 7]);
    
    % Draw the box and springs in each time step
    for i = 1:length(tlist)
        % Unpack current state
        x = Vlist(i, 1);
        y = Vlist(i, 2);
        theta = Vlist(i, 3);
        box_corners = compute_rbt_tabby(x, y, theta, boundary_pts);

        
        % Clear figure but avoid deleting plot objects
        clf;
        hold on;
        axis equal;
        axis([-10, 10, -7, 7]);
        plot(box_params.P_world(1, :), box_params.P_world(2, :), 'ko', 'MarkerFaceColor', 'k');  % Static anchor points
        
        % Plot the box (closing the loop)
        plot([box_corners(1, :), box_corners(1, 1)], ...
             [box_corners(2, :), box_corners(2, 1)], 'b-', 'LineWidth', 2);
        
        % Plot springs connecting the box to the anchor points with the zig-zag style
        % for j = 1:size(box_params.P_world)
        % spring_plot_struct = initialize_spring_plot(num_zigs, w);
        % 
        %     % Update spring with zig-zag pattern
        %     P1 = box_params.P_world(:, j);
        %     P2 = box_corners(:, j);
        %     update_spring_plot(spring_plot_struct, P1, P2);
        %     hold on;
        % end
        % for j = 1:size(box_params.P_world, 2)
        %     spring_plot_struct = initialize_spring_plot(num_zigs, w);
        %     update_spring_plot(spring_plot_struct, box_params.P_world(:, j), box_corners(:, j));
        % end
        spring_plot_struct = initialize_spring_plot(num_zigs, w);
        S1 = box_params.P_world(:, 1);
        S2 = box_corners(:, 1);
        update_spring_plot(spring_plot_struct, S1, S2);
        spring_plot_struct = initialize_spring_plot(num_zigs, w);
        S3 = box_params.P_world(:, 2);
        S4 = box_corners(:, 1);
        update_spring_plot(spring_plot_struct, S3, S4);
        spring_plot_struct = initialize_spring_plot(num_zigs, w);
        S5 = box_params.P_world(:, 3);
        S6 = box_corners(:, 2);
        update_spring_plot(spring_plot_struct, S5, S6);
        spring_plot_struct = initialize_spring_plot(num_zigs, w);
        S7 = box_params.P_world(:, 4);
        S8 = box_corners(:, 2);
        update_spring_plot(spring_plot_struct, S7, S8);
        spring_plot_struct = initialize_spring_plot(num_zigs, w);
        S9 = box_params.P_world(:, 6);
        S10 = box_corners(:, 3);
        update_spring_plot(spring_plot_struct, S9, S10);
        spring_plot_struct = initialize_spring_plot(num_zigs, w);
        S11 = box_params.P_world(:, 5);
        S12 = box_corners(:, 4);
        update_spring_plot(spring_plot_struct, S11, S12);
    % Update the plot
    drawnow;
    
    % Pause for a short duration to control the anim6ation speed
    %pause(0.01);
    end
    
end

function spring_plotting_example()

    num_zigs = 5;
    w = .1;
    hold on;
    spring_plot_struct = initialize_spring_plot(num_zigs,w);
    axis equal; axis square;
    axis([-3,3,-3,3]);
    for theta=linspace(0,6*pi,1000)
    P1 = [.5;.5];
    P2 = 2*[cos(theta);sin(theta)];
    update_spring_plot(spring_plot_struct,P1,P2)
    drawnow;
    end
end

%updates spring plotting object so that spring is plotted
%with ends located at points P1 and P2
function update_spring_plot(spring_plot_struct,P1,P2)
    dP = P2-P1;
    R = [dP(1),-dP(2)/norm(dP);dP(2),dP(1)/norm(dP)];
    plot_pts = R*spring_plot_struct.zig_zag;
    set(spring_plot_struct.line_plot,...
    'xdata',plot_pts(1,:)+P1(1),...
    'ydata',plot_pts(2,:)+P1(2));
    set(spring_plot_struct.point_plot,...
    'xdata',[P1(1),P2(1)],...
    'ydata',[P1(2),P2(2)]);
end

%create a struct containing plotting info for a single spring
%INPUTS:
%num_zigs: number of zig zags in spring drawing
%w: width of the spring drawing
function spring_plot_struct = initialize_spring_plot(num_zigs,w)
    spring_plot_struct = struct();
    zig_ending = [.25,.75,1; ...
    -1,1,0];
    zig_zag = zeros(2,3+3*num_zigs);
    zig_zag(:,1) = [-.5;0];
    zig_zag(:,end) = [num_zigs+.5;0];
    for n = 0:(num_zigs-1)
    zig_zag(:,(3+3*n):2+3*(n+1)) = zig_ending + [n,n,n;0,0,0];
    end
    zig_zag(1,:)=(zig_zag(1,:)-zig_zag(1,1))/(zig_zag(1,end)-zig_zag(1,1));
    zig_zag(2,:)=zig_zag(2,:)*w;
    spring_plot_struct.zig_zag = zig_zag;
    spring_plot_struct.line_plot = plot(0,0,'k','linewidth',2);
    spring_plot_struct.point_plot = plot(0,0,'ro','markerfacecolor','r','markersize',7);
end