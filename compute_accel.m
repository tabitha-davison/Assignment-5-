
%Computes the linear and angular acceleration of the box
%given its current position and orientation
%INPUTS:
%x: current x position of the box
%y: current y position of the box
%theta: current orientation of the box
%box_params: a struct containing the parameters that describe the system
%Fields:
%box_params.m: mass of the box
%box_params.I: moment of inertia w/respect to centroid
%box_params.g: acceleration due to gravity
%box_params.k_list: list of spring stiffnesses
%box_params.l0_list: list of spring natural lengths
%box_params.P_world: 2 x n list of static mounting
% points for the spring (in the world frame)
%box_params.P_box: 2 x n list of mounting points
% for the spring (in the box frame)
%
%box is wood .3mx.3m like 1 in thick  

%OUTPUTS
%ax: x acceleration of the box
%ay: y acceleration of the box
%atheta: angular acceleration of the box
function [ax,ay,atheta] = compute_accel(x,y,theta,box_params)
    % Unpack box parameters
    m = box_params.m;               % Mass of the box
    I = box_params.I;               % Moment of inertia about the centroid
    g = box_params.g;               % Gravity
    k_list = box_params.k_list;     % Spring stiffness list
    l0_list = box_params.l0_list;   % Natural lengths of the springs
    P_world = box_params.P_world   % Static spring mounting points in world frame
    P_box = box_params.P_box;       % Spring attachment points in box frame

    % Initialize total force and torque
    F_total = [0; -m * g];  % Initial force includes gravity acting on center of mass
    T_total = 0;            % Initial torque about the centroid

    % Transform P_box points into the world frame using the box position and orientation
    P_box_world = compute_rbt_tabby(x, y, theta, P_box);

    % Compute the position of the center of mass
    PC = [x; y];  % Position of the box's center of mass in world frame

    % Loop through each spring to compute forces and torques
    for i = 1:length(k_list)
        % Get spring parameters for the current spring
        k = k_list(i);
        l0 = l0_list(i);
        PA = P_world(:, i);        % Static mounting point in world frame
        PB = P_box_world(:, i);     % Attachment point in world frame (transformed)

        % Compute spring force vector in world frame
        spring_force = compute_spring_force_tabby(k, l0, PA, PB);

        % Sum the force contributions from each spring
        F_total = F_total + spring_force;

        % Compute the torque contribution for each spring about the centroid
        r_i = PB - PC;              % Vector from center of mass to attachment point
        T_total = T_total + cross([r_i; 0], [spring_force; 0]);  % Cross product in 3D
    end

    % Calculate linear accelerations
    ax = F_total(1) / m;
    ay = F_total(2) / m;

    % Calculate angular acceleration (z-component of torque / I)
    atheta = T_total(3) / I;
end





