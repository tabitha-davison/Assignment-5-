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
%OUTPUTS
%ax: x acceleration of the box
%ay: y acceleration of the box
%atheta: angular acceleration of the box
function [ax,ay,atheta] = compute_accel_tabby(x,y,theta,box_params)

    num_of_springs = length(k_list);
    % Force_x= [];
    % Force_y= [];
    
    PC = [x;y];
    PA_list = box_params.P_world;
    PB_list = compute_rbt_tabby(x,y,theta,box_params.P_box);

    force_total = [0;-box_params.m*box_params.g];
    torque_total = 0;

    for i = 1:length(num_of_springs)
        k = box_params.k_list(i);
        l0 = box_params.l0_list(i);
        PA = PA_list(:,i);
        PB = PB_list(:,i);

        moment_arm = PB-PC;

        spring_force = compute_spring_force_tabby(k,l0,PA,PB);
        spring_torque = moment_arm(1)*spring_force(2)-moment_arm(2)*spring_force(1);
        
        force_total = force_total+spring_force;
        torque_total = torque_total+spring_torque;
        

    
    end
        
        ax = force_total(1)/box_params.m;
        ay = force_total(2)/box_params.m;
        atheta = torque_total/box_params.I;
end