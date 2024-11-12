%Computes the rigid body transformation that maps a set
%of points in the box-frame to their corresponding
%world-frame coordinates
%INPUTS:
%x: the x position of the centroid of the box
%y: the y position of the centroid of the box
%theta: the orientation of the box
%Plist_box: a 2 x n matrix of points in the box frame
%OUTPUTS:
%Plist_world: a 2 x n matrix of points describing
%the world-frame coordinates of the points in Plist_box
function Plist_world = compute_rbt_tabby(x,y,theta,Plist_box)

Rotation_matrix = [cos(theta), -sin(theta); sin(theta), cos(theta)];

Plist_world = Rotation_matrix.*Plist_box + repmat([x;y],[1,size(Plist_box,2)]);

% Plist_world = Rotation_matrix.*Plist_box;
% Plist_world(1,:)=Plist_world(1,:)+x;
% Plist_world(2,:)=Plist_world(2,:)+y;

end