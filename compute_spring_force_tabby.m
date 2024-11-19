%computes the force exerted by the spring at one of its ends
%INPUTS:
%PA: the position of the first end of the spring
%PB: the position of the second end of the spring
%k: the stiffness of the spring
%l0: the natural length of the spring
%OUTPUTS:
%F: the force exerted by the spring at end B
function F = compute_spring_force_tabby(k,l0,PA,PB)
    %current length of the spring
    lx = (PB(1)-PA(1));
    ly = (PB(2)-PA(2));
    l = sqrt(lx^2+ly^2);
    %unit vector pointing from PA to PB
    e_s = (PB-PA)/l;
    %Force exerted by spring at point B
    F = -k*(l -l0).*e_s; 
end