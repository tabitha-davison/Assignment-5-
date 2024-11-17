function modal_anal(Q, Veq, my_rate_func)

    [U_mode,omega_n] = eig(Q, my_rate_func);
    
    %small numbers
    epsilon = 1e-5;
    V0 = Veq + epsilon*[U_mode;0;0;0];
    tspan = linspace(1,200,1);
    
    %run the integration of nonlinear system
    [tlist_nonlinear,Vlist_nonlinear] = ode45(my_rate_func,tspan,V0);
    
    x_modal = Veq(1)+epsilon*U_mode(1)*cos(omega_n*tlist_nonlinear);
    y_modal = Veq(2)+epsilon*U_mode(2)*cos(omega_n*tlist_nonlinear);
    theta_modal = Veq(3)+epsilon*U_mode(3)*cos(omega_n*tlist_nonlinear);
    
    figure(1)
    hold on
    plot(x_modal, tlist_nonlinear, 'or')
    plot(Vlist_nonlinear(1), tlist_nonlinear, '-b')
    title('Predicted Vibration Mode vs. Simulated Nonlinear Behavior')
    xlabel('Time (t)')
    ylabel('X-Axis')
    legend('Modal','nonlinear')
    hold off
    
    figure(2)
    hold on
    plot(y_modal, tlist_nonlinear, 'or')
    plot(Vlist_nonlinear(2), tlist_nonlinear, '-b')
    title('Predicted Vibration Mode vs. Simulated Nonlinear Behavior')
    xlabel('Time (t)')
    ylabel('X-Axis')
    legend('Modal','nonlinear')
    hold off
    
    figure(3)
    hold on
    plot(theta_modal, tlist_nonlinear, 'or')
    plot(Vlist_nonlinear(3), tlist_nonlinear, '-b')
    title('Predicted Vibration Mode vs. Simulated Nonlinear Behavior')
    xlabel('Time (t)')
    ylabel('X-Axis')
    legend('Modal','nonlinear')
    hold off

end