function [root] = multiNewton(test_function01,X0,varargin)
    %{
    arguements    
        func: testing function containing equations for line and derivative
        x0: initial guess/ starting x value
    returns
        x: approximated zero
    %}

    % set up stopping conditions
        % 1. for small change in x value
        % 2. for small abs(y) value
        % 3. for derivative of zero

    thresholds = [10^(-14), 10^(-14), 10^(-14)];

    max_iterations = 50;
    i = 0;
    use_analytical_jacobian = nargin==3 && varargin{1}(1);
    
    if use_analytical_jacobian == true
        [fx0, J0] = test_function01(X0);
    else
        fx0 = test_function01(X0);
        J0 = approximate_jacobian(test_function01, X0);
    end
    a = 0;
    while i < max_iterations
        i = i+1;

        % zero slope check
        if abs(det(J0*J0')) <= thresholds(1)
            x = 1
            return
        end

        % calculate new x
        X_delta =  - J0\fx0;
        X = X0 + X_delta;
        root = X;

        % check if close enough to a zero
        if use_analytical_jacobian == true
            [fx, J] = test_function01(X);
            
        else
            fx = test_function01(X);
            J = approximate_jacobian(test_function01, X);
        end
        a = a + 1;
        if or(norm(X_delta) < thresholds, norm(fx) < thresholds)
            root = X;
            fx1 = fx;
            return
        end
        
        % update x value
        X0 = X;
        fx0 = fx;
        J0 = J;
        
    end

end

function J = approximate_jacobian(fun, x)
    % Set the step size for numerical differentiation
    delta_x = 1e-6;
    n = length(x);          % number of inputs (size of x)
    f0 = fun(x);            % compute the output at x
    size = length(f0);         % number of outputs (size of fun(x))
    
    % Initialize the Jacobian matrix with zeros of size m-by-n
    J = zeros(size, n);
    for j = 1:n
        e_j = zeros(n, 1);
        e_j(j) = 1;
        
        % Compute the function values at x + delta_x * e_j and x - delta_x * e_j
        f_plus = fun(x + delta_x * e_j);
        f_minus = fun(x - delta_x * e_j);
        
        % Approximate the j-th column of the Jacobian
        J(:, j) = (f_plus - f_minus) / (2 * delta_x);
    end

end