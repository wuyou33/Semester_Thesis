function outer_pd(k_dxi_in, k_xi_in, k_omega_fix, k_eta_fix)

    clc;
    K_omega = k_omega_fix;
    K_eta = k_eta_fix;

    Ts = 0.01;                                      % Sample time
    T = 1/15;                                       % Actuator time constant

    % Actuator dynamic koefficients
    C1 = Ts/(Ts+2*T);
    C2 = (Ts-2*T)/(Ts+2*T);

    % Generate different colors for plotting
    colors = jet(size(k_dxi_in, 2)*size(k_xi_in, 2));
    clr = 1:(size(k_dxi_in, 2)*size(k_xi_in, 2)); 
    i  = 0;

    % Iterative computation of poles
    for k_dxi=k_dxi_in
        for k_xi=k_xi_in
            i = i+1;

            % Define transfer function xi_ref -> xi (symbolic)
            syms z K_d_xi K_xi
            TF_etaref_eta = ( (Ts^2*K_eta*K_omega*C1)*z^3 + ...  
                         (Ts^2*K_eta*K_omega*C1)*z^2 ) / ...
                       ( (Ts^2*K_eta*K_omega*C1 + Ts*K_omega*C1 + 1)*z^3 + ... 
                         (C2-2)*z^2 + (-Ts*K_omega*C1 - 2*C2 + 1)*z + C2 );

            TF_dxierr_dxi = K_d_xi * TF_etaref_eta * Ts*z/(z-1);     % Open loop
            TF_dxiref_dxi = TF_dxierr_dxi / (1 + TF_dxierr_dxi);     % Closed loop

            TF_xierr_xi = K_xi * TF_dxiref_dxi * Ts*z/(z-1);         % Open loop
            TF_xiref_xi = TF_xierr_xi / (1 + TF_xierr_xi);           % Closed

            % Extract numerator and denumerator from TF_xiref_xi
            [Num, Denum] = numden(TF_xiref_xi);

            % Substitute some values for K_dxi and K_xi in Denum
            Denum = subs(Denum, {K_d_xi, K_xi}, {k_dxi, k_xi});

            % Compute poles of TF_xiref_xi
            display(['K_dxi=', num2str(k_dxi), ', K_xi=', num2str(k_xi)])
            p = vpasolve(Denum, z)

            % Plot poles
            name = ['KDxi= ', num2str(k_dxi), ', KXi= ', num2str(k_xi)];
            plot(p, 'o', 'color', colors(clr(i),:), 'DisplayName', name); hold on;

            % Plot settings
            grid on;
            legend('Location', 'best')             
            xlabel('Re')
            ylabel('Im')

            % Plot unit circle
            r = 1;                                      % Radius
            c = [0 0];                                  % Center
            pos = [c-r 2*r 2*r];
            rectangle('Position', pos, 'Curvature', [1 1]);
            axis equal

        end
    end

end
