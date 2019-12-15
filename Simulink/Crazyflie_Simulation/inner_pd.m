function inner_pd(k_omega_in, k_eta_in)
    
    clc;
    Ts = 0.01;                                      % Sample time
    T = 1/15;                                       % Actuator time constant

    % Actuator dynamic koefficients
    C1 = Ts/(Ts+2*T);
    C2 = (Ts-2*T)/(Ts+2*T);

    % Generate different colors for plotting
    colors = jet(size(k_omega_in, 2)*size(k_eta_in, 2));
    clr = 1:(size(k_omega_in, 2)*size(k_eta_in, 2)); 
    i  = 0;

    % Iterative computation of poles
    for k_om=k_omega_in
        for k_et=k_eta_in
            i = i+1;

            % Define transfer function eta_ref -> eta (symbolic)
            syms z K_eta K_omega 
            TF_etaref_eta = ( (Ts^2*K_eta*K_omega*C1)*z^3 + ...  
                             (Ts^2*K_eta*K_omega*C1)*z^2 ) / ...
                           ( (Ts^2*K_eta*K_omega*C1 + Ts*K_omega*C1 + 1)*z^3 + ... 
                             (C2-2)*z^2 + (-Ts*K_omega*C1 - 2*C2 + 1)*z + C2 );

            % Extract numerator and denumerator from TF_etaref_eta
            [Num, Denum] = numden(TF_etaref_eta);

            % Substitute some values for K_omega and K_ref in Denum
            Denum = subs(Denum, {K_omega, K_eta}, {k_om, k_et});

            % Compute poles of TF_etaref_eta
            display(['K_omega=', num2str(k_om), ', K_eta=', num2str(k_et)])
            p = vpasolve(Denum, z)
            
            % Plot poles
            name = ['KOmega= ', num2str(k_om), ', KEta= ', num2str(k_et)];
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
