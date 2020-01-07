function inner_pd_cs(k_omega_in, k_eta_in)

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

            % Define transfer function eta_ref -> eta (Control Systems Toolbox)
            z = tf('z', Ts);
            K_omega = k_om; 
            K_eta = k_et; 
            TF_etaref_eta = ( (Ts^2*K_eta*K_omega*C1)*z^3 + ...  
                              (Ts^2*K_eta*K_omega*C1)*z^2 ) / ...
                            ( (Ts^2*K_eta*K_omega*C1 + Ts*K_omega*C1 + 1)*z^3 + ... 
                              (C2-2)*z^2 + (-Ts*K_omega*C1 - 2*C2 + 1)*z + C2 );

            % Compute poles of TF_etaref_eta
            display(['K_omega=', num2str(K_omega), ', K_eta=', num2str(K_eta)])
            p = pole(TF_etaref_eta)

            % Plot poles
            name = ['KOmega= ', num2str(K_omega), ', KEta= ', num2str(K_eta)];
            pzmap(TF_etaref_eta); hold on; axis equal;

            % Plot settings
            grid on; 

        end
    end

end
