function error = computeErrorFixedHeight(x, gamma, theta_c, target_r, target_theta, I_tg, d_initial)
    total_I = 0;
    x_tg = target_r * cos(target_theta);
    y_tg = target_r * sin(target_theta);
    
    for i = 1:6
        idx = (i-1)*3 + 1; % Now 3 parameters per LED
        I0 = x(idx);
        r = x(idx+1);
        theta = x(idx+2);
        d = d_initial(i); % Use fixed height
        
        % LED position in Cartesian
        x_LED = r * cos(theta);
        y_LED = r * sin(theta);
        z_LED = d;
        
        % Vector from LED to target
        dx = x_tg - x_LED;
        dy = y_tg - y_LED;
        dz = -z_LED;
        
        % Vector from LED to center (normal direction)
        ax = -x_LED;
        ay = -y_LED;
        az = -z_LED;
        
        % Compute angle theta
        dotAB = ax * dx + ay * dy + az * dz;
        normA = sqrt(ax^2 + ay^2 + az^2);
        normB = sqrt(dx^2 + dy^2 + dz^2);
        if normA == 0 || normB == 0
            theta_val = 0;
        else
            cos_theta = dotAB / (normA * normB);
            cos_theta = max(min(cos_theta, 1), -1);
            theta_val = acos(cos_theta);
        end
        
        % Intensity contribution
        exponent = -(theta_val / theta_c)^2;
        I_contrib = gamma * I0 * exp(exponent) / (normB^2);
        total_I = total_I + I_contrib;
    end
    
    error = (I_tg - total_I)^2;
end