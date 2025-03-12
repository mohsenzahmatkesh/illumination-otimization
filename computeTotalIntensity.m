function total_I = computeTotalIntensity(x, gamma, theta_c, x_tg, y_tg)
    total_I = 0;
    target_r = sqrt(x_tg^2 + y_tg^2);
    target_theta = atan2(y_tg, x_tg);
    
    for i = 1:6
        idx = (i-1)*4 + 1;
        I0 = x(idx);
        r = x(idx+1);
        theta = x(idx+2);
        d = x(idx+3);
        
        x_LED = r * cos(theta);
        y_LED = r * sin(theta);
        z_LED = d;
        
        dx = x_tg - x_LED;
        dy = y_tg - y_LED;
        dz = -z_LED;
        
        ax = -x_LED;
        ay = -y_LED;
        az = -z_LED;
        
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
        
        exponent = -(theta_val / theta_c)^2;
        I_contrib = gamma * I0 * exp(exponent) / (normB^2);
        total_I = total_I + I_contrib;
    end
end