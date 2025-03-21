function total_I = computeIntensity(Nag, I0s, radii, delta_theta, gamma, theta_c, ...
                                   led_height, base_angles, x_tg, y_tg)
    total_I = 0;
    for i = 1:Nag
        % Calculate LED position
        [x_led, y_led] = pol2cart(base_angles(i) + delta_theta, radii(i));
        
        % Calculate vector components
        
        dx = x_tg - x_led;
        dy = y_tg - y_led;
        dz = -led_height;
        
        % Calculate distance and angle
        distance = sqrt(dx^2 + dy^2 + dz^2);
        cos_theta = -dz/distance;  % From downward-facing normal
        theta = acos(cos_theta);
       
        total_I = total_I + gamma * I0s(i) * exp(-(theta/theta_c)^2) / distance^2;
    end
end