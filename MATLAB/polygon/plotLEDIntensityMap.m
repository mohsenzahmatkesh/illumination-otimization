function plotLEDIntensityMap(opt_angles, opt_radii, opt_I0, theta_c, led_height, gamma, x_tg, y_tg)
    % Create a polar grid
    r_max = max(opt_radii) + 3;  % Extent of the map
    theta_grid = linspace(0, 2*pi, 360);
    r_grid = linspace(0, r_max, 200);
    [Theta, R] = meshgrid(theta_grid, r_grid);
    
    % Convert to cartesian for plotting
    [X, Y] = pol2cart(Theta, R);
    
    % Initialize intensity map
    intensity_map = zeros(size(R));
    
    % Loop over each LED and accumulate intensities
    for i = 1:length(opt_angles)
        % LED position
        [x_led, y_led] = pol2cart(opt_angles(i), opt_radii(i));
        
        % Compute distance from each grid point to LED
        dx = X - x_led;
        dy = Y - y_led;
        dz = -led_height;
        
        distance = sqrt(dx.^2 + dy.^2 + dz.^2);
        cos_theta = -dz ./ distance;
        
        % Clamp invalid values
        cos_theta(cos_theta < 0) = 0;
        
        theta = acos(cos_theta);
        
        % Compute intensity at each grid point from this LED
        I = gamma * opt_I0(i) * exp(-(theta/theta_c).^2) ./ (distance.^2) ;
        
        % Add to total intensity map
        intensity_map = intensity_map + I;
    end
    
    % Plotting
    figure;
    % Use pcolor for a smooth 2D plot (or surf/mesh for 3D)
    subplot(1,2,1)
    pcolor(X, Y, intensity_map);
    shading interp;
    colormap hot;
    colorbar;
    axis equal;
    hold on;
    
    % Plot LEDs
    [x_leds, y_leds] = pol2cart(opt_angles, opt_radii);
    scatter(x_leds, y_leds, 50, 10 * opt_I0/max(opt_I0), 'DisplayName', 'LEDs');
    
    
    % Plot target point
    %[x_tgt, y_tgt] = pol2cart(target_theta, target_r);
    plot([x_tg;x_tg(1)], [y_tg;y_tg(1)], 'k:', 'DisplayName', 'Target');
    
    legend;
    title('LED Intensity Footprint Map');
    xlabel('X (cm)');
    ylabel('Y (cm)');
    grid on;
    
    subplot(1,2,2)
    pcolor(X, Y, intensity_map);
    shading interp;
    colormap gray;
    colorbar;
    axis equal;
    hold on;
    
    % Plot LEDs
    [x_leds, y_leds] = pol2cart(opt_angles, opt_radii);
    scatter(x_leds, y_leds, 50, 10 * opt_I0/max(opt_I0), 'DisplayName', 'LEDs');
    
    
    % Plot target point
    %[x_tgt, y_tgt] = pol2cart(target_theta, target_r);
    plot([x_tg;x_tg(1)], [y_tg;y_tg(1)], 'k:', 'DisplayName', 'Target');
    
    legend;
    title('LED Intensity Footprint Map');
    xlabel('X (cm)');
    ylabel('Y (cm)');
    grid on;

end
