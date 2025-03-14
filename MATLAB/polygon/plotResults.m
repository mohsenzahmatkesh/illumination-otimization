function plotResults(base_angles, init_radii, opt_angles, opt_radii, ...
                    led_height, opt_I0, init_I0, x_tg, y_tg, theta_c)
    % Polar position comparison
    figure;
    polarplot([base_angles, opt_angles]', [init_radii', opt_radii]', 'k--');
    hold on;
    polarscatter(base_angles, init_radii, 100, 'filled', 'DisplayName','Initial');
    polarscatter(opt_angles, opt_radii, 100, 'd', 'DisplayName','Optimized');
    [target_r, target_theta] = cart2pol(x_tg,y_tg);
    polarscatter(target_theta, target_r, 20, 'r*', 'DisplayName','Target');
    % Plot illumination patterns for each optimized LED
    

    title('Optimized LED Configuration');
    legend;
    rlim([0 max([init_radii' opt_radii])+1]);
end