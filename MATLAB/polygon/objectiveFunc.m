function error = objectiveFunc(x, gamma, theta_c, x_tg, y_tg, ...
                              I_tg, led_height, base_angles, r_initial, I0_initial, Nag)
    
    % Preallocate total intensity vector
    total_I = zeros(length(I_tg), 1);
    
    % Compute total intensity for each target point
    for i = 1:length(I_tg)
        total_I(i) = computeIntensity( ...
            x(1:Nag), ...                   % LED intensities
            x(Nag+1:2*Nag), ...             % LED radii
            x(2*Nag+1), ...                 % rotation
            gamma, theta_c, ...
            led_height, ...
            base_angles, ...
            x_tg(i), y_tg(i));              % Target x and y
    end
    
    % Error calculation
    intensity_error = 10 * sum((I_tg - total_I').^2);  % Sum of squared differences
    
    % Regularization terms to prevent drifting too far
    reg_radii = 0.001 * sum((x(Nag+1:2*Nag) - r_initial').^2);
    reg_I0    = 0.001 * sum((x(1:Nag) - I0_initial').^2);
    
    % Total error (objective function output)
    error = intensity_error + reg_radii + reg_I0;

%     % Ensure scalar, real, and finite
%     if ~isfinite(error) || isnan(error)
%         error = 1e6;  % Penalize bad solutions
%     end
end