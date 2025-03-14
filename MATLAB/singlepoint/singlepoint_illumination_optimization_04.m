clc; clear; close all;


gamma = 1;                  % Efficiency factor
theta_c = 30 * pi/180;      % cutoff angle
I0_initial = ones(6, 1);    
r_initial = 6 * ones(6, 1); % cm
base_angles = deg2rad([0, 60, 120, 180, 240, 300])';
led_height = 15;           


target_r = 3.0;             % cm
target_theta = 30*pi/180;   
I_tg = 100;                


[x_tg, y_tg] = pol2cart(target_theta, target_r);

% Optimization variables: [I0s(6); radii(6); delta_theta(1)]
initialGuess = [I0_initial.*(1+(2*rand(6,1)-1)); r_initial; 0]; 


lb = zeros(13,1);
ub = inf(13,1);
lb(1:6) = 0;       
ub(1:6) = 4000;     
lb(7:12) = 1;      
ub(7:12) = 8;
lb(13) = 0;         
ub(13) = 2*pi;


options = optimoptions('fmincon', 'Display', 'iter', 'Algorithm', 'sqp', ...
                      'MaxFunctionEvaluations', 1e4);
[x_opt, ~] = fmincon(@(x) objectiveFunc(x, gamma, theta_c, target_r, ...
                      target_theta, I_tg, led_height, base_angles, r_initial, I0_initial), ...
                      initialGuess, [], [], [], [], lb, ub, [], options);


opt_I0 = x_opt(1:6);
opt_radii = x_opt(7:12);
opt_rotation = x_opt(13);
opt_angles = base_angles + opt_rotation;


final_I = computeIntensity(opt_I0, opt_radii, opt_rotation, gamma, ...
                         theta_c, led_height, base_angles, x_tg, y_tg);


plotResults(base_angles, r_initial, opt_angles, opt_radii, led_height, ...
            opt_I0, I0_initial, target_r, target_theta);


function error = objectiveFunc(x, gamma, theta_c, target_r, target_theta, ...
                              I_tg, led_height, base_angles, r_initial, I0_initial)
    [x_tg, y_tg] = pol2cart(target_theta, target_r);
    total_I = computeIntensity(x(1:6), x(7:12), x(13), gamma, theta_c, ...
                              led_height, base_angles, x_tg, y_tg);
    error = (I_tg - total_I)^2 + 0.001 * sum((x(7:12) - r_initial).^2) + 0.001 * sum((x(1:6) - I0_initial).^2);
end

function total_I = computeIntensity(I0s, radii, delta_theta, gamma, theta_c, ...
                                   led_height, base_angles, x_tg, y_tg)
    total_I = 0;
    for i = 1:6
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

%% Visualization Functions
function plotResults(base_angles, init_radii, opt_angles, opt_radii, ...
                    led_height, opt_I0, init_I0, target_r, target_theta)
    % Polar position comparison
    figure;
    polarplot([base_angles, opt_angles]', [init_radii, opt_radii]', 'k--');
    hold on;
    polarscatter(base_angles, init_radii, 100, 'filled', 'DisplayName','Initial');
    polarscatter(opt_angles, opt_radii, 100, 'd', 'DisplayName','Optimized');
    polarscatter(target_theta, target_r, 200, 'r*', 'DisplayName','Target');
    title('Optimized LED Configuration');
    legend;
    rlim([0 max([init_radii; opt_radii])+1]);
    
    % Intensity comparison
    figure;
    subplot(1,2,1);
    bar(init_I0, 'FaceColor', [0.7 0.7 0.7]);
    title('Initial Intensities');
    ylabel('I_0');
    ylim([0 max(opt_I0)*1.1]);
    
    subplot(1,2,2);
    bar(opt_I0, 'FaceColor', [0.2 0.6 0.9]);
    title('Optimized Intensities');
    ylabel('I_0'); 
    ylim([0 max(opt_I0)*1.1]);
    
    % 3D visualization
    figure;
    [xi, yi] = pol2cart(base_angles, init_radii);
    [xo, yo] = pol2cart(opt_angles, opt_radii);
    scatter3(xi, yi, ones(6,1)*led_height, 100, 'filled', 'DisplayName','Initial');
    hold on;
    scatter3(xo, yo, ones(6,1)*led_height, 100, 'd', 'DisplayName','Optimized');
    scatter3(target_r*cos(target_theta), target_r*sin(target_theta), 0, ...
            200, 'r*', 'DisplayName','Target');
    title('3D System Configuration');
    xlabel('X (cm)'); ylabel('Y (cm)'); zlabel('Z (cm)');
    legend; 
    grid on;
    view(25, 35);
end