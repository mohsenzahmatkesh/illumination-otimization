clc; clear; close all;
%% Initialisation
Nag = 6;% Number of LEDs (adjust if needed)
gamma = 1;                  % Efficiency factor
theta_c = 30 * pi/180;      % cutoff angle
I0_initial = ones(Nag, 1) * 10;    
r_initial = 6 * ones(Nag, 1); % cm
base_angles = 0:2*pi/Nag:(Nag - 1) * 2 * pi/Nag';
led_height = 5;           


target_c = [1 0.1];
target_r = 1;             % cm
target_theta = 133.4*pi/180; 
Ntg = 6;
% pgon = nsidedpoly(Ntg, 'Center', target_c, 'Radius', target_r);
% pgon = rotate(pgon, target_theta);
r_lim = 1.5; r_min = 0.25; r_max = 3.5;
[pgon, center, vertex_coords] = generateRandomConvexPolygon(Ntg, r_lim, r_min, r_max);
I_tg = 100;                

if isempty(pgon)
    [x_tg, y_tg] = pol2cart(target_theta, target_r);
else
    x_tg = pgon.Vertices(:,1);
    y_tg = pgon.Vertices(:,2);
    I_tg = I_tg * ones(1,length(x_tg));
end
% Optimization variables: [I0s(6); radii(6); delta_theta(1)]
initialGuess = [I0_initial.*(1+(2*rand(Nag,1)-1)); r_initial; 0]; 


% Define the bounds for the problem
lb = zeros(2*Nag + 1, 1);
ub = inf(2*Nag + 1, 1);
lb(1:Nag) = 0;           % I0 lower bounds
ub(1:Nag) = 4000;        % I0 upper bounds
lb(Nag+1:2*Nag) = 1;     % radii lower bounds
ub(Nag+1:2*Nag) = 8;     % radii upper bounds
lb(2*Nag+1) = 0;         % rotation lower bound
ub(2*Nag+1) = 2*pi;      % rotation upper bound

%% GA Options
populationSize = 10000;      % Larger population gives better exploration
eliteCount = 10;           % Top individuals that survive unchanged
crossoverFraction = 0.8;   % 80% of children come from crossover
mutationRate = 0.1;       % Probability of mutation (affects diversity)
maxgeneration = 1000;

options = optimoptions('ga', ...
    'Display', 'iter', ...
    'PopulationSize', populationSize, ...
    'EliteCount', eliteCount, ...
    'CrossoverFraction', crossoverFraction, ...
    'MutationFcn', {@mutationadaptfeasible}, ...   % Adaptive feasible mutation
    'MaxGenerations', maxgeneration, ...           % Increase if convergence is slow
    'FunctionTolerance', 1e-12, ...
    'UseParallel', false, ...
    'PlotFcn',{'gaplotbestf','gaplotbestindiv'});  % Speed up if multiple cores are available

%% Run GA optimization
%test for objective function:
% test_x = rand(2*Nag+1, 1);
% test_error = objectiveFunc(test_x, gamma, theta_c, x_tg, y_tg, ...
%                            I_tg, led_height, base_angles, r_initial, I0_initial, Nag)
% 
% t=1

[x_opt, fval, exitflag, output] = ga(@(x) objectiveFunc(x, gamma, theta_c, x_tg, ...
                      y_tg, I_tg, led_height, base_angles, r_initial, I0_initial, Nag), ...
                      2*Nag+1, [], [], [], [], lb, ub, [], options);

% Extract optimized variables
opt_I0 = x_opt(1:Nag);
opt_radii = x_opt(Nag+1:2*Nag);
opt_rotation = x_opt(2*Nag+1);
opt_angles = base_angles + opt_rotation;

%% Final Res and Visualization

for i = 1:length(x_tg)
    final_I(i) = computeIntensity(opt_I0, opt_radii, opt_rotation, gamma, ...
                         theta_c, led_height, base_angles, x_tg(i), y_tg(i));
end


plotResults(base_angles, r_initial, opt_angles, opt_radii, led_height, ...
            opt_I0, I0_initial, x_tg, y_tg, theta_c);

plotLEDIntensityMap(opt_angles, opt_radii, opt_I0, theta_c, led_height, gamma, x_tg, y_tg);

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
    pcolor(X, Y, intensity_map);
    shading interp;
    colormap hot;
    colorbar;
    axis equal;
    hold on;
    
    % Plot LEDs
    [x_leds, y_leds] = pol2cart(opt_angles, opt_radii);
    scatter(x_leds, y_leds, 50, 'filled', 'MarkerFaceColor', 'cyan', 'DisplayName', 'LEDs');
    
    % Plot target point
    %[x_tgt, y_tgt] = pol2cart(target_theta, target_r);
    scatter(x_tg, y_tg, 20, 'r*', 'DisplayName', 'Target');
    
    legend;
    title('LED Intensity Footprint Map');
    xlabel('X (cm)');
    ylabel('Y (cm)');
    grid on;
end
