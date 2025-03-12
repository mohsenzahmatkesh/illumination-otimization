clc
clear
close all

% LED Model Parameters
gamma = 0.9;           % LED efficiency factor (0 to 1)
theta_c = deg2rad(20); % LED cutoff angle in radians
I0_max = 1.0;          % Maximum LED intensity command (arbitrary units)

% Geometry of LEDs (Radial Coordinates)
N_leds = 6;          % Number of LEDs
r_ag = 6;            % Radial distance from center (cm)
theta_ag = deg2rad([0, 60, 120, 180, 240, 300]); % Angular positions of LEDs in radians
d_ag = 10;           % LED height from illuminated plane (cm)

% Target Cell Parameters
r_tg = 0;            % Radial position of target (cm)
theta_tg = deg2rad(120); % Angular position of target (radians)
I_tg = 1;          % Target intensity

% Compute intensity contribution from each LED at the target cell

I_total = 0; % Initialize total intensity at the target cell

for i = 1:N_leds
    % Compute distance from LED to target cell
    dx = r_tg * cos(theta_tg) - r_ag * cos(theta_ag(i));
    dy = r_tg * sin(theta_tg) - r_ag * sin(theta_ag(i));
    d = sqrt(dx^2 + dy^2 + d_ag^2); % 3D distance from LED to target
    
    % Compute angle theta between LED normal and target
    theta = abs(theta_tg - theta_ag(i)); 
    
    % Compute intensity contribution from this LED
    I_i = gamma * I0_max * exp(- (theta / theta_c)^2 ) * (1 / d^2);
    
    % Sum up the intensity from all LEDs
    I_total = I_total + I_i;
    
    % Display individual LED contribution
    disp(["LED ", num2str(i), " contributes I = ", num2str(I_i)]);
end

% Display the total intensity at the target cell
disp(["Total Intensity at Target = ", num2str(I_total)]);

% Optimization Setup
I0_init = ones(N_leds, 1) * I0_max / 2; % Initial intensities (half max)
theta_init = theta_ag; % Initial angles (current setup)
r_init = ones(N_leds, 1) * r_ag; % Initial radial distances

% Bounds on variables
I0_lb = zeros(N_leds, 1); % Lower bound for intensities
I0_ub = ones(N_leds, 1) * I0_max; % Upper bound for intensities

r_lb = ones(N_leds, 1) * 0.5; % Increase min radial distance to 4.5 cm
r_ub = ones(N_leds, 1) * 8; % Keep max at 8 cm

theta_lb = zeros(N_leds, 1); % Min angle 0
theta_ub = ones(N_leds, 1) * 2 * pi; % Max angle 2π

% Combine all variables
x0 = [I0_init; r_init; theta_init'];
lb = [I0_lb; r_lb; theta_lb]; % Lower bounds
ub = [I0_ub; r_ub; theta_ub]; % Upper bounds

% Objective function (error minimization)
obj_fun = @(x) compute_illumination_error(x, N_leds, gamma, theta_c, d_ag, r_tg, theta_tg, I_tg);

% Solve using fmincon
options = optimoptions('fmincon', 'Display', 'iter', 'Algorithm', 'sqp', ...
                       'OptimalityTolerance', 1e-10, 'MaxIterations', 2000);

%x_opt = fmincon(obj_fun, x0, [], [], [], [], lb, ub, [], options);
% Define a function to enforce minimum angular separation of 30 degrees
A_theta = zeros(N_leds - 1, length(x0));
for i = 1:N_leds-1
    A_theta(i, 2*N_leds + i) = 1;  % Select theta(i)
    A_theta(i, 2*N_leds + i + 1) = -1; % Subtract theta(i+1)
end
b_theta = -deg2rad(30) * ones(N_leds - 1, 1); % Minimum spacing 30 degrees

% Add constraints in fmincon
x_opt = fmincon(obj_fun, x0, A_theta, b_theta, [], [], lb, ub, [], options);

% Extract optimized values
I0_opt = x_opt(1:N_leds);
r_opt = x_opt(N_leds+1:2*N_leds);
theta_opt = x_opt(2*N_leds+1:end);

% Display results
disp("Optimized LED Intensities:"); disp(I0_opt');
disp("Optimized LED Radii:"); disp(r_opt');
disp("Optimized LED Angles (degrees):"); disp(rad2deg(theta_opt)');

function error = compute_illumination_error(x, N_leds, gamma, theta_c, d_ag, r_tg, theta_tg, I_tg)
    I0 = x(1:N_leds);
    r_ag = x(N_leds+1:2*N_leds);
    theta_ag = x(2*N_leds+1:end);
    
    I_total = 0; % Initialize total intensity
    
    for i = 1:N_leds
        % Compute distance from LED to target
        dx = r_tg * cos(theta_tg) - r_ag(i) * cos(theta_ag(i));
        dy = r_tg * sin(theta_tg) - r_ag(i) * sin(theta_ag(i));
        d = sqrt(dx^2 + dy^2 + d_ag^2);
        
        % Compute angle theta between LED normal and target
        theta = abs(theta_tg - theta_ag(i)); 
        
        % Compute intensity contribution
        %I_i = gamma * I0(i) * exp(- (theta / theta_c)^2 ) * (1 / d^2);
        % Normalize intensity contribution to avoid underflow issues
        I_i = gamma * I0(i) * exp(- (theta / theta_c)^2 ) / (d^2 + 1e-6);

        
        % Sum intensity
        I_total = I_total + I_i;
    end
    
    % Compute error as absolute difference from target intensity
    error = abs(I_tg - I_total);
end





%% Plot

% Create a figure for visualization
figure; hold on; grid on;
axis equal;
xlim([-10, 10]); ylim([-10, 10]);

% Plot Initial LED Positions
x_init = r_ag * cos(theta_ag);
y_init = r_ag * sin(theta_ag);
scatter(x_init, y_init, 100, 'b', 'filled'); 
text(x_init, y_init, "Initial", 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');

% Plot Optimized LED Positions
x_opt = r_opt .* cos(theta_opt);
y_opt = r_opt .* sin(theta_opt);
scatter(x_opt, y_opt, 100, 'r', 'filled');
text(x_opt, y_opt, "Optimized", 'VerticalAlignment', 'top', 'HorizontalAlignment', 'left');

% Plot Target Position
x_tg = r_tg * cos(theta_tg);
y_tg = r_tg * sin(theta_tg);
scatter(x_tg, y_tg, 150, 'g', 'filled');
text(x_tg, y_tg, "Target", 'VerticalAlignment', 'middle', 'HorizontalAlignment', 'right');

% Connect LEDs to the Target
for i = 1:N_leds
    plot([x_opt(i), x_tg], [y_opt(i), y_tg], '--k'); % Optimized lines
end

% Labels and Title
xlabel('X Position (cm)');
ylabel('Y Position (cm)');
title('Optimized LED Placement vs Initial Placement');
legend({'Initial LEDs', 'Optimized LEDs', 'Target Cell'}, 'Location', 'best');
hold off;



% Global variable to store error history
global error_history;
error_history = [];

% Modify objective function to track error over iterations
obj_fun = @(x) track_optimization_error(x, N_leds, gamma, theta_c, d_ag, r_tg, theta_tg, I_tg);

% Solve using fmincon with tracking
options = optimoptions('fmincon', 'Display', 'iter', 'Algorithm', 'sqp');
x_opt = fmincon(obj_fun, x0, [], [], [], [], lb, ub, [], options);

% Extract optimized values
I0_opt = x_opt(1:N_leds);
r_opt = x_opt(N_leds+1:2*N_leds);
theta_opt = x_opt(2*N_leds+1:end);

% Plot optimization convergence
figure;
plot(1:length(error_history), error_history, '-o', 'LineWidth', 2);
xlabel('Iteration Number');
ylabel('Error \epsilon_\alpha');
title('Optimization Convergence');
grid on;

function error = track_optimization_error(x, N_leds, gamma, theta_c, d_ag, r_tg, theta_tg, I_tg)
    global error_history;

    % Compute the current error
    error = compute_illumination_error(x, N_leds, gamma, theta_c, d_ag, r_tg, theta_tg, I_tg);

    % Store error for convergence plotting
    error_history = [error_history; error];
end

