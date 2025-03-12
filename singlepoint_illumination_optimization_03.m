clc; clear; close all;

% LED Parameters
gamma = 1;                  % Efficiency factor
theta_c = 30 * pi/180;      % Cutoff angle (30 degrees)
I0_initial = ones(6, 1);    % Initial intensities
r_initial = 6 * ones(6, 1); % Initial radii (cm)
base_angles = deg2rad([0, 60, 120, 180, 240, 300])'; % Base angular positions
d_initial = 15 * ones(6, 1); % Fixed heights (cm)

% Target Parameters
target_r = 6.0;             % Center target
target_theta = 60*pi/180;           % Target angle
I_tg = 100;                 % Target intensity

% Convert target to Cartesian
x_tg = target_r * cos(target_theta);
y_tg = target_r * sin(target_theta);

% Optimization variables: [I0s; radii; delta_theta]
initialGuess = [I0_initial; r_initial; 0]; % Initial rotation offset = 0

% Set bounds
lb = zeros(13,1);
ub = inf(13,1);

% Intensity bounds
lb(1:6) = 0;         % I0 >= 0

% Radial bounds
lb(7:12) = 1;        % Minimum radius = 1 cm
ub(7:12) = 8;        % Maximum radius = 6 cm

% Rotation bounds
lb(13) = 0;          % Minimum rotation
ub(13) = 2*pi;       % Maximum rotation

% Define objective
objective = @(x) computePolarError(x, gamma, theta_c, target_r, target_theta, I_tg, d_initial, base_angles);

% Optimize using fmincon
% options = optimoptions('fmincon', 'Display', 'iter', 'Algorithm', 'sqp', 'MaxFunctionEvaluations', 1e4);
options = optimoptions('fmincon', 'Display', 'iter', 'Algorithm', 'sqp', ...
    'MaxFunctionEvaluations', 1e4, 'OutputFcn', @(x,optimValues,state) saveHistory(x,optimValues,state));
[x_opt, fval] = fmincon(objective, initialGuess, [], [], [], [], lb, ub, [], options);

% Extract optimized parameters
opt_I0 = x_opt(1:6);
opt_radii = x_opt(7:12);
opt_rotation = x_opt(13);
opt_angles = base_angles + opt_rotation;

% Calculate final intensity
final_I = computeTotalIntensity(opt_I0, opt_radii, opt_rotation, gamma, theta_c, d_initial, base_angles, x_tg, y_tg);
disp(['Achieved Intensity: ', num2str(final_I)]);
disp(['Target Intensity: ', num2str(I_tg)]);
disp(['Error: ', num2str(abs(I_tg - final_I))]);

% Visualization
% plotResults(base_angles, r_initial, opt_angles, opt_radii, d_initial, opt_I0, I0_initial);

%% Helper Functions
function error = computePolarError(x, gamma, theta_c, target_r, target_theta, I_tg, d_initial, base_angles)
    % Extract parameters
    I0s = x(1:6);
    radii = x(7:12);
    delta_theta = x(13);
    
    % Calculate total intensity
    total_I = computeTotalIntensity(I0s, radii, delta_theta, gamma, theta_c, d_initial, base_angles,...
                   target_r*cos(target_theta), target_r*sin(target_theta));
    
    % Calculate error
    error = (I_tg - total_I)^2;
end

function total_I = computeTotalIntensity(I0s, radii, delta_theta, gamma, theta_c, d_initial, base_angles, x_tg, y_tg)
    total_I = 0;
    
    for i = 1:6
        % Current LED parameters
        I0 = I0s(i);
        r = radii(i);
        theta = base_angles(i) + delta_theta;
        d = d_initial(i);
        
        % LED position
        x_LED = r * cos(theta);
        y_LED = r * sin(theta);
        
        % Vector calculations
        dx = x_tg - x_LED;
        dy = y_tg - y_LED;
        dz = -d;
        
        normB = sqrt(dx^2 + dy^2 + dz^2);
        ax = -x_LED;
        ay = -y_LED;
        az = -d;
        
        % Angle calculation
        dotAB = ax*dx + ay*dy + az*dz;
        normA = sqrt(ax^2 + ay^2 + az^2);
        cos_theta = max(min(dotAB/(normA*normB), 1), -1);
        theta_val = acos(cos_theta);
        
        % Intensity contribution
        exponent = -(theta_val/theta_c)^2;
        total_I = total_I + gamma * I0 * exp(exponent) / normB^2;
    end
end

% Add this function before main plotting section
function stop = saveHistory(~, optimValues, state)
    persistent fvals;
    stop = false;
    
    if strcmp(state, 'init')
        fvals = [];  % Reset at initialization
    end
    fvals = [fvals; optimValues.fval];  % Append new error value
    
    assignin('base', 'optim_history', fvals);
end



%% plot1
% Radial Position Comparison
figure;
subplot(2,2,1);
polarplot(base_angles, r_initial, 'ro', 'MarkerSize', 8, 'LineWidth', 1.5, 'DisplayName', 'Initial');
hold on;
polarplot(opt_angles, opt_radii, 'bx', 'MarkerSize', 8, 'LineWidth', 1.5, 'DisplayName', 'Optimized');
title('Radial Position Comparison');
legend('Initial', 'Optimized', 'Location', 'best');
rlim([0 max([r_initial; opt_radii])+1]);
grid on;

% Ensure history exists before plotting
subplot(2,2,2);
if exist('optim_history', 'var') && ~isempty(optim_history)
    plot(1:length(optim_history), sqrt(optim_history), 'k-o', 'LineWidth', 2, 'MarkerSize', 6);
    xlabel('Iteration');
    ylabel('Error (\epsilon_{\alpha})');
    title('Optimization Convergence');
    grid on;
else
    text(0.5, 0.5, 'No convergence data', 'FontSize', 12, 'HorizontalAlignment', 'center');
end

% Intensity Comparison
subplot(2,2,3);
bar([I0_initial, opt_I0], 'FaceColor', 'flat');
title('LED Intensity Comparison');
ylabel('I_0');
xticklabels({'LED1', 'LED2', 'LED3', 'LED4', 'LED5', 'LED6'});
legend({'Initial', 'Optimized'}, 'Location', 'northwest');
grid on;


% 3D Position Visualization
subplot(2,2,4);
[xi, yi] = pol2cart(base_angles, r_initial);
[xo, yo] = pol2cart(opt_angles, opt_radii);

% Scatter for initial positions (red circles)
scatter3(xi, yi, d_initial, 100, 'ro', 'filled', 'DisplayName', 'Initial');
hold on;

% Scatter for optimized positions (blue crosses)
scatter3(xo, yo, d_initial, 150, 'bx', 'LineWidth', 2, 'DisplayName', 'Optimized'); 

% Labels and formatting
title('3D Position Comparison');
xlabel('X (cm)');
ylabel('Y (cm)');
zlabel('Height (cm)');
legend('Initial', 'Optimized', 'Location', 'best');
grid on;
view(3); % Ensures 3D view



% Angular Movement Visualization
figure;
polarplot([base_angles base_angles+opt_rotation]', [ones(6,1) ones(6,1)]', 'k--', 'LineWidth', 1.5);
hold on;
polarplot(base_angles, r_initial, 'ro', 'MarkerSize', 10, 'LineWidth', 1.5, 'DisplayName','Initial');
polarplot(opt_angles, opt_radii, 'bx', 'MarkerSize', 10, 'LineWidth', 1.5, 'DisplayName','Optimized');
title('Angular Position Changes');
rlim([0 max([r_initial; opt_radii])+1]);
legend('Rotation Path', 'Initial', 'Optimized', 'Location', 'best');
grid on;











