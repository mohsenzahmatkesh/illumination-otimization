clc
clear 
close all
% LED Parameters
gamma = 1;                  % Efficiency factor
theta_c = 30 * pi/180;      % Cutoff angle in radians (30 degrees)
I0_initial = ones(6, 1);    % Initial LED intensities (arbitrary)
r_initial = 6 * ones(6, 1); % Radial positions in cm
theta_initial = deg2rad([0, 60, 120, 180, 240, 300])'; % Angular positions
d_initial = 15 * ones(6, 1); % Heights in cm

% Target Parameters
target_r = 0.0;               % Target at center (queen cell)
target_theta = 0;           % Target angle
I_tg = 100;                 % Target intensity

% Convert target to Cartesian coordinates
x_tg = target_r * cos(target_theta);
y_tg = target_r * sin(target_theta);

% Precompute coefficients for each LED (c) based on their positions
c = zeros(6, 1);
for ag = 1:6
    x_LED = r_initial(ag) * cos(theta_initial(ag));
    y_LED = r_initial(ag) * sin(theta_initial(ag));
    z_LED = d_initial(ag);
    
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
    cos_theta = dotAB / (normA * normB);
    cos_theta = max(min(cos_theta, 1), -1); % Clamp to avoid numerical issues
    theta = acos(cos_theta);
    
    % Compute distance from LED to target
    d = normB;
    
    % Coefficient for this LED
    c(ag) = gamma * exp(-(theta / theta_c)^2) / d^2;
end

% Solve for optimal I0 using non-negative least squares
C = c';
I0_opt = lsqnonneg(C, I_tg);

% Calculate achieved intensity and error
total_I = C * I0_opt;
error = I_tg - total_I;

disp('Optimal LED Intensities:');
disp(I0_opt);
disp(['Achieved Intensity: ', num2str(total_I)]);
disp(['Error: ', num2str(error)]);

% Combine initial guess into a vector [I0, r, theta] for each LED (d is fixed)
initialGuess = [];
for i = 1:6
    initialGuess = [initialGuess; I0_initial(i); r_initial(i); theta_initial(i)]; % Removed d
end

% Set bounds for optimization variables (now 18 variables instead of 24)
lb = zeros(18, 1);  % Lower bounds
ub = inf(18, 1);     % Upper bounds

% I0 >= 0
lb(1:3:end) = 0;

% Radial position bounds (5 cm to 7 cm)
lb(2:3:end) = 5;
ub(2:3:end) = 7;

% Theta bounds (0 to 2π)
lb(3:3:end) = 0;
ub(3:3:end) = 2*pi;

% Define modified objective function with fixed heights
objective = @(x) computeErrorFixedHeight(x, gamma, theta_c, target_r, target_theta, I_tg, d_initial);

% Run optimization with fmincon
options = optimoptions('fmincon', 'Display', 'iter', 'Algorithm', 'sqp', 'MaxFunctionEvaluations', 1e4);
[x_opt, fval] = fmincon(objective, initialGuess, [], [], [], [], lb, ub, [], options);

% Display optimized parameters
disp('Optimized Parameters (I0, r, theta, d for each LED):');
disp(x_opt);


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



%% plot (Updated for fixed heights)
% Extract optimized parameters
opt_I0 = x_opt(1:3:end);
opt_r = x_opt(2:3:end);
opt_theta = x_opt(3:3:end);
opt_d = d_initial; % Heights remain unchanged

% Initial vs optimized intensities
figure;
subplot(1,2,1);
bar(I0_initial);
title('Initial Intensities');
ylabel('I_0');
ylim([0 max(opt_I0)*1.2]);

subplot(1,2,2);
bar(opt_I0);
title('Optimized Intensities');
ylabel('I_0');
ylim([0 max(opt_I0)*1.2]);

% Position comparison
figure;
subplot(1,2,1);
polarplot(theta_initial, r_initial, 'o', 'DisplayName', 'Initial');
hold on;
polarplot(opt_theta, opt_r, 'x', 'DisplayName', 'Optimized');
title('Radial Position Comparison');
legend;
rlim([0 10]);

subplot(1,2,2);
plot(1:6, d_initial, 'o-', 'DisplayName', 'Initial');
hold on;
plot(1:6, opt_d, 'x-', 'DisplayName', 'Optimized');
title('Height Comparison (Fixed)');
ylabel('Height (cm)');
xlabel('LED Index');
legend;

% 3D plot with fixed heights
[xi, yi] = pol2cart(theta_initial, r_initial);
[xo, yo] = pol2cart(opt_theta, opt_r);

figure;
scatter3(xi, yi, d_initial, 100, 'filled', 'DisplayName', 'Initial');
hold on;
scatter3(xo, yo, opt_d, 100, 'x', 'LineWidth', 2, 'DisplayName', 'Optimized');
title('3D Position Optimization (Fixed Height)');
xlabel('X (cm)');
ylabel('Y (cm)');
zlabel('Height (cm)');
legend;
grid on;