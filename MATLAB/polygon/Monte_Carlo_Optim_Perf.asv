function [opt_I0, opt_radii, opt_rotation, opt_angles, bestFitnessHistory, intensity_map, pgon] = Monte_Carlo_Optim_Perf(Nag, I_tg,  I0, r0, led_height, r_lim, r_min, r_max, Ntg, target_theta)
%% Initialisation
% Monte_Carlo_Optim_Perf
% Core function for statistical performance analysis.
%
% Inputs:
%   Nag - Number of LEDs for illumination
%   I_tg - final illumination intensity
%   I0 - Initial intensity
%   r0 - initial radial position (cm)
%   led_height - the distance between LEDs and the illumination surface (cm)
%   r_lim  - Maximum radius from the origin for polygon center
%   r_min  - Minimum radius from polygon center to a vertex
%   r_max  - Maximum radius from polygon center to a vertex
%   target_theta - the target illumination polygon rotation (rad)
%
% Outputs:
%   pgon         - polyshape object for plotting
%   center       - 1x2 array with center [x, y]
%   vertex_coords - Nx2 matrix with vertex coordinates [x_i, y_i]


%%%%%%%%%% LED characteristics %%%%%%%%%%%%%%%%%
gamma = 1;                  % Efficiency factor
theta_c = 30 * pi/180;      % cutoff angle
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

I0_initial = ones(Nag, 1) * I0;    
r_initial = r0 * ones(Nag, 1); % cm
base_angles = 0:2*pi/Nag:(Nag - 1) * 2 * pi/Nag';
% led_height = 5;           
global bestFitnessHistory  % declare the variable outside
bestFitnessHistory = 1000;
% target_c = [1 0.1];
% target_r = 1;             % cm
%target_theta = 133.4*pi/180; 
% Ntg = 6;
% pgon = nsidedpoly(Ntg, 'Center', target_c, 'Radius', target_r);
% pgon = rotate(pgon, target_theta);
% r_lim = 1.5; r_min = 2; r_max = 2.75;
[pgon, center, vertex_coords] = generateRandomConvexPolygon(Ntg, r_lim, r_min, r_max);
% I_tg = 100;                

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
populationSize = 1000;      % Larger population gives better exploration
eliteCount = 10;           % Top individuals that survive unchanged
crossoverFraction = 0.8;   % 80% of children come from crossover
mutationRate = 0.1;       % Probability of mutation (affects diversity)
maxgeneration = 1000;

options = optimoptions('ga', ...
    'Display', 'iter', ...
    'PopulationSize', populationSize, ...
    'SelectionFcn','selectionuniform',...
    'CrossoverFcn','crossoverheuristic',...
    'EliteCount', eliteCount, ...
    'CrossoverFraction', crossoverFraction, ...
    'MutationFcn', {@mutationadaptfeasible}, ...   % Adaptive feasible mutation
    'MaxGenerations', maxgeneration, ...           % Increase if convergence is slow
    'OutputFcn', @gaOutputFcn,...                  % For saving the best value of objective function in each generation
    'FunctionTolerance', 1e-12, ...
    'UseParallel', true, ...
    'PlotFcn',{'gaplotbestf','gaplotbestindiv'});  % Speed up if multiple cores are available

%% Run GA optimization
%test for objective function:
% test_x = rand(2*Nag+1, 1);
% test_error = objectiveFunc(test_x, gamma, theta_c, x_tg, y_tg, ...
%                            I_tg, led_height, base_angles, r_initial, I0_initial, Nag)
% 
% t=1

[x_opt, fval, exitflag, output, population,scores] = ga(@(x) objectiveFunc_Intesity_dist(x, gamma, theta_c, x_tg, ...
                      y_tg, I_tg, led_height, base_angles, r_initial, I0_initial, Nag), ...
                      2*Nag+1, [], [], [], [], lb, ub, [], options);

% Extract optimized variables
opt_I0 = x_opt(1:Nag);
opt_radii = x_opt(Nag+1:2*Nag);
opt_rotation = x_opt(2*Nag+1);
opt_angles = base_angles + opt_rotation;

%% Final Res and Visualization

for i = 1:length(x_tg)
    final_I(i) = computeIntensity(Nag, opt_I0, opt_radii, opt_rotation, gamma, ...
                         theta_c, led_height, base_angles, x_tg(i), y_tg(i));
end


plotResults(base_angles, r_initial, opt_angles, opt_radii, led_height, ...
            opt_I0, I0_initial, x_tg, y_tg, theta_c);

% plotLEDIntensityMap(opt_angles, opt_radii, opt_I0, theta_c, led_height, gamma, x_tg, y_tg);

[intensity_map, X_grid, Y_grid] = IntensityMap(opt_angles, opt_radii, opt_I0, theta_c, led_height, gamma, x_tg, y_tg);


% figure(4)
% surf(intensity_map)
% colorbar

%% Visualization Functions


