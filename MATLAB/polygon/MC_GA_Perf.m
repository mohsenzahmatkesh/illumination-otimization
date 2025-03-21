clear
clc

% Get SLURM Array Task ID
sim_id_str = getenv('SLURM_ARRAY_TASK_ID');
if isempty(sim_id_str)
    sim_id = randi(1000); % Default for local testing
else
    sim_id = str2double(sim_id_str);
end

% Seed RNG with sim_id to make random numbers reproducible and unique
rng(sim_id);

% Randomized parameters
Nag = 2 + randi(6);  % Number of LEDs for illumination
I_tg = 75 + (2 * rand - 1) * 25;  % Final illumination intensity
I0 = 10;  % Initial intensity
r0 = 5 + (2 * rand - 1);  % Initial radial position (cm)
led_height = 5;  % Distance between LEDs and the illumination surface (cm)
r_lim = 1.5; 
r_min = 2; 
r_max = 2.75;  % Radius limits
Ntg = 2 + randi(10);
target_theta = 123.78 * pi / 180;

% Ensure Res_Data directory exists
if ~exist('Res_Data', 'dir')
    mkdir('Res_Data');
end

% Run your optimization function
[opt_I0, opt_radii, opt_rotation, opt_angles, bestFitnessHistory, intensity_map, pgon] = ...
    Monte_Carlo_Optim_Perf(Nag, I_tg, I0, r0, led_height, r_lim, r_min, r_max, Ntg, target_theta);

% Build file name: includes sim_id and Nag for clarity
file_name = fullfile('Res_Data', ...
    ['Sim_', num2str(sim_id), '_NumLED_', num2str(Nag), '.mat']);

% Save results
save(file_name);

disp(['Saved file: ', file_name]);
