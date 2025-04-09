function Result = Illum_Optim_Perf(dataFolder)


% dataFolder = 'D:\ROBORYALE\Data_Collection\control_theory_papers_data\control_theory_papers_data\2023-10-10-00-00-02-1hour\MC_RES_Mat\08'; % <-- Change this to your folder path

% Get a list of all .mat files matching your pattern
filePattern = fullfile(dataFolder, 'Sim_*_NumLED_*.mat');
matFiles = dir(filePattern);

% Initialize an empty structure
ResultsByNag = struct();

I_tg = 0; Nag = 3; Ntg = 3;

BestFittnessHist = []; IntensityMap = []; Opt_I0 = [];
Opt_angle = []; Opt_radii = []; Opt_rot = 0;
pgon = [];

k3 = 0; k4 = 0; k5 = 0; k6 = 0; k7 = 0; k8 = 0;
Intens_Normalised3 = [];    Intens_Normalised4 = [];    Intens_Normalised5 = [];    Intens_Normalised6 = [];    Intens_Normalised7 = [];    Intens_Normalised8 = [];     
geo_cov3 = [];  geo_cov4 = []; geo_cov5 = []; geo_cov6 = []; geo_cov7 = []; geo_cov8 = [];
Best_fitness3 = {};    Best_fitness4 = {};     Best_fitness5 = {};     Best_fitness6 = {};     Best_fitness7 = {};     Best_fitness8 = {}; 
ga_gen3 = [];   ga_gen4 = [];   ga_gen5 = [];   ga_gen6 = [];   ga_gen7 = [];   ga_gen8 = [];

T_ths = 35;
% Loop over each file
for k = 1:length(matFiles)
    % Get the file name
    fileName = matFiles(k).name;
    
    % Use regular expressions to extract Nag and sim_num from the filename
    tokens = regexp(fileName, 'Sim_(\d+)_NumLED_(\d+)\.mat', 'tokens');% ResData_agent_(\d+)_batch_(\d+)\
    
    if isempty(tokens)
        warning('Filename "%s" does not match the expected pattern.', fileName);
        continue;
    end
    
    % Extract Nag and sim_num as numbers
    Nag = str2double(tokens{1}{2});
    sim_num = str2double(tokens{1}{1});
    
    % Load the .mat file
    data = load(fullfile(dataFolder, fileName));
    
    % Assume the variable inside is called isSuccessful
    try
        switch Nag
            case 3
                k3 = k3 + 1;
                Intens_Normalised3(:,:,k3) = data.intensity_map / data.I_tg;
                pgon_tgt = data.pgon; 
                [xp3,yp3] = pol2cart(data.opt_angles',data.opt_radii');
                pgon_ag = polyshape(xp3,yp3);
                pgon_intersect = intersect(pgon_ag,pgon_tgt);
                geo_cov3(k3) = area(pgon_intersect) / area(pgon_tgt);
                Best_fitness3{k3} = data.bestFitnessHistory;
                ga_gen3(k3) = length(data.bestFitnessHistory);
            case 4
                k4 = k4 + 1;
                Intens_Normalised4(:,:,k4) = data.intensity_map / data.I_tg;
                pgon_tgt = data.pgon; 
                [xp4,yp4] = pol2cart(data.opt_angles',data.opt_radii');
                pgon_ag = polyshape(xp4,yp4);
                pgon_intersect = intersect(pgon_ag,pgon_tgt);
                geo_cov4(k4) = area(pgon_intersect) / area(pgon_tgt);
                Best_fitness4{k4} = data.bestFitnessHistory;
                ga_gen4(k4) = length(data.bestFitnessHistory);
            case 5 
                k5 = k5 + 1;
                Intens_Normalised5(:,:,k5) = data.intensity_map / data.I_tg;
                pgon_tgt = data.pgon; 
                [xp5,yp5] = pol2cart(data.opt_angles',data.opt_radii');
                pgon_ag = polyshape(xp5,yp5);
                pgon_intersect = intersect(pgon_ag,pgon_tgt);
                geo_cov5(k5) = area(pgon_intersect) / area(pgon_tgt);
                Best_fitness5{k5} = data.bestFitnessHistory;
                ga_gen5(k5) = length(data.bestFitnessHistory);
            case 6 
                k6 = k6 + 1;
                Intens_Normalised6(:,:,k6) = data.intensity_map / data.I_tg;
                pgon_tgt = data.pgon; 
                [xp6,yp6] = pol2cart(data.opt_angles',data.opt_radii');
                pgon_ag = polyshape(xp6,yp6);
                pgon_intersect = intersect(pgon_ag,pgon_tgt);
                geo_cov6(k6) = area(pgon_intersect) / area(pgon_tgt);
                Best_fitness6{k6} = data.bestFitnessHistory;
                ga_gen6(k6) = length(data.bestFitnessHistory);
            case 7 
                k7 = k7 + 1;
                Intens_Normalised7(:,:,k7) = data.intensity_map / data.I_tg;
                pgon_tgt = data.pgon; 
                [xp7,yp7] = pol2cart(data.opt_angles',data.opt_radii');
                pgon_ag = polyshape(xp7,yp7);
                geo_cov7(k7) = area(pgon_intersect) / area(pgon_tgt);
                Best_fitness7{k7} = data.bestFitnessHistory;
                ga_gen7(k7) = length(data.bestFitnessHistory);
            case 8 
                k8 = k8 + 1;
                Intens_Normalised8(:,:,k8) = data.intensity_map / data.I_tg;
                pgon_tgt = data.pgon; 
                [xp8,yp8] = pol2cart(data.opt_angles',data.opt_radii');
                pgon_ag = polyshape(xp8,yp8);
                
                pgon_intersect = intersect(pgon_ag,pgon_tgt);
                geo_cov8(k8) = area(pgon_intersect) / area(pgon_tgt);
                Best_fitness8{k8} = data.bestFitnessHistory;
                ga_gen8(k8) = length(data.bestFitnessHistory);
        end
    catch
        continue
    end
    
end

Result.k3 = k3;     Result.k4 = k4;     Result.k5 = k5;     Result.k6 = k6;     Result.k7 = k7;     Result.k8 = k8;
Result.I_N3 = Intens_Normalised3;   Result.I_N4 = Intens_Normalised4;       Result.I_N5 = Intens_Normalised5;       Result.I_N6 = Intens_Normalised6;       Result.I_N7 = Intens_Normalised7;       Result.I_N8 = Intens_Normalised8;
Result.geo_cov3 = geo_cov3;     Result.geo_cov4 = geo_cov4;     Result.geo_cov5 = geo_cov5;     Result.geo_cov6 = geo_cov6;     Result.geo_cov7 = geo_cov7;     Result.geo_cov8 = geo_cov8;
Result.BestFit3 = Best_fitness3;    Result.BestFit4 = Best_fitness4;    Result.BestFit5 = Best_fitness5;    Result.BestFit6 = Best_fitness6;    Result.BestFit7 = Best_fitness7;    Result.BestFit8 = Best_fitness8;
Result.ga_gen3 = ga_gen3;   Result.ga_gen4 = ga_gen4;   Result.ga_gen5 = ga_gen5;   Result.ga_gen6 = ga_gen6;   Result.ga_gen7 = ga_gen7;   Result.ga_gen8 = ga_gen8;
