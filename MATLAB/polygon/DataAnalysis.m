

clear
clc



%% ***** Load data ******
dataFolder = 'Matlab/Res_Data';
Result = Illum_Optim_Perf(dataFolder);


IN3_mean = mean(Result.I_N3,3);
IN3_q1 = quantile(Result.I_N3,0.25,3);
IN3_q3 = quantile(Result.I_N3,0.75,3);