function [state, options, optchanged] = gaOutputFcn(options, state, flag)
    % gaOutputFcn stores the best fitness value at each generation in a global variable
    global bestFitnessHistory  % or persistent if you prefer
    optchanged = false;  % By default, no changes to options

    switch flag
        case 'init'
            % Initialize history at the beginning of the run
            bestFitnessHistory = [];
        case 'iter'
            % Record the best fitness in this generation
            bestFitnessHistory(end+1) = state.Best(end);
        case 'done'
            % Final cleanup or display if needed
            disp('GA optimization finished.');
    end
end
