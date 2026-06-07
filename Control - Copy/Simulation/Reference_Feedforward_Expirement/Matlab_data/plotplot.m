% =========================================================
% 1. DEFINE FILE NAMES & SETTINGS
% =========================================================
% List all 8 of your base file names here (without the extension)
file_names = {
    '1rad.s_180deg_withRff',
    '2rad.s_180deg_withRff',
    '3rad.s_180deg_withRff',
    '4rad.s_180deg_withRff',
    '1rad.s_180deg_withoutRff',
    '2rad.s_180deg_withoutRff',
    '3rad.s_180deg_withoutRff',
    '4rad.s_180deg_withoutRff'
};

% We are now explicitly looking for MATLAB data files
file_extension = '.mat'; 

num_files = length(file_names);

% Preallocate arrays for metrics
rmse_all = zeros(num_files, 1);
mae_all = zeros(num_files, 1);
max_error_all = zeros(num_files, 1);

figure('Name', 'Feedforward (Rff) Tracking Comparison', 'Position', [50, 50, 1400, 800]);

% =========================================================
% 2. LOOP THROUGH ALL 8 FILES
% =========================================================
for i = 1:num_files
    
    full_file_name = strcat(file_names{i}, file_extension);
    
    try
        % --- A. Load the .mat file ---
        % This loads all variables from the .mat file into a structure
        file_data = load(full_file_name); 
        
        % We don't know exactly what you named the dataset variable when saving 
        % (e.g., 'logsout', 'ans', 'Dataset'), so we search the file for it automatically.
        var_names = fieldnames(file_data);
        dataset_var = '';
        for v = 1:length(var_names)
            if isa(file_data.(var_names{v}), 'Simulink.SimulationData.Dataset')
                dataset_var = var_names{v};
                break;
            end
        end
        
        if isempty(dataset_var)
            warning('Could not find a Simulink Dataset in %s. Skipping.', full_file_name);
            continue;
        end
        
        % Extract the main Dataset object
        ds = file_data.(dataset_var);
        
        % --- B. Unpack the Simulink Signals ---
        % Using the exact names from your screenshot
        target_signal = ds.getElement('velocity');
        actual_signal = ds.getElement('real_velocity');
        
        % Extract the raw numbers from the Simulink objects
        t        = target_signal.Values.Time;
        y_target = target_signal.Values.Data;
        y_actual = actual_signal.Values.Data;
        
        % Force them to be standard column vectors for math
        t = t(:); 
        y_target = y_target(:); 
        y_actual = y_actual(:);
        
    catch ME
        warning('Error processing %s: %s', full_file_name, ME.message);
        continue; 
    end
    
    % --- C. Calculate Errors ---
    error_signal = y_target - y_actual;
    absolute_error = abs(error_signal);
    
    rmse_all(i) = sqrt(mean(error_signal.^2));
    mae_all(i) = mean(absolute_error);
    max_error_all(i) = max(absolute_error);
    
    % --- D. Plot the Graph ---
    if contains(file_names{i}, 'withoutRff')
        plot_position = (i - 4) * 2; % Right column
    else
        plot_position = i * 2 - 1;   % Left column
    end
    
    subplot(4, 2, plot_position);
    plot(t, y_target, 'Color', [0.8500 0.3250 0.0980], 'LineWidth', 1.5); hold on;
    plot(t, y_actual, 'Color', [0 0.4470 0.7410], 'LineWidth', 1);
    
    title(strrep(file_names{i}, '_', ' '), 'Interpreter', 'none', 'FontSize', 10);
    xlabel('Time (s)');
    ylabel('Velocity (rad/s)');
    grid on;
    
    if i == 1
        legend('Target (velocity)', 'Actual (real\_velocity)', 'Location', 'best');
    end
end

% =========================================================
% 3. DISPLAY CONSOLIDATED RESULTS TABLE
% =========================================================
fprintf('\n========================================================================\n');
fprintf('                      TRACKING PERFORMANCE SUMMARY                        \n');
fprintf('========================================================================\n');
fprintf('%-30s | %-10s | %-10s | %-10s\n', 'File Name', 'RMSE', 'Mean Abs', 'Max Error');
fprintf('------------------------------------------------------------------------\n');

for i = 1:num_files
    clean_name = strrep(file_names{i}, '_', ' '); 
    fprintf('%-30s | %-10.4f | %-10.4f | %-10.4f\n', ...
        clean_name, rmse_all(i), mae_all(i), max_error_all(i));
end
fprintf('========================================================================\n');