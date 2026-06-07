% =========================================================
% 1. DEFINE FILE NAMES & DIRECTORIES
% =========================================================
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

file_extension = '.mat'; 
num_files = length(file_names);

% --- DEFINE SAVE DIRECTORY ---
save_dir = 'C:\Studio_2.68_G11\Control_by_due\Control\Simulation\Reference_Feedforward_Expirement\Result_figure';

% Check if the folder exists. If not, create it.
if ~exist(save_dir, 'dir')
    mkdir(save_dir);
    fprintf('Created directory: %s\n', save_dir);
end

% Preallocate arrays for metrics
rmse_all = zeros(num_files, 1);
mae_all = zeros(num_files, 1);
max_error_all = zeros(num_files, 1);

% --- Setup the Master Summary Figure ---
summary_fig = figure('Name', 'Summary of All Tracking Profiles', 'Position', [50, 50, 1400, 800]);

% =========================================================
% 2. LOOP THROUGH ALL 8 FILES
% =========================================================
for i = 1:num_files
    
    full_file_name = strcat(file_names{i}, file_extension);
    clean_name = strrep(file_names{i}, '_', ' '); 
    
    try
        % --- A. Load the .mat file & Extract Simulink Data ---
        file_data = load(full_file_name); 
        
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
        
        ds = file_data.(dataset_var);
        
        target_signal = ds.getElement('velocity');
        actual_signal = ds.getElement('real_velocity');
        
        t        = target_signal.Values.Time(:);
        y_target = target_signal.Values.Data(:);
        y_actual = actual_signal.Values.Data(:);
        
    catch ME
        warning('Error processing %s: %s', full_file_name, ME.message);
        continue; 
    end
    
    % --- B. Calculate Errors ---
    error_signal = y_target - y_actual;
    absolute_error = abs(error_signal);
    
    rmse_all(i) = sqrt(mean(error_signal.^2));
    mae_all(i) = mean(absolute_error);
    max_error_all(i) = max(absolute_error);
    
    % --- C. Create & Save INDIVIDUAL Figure ---
    indiv_fig = figure('Name', clean_name, 'Position', [150 + (i*20), 150 + (i*20), 700, 400]);
    plot(t, y_target, 'Color', [0.8500 0.3250 0.0980], 'LineWidth', 1.5, 'DisplayName', 'Target Velocity'); 
    hold on;
    plot(t, y_actual, 'Color', [0 0.4470 0.7410], 'LineWidth', 1.5, 'DisplayName', 'Actual Velocity');
    
    title(clean_name, 'Interpreter', 'none', 'FontSize', 12);
    xlabel('Time (s)');
    ylabel('Velocity (rad/s)');
    legend('Location', 'best');
    grid on;
    
    % SAVE THE INDIVIDUAL FIGURE
    indiv_fig_path = fullfile(save_dir, sprintf('%s.png', file_names{i}));
    %% 
    saveas(indiv_fig, indiv_fig_path);
    
    % --- D. Add SUBPLOT to SUMMARY Figure ---
    figure(summary_fig); 
    
    if i <= 4
        subplot_idx = (i * 2) - 1; % Left column (With Rff)
    else
        subplot_idx = (i - 4) * 2; % Right column (Without Rff)
    end
    
    subplot(4, 2, subplot_idx);
    plot(t, y_target, 'Color', [0.8500 0.3250 0.0980], 'LineWidth', 1.5, 'DisplayName', 'Target'); hold on;
    plot(t, y_actual, 'Color', [0 0.4470 0.7410], 'LineWidth', 1.2, 'DisplayName', 'Actual');
    
    title(clean_name, 'Interpreter', 'none', 'FontSize', 10);
    xlabel('Time (s)');
    ylabel('Velocity (rad/s)');
    grid on;
    
    if subplot_idx == 1
        legend('Location', 'best');
    end
end

% Finish Summary Figure Formatting
sgtitle('Combined Summary: Individual Tracking Profiles (Left: With RFF | Right: Without RFF)', 'FontSize', 14, 'FontWeight', 'bold');

% SAVE THE SUMMARY FIGURE
summary_fig_path = fullfile(save_dir, 'Summary_All_Profiles.png');
saveas(summary_fig, summary_fig_path);

% =========================================================
% 3. CREATE AND SAVE THE RESULTS TABLE (.CSV & Command Window)
% =========================================================

% Print to Command Window so you can still read it immediately
fprintf('\n========================================================================\n');
fprintf('                      TRACKING PERFORMANCE SUMMARY                        \n');
fprintf('========================================================================\n');
fprintf('%-30s | %-10s | %-10s | %-10s\n', 'File Name', 'RMSE', 'Mean Abs', 'Max Error');
fprintf('------------------------------------------------------------------------\n');
for i = 1:num_files
    fprintf('%-30s | %-10.4f | %-10.4f | %-10.4f\n', ...
        strrep(file_names{i}, '_', ' '), rmse_all(i), mae_all(i), max_error_all(i));
end
fprintf('========================================================================\n');


% Compile the data into a MATLAB Table variable (forcing column orientation)
ResultsTable = table(file_names(:), rmse_all(:), mae_all(:), max_error_all(:), ...
    'VariableNames', {'FileName', 'RMSE', 'MeanAbsoluteError', 'MaxError'});

% Save the Table as a CSV file in your chosen directory
csv_path = fullfile(save_dir, 'Tracking_Performance_Results.csv');
writetable(ResultsTable, csv_path);

fprintf('\nSUCCESS: All files successfully saved to:\n%s\n\n', save_dir);
%% 
% =========================================================
% 1. DEFINE NEW FILE NAMES & DIRECTORIES
% =========================================================
% Updated to test varying distances (5, 35, 65, 95 deg) at 1 rad/s
file_names = {
    '1rad.s_5deg_withRff',
    '1rad.s_35deg_withRff',
    '1rad.s_65deg_withRff',
    '1rad.s_95deg_withRff',
    '1rad.s_5deg_withoutRff',
    '1rad.s_35deg_withoutRff',
    '1rad.s_65deg_withoutRff',
    '1rad.s_95deg_withoutRff'
};

file_extension = '.mat'; 
num_files = length(file_names);

% --- DEFINE SAVE DIRECTORY ---
save_dir = 'C:\Studio_2.68_G11\Control_by_due\Control\Simulation\Reference_Feedforward_Expirement\Result_figure';

% Check if the folder exists. If not, create it.
if ~exist(save_dir, 'dir')
    mkdir(save_dir);
    fprintf('Created directory: %s\n', save_dir);
end

% Preallocate arrays for metrics
rmse_all = zeros(num_files, 1);
mae_all = zeros(num_files, 1);
max_error_all = zeros(num_files, 1);

% --- Setup the Master Summary Figure ---
summary_fig = figure('Name', 'Summary of All Tracking Profiles', 'Position', [50, 50, 1400, 800]);

% =========================================================
% 2. LOOP THROUGH ALL 8 FILES
% =========================================================
for i = 1:num_files
    
    full_file_name = strcat(file_names{i}, file_extension);
    clean_name = strrep(file_names{i}, '_', ' '); 
    
    try
        % --- A. Load the .mat file & Extract Simulink Data ---
        file_data = load(full_file_name); 
        
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
        
        ds = file_data.(dataset_var);
        
        target_signal = ds.getElement('velocity');
        actual_signal = ds.getElement('real_velocity');
        
        t        = target_signal.Values.Time(:);
        y_target = target_signal.Values.Data(:);
        y_actual = actual_signal.Values.Data(:);
        
    catch ME
        warning('Error processing %s: %s', full_file_name, ME.message);
        continue; 
    end
    
    % --- B. Calculate Errors ---
    error_signal = y_target - y_actual;
    absolute_error = abs(error_signal);
    
    rmse_all(i) = sqrt(mean(error_signal.^2));
    mae_all(i) = mean(absolute_error);
    max_error_all(i) = max(absolute_error);
    
    % --- C. Create & Save INDIVIDUAL Figure ---
    indiv_fig = figure('Name', clean_name, 'Position', [150 + (i*20), 150 + (i*20), 700, 400]);
    plot(t, y_target, 'Color', [0.8500 0.3250 0.0980], 'LineWidth', 1.5, 'DisplayName', 'Target Velocity'); 
    hold on;
    plot(t, y_actual, 'Color', [0 0.4470 0.7410], 'LineWidth', 1.5, 'DisplayName', 'Actual Velocity');
    
    title(clean_name, 'Interpreter', 'none', 'FontSize', 12);
    xlabel('Time (s)');
    ylabel('Velocity (rad/s)');
    legend('Location', 'best');
    grid on;
    
    % SAVE THE INDIVIDUAL FIGURE
    indiv_fig_path = fullfile(save_dir, sprintf('%s.png', file_names{i}));
    saveas(indiv_fig, indiv_fig_path);
    
    % --- D. Add SUBPLOT to SUMMARY Figure ---
    figure(summary_fig); 
    
    if i <= 4
        subplot_idx = (i * 2) - 1; % Left column (With Rff)
    else
        subplot_idx = (i - 4) * 2; % Right column (Without Rff)
    end
    
    subplot(4, 2, subplot_idx);
    plot(t, y_target, 'Color', [0.8500 0.3250 0.0980], 'LineWidth', 1.5, 'DisplayName', 'Target'); hold on;
    plot(t, y_actual, 'Color', [0 0.4470 0.7410], 'LineWidth', 1.2, 'DisplayName', 'Actual');
    
    title(clean_name, 'Interpreter', 'none', 'FontSize', 10);
    xlabel('Time (s)');
    ylabel('Velocity (rad/s)');
    grid on;
    
    if subplot_idx == 1
        legend('Location', 'best');
    end
end

% Finish Summary Figure Formatting
sgtitle('Combined Summary: Varying Angles at 1 rad/s (Left: With RFF | Right: Without RFF)', 'FontSize', 14, 'FontWeight', 'bold');

% SAVE THE SUMMARY FIGURE
% Renamed to avoid overwriting your previous 180deg tests
summary_fig_path = fullfile(save_dir, 'Summary_VaryingAngles_Profiles.png');
saveas(summary_fig, summary_fig_path);

% =========================================================
% 3. CREATE AND SAVE THE RESULTS TABLE (.CSV & Command Window)
% =========================================================

% Print to Command Window
fprintf('\n========================================================================\n');
fprintf('                 TRACKING PERFORMANCE SUMMARY (Varying Angles)            \n');
fprintf('========================================================================\n');
fprintf('%-30s | %-10s | %-10s | %-10s\n', 'File Name', 'RMSE', 'Mean Abs', 'Max Error');
fprintf('------------------------------------------------------------------------\n');
for i = 1:num_files
    fprintf('%-30s | %-10.4f | %-10.4f | %-10.4f\n', ...
        strrep(file_names{i}, '_', ' '), rmse_all(i), mae_all(i), max_error_all(i));
end
fprintf('========================================================================\n');

% Compile the data into a MATLAB Table variable (forcing column orientation)
ResultsTable = table(file_names(:), rmse_all(:), mae_all(:), max_error_all(:), ...
    'VariableNames', {'FileName', 'RMSE', 'MeanAbsoluteError', 'MaxError'});

% Save the Table as a CSV file in your chosen directory
% Renamed to avoid overwriting your previous 180deg results
csv_path = fullfile(save_dir, 'Tracking_Performance_Results_Angles.csv');
writetable(ResultsTable, csv_path);

fprintf('\nSUCCESS: All files successfully saved to:\n%s\n\n', save_dir);