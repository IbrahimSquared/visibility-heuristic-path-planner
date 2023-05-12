clear; clc;

% Clear all figures
FigList = findall(groot, 'Type', 'figure');
for iFig = 1:numel(FigList)
    try
        clf(FigList(iFig));
    catch
        % Nothing to do
    end
end

% fid = fopen('main.bat','w');
% fprintf(fid,'%s\n','set path=%path:C:\Program Files\MATLAB\R2020b\bin\win64;=%');
% fprintf(fid,'%s\n','visibility_heuristic_planner.exe');
% fclose(fid);
system('visibility_heuristic_planner.bat');

%% Parser - parse settings

% Open the file for reading
fid = fopen('config/settings.config', 'r');

% Define the format string for textscan
formatSpec = '%s';

% Read the settings using textscan
settingsCell = textscan(fid, formatSpec, 'Delimiter', '=', 'CommentStyle', '#');

% Close the file
fclose(fid);

% Convert the cell array to a struct
settings = struct();
for i = 1:2:length(settingsCell{1})
    key = settingsCell{1}(i);
    value = settingsCell{1}(i+1);
    % Check if the value is a number
    if ~isnan(str2double(value{1}))
        % Convert the value to a number
        value = str2double(value{1});
    end
    % Save the key-value pair in the settings struct
    if strcmp(key{1}, 'imagePath') || strcmp(key{1}, 'start')...
            || strcmp(key{1}, 'end')
        % Don't convert imagePath and initialFrontline to numbers
        settings.(key{1}) = value{1};
    else
        settings.(key{1}) = value;
    end
    
    if strcmp(key{1}, 'start') || strcmp(key{1}, 'end')
        settings.(key{1}) = eval(strrep(strrep(value{1}, '{', '['), '}', ']'));
    end
end

%% Visibility field (basically occupancy grid)
filename_visibilityField = "output/visibilityField.txt";
T_visibilityField = readtable(filename_visibilityField,'Delimiter',' ');
visibilityField = T_visibilityField.Variables;
plotVisibilityField = true;

if plotVisibilityField
    [nx, ny] = size(visibilityField);

    figure(1)
    set(gcf, 'Name', 'Visibility field (occupancy grid)')
    mesh(visibilityField,'FaceLighting','phong','FaceColor','interp',...
        'AmbientStrength',1.0, 'EdgeColor', 'interp','FaceAlpha','1.0');
    colormap(gray)
    view(0,90)
    axis equal
    axis([1 ny 1 nx])
    hold on

    grid off
    set(gca, 'xtick', [-1e6 1e6]);
    set(gca, 'ytick', [-1e6 1e6]);
    set(gca,'LooseInset',get(gca,'TightInset'));
end

%% Visibility heuristic path planning plots
filename_visibility = "output/VisibilityMap.txt";
T_visibility = readtable(filename_visibility,'Delimiter',' ');
map_visibility = T_visibility.Variables;

figure(2)
set(gcf, 'Name', 'Visibility heuristic path planning')
mesh(imcomplement(visibilityField),'FaceLighting','none','FaceColor','red',...
    'AmbientStrength',1,'EdgeLighting','flat',...
    'EdgeColor', 'red', 'FaceAlpha','1');
hold on
mesh(map_visibility+0.01,'FaceLighting','phong','FaceColor','interp',...
        'AmbientStrength',0.5, 'EdgeColor', 'interp','FaceAlpha','1.0');
colormap(gray)

view(0,90)
axis equal
axis([1 ny 1 nx])
hold on

grid off
set(gca, 'xtick', [-1e6 1e6]);
set(gca, 'ytick', [-1e6 1e6]);

filename_lightSources = "output/lightSources.txt";
T = readtable(filename_lightSources, 'Delimiter',' ');
pivots = T.Variables;
pivots = pivots + 1;

for i = 1:size(pivots,1)
    pt_ = pivots(i,:);
    plot3(pt_(1), pt_(2),1.15,'o','MarkerFaceColor','magenta',...
      'MarkerEdgeColor','black','MarkerSize', 14, 'LineWidth', 1)
end

sp_o = settings.start + 1;
ep_o = settings.end + 1;
pt = ep_o;

filename_cameFrom = "output/cameFrom.txt";
T = readtable(filename_cameFrom, 'Delimiter',' ');
cameFrom = T.Variables + 1;

% Plot the path as lines
path = [pt];
total_distance = 0;
while true 
    if pt(1) == sp_o(1) && pt(2) == sp_o(2)
        break
    end

    plot3(pt(1), pt(2),1.15,'o','MarkerFaceColor','magenta',...
          'MarkerEdgeColor','black','MarkerSize', 14, 'LineWidth', 1)
    pt = pivots(cameFrom(pt(2), pt(1)),:);

    path(end+1,:) = pt;

    total_distance = total_distance + norm(path(end,:) - path(end-1,:));
    line([path(end,1), path(end-1,1)], [path(end,2), path(end-1,2)], ...
        [1.05 1.05], 'LineWidth', 2, "color", "magenta")
    % pause(0.01)
end
sprintf("Total distance: %0.2f", total_distance)

plot3(sp_o(1), sp_o(2),1.15,'o','MarkerFaceColor','green',...
          'MarkerEdgeColor','black','MarkerSize', 14, 'LineWidth', 1)
plot3(ep_o(1), ep_o(2),1.15,'o','MarkerFaceColor','blue',...
          'MarkerEdgeColor','black','MarkerSize', 14, 'LineWidth', 1)

grid off
set(gca, 'xtick', [-1e6 1e6]);
set(gca, 'ytick', [-1e6 1e6]);
set(gca,'LooseInset',get(gca,'TightInset'));

%% Expfig
% addpath(genpath('MATLAB_code/expfig'))
% export_fig misc_example -r400 -transparent -jpg