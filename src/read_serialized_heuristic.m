clear; clc;
clf

% fid = fopen('main.bat','w');
% fprintf(fid,'%s\n','set path=%path:C:\Program Files\MATLAB\R2022b\bin;=%');
% fprintf(fid,'%s\n','main.exe');
% fclose(fid);
system('main.bat');

filename_visibility_env  = "output/VisibilityMap_env.txt";
filename_visibility = "output/VisibilityMap.txt";
filename_local_visibility = "output/LocalVisibilityMap.txt";
filename_lightSources = "output/lightSources.txt";
filename_lightSource_Eum = "output/LightSourceEnum.txt";

T = readtable(filename_lightSource_Eum, 'Delimiter',' ');
lightSource_enum = T.Variables;

T = readtable(filename_local_visibility, 'Delimiter',' ');
map_local_visibility = T.Variables;

T = readtable(filename_lightSources, 'Delimiter',' ');
pivots = T.Variables;
pivots = pivots + 1;

T_visibility = readtable(filename_visibility,'Delimiter',' ');
map_visibility = T_visibility.Variables;

T_visibility_env = readtable(filename_visibility_env,'Delimiter',' ');
map_visibility_env = T_visibility_env.Variables;

nx = size(map_visibility,1);
ny = size(map_visibility,2);

%%
clf
mesh(imcomplement(map_visibility_env),'FaceLighting','none','FaceColor','red',...
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
% set(gca,'LooseInset',get(gca,'TightInset'));

% return

for i = 1:size(pivots,1)
    pt_ = pivots(i,:);
    if i == 1
        plot3(pt_(2), pt_(1),1.15,'o','MarkerFaceColor','green',...
          'MarkerEdgeColor','black','MarkerSize', 14, 'LineWidth', 1)
    elseif i == size(pivots,1)
        plot3(pt_(2), pt_(1),1.15,'o','MarkerFaceColor','blue',...
          'MarkerEdgeColor','black','MarkerSize', 14, 'LineWidth', 1)
    else
        plot3(pt_(2), pt_(1),1.15,'o','MarkerFaceColor','cyan',...
          'MarkerEdgeColor','black','MarkerSize', 14, 'LineWidth', 1)
    end
end

% Start point and target point
% sp_o = [1 1];
% ep_o = [101 101];
sp_o = [170 316] + 1;
ep_o = [155 6] + 1;
pt = ep_o;

% Plot the path as lines
pointz = [pt];
d_1 = 0;
while true 
    if pt(1) == sp_o(1) && pt(2) == sp_o(2)
        break
    end
    plot3(pt(2), pt(1),1.15,'o','MarkerFaceColor','magenta',...
          'MarkerEdgeColor','black','MarkerSize', 14, 'LineWidth', 1)
    pt = pivots(lightSource_enum(pt(1),pt(2))+1,:);
    pointz(end+1,:) = pt;
    d_1 = d_1 + norm(pointz(end,:)-pointz(end-1,:));
    line([pointz(end,2),pointz(end-1,2)],[pointz(end,1),pointz(end-1,1)],[1.05 1.05], 'LineWidth', 2, "color", "magenta")
    % pause(0.01)
end
d_1

plot3(sp_o(2), sp_o(1),1.15,'o','MarkerFaceColor','green',...
          'MarkerEdgeColor','black','MarkerSize', 14, 'LineWidth', 1)
plot3(ep_o(2), ep_o(1),1.15,'o','MarkerFaceColor','blue',...
          'MarkerEdgeColor','black','MarkerSize', 14, 'LineWidth', 1)
%% Expfig
% addpath(genpath('expfig'))
% set(gca,'LooseInset',get(gca,'TightInset'));
% export_fig blebla -r400 -transparent