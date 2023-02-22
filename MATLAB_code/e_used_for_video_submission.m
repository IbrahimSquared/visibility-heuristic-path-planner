clear; clc; clf;
% close all;
% clf;

addpath(genpath("visibility"))
%% Init
% In the paper, our threshold is set to 0.5, but it can be a bit more
% permissive, considering cells having visibility values > threshold to be
% visible
threshold = 0.5;

%% Seed - for repeatability
% rngstate = rng();
% rng(rngstate)
% save("rnd_2","rngstate");
load rnd_1
rng(rngstate)

%%
dx = 1; dy = 1;
nrows = 1000; ncols = 1000;
x = meshgrid(0:dx:nrows-dx,0:dy:ncols-dy); y = x';
% nx = 300; ny = nx;
[nx, ny] = size(x);

alpha = 1;

sp_o = [50*1/dy 50*1/dx];
ep_o = [950*1/dy 950*1/dx];
lightStrength = 1;

for repeat = 1:20
clf
% A grid enumerating the parent of each explored cell
lightSource_enum = nan(nx, ny);
% Set the starting point to have itself as a parent
lightSource_enum(sp_o(1),sp_o(2)) = 1;

%% Generate environment
obstacle = false(nx, ny);

nb_of_obstacles = 25;
min_width = 20*1/dy; max_width = 200*1/dy;
min_height = 20*1/dx; max_height = 200*1/dx;

for i = 1:nb_of_obstacles
    row_1 = 1 + randi(nx);
    row_2 = row_1 + min_width + randi(max_width - min_width);
    row_1 = min(row_1, nx-1); row_2 = min(row_2, nx-1);

    col_1 = 1 + randi(ny);
    col_2 = col_1 + min_height + randi(max_height - min_height);
    col_1 = min(col_1, ny-1); col_2 = min(col_2, ny-1);

    cond_1 = (row_1 <= sp_o(1)) && (sp_o(1) <= row_2);
    cond_2 = (col_1 <= sp_o(2)) && (sp_o(2) <= col_2);

    cond_3 = (row_1 <= ep_o(1)) && (ep_o(1) <= row_2);
    cond_4 = (col_1 <= ep_o(2)) && (ep_o(2) <= col_2);

    if  ~((cond_1 && cond_2) || (cond_3 && cond_4))
        obstacle(row_1:row_2,col_1:col_2) = true;
    end
end


%% Accessibility Map Init
accessibilityMap = ones(nx, ny);
% Get accessibillity map planner is the same as getAccessibilityMap but
% updates lightsourceEnum within
% Initial visibility for the starting point
[accessibilityMap_s, lightSource_enum] = getAccessibilityMapPlanner(alpha,lightStrength,sp_o,1-obstacle,accessibilityMap,lightSource_enum,1,threshold);
% Can be also computed for end point
% accessibilityMap_e = getAccessibilityMap(alpha,lightStrength,ep_o,1-obstacle,accessibilityMap);

%% Init visibility plots
% accessibilityMap_s = accessibilityMap_s >= 0.5;
m1 = mesh(obstacle,'FaceLighting','none','FaceColor','red',...
    'AmbientStrength',1,'EdgeLighting','flat',...
    'EdgeColor', 'red', 'FaceAlpha','1');
% imagesc(accessibilityMap);
hold on
m2 = mesh(accessibilityMap_s+0.01,'FaceLighting','phong','FaceColor','interp',...
        'AmbientStrength',0.5, 'EdgeColor', 'interp','FaceAlpha','1.0');

plot3(sp_o(2), sp_o(1), 1.15,'o','MarkerFaceColor','green',...
      'MarkerEdgeColor','black','MarkerSize', 18, 'LineWidth', 1.5)
plot3(ep_o(2), ep_o(1), 1.15,'o','MarkerFaceColor','blue',...
      'MarkerEdgeColor','black','MarkerSize', 18, 'LineWidth', 1.5)

grid off
set(gca, 'xtick', [-1e6 1e6]);
set(gca, 'ytick', [-1e6 1e6]);
set(gca,'LooseInset',get(gca,'TightInset'));

colormap(gray); 
axis equal
axis([1 nx 1 ny])
view(0,90)
pause(0.2)
% return

%% Explore
% initial waypoint
wp = sp_o;
% This collects visibility (does the union we speak about in the paper)
map_builder = accessibilityMap_s;

iter = 1;
sol = 0;

wp_old = wp;
% holders for waypoint (the set W)
wps_holder = [];
wps_holder(1,:) = wp;

% If seen intially (if trivial solution, return)
if accessibilityMap_s(ep_o(1),ep_o(2)) > threshold
    wps_holder(2,:) = ep_o;
    sprintf("Target visual confirmed")
    line([ep_o(2) sp_o(2)], [ep_o(1) sp_o(1)], [1.05 1.05], 'linewidth', 2, ...
         'color', 'blue','LineStyle','-')
    return
end

while true
    % Queue visibile region/set and compute heuristic for it
    [xq, yq] = find(map_builder > threshold);
    v = zeros(length(xq),1);
    d_to_target = v; d_from_parent = v; d_from_previous_waypoint = v;
    for i = 1:length(xq)
        v(i) = map_builder(xq(i),yq(i));
        % bias towards goal
        d_to_target(i) = sqrt( (xq(i)-ep_o(1))^2 + (yq(i)-ep_o(2))^2 );
        parent = wps_holder(lightSource_enum(xq(i),yq(i)), :);
        % bias from parent
        % d_from_parent(i) = sqrt( (xq(i) -  parent(1))^2 + (yq(i) - parent(2))^2);
        % bias from previous waypoint
        d_from_previous_waypoint(i) = sqrt( (xq(i) -  wp_old(1))^2 + (yq(i) - wp_old(2))^2);
    end
    
    d_tot = d_to_target + d_from_previous_waypoint;
    % Scale visibility
    v = (max(d_tot)-min(d_tot))*(v-min(v))/(max(v)-min(v)) + min(d_tot);
    % Fun is the heuristic, it is a bit different from the paper in this
    % specific code but it can be edited. We multiply instead of add.
    fun = v + d_tot;
    [m, ind] = min(fun);
     
    wp = [xq(ind),yq(ind)];
    iter = iter + 1;
    wps_holder(iter,:) = wp;

    plot3(wp(2),wp(1),1.15,'o','MarkerFaceColor','cyan',...
          'MarkerEdgeColor','black', 'MarkerSize', 18, 'LineWidth', 1.5)
      
    wp_old = wp;

    % Compute new visibility polygon for the new waypoint
    [accessibilityMap_wp, lightSource_enum] = getAccessibilityMapPlanner(alpha,lightStrength,wp,1-obstacle,accessibilityMap,lightSource_enum,iter,threshold);
    map_builder = max(map_builder,accessibilityMap_wp);
    m2 = mesh(map_builder+0.01,'FaceLighting','phong','FaceColor','interp',...
        'AmbientStrength',0.5, 'EdgeColor', 'interp','FaceAlpha','1.0');
    hold on
    
    % pause for visualization purposes
    pause(0.2)

    % If stopping criterion => stop
    if accessibilityMap_wp(ep_o(1),ep_o(2)) > threshold
        sprintf("Target visual confirmed in %i iterations.", iter)
        sol = 1;
        break
    end
    
    delete(m2);
end

pause(0.7)
%% Plot the path, backtrack parents of waypoints starting from end point
pt = ep_o;
while (pt(1) ~= sp_o(1) || pt(2) ~= sp_o(2)) 
    pt_1 = wps_holder(lightSource_enum(pt(1),pt(2)),:);
    plot3(pt_1(2),pt_1(1),1.15,'o','MarkerFaceColor','magenta',...
          'MarkerEdgeColor','black', 'MarkerSize', 18, 'LineWidth', 1.5)
    line([pt_1(2) pt(2)], [pt_1(1) pt(1)], [1.15 1.15],'linewidth', 2, ...
                            'color', 'magenta','LineStyle','-');
    pt = pt_1;
end
plot3(sp_o(2), sp_o(1), 1.15,'o','MarkerFaceColor','green',...
      'MarkerEdgeColor','black','MarkerSize', 18, 'LineWidth', 1.5)
plot3(ep_o(2), ep_o(1), 1.15,'o','MarkerFaceColor','blue',...
      'MarkerEdgeColor','black','MarkerSize', 18, 'LineWidth', 1.5)
pause(0.2)
end