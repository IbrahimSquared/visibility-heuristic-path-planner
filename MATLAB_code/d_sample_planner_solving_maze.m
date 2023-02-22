clear; clc; clf;
% close all;
% clf;

addpath(genpath("visibility"))
addpath(genpath("mazes"))
%% Init
% In the paper, our threshold is set to 0.5, but it can be a bit more
% permissive, considering cells having visibility values >= threshold to be
% visible
threshold = 0.2;

% Read the image maze.png
I = double(imread('maze_1.png'));
% I = double(imread('maze_2.png'));
% I = double(imread('maze_3.png'));
obstacle = im2bw(I);
obstacle = imcomplement(im2bw(I));
nrows = size(I,1); ncols = size(I,2);

% The code below can be used for obstacle inflation
% obstacle = imcomplement(im2bw(I));
% Inflate obstacle (if needed)
% u = strel("disk",2).Neighborhood;
% obstacle = conv2(obstacle, u, 'same');
% % obstacle_s = ones(nx,ny) + obstacle;
% obstacle_v = imcomplement(obstacle);
% obstacle_v = max(obstacle_v, 0);
% % obstacle_v = min(obstacle_v,1);
% obstacle = 1-obstacle_v;

% Refer to previous code for comments on those variables
lightStrength = 1;
alpha = 1;
lightPos = [315 169];
sp_o = lightPos;
% End position
ep_o = [7 153];

% A grid enumerating the parent of each explored cell
lightSource_enum = nan(nrows, ncols);
% Set the starting point to have itself as a parent
lightSource_enum(sp_o(1),sp_o(2)) = 1;

%% Accessibility Map Init
accessibilityMap = ones(nrows, ncols);
% Get accessibillity map planner is the same as getAccessibilityMap but
% updates lightsourceEnum within
% Initial visibility for the starting point
[accessibilityMap_s, lightSource_enum] = getAccessibilityMapPlanner(alpha,lightStrength,sp_o,1-obstacle,accessibilityMap,lightSource_enum,1,threshold);
% Can be also computed for end point
% accessibilityMap_e = getAccessibilityMap(alpha,lightStrength,ep_o,1-obstacle,accessibilityMap);

%% Init visibility plots
% accessibilityMap_s = accessibilityMap_s >= 0.5;
mesh(obstacle,'FaceLighting','none','FaceColor','red',...
    'AmbientStrength',0.5,'EdgeLighting','flat',...
    'EdgeColor', 'red', 'FaceAlpha','1');
% imagesc(accessibilityMap);
hold on
plot3(sp_o(2), sp_o(1), 1.15,'o','MarkerFaceColor','green',...
      'MarkerEdgeColor','black','MarkerSize', 18, 'LineWidth', 1)
plot3(ep_o(2), ep_o(1), 1.15,'o','MarkerFaceColor','blue',...
      'MarkerEdgeColor','black','MarkerSize', 18, 'LineWidth', 1)

grid off
set(gca, 'xtick', [-1e6 1e6]);
set(gca, 'ytick', [-1e6 1e6]);
set(gca,'LooseInset',get(gca,'TightInset'));

colormap(gray); 
axis equal
axis([1 nrows 1 ncols])
view(0,90)

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
    % d_to_target = v; d_from_parent = v;
    for i = 1:length(xq)
        v(i) = map_builder(xq(i),yq(i));
    end
    
    % d_tot = d_to_target + d_from_parent;
    % or 
    % d_tot = d_to_target + d_from_previous_waypoint
    % or any other better implementation

    % bias towards goal
    d_ep = sqrt( (xq-ep_o(1)).^2 + (yq-ep_o(2)).^2);
    d_sp = sqrt( (xq-wp(1)).^2 + (yq-wp(2)).^2);
    d_tot = d_ep + d_sp;

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
          'MarkerEdgeColor','black', 'MarkerSize', 18, 'LineWidth', 1)
      
    wp_old = wp;

    % Compute new visibility polygon for the new waypoint
    [accessibilityMap_wp, lightSource_enum] = getAccessibilityMapPlanner(alpha,lightStrength,wp,1-obstacle,accessibilityMap,lightSource_enum,iter,threshold);
    map_builder = max(map_builder,accessibilityMap_wp);
    m = mesh(map_builder,'FaceLighting','phong','FaceColor','interp',...
        'AmbientStrength',0.5, 'EdgeColor', 'interp','FaceAlpha','1.0');
    hold on
    
    % If stopping criterion => stop
    if accessibilityMap_wp(ep_o(1),ep_o(2)) > threshold
        sprintf("Target visual confirmed in %i iterations.", iter)
        sol = 1;
        break
    end
    % pause for visualization purposes
    pause(0.1)
    delete(m);
end

%% Plot the path, backtrack parents of waypoints starting from end point
pt = ep_o;
while ((pt(1) ~= sp_o(1)) || (pt(2) ~= sp_o(2))) 
    pt_1 = wps_holder(lightSource_enum(pt(1),pt(2)),:);
    plot3(pt_1(2),pt_1(1),1.15,'o','MarkerFaceColor','magenta',...
          'MarkerEdgeColor','black', 'MarkerSize', 18, 'LineWidth', 1)
    line([pt_1(2) pt(2)], [pt_1(1) pt(1)], [1.15 1.15],'linewidth', 2, ...
                            'color', 'magenta','LineStyle','-');
    pt = pt_1;
end
plot3(sp_o(2), sp_o(1), 1.15,'o','MarkerFaceColor','green',...
      'MarkerEdgeColor','black','MarkerSize', 18, 'LineWidth', 1)
plot3(ep_o(2), ep_o(1), 1.15,'o','MarkerFaceColor','blue',...
      'MarkerEdgeColor','black','MarkerSize', 18, 'LineWidth', 1)

%% Expfig - requires the folder expfig or the path to it
% addpath(genpath('expfig'))
% % set(gca,'LooseInset',get(gca,'TightInset'));
% export_fig maze_sol_x -r400 -png