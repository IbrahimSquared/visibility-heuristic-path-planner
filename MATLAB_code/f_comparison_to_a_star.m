clear; clc; clf;
% close all;
% clf;

addpath(genpath("visibility"))
addpath(genpath('Astar'))
%% Init
% In the paper, our threshold is set to 0.5, but it can be a bit more
% permissive, considering cells having visibility values > threshold to be
% visible
threshold = 0.2;

%% Seed - for repeatability
% rngstate = rng();
% rng(rngstate)
% save("rnd_2","rngstate");
load rnd_1
rng(rngstate)


%%
dx = 1; dy = 1;
nrows = 100; ncols = 100;
x = meshgrid(0:dx:nrows-dx,0:dy:ncols-dy); y = x';
% nx = 300; ny = nx;
[nx, ny] = size(x);

alpha = 1;

sp_o = [5*1/dy 5*1/dx];
ep_o = [95*1/dy 95*1/dx];
% A star map has to be flipped so we also flip start and end points
sp_o_star = [95*1/dy 5*1/dx];
ep_o_star = [5*1/dy 95*1/dx];

lightStrength = 1;

repititions = 10;
times_visibility = zeros(repititions,1);
distances_visibility = zeros(repititions,1);
times_a_star = zeros(repititions,1);
distances_a_star = zeros(repititions,1);
repeat = 1;
%% 
while repeat <= repititions
    clf
    % A grid enumerating the parent of each explored cell
    lightSource_enum = nan(nx, ny);
    % Set the starting point to have itself as a parent
    lightSource_enum(sp_o(1),sp_o(2)) = 1;

    %% Generate environment
    obstacle = false(nx, ny);

    nb_of_obstacles = 25;
    min_width = 2*1/dy; max_width = 20*1/dy;
    min_height = 2*1/dx; max_height = 20*1/dx;

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
          'MarkerEdgeColor','black','MarkerSize', 14, 'LineWidth', 1)
    plot3(ep_o(2), ep_o(1), 1.15,'o','MarkerFaceColor','red',...
          'MarkerEdgeColor','black','MarkerSize', 14, 'LineWidth', 1)

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
        continue;
    end

    tic
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

        wp_old = wp;

        % Compute new visibility polygon for the new waypoint
        [accessibilityMap_wp, lightSource_enum] = getAccessibilityMapPlanner(alpha,lightStrength,wp,1-obstacle,accessibilityMap,lightSource_enum,iter,threshold);
        map_builder = max(map_builder,accessibilityMap_wp);

        % If stopping criterion => stop
        if accessibilityMap_wp(ep_o(1),ep_o(2)) > threshold
            % sprintf("Target visual confirmed in %i iterations.", iter)
            sol = 1;
            break
        end
    end
    time_visibility = toc;
    delete(m2)
    m2 = mesh(map_builder+0.01,'FaceLighting','phong','FaceColor','interp',...
            'AmbientStrength',0.5, 'EdgeColor', 'interp','FaceAlpha','1.0');
    % sprintf("Found solution with visibility")
    pause(0.1)
    
    %% Plot the path, backtrack parents of waypoints starting from end point
    pt = ep_o;
    d_tot_1 = 0;
    while (pt(1) ~= sp_o(1) || pt(2) ~= sp_o(2)) 
        pt_1 = wps_holder(lightSource_enum(pt(1),pt(2)),:);
        plot3(pt_1(2),pt_1(1),1.15,'o','MarkerFaceColor','magenta',...
              'MarkerEdgeColor','black', 'MarkerSize', 14, 'LineWidth', 1)
        line([pt_1(2) pt(2)], [pt_1(1) pt(1)], [1.15 1.15],'linewidth', 2, ...
                                'color', 'magenta','LineStyle','-');
        d_tot_1 = d_tot_1 + norm(pt_1 - pt);
        pt = pt_1;
    end
    plot3(sp_o(2), sp_o(1), 1.15,'o','MarkerFaceColor','green',...
          'MarkerEdgeColor','black','MarkerSize', 14, 'LineWidth', 1)
    plot3(ep_o(2), ep_o(1), 1.15,'o','MarkerFaceColor','red',...
          'MarkerEdgeColor','black','MarkerSize', 14, 'LineWidth', 1)
    pause(0.1)
    %% astar
    obstacle_temp = false(nrows, ncols);
    for i = 1:size(obstacle,1)-1
        for j = 1:size(obstacle,2)-1
            if obstacle(i,j) == 1
                obstacle_temp(nrows-i,j) = 1;
            end
        end
    end
    MAP = obstacle_temp;

    % sp_o = [95,5];
    % ep_o = [10,95];
    % sp_o = [5, 5];
    % ep_o = [90, 95];

    % sp_o = [95*1/dy 5*1/dx];
    % ep_o = [5*1/dy 95*1/dx];

    %Start Positions
    StartX=sp_o_star(2);
    StartY=sp_o_star(1);
    % tic
    %Generating goal nodes, which is represented by a matrix. Several goals can be speciefied, in which case the pathfinder will find the closest goal. 
    %a cell with the value 1 represent a goal cell
    GoalRegister=int8(zeros(nrows,ncols));
    GoalRegister(ep_o_star(1),ep_o_star(2))=1;

    %Number of Neighboors one wants to investigate from each cell. A larger
    %number of nodes means that the path can be alligned in more directions. 
    %Connecting_Distance=1-> Path can  be alligned along 8 different direction.
    %Connecting_Distance=2-> Path can be alligned along 16 different direction.
    %Connecting_Distance=3-> Path can be alligned along 32 different direction.
    %Connecting_Distance=4-> Path can be alligned along 56 different direction.
    %ETC......

    Connecting_Distance=4; %Avoid to high values Connecting_Distances for reasonable runtimes. 

    % Running PathFinder
    try
        tic
        OptimalPath=ASTARPATH(StartX,StartY,MAP,GoalRegister,Connecting_Distance);
        time_a_star = toc;
        % End. 

        if size(OptimalPath,2)>1
        % figure(10)
        % imagesc((MAP))
        %     colormap(flipud(gray));

        % hold on
        % plot(OptimalPath(1,2),OptimalPath(1,1),'o','color','k')
        % plot(OptimalPath(end,2),OptimalPath(end,1),'o','color','b')
        % plot(OptimalPath(:,2),OptimalPath(:,1),'r')
        % legend('Goal','Start','Path')

        else 
             pause(1);
         h=msgbox('Sorry, No path exists to the Target!','warn');
         uiwait(h,5);
        end

        showNeighboors=0; %Set to 1 if you want to visualize how the possible directions of path. The code
        %below are purley for illustrating purposes. 
        if showNeighboors==1
        %

        %2
        NeigboorCheck=[0 1 0 1 0;1 1 1 1 1;0 1 0 1 0;1 1 1 1 1;0 1 0 1 0]; %Heading has 16 possible allignments
        [row col]=find(NeigboorCheck==1);
        Neighboors=[row col]-(2+1);
        figure(2)

        % for p=1:size(Neighboors,1)
        %   i=Neighboors(p,1);
        %        j=Neighboors(p,2);
        %       
        %      plot([0 i],[0 j],'k')
        %  hold on
        %  axis equal
        %  
        % grid on
        % title('Connecting distance=2')
        % end

        %3
        NeigboorCheck=[0 1 1 0 1 1 0;1 0 1 0 1 0 1;1 1 1 1 1 1 1;0 0 1 0 1 0 0;1 1 1 1 1 1 1;1 0 1 0 1 0 1;0 1 1 0 1 1 0]; %Heading has 32 possible allignments
        figure(3)
        [row col]=find(NeigboorCheck==1);
        Neighboors=[row col]-(3+1);

        % for p=1:size(Neighboors,1)
        %   i=Neighboors(p,1);
        %        j=Neighboors(p,2);
        %       
        %      plot([0 i],[0 j],'k')
        %  hold on
        %  axis equal
        %  grid on
        % title('Connecting distance=3')
        % 
        % end

        %4
        NeigboorCheck=[0 1 1 1 0 1 1 1 0;1 0 1 1 0 1 1 0 1;1 1 0 1 0 1 0 1 1;1 1 1 1 1 1 1 1 1;0 0 0 1 0 1 0 0 0;1 1 1 1 1 1 1 1 1;1 1 0 1 0 1 0 1 1 ;1 0 1 1 0 1 1 0 1 ;0 1 1 1 0 1 1 1 0];  %Heading has 56 possible allignments
        figure(4)
        [row col]=find(NeigboorCheck==1);
        Neighboors=[row col]-(4+1);

        % for p=1:size(Neighboors,1)
        %   i=Neighboors(p,1);
        %        j=Neighboors(p,2);
        %       
        %      plot([0 i],[0 j],'k')
        %  hold on
        %  axis equal
        % grid on
        % title('Connecting distance=4')
        % 
        % end
        %1
        NeigboorCheck=[1 1 1;1 0 1;1 1 1];
        figure(1)
        [row col]=find(NeigboorCheck==1);
        Neighboors=[row col]-(1+1);

        % for p=1:size(Neighboors,1)
        %   i=Neighboors(p,1);
        %        j=Neighboors(p,2);
        %       
        %      plot([0 i],[0 j],'k')
        %  hold on
        %  axis equal
        % grid on
        % title('Connecting distance=1')
        % 
        % end
        end

        d_tot_2 = 0;
        for i = 1:length(OptimalPath)-1
            d_tot_2 = d_tot_2 + norm(OptimalPath(i+1,:) - OptimalPath(i,:));
        end

        %%
        hold on
        OptimalPath_edit = [OptimalPath(:,2) 100-OptimalPath(:,1)];
        plot3(OptimalPath_edit(:,1),OptimalPath_edit(:,2),1.15*ones(length(OptimalPath_edit),1),'-*',...
            'MarkerFaceColor','green','markersize',8);
    catch
        time_a_star = nan;
        d_tot_2 = nan;
    end

    %%

    % sprintf("Distance - Visibility: %f, A_star: %f", d_tot_1, d_tot_2)
    % sprintf("Distance Error: %f%%", 100 * (d_tot_1-d_tot_2) / d_tot_2)

    % sprintf("Time - Visibility: %f, A_star: %f", time_visibility, time_a_star)
    % sprintf("Time Error: %f%%", 100 * (time_a_star-time_visibility) / time_visibility)
    
    times_visibility(repeat,1) = time_visibility;
    distances_visibility(repeat,1) = d_tot_1;
    times_a_star(repeat,1) = time_a_star;
    distances_a_star(repeat,1) = d_tot_2;
    sprintf("Repeat: %i", repeat)
    repeat = repeat + 1;
    pause(0.1)
end

%% 
[mean(distances_visibility) min(distances_visibility) max(distances_visibility)]
[mean(distances_a_star) min(distances_a_star) max(distances_a_star)]

[mean(times_visibility) min(times_visibility) max(times_visibility)]
[mean(times_a_star) min(times_a_star) max(times_a_star)]