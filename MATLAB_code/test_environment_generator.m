clear; clc; clf;

dx = 1; dy = 1;
nrows = 100; ncols = 100;
x = meshgrid(0:dx:nrows-dx,0:dy:ncols-dy); y = x';
% nx = 300; ny = nx;
[nx, ny] = size(x);

sp_o = [5*1/dy 5*1/dx];
ep_o = [95*1/dy 95*1/dx];

%% Generate environment
obstacle = false(nx, ny);

nb_of_obstacles = 25;
min_width = 2*1/dy; max_width = 20*1/dy;
min_height = 2*1/dx; max_height = 20*1/dx;

    for i = 1:nb_of_obstacles
        row_1 = 0 + randi(nx);
        row_2 = row_1 + min_width + randi(max_width - min_width);
        row_1 = min(row_1, nx); row_2 = min(row_2, nx);

        col_1 = 0 + randi(ny);
        col_2 = col_1 + min_height + randi(max_height - min_height);
        col_1 = min(col_1, ny); col_2 = min(col_2, ny);

        cond_1 = (row_1 <= sp_o(2)) && (sp_o(2) <= row_2);
        cond_2 = (col_1 <= sp_o(1)) && (sp_o(1) <= col_2);

        cond_3 = (row_1 <= ep_o(2)) && (ep_o(2) <= row_2);
        cond_4 = (col_1 <= ep_o(1)) && (ep_o(1) <= col_2);

        if  ~((cond_1 && cond_2) || (cond_3 && cond_4))
            obstacle(row_1:row_2,col_1:col_2) = true;
        end
    end

[obstacle(ep_o(1),ep_o(2)) obstacle(sp_o(1),sp_o(2))]

mesh(obstacle,'FaceLighting','none','FaceColor','interp',...
    'AmbientStrength',1,'EdgeLighting','flat',...
    'EdgeColor', 'interp', 'FaceAlpha','1');

hold on
plot3(sp_o(2), sp_o(1), 1.15,'o','MarkerFaceColor','green',...
          'MarkerEdgeColor','black','MarkerSize', 14, 'LineWidth', 1)
plot3(ep_o(2), ep_o(1), 1.15,'o','MarkerFaceColor','blue',...
      'MarkerEdgeColor','black','MarkerSize', 14, 'LineWidth', 1)
  
grid off
set(gca, 'xtick', [-1e6 1e6]);
set(gca, 'ytick', [-1e6 1e6]);
set(gca,'LooseInset',get(gca,'TightInset'));

colormap(gray); 
axis equal
axis([1 nx 1 ny])
view(0,90)
      