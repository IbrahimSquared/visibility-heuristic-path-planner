clear; clc;
% close all;
clf;

addpath(genpath("visibility"));

%% Seed, for repeatability
% 3 lines below will create a new seed and save it, which can be loaded
% then
% rngstate = rng();
% rng(rngstate)
% save("rnd_2","rngstate");
% Comment those two lines to change the randomly generated environment
load rnd_1
rng(rngstate)

%%
% step sizes
dx = 0.5; dy = 0.5;
% grid dimensions and setting up the grid
nrows = 300; ncols = 300;
x = meshgrid(0:dx:nrows-dx,0:dy:ncols-dy); y = x';
[nx, ny] = size(x);

% decay factor (only decrease a little bit like 0.995 because it's effect
% is exponential)
alpha = 1;

% Starting position (intial light position)
sp_o = [140*1/dy 150*1/dx];
% sp_o = [50*1/dy 50*1/dx];

%% Generate environment - environment generator
obstacle = false(nx, ny);

nb_of_obstacles = 25;
% obstacle dimensions
min_width = 2*1/dy; max_width = 40*1/dy;
min_height = 2*1/dx; max_height = 40*1/dx;

for i = 1:nb_of_obstacles
    row_1 = 0 + randi(nx);
    row_2 = row_1 + min_width + randi(max_width - min_width);
    row_1 = min(row_1, nx-0); row_2 = min(row_2, nx-0);

    col_1 = 0 + randi(ny);
    col_2 = col_1 + min_height + randi(max_height - min_height);
    col_1 = min(col_1, ny-0); col_2 = min(col_2, ny-0);
    
    % Conditions to make sure that the starting point is not inside an obstacle 
    cond_1 = (row_1 <= sp_o(1)) && (sp_o(1) <= row_2);
    cond_2 = (col_1 <= sp_o(2)) && (sp_o(2) <= col_2);
    
    if  ~(cond_1 && cond_2)
        obstacle(row_1:row_2,col_1:col_2) = true;
    end
end
    
%% Adding non-polygonal shapes to the generated environment like circles
% Those are fixed, comment them if needed
X = meshgrid(1:nx, 1:ny); Y = X;
R = 15*1/dx;
t1 = (X-255*1/dx).^2 + (Y'-157*1/dy).^2 <= R^2;
R = 10*1/dx;
t2 = (X-255*1/dx).^2 + (Y'-157*1/dy).^2 >= R^2;
t = min(t1,t2);
obstacle(t) = true;
obstacle((147+7)*1/dx:(153+7)*1/dx,235*1/dy:end) = false;

R = 15*1/dx;
t = (X-155*1/dx).^2 + (Y'-50*1/dy).^2 <= R^2;
obstacle(t) = true;
% obstacle(290:343,727:828) = false;
% obstacle(160:343,727-50:827-50) = false;

R = 10*1/dx;
t = (X-180*1/dx).^2 + (Y'-95*1/dy).^2 <= R^2;
obstacle(t) = true;

%% Instead, one can manually generate an environment 
% obstacle = zeros(nx, ny);
% obstacle(30:40, 30:40) = 1;
% obstacle(90:100, 30:40) = 1;

% obstacle(180:220,140:180) = 1;
% obstacle(280:430,140:180) = 1;
% obstacle(70:190,440:480) = 1;

%% some settings
% light strength can be changed meaning the starting visibility value can
% be set to be different than 1
lightStrength = 1;

% initial light position is the starting position
lightPos = sp_o;

% Factor by which to change c(x,y), for example fac = 2.5 => c = 2.5*y/x.
% See a_quiver_plots.m and the paper to see the effect this does.
fac = 1; 

%% Compute the accessibility/visibility map/polygon
% Init
accessibilityMap = ones(nx, ny);
% Compute the map/polygon
accessibilityMap = getAccessibilityMap(alpha,lightStrength,...
    lightPos,1-obstacle,accessibilityMap,fac);

%% Plotting
% Uncomment if binary visibility polygon is needed (1 = visible, 0 =
% invisible), otherwise the map has some dispersion/gradient
% accessibilityMap = accessibilityMap >= 0.5;

% adding 0.01 is only for visualization purposes because we plot the
% obstacles over the same map (to differentiate obstacle plot from
% visibility plot)
mesh(accessibilityMap+0.01,'FaceLighting','none','FaceColor','interp',...
    'AmbientStrength',0.5,'EdgeLighting','flat',...
    'EdgeColor', 'interp','FaceAlpha','1.0');
hold on
mesh(obstacle,'FaceLighting','none','FaceColor','red',...
    'AmbientStrength',0.5,'EdgeLighting','flat',...
    'EdgeColor', 'red', 'FaceAlpha','1');

% Add starting point as a yellow ball
plot3(sp_o(2), sp_o(1), 1.05,'o','MarkerFaceColor','yellow',...
  'MarkerEdgeColor','black','MarkerSize', 14, 'LineWidth', 1)

grid off
set(gca, 'xtick', [-1e6 1e6]);
set(gca, 'ytick', [-1e6 1e6]);
set(gca,'LooseInset',get(gca,'TightInset'));
% Another way to visualize the map instead of mesh
% imagesc(accessibilityMap);
% hold on

% shading flat
% Can be changed to different colormaps, test if you want with jet, prism,
% lines, hot instead of gray
colormap(gray); 
axis equal
axis([1 ny 1 nx])
view(0,90)

%% For saving purposes using expfig
% addpath(genpath('expfig'))
% the output file name would be sample.jpg for the line below
% export_fig sample -r400 -transparent -jpg