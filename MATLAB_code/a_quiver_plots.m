clc; clear;
%% Quiver plots
clf
nrows = 25; ncols = nrows;

x = meshgrid(-nrows+1:nrows); 
y = x';

%% For saving figures using expfig - uncomment if needed
% addpath(genpath('expfig'))

%% For visibility polygon
figure
u = x./sqrt(2.5*x.^2 + y.^2); 
v = y./sqrt(2.5*x.^2 + y.^2);
quiver(x,y,u,v,'color','black');
% axis equal
axis([-nrows ncols -nrows ncols])
set(gca, 'xtick', [-1e6 1e6]);
set(gca, 'ytick', [-1e6 1e6]);
set(gca,'LooseInset',get(gca,'TightInset'));
% uncomment the line below to save the figure as quiver_1.jpg, needs expfig
% export_fig quiver_1 -r600 -transparent -jpg

%% For an example visibility curve
u = 2.5*x./sqrt(2.5*x.^2 + y.^2); 
v = y./sqrt(2.5*x.^2 + y.^2);

figure
quiver(x,y,u,v,'color','black');
% axis equal
axis([-nrows ncols -nrows ncols])
set(gca, 'xtick', [-1e6 1e6]);
set(gca, 'ytick', [-1e6 1e6]);
set(gca,'LooseInset',get(gca,'TightInset'));
% uncomment the line below to save the figure as quiver_2.jpg, needs expfig
% export_fig quiver_2 -r600 -transparent -jpg