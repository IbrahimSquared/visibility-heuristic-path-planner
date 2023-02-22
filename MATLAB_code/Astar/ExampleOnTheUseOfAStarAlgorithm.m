%Example on the use of AStar Algorithm in an occupancy grid. 
clear, clc


%%% Generating a MAP
%1 represent an object that the path cannot penetrate, zero is a free path
% MAP=int8(zeros(128,140));
% MAP(1:64,1)=1;
% MAP(120,3:100)=1;
% MAP(125:128,40:60)=1;
% MAP(120:128,100:120)=1;
% MAP(126,100:118)=0;
% MAP(120:126,118)=0;
% MAP(100:120,100)=1;
% MAP(114:124,112:118)=0;
% MAP(1,1:128)=1;
% MAP(128,1:128)=1;
% MAP(100,1:130)=1;
% MAP(50,28:128)=1;
% MAP(20:30,50)=1;
% MAP(1:128,1)=1;
% MAP(1:65,128)=1;
% MAP(1,1:128)=1;
% MAP(128,1:128)=1;
% MAP(10,1:50)=1;
% MAP(25,1:50)=1;
% MAP(40,40:50)=1;
% MAP(40,40:45)=1;
% MAP(80,20:40)=1;
% MAP(80:100,40)=1;
% MAP(80:100,120)=1;
% MAP(120:122,120:122)=1;
% MAP(120:122,20:25)=1;
% MAP(120:122,10:11)=1;
% MAP(125:128,10:11)=1;
% MAP(100:110,30:40)=1;
% MAP(1:20,100:128)=1;
% MAP(10:20,80:128)=1;
% MAP(20:40,80:90)=1;
% MAP(1:40,90:90)=1;
% MAP(100:105,70:80)=1;


% obstacle = int8(zeros(128,140));
% obstacle(129-(1:60),90:91) = true;
% obstacle(129-(40:128),80:81) = true;
% obstacle(129-(1:60),70:71) = true;
% obstacle(129-(40:128),60:61) = true;
% obstacle(129-(1:60),50:51) = true;
% obstacle(129-(40:128),40:41) = true;
% obstacle(129-(1:60),30:31) = true;
% obstacle(129-(40:128),20:21) = true;
% obstacle(129-(1:60),10:11) = true;
nrows = 100;
ncols = 100;
obstacle = false(nrows, ncols);
obstacle(40:100,90:91) = true;
obstacle(1:60,80:81) = true;
obstacle(40:100,70:71) = true;
obstacle(1:60,60:61) = true;
obstacle(40:100,50:51) = true;
obstacle(1:60,40:41) = true;
obstacle(40:100,30:31) = true;
obstacle(1:60,20:21) = true;
obstacle(40:100,10:11) = true;

MAP = obstacle;

% mesh(obstacle)
% view(0,90)
% 
% return

ep_o = [85,5];
sp_o = [10,95];

%Start Positions
StartX=sp_o(2);
StartY=sp_o(1);

%Generating goal nodes, which is represented by a matrix. Several goals can be speciefied, in which case the pathfinder will find the closest goal. 
%a cell with the value 1 represent a goal cell
GoalRegister=int8(zeros(nrows,ncols));
GoalRegister(ep_o(1),ep_o(2))=1;

%Number of Neighboors one wants to investigate from each cell. A larger
%number of nodes means that the path can be alligned in more directions. 
%Connecting_Distance=1-> Path can  be alligned along 8 different direction.
%Connecting_Distance=2-> Path can be alligned along 16 different direction.
%Connecting_Distance=3-> Path can be alligned along 32 different direction.
%Connecting_Distance=4-> Path can be alligned along 56 different direction.
%ETC......

Connecting_Distance=8; %Avoid to high values Connecting_Distances for reasonable runtimes. 

% Running PathFinder
tic
OptimalPath=ASTARPATH(StartX,StartY,MAP,GoalRegister,Connecting_Distance);
toc
% End. 

if size(OptimalPath,2)>1
figure(10)
imagesc((MAP))
    colormap(flipud(gray));

hold on
plot(OptimalPath(1,2),OptimalPath(1,1),'o','color','k')
plot(OptimalPath(end,2),OptimalPath(end,1),'o','color','b')
plot(OptimalPath(:,2),OptimalPath(:,1),'r')
legend('Goal','Start','Path')

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

for p=1:size(Neighboors,1)
  i=Neighboors(p,1);
       j=Neighboors(p,2);
      
     plot([0 i],[0 j],'k')
 hold on
 axis equal
 
grid on
title('Connecting distance=2')
end

%3
NeigboorCheck=[0 1 1 0 1 1 0;1 0 1 0 1 0 1;1 1 1 1 1 1 1;0 0 1 0 1 0 0;1 1 1 1 1 1 1;1 0 1 0 1 0 1;0 1 1 0 1 1 0]; %Heading has 32 possible allignments
figure(3)
[row col]=find(NeigboorCheck==1);
Neighboors=[row col]-(3+1);

for p=1:size(Neighboors,1)
  i=Neighboors(p,1);
       j=Neighboors(p,2);
      
     plot([0 i],[0 j],'k')
 hold on
 axis equal
 grid on
title('Connecting distance=3')

end
 
%4
NeigboorCheck=[0 1 1 1 0 1 1 1 0;1 0 1 1 0 1 1 0 1;1 1 0 1 0 1 0 1 1;1 1 1 1 1 1 1 1 1;0 0 0 1 0 1 0 0 0;1 1 1 1 1 1 1 1 1;1 1 0 1 0 1 0 1 1 ;1 0 1 1 0 1 1 0 1 ;0 1 1 1 0 1 1 1 0];  %Heading has 56 possible allignments
figure(4)
[row col]=find(NeigboorCheck==1);
Neighboors=[row col]-(4+1);

for p=1:size(Neighboors,1)
  i=Neighboors(p,1);
       j=Neighboors(p,2);
      
     plot([0 i],[0 j],'k')
 hold on
 axis equal
grid on
title('Connecting distance=4')

end
%1
NeigboorCheck=[1 1 1;1 0 1;1 1 1];
figure(1)
[row col]=find(NeigboorCheck==1);
Neighboors=[row col]-(1+1);

for p=1:size(Neighboors,1)
  i=Neighboors(p,1);
       j=Neighboors(p,2);
      
     plot([0 i],[0 j],'k')
 hold on
 axis equal
grid on
title('Connecting distance=1')

end
end

d_tot = 0;
for i = 1:length(OptimalPath)-1
    d_tot = d_tot + norm(OptimalPath(i+1,:) - OptimalPath(i,:));
end
d_tot