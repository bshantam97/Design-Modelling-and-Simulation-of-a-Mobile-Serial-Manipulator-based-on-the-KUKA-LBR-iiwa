%% Designing Robot manipulator algorithms
clearvars

robot = importrobot('iiwa14.urdf');
show(robot)

%%configuration = randomConfiguration(robot)
%%show(robot,configuration)

%% Create a set of waypoints

wayPoints = [0 0 0.2 ;0.15 0 0.28;0.15 0.05 0.2]
hold on 
trajectory = cscvn(wayPoints')
fnplt(trajectory , 'r' , 2)