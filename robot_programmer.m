%% Project Setup

clear all
global vrep;
global clientID;
global wheel_separation_width;
global wheel_radius;
global wheel_separatation_length;
global KMR_Front_Right_Wheel;
global KMR_Front_Left_Wheel;
global KMR_Rear_Right_Wheel;
global KMR_Rear_Left_Wheel;
global KUKA_Joint1;
global KUKA_Joint2;
global KUKA_Joint3;
global KUKA_Joint4;
global KUKA_Joint5;
global KUKA_Joint6;
global KUKA_Joint7;
global ik;
global kuka;



wheel_separation_width=.5/2;%m
wheel_radius=2.5400E-01; %meters
wheel_separatation_length=.625/2; %m


kuka=importrobot("iiwa14.urdf");
ik = robotics.InverseKinematics('RigidBodyTree',kuka);

%% Connect to VRep

vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections

clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);


if (clientID<=-1) % If not true, we've connected
    disp('Cannot connect to VRep. Make sure the Vrep simulation is already running');
    return;
end

disp('Connected to vrep. Ready to go!');

%% Get Object Handles for all Parts

[~, KMR_Front_Right_Wheel]=vrep.simxGetObjectHandle(clientID, 'Omnirob_FRwheel_motor', vrep.simx_opmode_blocking);
[~, KMR_Front_Left_Wheel]=vrep.simxGetObjectHandle(clientID, 'Omnirob_FLwheel_motor', vrep.simx_opmode_blocking);
[~, KMR_Rear_Right_Wheel]=vrep.simxGetObjectHandle(clientID, 'Omnirob_RRwheel_motor', vrep.simx_opmode_blocking); 
[~, KMR_Rear_Left_Wheel]=vrep.simxGetObjectHandle(clientID, 'Omnirob_RLwheel_motor', vrep.simx_opmode_blocking);
[~, KUKA_Joint1]=vrep.simxGetObjectHandle(clientID, 'LBR_iiwa_14_R820_joint1', vrep.simx_opmode_blocking);
[~, KUKA_Joint2]=vrep.simxGetObjectHandle(clientID, 'LBR_iiwa_14_R820_joint2', vrep.simx_opmode_blocking);
[~, KUKA_Joint3]=vrep.simxGetObjectHandle(clientID, 'LBR_iiwa_14_R820_joint3', vrep.simx_opmode_blocking);
[~, KUKA_Joint4]=vrep.simxGetObjectHandle(clientID, 'LBR_iiwa_14_R820_joint4', vrep.simx_opmode_blocking);
[~, KUKA_Joint5]=vrep.simxGetObjectHandle(clientID, 'LBR_iiwa_14_R820_joint5', vrep.simx_opmode_blocking);
[~, KUKA_Joint6]=vrep.simxGetObjectHandle(clientID, 'LBR_iiwa_14_R820_joint6', vrep.simx_opmode_blocking);
[~, KUKA_Joint7]=vrep.simxGetObjectHandle(clientID, 'LBR_iiwa_14_R820_joint7', vrep.simx_opmode_blocking);



%% Drive the Cart
%pause(10);
drive(5,5);
rotate(90);
drive(3,5);
rotate(90);
drive(7,5);

%% Object Retrieval Pose

%The object to retrieve is a disc on a spindle. The spindle is a distance
%l_obj along the (manipulator) Xo axis, h_obj along the (manipulator) Zo
%axis. It is then rotated by theta_obj about the Z axis, and by alpha_obj
%about the X axis. 

theta_obj=deg2rad(30);
h_obj=-4;
l_obj=3;
alpha_obj=deg2rad(45);


Rotz_obj=[cos(theta_obj) sin(theta_obj)  0 0;
        -sin(theta_obj) cos(theta_obj) 0 0;
       0 0 1 0;
       0 0 0 1];

Transz_obj=[1 0 0 0;
         0 1 0 0;
         0 0 1 h_obj;
         0 0 0 1];

Transx_obj=eye(4);

Rotx_obj=[1 0 0 0;
       0 cos(alpha_obj) sin(alpha_obj) 0;
       0 -sin(alpha_obj) cos(alpha_obj) 0
       0 0 0 1];


A_obj=Rotz_obj*Transz_obj*Transx_obj*Rotx_obj;


movearm(A_obj)


%% Object Removal
%To remove the disc from the spindle, it must be slide a distance d_remove
%in the Z_obj direction

d_remove=3;

Rotz_remove=eye(4);
   
Transz_remove=[1 0 0 0;
         0 1 0 0;
         0 0 1 d_remove;
         0 0 0 1];
     
Transx_remove=eye(4);
Rotx_remove=eye(4);
A_remove=Rotz_remove*Transz_remove*Transx_remove*Rotx_remove;

A_0_remove=A_obj*A_remove;

movearm(A_0_remove)












vrep.simxFinish(-1); % Close the connection


    
vrep.delete(); % Call the vrep destructor




%% Functions

function driver=drive(distance, speed)% Drive Forward (direction =1), Reverse (direction=-1)
global clientID;
global vrep;
global KMR_Front_Right_Wheel;
global KMR_Front_Left_Wheel;
global KMR_Rear_Right_Wheel;
global KMR_Rear_Left_Wheel;

if(speed>0)
    disp("Driving Foward!");
elseif(speed<0)
    disp("Beep beep beep - driving backwards!");
end
        wheel_velocity=speed;
        duration=abs(distance/(.3*speed));
        %spin wheels
        [~]=vrep.simxSetJointTargetVelocity(clientID,KMR_Front_Left_Wheel,wheel_velocity,vrep.simx_opmode_blocking);
        [~]=vrep.simxSetJointTargetVelocity(clientID,KMR_Front_Right_Wheel,-wheel_velocity,vrep.simx_opmode_blocking);
        [~]=vrep.simxSetJointTargetVelocity(clientID,KMR_Rear_Right_Wheel,-wheel_velocity,vrep.simx_opmode_blocking);
        [~]=vrep.simxSetJointTargetVelocity(clientID,KMR_Rear_Left_Wheel,wheel_velocity,vrep.simx_opmode_blocking);
        pause(duration)
        %stop wheels
        [~]=vrep.simxSetJointTargetVelocity(clientID,KMR_Front_Left_Wheel,0,vrep.simx_opmode_blocking);
        [~]=vrep.simxSetJointTargetVelocity(clientID,KMR_Front_Right_Wheel,0,vrep.simx_opmode_blocking);
        [~]=vrep.simxSetJointTargetVelocity(clientID,KMR_Rear_Left_Wheel,0,vrep.simx_opmode_blocking);
        [~]=vrep.simxSetJointTargetVelocity(clientID,KMR_Rear_Right_Wheel,0,vrep.simx_opmode_blocking);
        pause(.5);
end


% function driver=drive(wheel_velocity,duration,direction)% Drive
% Forward (direction =1), Reverse (direction=-1)
%Original drive function, uses wheel_speed and time
% global clientID;
% global vrep;
% global KMR_Front_Right_Wheel;
% global KMR_Front_Left_Wheel;
% global KMR_Rear_Right_Wheel;
% global KMR_Rear_Left_Wheel;
% 
% if(direction==1)
%     disp("Driving Foward!");
% elseif(direction==-1)
%     disp("Beep beep beep - driving backwards!");
% end
%         wheel_velocity=wheel_velocity*direction;
%         %spin wheels
%         [~]=vrep.simxSetJointTargetVelocity(clientID,KMR_Front_Left_Wheel,wheel_velocity,vrep.simx_opmode_blocking);
%         [~]=vrep.simxSetJointTargetVelocity(clientID,KMR_Front_Right_Wheel,-wheel_velocity,vrep.simx_opmode_blocking);
%         [~]=vrep.simxSetJointTargetVelocity(clientID,KMR_Rear_Right_Wheel,-wheel_velocity,vrep.simx_opmode_blocking);
%         [~]=vrep.simxSetJointTargetVelocity(clientID,KMR_Rear_Left_Wheel,wheel_velocity,vrep.simx_opmode_blocking);
%         pause(duration)
%         %stop wheels
%         [~]=vrep.simxSetJointTargetVelocity(clientID,KMR_Front_Left_Wheel,0,vrep.simx_opmode_blocking);
%         [~]=vrep.simxSetJointTargetVelocity(clientID,KMR_Front_Right_Wheel,0,vrep.simx_opmode_blocking);
%         [~]=vrep.simxSetJointTargetVelocity(clientID,KMR_Rear_Left_Wheel,0,vrep.simx_opmode_blocking);
%         [~]=vrep.simxSetJointTargetVelocity(clientID,KMR_Rear_Right_Wheel,0,vrep.simx_opmode_blocking);
% end


function pivot=rotate(angle) % Rotate about central axis (deg)
    global clientID;
    global vrep;
    global KMR_Front_Right_Wheel;
    global KMR_Front_Left_Wheel;
    global KMR_Rear_Right_Wheel;
    global KMR_Rear_Left_Wheel;
    global wheel_radius;
    global wheel_separation_width
    global wheel_separatation_length
    angle=deg2rad(angle);
    
    disp("Turning!");
    % https://robohub.org/drive-kinematics-skid-steer-and-mecanum-ros-twist-included/
    v_wheel_front_left = -(1/wheel_radius)*((wheel_separation_width + wheel_separatation_length)*angle);
    v_wheel_front_right = -(1/wheel_radius)*((wheel_separation_width + wheel_separatation_length)*angle);
    v_wheel_rear_left = -(1/wheel_radius)*((wheel_separation_width + wheel_separatation_length)*angle);
    v_wheel_rear_right = -(1/wheel_radius)*((wheel_separation_width + wheel_separatation_length)*angle);
    
    %spin wheels
    [~]=vrep.simxSetJointTargetVelocity(clientID,KMR_Front_Left_Wheel,v_wheel_front_left,vrep.simx_opmode_blocking);
    [~]=vrep.simxSetJointTargetVelocity(clientID,KMR_Front_Right_Wheel, v_wheel_front_right,vrep.simx_opmode_blocking);
    [~]=vrep.simxSetJointTargetVelocity(clientID,KMR_Rear_Right_Wheel,v_wheel_rear_left,vrep.simx_opmode_blocking);
    [~]=vrep.simxSetJointTargetVelocity(clientID,KMR_Rear_Left_Wheel,v_wheel_rear_right,vrep.simx_opmode_blocking);
    pause(1)
    %stop wheels
    [~]=vrep.simxSetJointTargetVelocity(clientID,KMR_Front_Left_Wheel,0,vrep.simx_opmode_blocking);
    [~]=vrep.simxSetJointTargetVelocity(clientID,KMR_Front_Right_Wheel,0,vrep.simx_opmode_blocking);
    [~]=vrep.simxSetJointTargetVelocity(clientID,KMR_Rear_Left_Wheel,0,vrep.simx_opmode_blocking);
    [~]=vrep.simxSetJointTargetVelocity(clientID,KMR_Rear_Right_Wheel,0,vrep.simx_opmode_blocking);
    pause(.5);
end








function movearm=movearm(pose)
    global vrep;
    global kuka;
    global ik;
    global clientID;
    global KUKA_Joint1;
    global KUKA_Joint2;
    global KUKA_Joint3;
    global KUKA_Joint4;
    global KUKA_Joint5;
    global KUKA_Joint6;
    global KUKA_Joint7;  
    disp("Attempting to solve Inverse Kinematics for that pose...");
    weights = [1 1 1 1 1 1];
    initialguess = homeConfiguration(kuka);
    [configSoln,solnInfo] = ik('iiwa_link_7',pose,weights,initialguess);
    disp("Done solving inverse kinematics; Ready to move!");


    j1=configSoln(1).JointPosition;
    j2=configSoln(2).JointPosition;
    j3=configSoln(3).JointPosition;
    j4=configSoln(4).JointPosition;
    j5=configSoln(5).JointPosition;
    j6=configSoln(6).JointPosition;
    j7=configSoln(7).JointPosition;

    disp("Moving robot arm");
    
    [~]=vrep.simxSetJointTargetPosition(clientID,KUKA_Joint1,j1,vrep.simx_opmode_blocking);
    [~]=vrep.simxSetJointTargetPosition(clientID,KUKA_Joint2,j2,vrep.simx_opmode_blocking);
    [~]=vrep.simxSetJointTargetPosition(clientID,KUKA_Joint3,j3,vrep.simx_opmode_blocking);
    [~]=vrep.simxSetJointTargetPosition(clientID,KUKA_Joint4,j4,vrep.simx_opmode_blocking);
    [~]=vrep.simxSetJointTargetPosition(clientID,KUKA_Joint5,j5,vrep.simx_opmode_blocking);
    [~]=vrep.simxSetJointTargetPosition(clientID,KUKA_Joint6,j6,vrep.simx_opmode_blocking);
    [~]=vrep.simxSetJointTargetPosition(clientID,KUKA_Joint7,j7,vrep.simx_opmode_blocking);
    
    disp("Done moving");
end


