%Test code for the Crustcrawler robotic manipulator
%Group 365, Robotics 3. semester, 2021
%Usage of this script requires Robotics Toolbox by Peter Corke

clear all; %Clear all previous variables

%%%% The link lengths as defined by the report
% The lengths are found by utilizing the CAD-files supplied for the project
% in Solidworks.

l_base = 239;   %mm
l_2_3 = 219.8;   %mm
l_ee = 221.17;    %mm

%Using Robotic Toolbox
% Create the links of the robot, based on the DH-parameters

L3(1) = Link('alpha', 0       , 'a', 0        , 'd'  , l_base                       , 'modified');
L3(2) = Link('alpha', pi/2    , 'a', 0        , 'd'  , 0      ,'offset' , pi/2      , 'modified');
L3(3) = Link('alpha', 0       , 'a', l_2_3    , 'd'  , 0                            , 'modified');
L3(4) = Link('alpha', 0       , 'a', l_ee     , 'd'  , 0                            ,  'modified');

r=SerialLink(L3 ,'name', 'Crustcrawler') %Link the above and call the result Crustcrawler
r.teach('eul') %Call teach function of r, with option 'eul' to gain correct angles

%% Verification calculations

clear all;

%Verification of DH-parameters Forward kinematics
%GGroup 365, ROB 3, 2021
%The following script requires the usage of TDH.m and eulerZYX
%Obtained during ROB 2, 2021.
%The usage of these functions requires that they are placed in the same
%folder or added to the path of matlab
%% Theta values for testing
syms PI theta1 theta2 theta3 theta4 theta5 theta6 theta7

%Input the desired theta values in the below function calls
%in degrees.
theta1=deg2rad(50);
theta2=deg2rad(60);
theta3=deg2rad(70);

%% Link measurements
%Lengths found from CAD files in Solidworks
L1 = 239;   %mm
L2 = 219;   %mm
L3 = 221;    %mm

%% DH-parameters conversion to transformation matrices
T01=TDH(0       ,   0    ,    0     ,theta1     );
T12=TDH(0       ,   0    , pi/2     ,theta2+pi/2);
T23=TDH(L2     ,   0    ,    0     ,theta3     );

%% Calculation of the total transformation matrix
T03 = T01*T12*T23;

%% The transformation matrix from base to end-effector center point
% The transformation matrix along the z-axis for base to frame 0
TB0= [   1       0       0       0;
            0       1       0       0;
            0       0       1     L1;
            0       0       0       1];


% Transalation of frame 3 to frame ee along X-axis with L3

T3EE= [  1       0       0     L3;
            0       1       0       0;
            0       0       1       0;
            0       0       0       1];

TBEE = TB0*T03*T3EE;

%% Transform TBW the to Euler angles and ZYX positions and euler angles
R = tr2eul(TBEE);
RPY = R*180/pi;
POS=[TBEE(1,4),TBEE(2,4),TBEE(3,4),R*180/pi]


x = TBEE(1,4)
y = TBEE(2,4)
z = TBEE(3,4)
poll  = RPY(1)
pitch = RPY(2)
yaw   = RPY(3)