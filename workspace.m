%Test code for the Crustcrawler robotic manipulator
%Group 365, Robotics 3. semester, 2021
%This script utilises the kinematic chain to determine the workspace of
% the CrustCrawler

clear all; %Clear all previous variables
figure(1)
clf
figure(2)
clf

%% The link lengths as defined by the report
% The lengths are found by utilizing the CAD-files supplied for the project
% in Solidworks.

l_base = 239;   %mm
l_2_3 = 219.8;   %mm
l_ee = 221.17;    %mm

%% Using Robotic Toolbox
% Create the links of the robot, based on the DH-parameters

L3(1) = Link('alpha', 0       , 'a', 0        , 'd'  , 0                       , 'modified');
L3(2) = Link('alpha', pi/2    , 'a', 0        , 'd'  , 0      ,'offset' , pi/2      , 'modified');
L3(3) = Link('alpha', 0       , 'a', l_2_3    , 'd'  , 0                            , 'modified');
L3(4) = Link('alpha', 0       , 'a', l_ee     , 'd'  , 0                            ,  'modified');

r=SerialLink(L3 ,'name', 'Crustcrawler'); %Link the above and call the result Crustcrawler



%% Check all positions possible for the CrustCrawler in the X-Y plane

M1_min = 45;
M1_max = 315;
M2_max = 110;
M2_min = -110;
M3_max = 110;
M3_min = -110;
M4_min = 0;
M4_max = 0;


for i = M1_min:1:M1_max
                A = r.A([1 2 3 4], [i*pi/180, 90*pi/180, 0, 0]);
                T = A.t;
                X=T(1);
                Y=T(2);
                Z=T(3);
                figure(1)
                plot(X,Y, '-bo')
                hold on;
end
    for k = 0:3:M3_max
        A = r.A([1 2 3 4], [M1_max*pi/180, 90*pi/180, k*pi/180, 0]);
        T = A.t;
        X=T(1);
        Y=T(2);
        Z=T(3);
        figure(1)
        plot(X,Y, '-bo')
        hold on;
    end
for q = 90:2:M2_max

        A = r.A([1 2 3 4], [M1_max*pi/180, q*pi/180, M3_max*pi/180, 0]);
        T = A.t;
        X=T(1);
        Y=T(2);
        Z=T(3);
        figure(1)
        plot(X,Y, '-bo')
        hold on;

end
for k = 0:3:M3_max
        A = r.A([1 2 3 4], [M1_min*pi/180, 90*pi/180, k*pi/180, 0]);
        T = A.t;
        X=T(1);
        Y=T(2);
        Z=T(3);
        figure(1)
        plot(X,Y, '-bo')
        hold on;
end

for q = 90:2:M2_max
        A = r.A([1 2 3 4], [M1_min*pi/180, q*pi/180, M3_max*pi/180, 0]);
        T = A.t;
        X=T(1);
        Y=T(2);
        Z=T(3);
        figure(1)
        plot(X,Y, '-bo')
        hold on;
end
for i = M1_min:3:M1_max
    A = r.A([1 2 3 4], [i*pi/180, M2_max*pi/180, M3_max*pi/180, 0]);
    T = A.t;
    X = T(1);
    Y = T(2);
    Z = T(3);
    figure(1)
    plot(X,Y, '-bo')
    hold on
end
 
%% Check all extremum positions for crustcrawler in X-Z plane
figure(2)
clf


for q = M2_min:1:M2_max
        A = r.A([1 2 3 4], [0, q*pi/180, 0*pi/180, 0]);
        T = A.t;
        X = T(1);
        Y = T(2);
        Z = T(3);
        figure(2)
        plot(X,Z, '-bo')
        hold on
end

for q = 0:2:M3_max
        A = r.A([1 2 3 4], [0, M2_max*pi/180, q*pi/180, 0]);
        T = A.t;
        X = T(1);
        Y = T(2);
        Z = T(3);
        figure(2)
        plot(X,Z, '-bo')
        hold on
end

for q = M2_max:-2:M2_min
        A = r.A([1 2 3 4], [0, q*pi/180, M3_max*pi/180, 0]);
        T = A.t;
        X = T(1);
        Y = T(2);
        Z = T(3);
        figure(2)
        plot(X,Z, '-bo')
        hold on
end

for q = M3_min:2:0
        A = r.A([1 2 3 4], [0, M2_min*pi/180, q*pi/180, 0]);
        T = A.t;
        X = T(1);
        Y = T(2);
        Z = T(3);
        figure(2)
        plot(X,Z, '-bo')
        hold on
end

for q = M2_min:2:0
    A = r.A([1 2 3 4], [0, q*pi/180, M3_min*pi/180, 0]);
        T = A.t;
        X = T(1);
        Y = T(2);
        Z = T(3);
        figure(2)
        plot(X,Z, '-bo')
        hold on
end