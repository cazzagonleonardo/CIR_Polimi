%% Kinematic model of the robot
clear all
close all
clc
%%%%%%%%%%%%%%%%%%%
%%%%%% LAB 5 %%%%%%
%%%%%%%%%%%%%%%%%%%
%% data
%DH parameters UR5
alpha = [pi/2,0,0,pi/2,-pi/2,0]; %[in rad]
a = [0,-0.425,-0.39225,0,0,0]; %[in m]
d = [0.08916,0,0,0.10915,0.09465,0.0823]; %[in m]

%generate the link object associated to the robot model (DH param); 
L1 = Link('d', d(1), 'a', a(1), 'alpha', alpha(1)); %each link is an object which has methods
L2 = Link('d', d(2), 'a', a(2), 'alpha', alpha(2));
L3 = Link('d', d(3), 'a', a(3), 'alpha', alpha(3));
L4 = Link('d', d(4), 'a', a(4), 'alpha', alpha(4));
L5 = Link('d', d(5), 'a', a(5), 'alpha', alpha(5));
L6 = Link('d', d(6), 'a', a(6), 'alpha', alpha(6));

%assign dynamic parameters
%mass of each link in kg
m=[3.7,8.393,2.275,1.219,1.219,0.1879];
%positions of the centers of masses expressed w.r.t the reference frame attached to the associated link [m]
p_cm1 = [0,-25.61,1.93]*10^(-3);
p_cm2 = [212.5,0,113.36]*10^(-3);
p_cm3 = [119.93,0,26.5]*10^(-3);
p_cm4 = [0,-1.8,16.34]*10^(-3);
p_cm5 = [0,1.8,16.34]*10^(-3);
p_cm6 = [0,0,-1.159]*10^(-3);

%inertia tensors expressed wrt the frame attached to the associated link (symmetric matrix, diagonal -> they are assumed cylindric)
In_1 = [84,0,0;0,64,0;0,0,84]*10^(-4);
In_2 = [78,0,0;0,21,0;0,0,21]*10^(-4);
In_3 = [16,0,0;0,462,0;0,0,462]*10^(-4);
In_4 = [16,0,0;0,16,0;0,0,9]*10^(-4);
In_5 = [16,0,0;0,16,0;0,0,9]*10^(-4);
In_6 = eye(3)*10^(-4);

%Moment of inertia of the motors around their own axes (indicative values)
%in [kg m^2]
Jm1 = 10^(-5); 
Jm2 = Jm1;
Jm3 = Jm1;
Jm4 = 5*10^(-6);
Jm5 = Jm4;
Jm6 = Jm4;

%gear ratios
n1 = 100;
n2 = n1;
n3 = n1;
n4 = n1;
n5 = n1;
n6 = n1;
%toolbox convention
%%LINK 1
L1.m = m(1); %link mass
L1.r = p_cm1; %link COG wrt link coordinate
L1.I = In_1; %link inertia matrix
L1.G = n1; %link transmission (gear) ratio
L1.Jm = Jm1; %link moment of inertia of the motor
%%LINK 2
L2.m = m(2); %link mass
L2.r = p_cm2; %link COG wrt link coordinate
L2.I = In_2; %link inertia matrix
L2.G = n2; %link transmission (gear) ratio
L2.Jm = Jm2; %link moment of inertia of the motor
%%LINK 3
L3.m = m(3); %link mass
L3.r = p_cm3; %link COG wrt link coordinate
L3.I = In_3; %link inertia matrix
L3.G = n3; %link transmission (gear) ratio
L3.Jm = Jm3; %link moment of inertia of the motor
%%LINK 4
L4.m = m(4); %link mass
L4.r = p_cm4; %link COG wrt link coordinate
L4.I = In_4; %link inertia matrix
L4.G = n4; %link transmission (gear) ratio
L4.Jm = Jm4; %link moment of inertia of the motor
%%LINK 5
L5.m = m(5); %link mass
L5.r = p_cm5; %link COG wrt link coordinate
L5.I = In_5; %link inertia matrix
L5.G = n5; %link transmission (gear) ratio
L5.Jm = Jm5; %link moment of inertia of the motor
%%LINK 6
L6.m = m(6); %link mass
L6.r = p_cm6; %link COG wrt link coordinate
L6.I = In_6; %link inertia matrix
L6.G = n6; %link transmission (gear) ratio
L6.Jm = Jm6; %link moment of inertia of the motor

%% ex 1.1
UR5 = SerialLink([L1 L2 L3 L4 L5 L6]);
UR5.name = 'UR5';
UR5.gravity = [0;0;9.81]; %gravity acceleration vector expressed in the base frame 

%% ex 1.2 
%Plan the trajectory with trapezoidal velocity profile

%Initial instant of motion
t0=0.1;

%Travel time
t_tot=0.5;

%Maximum speed
x_dot_max=1;

%Distance to cover
x_displacement=0.4;

%Compute the acceleration time for the assigned trapezoidal velocity
%profile
t_acc=(t_tot*x_dot_max-x_displacement)/x_dot_max;

%Compute the maximum acceleration for the assigned trapezoidal velocity
%profile
x_dotdot=x_dot_max/t_acc;

%The robot is initially at steady-state in position x0=0.2, y0=0, z0=0
x0=0.2;
y0=0;
z0=0;

%Compute the inverse kinematics to obtain the robot initial joint
%configuration (put it in the Simulink robot block) corresponding to the ee
%pose (0.2, 0, 0) and rotated as the base frame. Use the provided function
%qreturned = ComputeIKInit(x_ee,y_ee,z_ee,Ree,q_prev), where q_prev is a column vector
q0=[0 0 0 0 0 0]';
q_init=ComputeIKInit(x0,y0,z0,eye(3),q0);


%% ex 1.4
% Design the speed and position controllers for the six joints
%moment of inertia of the load
Jl1 = 1.548; %[kg m^2]
Jl2 = 1.8; %[kg m^2]
Jl3 = 0.58; %[kg m^2]
Jl4 = 1.76*10^(-2); %[kg m^2]
Jl5 = 2.23*10^(-3); %[kg m^2]
Jl6 = 10^(-4); %[kg m^2]

%cross-over frequencies
w_cp = 60; %[rad/s] (~10 Hz)
w_cv = 10*w_cp; %[rad/s]

%position controllers
kpp = w_cp;

%speed controllers
Tiv = 1/(0.2*w_cv);
%axis 1
mu1 = 1/(Jm1+Jl1/n1^2);
kpv1 = w_cv/mu1;
%axis 2
mu2 = 1/(Jm2+Jl2/n2^2);
kpv2 = w_cv/mu2;
%axis 3
mu3 = 1/(Jm3+Jl3/n3^2);
kpv3 = w_cv/mu3;
%axis 4
mu4 = 1/(Jm4+Jl4/n4^2);
kpv4 = w_cv/mu4;
%axis 5
mu5 = 1/(Jm5+Jl5/n5^2);
kpv5 = w_cv/mu5;
%axis 6
mu6 = 1/(Jm6+Jl6/n6^2);
kpv6 = w_cv/mu6;

