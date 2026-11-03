

FASTEXECUTION = false;

Franka_robot = importrobot("RobotFiles/Franka_Panda/Franka_arm.urdf");

Franka_robot.DataFormat = 'column';
% define gravity
Franka_robot.Gravity = [0 0 -9.81];


% Nullspace Damping
Kn = 20*ones(7,1); 
Dn = 20*ones(7,1);



% define initial/home configuration for the simulation
q_init = [0 -pi/4  0  -(3/4)*pi  0  pi/2  pi/4]';       
% for Controller_kinesthetic_02_TubeWiping only!!!
% q_init = [0 -pi/4  0  -(3/4)*pi  0  pi/2  -pi/4]';       


% Trajectory
% for Free movement - damping evaluation
q_home = [0.003542735427650;-0.076954344513307;-0.003006835131443;-1.362053944077202;-5.624494697578509e-04;1.279919277568267;0.786445509714290];
% For wiping teachen 
% q_home = q_init;
H_home = getTransform(Franka_robot, q_home, 'panda_link8','panda_link0');

Rz = @(x) [cos(x),-sin(x),0; sin(x), cos(x), 0; 0,0,1];
Ry = @(x) [cos(x),0,sin(x);0,1,0; -sin(x), 0,cos(x)];
Rx = @(x) [1,0,0;0,cos(x),-sin(x);0, sin(x),cos(x)];


% Homo Trans from the point of reference (red calibration point on the
% tube) to the base coordinate system in the Franka base. The translation
% can be read from the franka eef
% H_0_pr = [[0,-1,0;1,0,0;0,0,1], [0.3253;0.0047;0.0329];
%             zeros(1,3), 1];                                 % Configuration if the tube is perpendicular to the robot
H_0_pr = [eye(3), [0.5596;0.1927;-0.07353];
            zeros(1,3), 1];                                 % Configuration if the tube is in line/ parallel to the robot
% Homo Trans from the center of the tube to the point of reference 
H_pr_cy = [eye(3), [0;-0.2015;0.1355];zeros(1,3), 1];
H_0_cy = H_0_pr*H_pr_cy;
H_0_cy_INV = [transpose(H_0_cy(1:3,1:3)), -transpose(H_0_cy(1:3,1:3))*H_0_cy(1:3,4); zeros(1,3), 1];



% KMatTest = diag([200,0,200]);
% KMatTest = Rz(pi/4)*KMatTest;


if(FASTEXECUTION == true)
    slowDown = 1.0;
    % 1 = is standard time
    % 2 = takes double amount of time
    
    
    Tfinal = ceil(slowDown*T_opti*1000)/1000;
    s = 0;
    interCnt = FUNC_findInterval(0, sBodVal);
    alpha_3 = ALPHA(1:dimData,interCnt);
    alpha_2 = ALPHA((dimData+1):(2*dimData),interCnt);
    alpha_1 = ALPHA((2*dimData+1):(3*dimData),interCnt);
    alpha_0 = ALPHA((3*dimData+1):(4*dimData),interCnt);
    q_init = alpha_3.*s^3 + alpha_2.*s^2 + alpha_1.*s + alpha_0;

end






