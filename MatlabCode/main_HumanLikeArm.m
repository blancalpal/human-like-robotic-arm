%% Human-like code for MATLAB
clear;clc;close all;
%% Load the robot model
robot = loadrobot("universalUR3");
T = [1 0 0 0;0 0.7071 0.7071 0;0 -0.7071 0.7071 0;0 0 0 1];
HombroLim = [-3.14,3.14];
HombroLim2 = [-2, 1.5];
Wrist4Lim = [-4*pi/3,pi/3];
% Body values
weights = [2, 3.42, 1.26, 0.8, 0.8, 0.35];
configSoln = robot.homeConfiguration;

%% robot for checkconfig

% robot2 = loadrobot("universalUR3");
% T2 = [1 0 0 0.07;0 0.7071 0.7071 0.13;0 -0.7071 0.7071 1.15;0 0 0 1];
% setFixedTransform(robot2.Bodies{1,1}.Joint,T2);
% configSoln2 = robot2.homeConfiguration;

%% Collision environment
robotModel = loadrobot("universalUR3","DataFormat","column");
T1 = [1 0 0 0.07;0 0.7071 0.7071 0.13;0 -0.7071 0.7071 1.15;0 0 0 1];
setFixedTransform(robotModel.Bodies{1,1}.Joint,T1);
ss = manipulatorStateSpace(robotModel);
sv = manipulatorCollisionBodyValidator(ss);
sv.ValidationDistance = 0.1;
sv.IgnoreSelfCollision = true;

% Load the body as a collision object
cagedata=stlread('body1.stl');
body1 = collisionMesh(cagedata.Points./1000);
% body1.Vertices(:) = body1.Vertices(:)*1.03;
cagedata=stlread('body2.stl');
body2 = collisionMesh(cagedata.Points./1000);
% body2.Vertices(:) = body2.Vertices(:)*1.03;
cagedata=stlread('body3.stl');
body3 = collisionMesh(cagedata.Points./1000);
% body3.Vertices(:) = body3.Vertices(:)*1.03;
cagedata=stlread('body4.stl');
body4 = collisionMesh(cagedata.Points./1000);
% body4.Vertices(:) = body4.Vertices(:)*1.03;
matrizRot = [0 -1 0 0;1 0 0 0;0 0 1 -0.03;0 0 0 1]; %(0.05248825/2)-0.025
body1.Pose = matrizRot;
body2.Pose = matrizRot;
body3.Pose = matrizRot;
body4.Pose = matrizRot;
env = {body1 body2 body3 body4};
sv.Environment = env;

%% Inverse kinematics solver
%% Read the files
numTest = 7;
path = ['..\HumanData\Prueba',num2str(numTest),'\DatosBrazoHumano.csv'];
path2 = ['..\HumanData\Prueba',num2str(numTest),'\CodoHumano.csv'];
matrixRead = readmatrix(path);
CodoRead = readmatrix(path2);
CodoOrganizado = [];
MEJORES = [];
PEORES = [];
iter = length(matrixRead);
iter2 = length(CodoRead);
k=0;
configFinal = [];
almacenamiento = [];
RES=[];
ROTMAT = [];
VecCODO = [];
CODRob = [];
% Organise the elbow values
for j=1:3:iter2
    CodoOrganizado = [CodoOrganizado; CodoRead(j,1) CodoRead(j+1) CodoRead(j+2)];
end

%% Main Optimization Loop 
% Weigths values
W_rax = 1; %Position weigth
W_rao = 10; %Orientation weigth

%Evaluation loop
gola = [];
gola2 = [];
matrixCodo = zeros(4,4);

mejor_ii = 0;
for i=1:4:iter
    k = k+1;
    DistDif = 10000000;
    DistDigOG = DistDif;
    iteration = 0;
    check = true;
    
    % End efector matrix
    matrix = [matrixRead(i,1) matrixRead(i,2) matrixRead(i,3) matrixRead(i,4);
        matrixRead(i+1,1) matrixRead(i+1,2) matrixRead(i+1,3) matrixRead(i+1,4);
        matrixRead(i+2,1) matrixRead(i+2,2) matrixRead(i+2,3) matrixRead(i+2,4);
        matrixRead(i+3,1) matrixRead(i+3,2) matrixRead(i+3,3) matrixRead(i+3,4)];
    matrix2 = inv(T) * matrix;
    matrix(1:3,1:3)  = rotz(180) * matrix(1:3,1:3);
    matrix(1:2,4) = -matrix(1:2,4);
    matrix = T * matrix;
    
    % Elbow matrix
    VectorCodo = [CodoOrganizado(k,1), CodoOrganizado(k,2),CodoOrganizado(k,3)];
    VecCODO = [VecCODO;VectorCodo];
    X_RAGoal = [matrix2(1,4) matrix2(2,4) matrix2(3,4)];
    gola = [gola;[matrixRead(i,4) matrixRead(i+1,4) matrixRead(i+2,4)]];
    gola2 = [gola2;X_RAGoal];
    Rot_mat = [matrix2(1,1) matrix2(1,2) matrix2(1,3);
        matrix2(2,1) matrix2(2,2) matrix2(2,3);
        matrix2(3,1) matrix2(3,2) matrix2(3,3)];
    EulerAngles = rotm2eul(Rot_mat);
    quat_h = eul2quat(EulerAngles);
    
    disp("------------- NEW ITERATION -------------")
    disp(k)
   
    % IK using UR3_Inverse_Kinematics
    try
        IK_UR
        keep_best = 0;
        if mejor_ii ~= 0
            ii = mejor_ii;
            evaluate_configuration
        end
        if ~keep_best

            for ii = 1:1:8
                evaluate_configuration
            end
        end

        if DistDif ~= DistDigOG
            Wrist_old = MejorConfig(1,4);
            Elbow_old = MejorConfig(1,3);

            MEJORES = [MEJORES;mejor_ii];
            configFinal = [configFinal;MejorConfig];
            ROTMAT = [ROTMAT;Rot_mat];
        else
            disp('Not valid configuration found')
            worst = k;
            PEORES = [PEORES;worst];
            dif
        end
        
    catch
        disp('Wrong IK value')
    end   
    
end
% Filter the incorrect data and correct the position values
gola(PEORES,:) = [];
gola2(PEORES,:) = [];
gola =[gola(:,1)+0.07,gola(:,2)+0.13,gola(:,3)+1.15];

%% Plot the movements of the robotic arm and the human elbow and wrist positions
robot = loadrobot("universalUR3");
T = [1 0 0 0.07;0 0.7071 0.7071 0.13;0 -0.7071 0.7071 1.15;0 0 0 1];
setFixedTransform(robot.Bodies{1,1}.Joint,T);
configuraciones = robot.homeConfiguration;
k=0;
DISTANCIAS = [];
END = [];
ROTEND = [];
quatEND = [];
X =[];
Y = [];
Z = [];


for i=1:1:length(configFinal)
    k= k+1
    configuraciones(1).JointPosition = configFinal(i,1);
    configuraciones(2).JointPosition = configFinal(i,2);
    configuraciones(3).JointPosition = configFinal(i,3);
    configuraciones(4).JointPosition = configFinal(i,4);
    configuraciones(5).JointPosition = configFinal(i,5);
    configuraciones(6).JointPosition = configFinal(i,6);
    zoom(gca,'on');
    show(robot,configuraciones,"Collisions","off");
    CodoR = getTransform(robot,configuraciones,'forearm_link','base_link');
    CODRob = [CODRob;CodoR(1,4) CodoR(2,4) CodoR(3,4)];
    camzoom(1.7);
    hold on
    show(env{1});
    show(env{2});
    show(env{3});
    show(env{4});
    hold on
    plot3(VecCODO(k,1)+0.07,VecCODO(k,2)+0.13,VecCODO(k,3)+1.15,'o','Color','g','MarkerSize',10,'MarkerFaceColor','g')
    plot3(gola(k,1),gola(k,2),gola(k,3),'o','Color','r','MarkerSize',10,'MarkerFaceColor','r')
    hold on
    PuntoEnd = getTransform(robot,configuraciones,'tool0','base_link');
    Punto = [gola(k,1),gola(k,2),gola(k,3)];
    END = [END;PuntoEnd(1,4),PuntoEnd(2,4),PuntoEnd(3,4)];
    Rot_matrix = [PuntoEnd(1,1),PuntoEnd(1,2),PuntoEnd(1,3);PuntoEnd(2,1),PuntoEnd(2,2),PuntoEnd(2,3);PuntoEnd(3,1),PuntoEnd(3,2),PuntoEnd(3,3)];
    ROTEND = [ROTEND;Rot_matrix];
    EulerAnglesEF = rotm2eul(Rot_matrix);    
    quat = eul2quat(EulerAnglesEF, 'ZYX');  % n2x4 matrix
    quatEND = [quatEND;quat];
    drawnow;
    pause(0.001);
    hold off
end

%% Plot in 3D the EE trajectory
plot3(gola2(:,1),gola2(:,2),gola2(:,3),'Color','g');
hold on
xlabel('X [m]')
ylabel('Y [m]')
zlabel('Z [m]')
plot3(END(:,1),END(:,2),END(:,3),'Color','b');
plot3(gola2(1,1),gola2(1,2),gola2(1,3),'o','Color','r','MarkerSize',8,'MarkerFaceColor','r');
plot3(gola2(length(gola2),1),gola2(length(gola2),2),gola2(length(gola2),3),'o','Color','m','MarkerSize',8,'MarkerFaceColor','m');
axis('equal')
legend({'Robot data','Human data','Initial point','Final point'},'Location', 'best', 'Orientation', 'horizontal')
grid on;
hold off

%% Plot the error values of the end efector and the joint values
PlotError

% Create a color map for different joints for better distinction
colors = [
    155/255, 93/255, 229/255;  % Purple
    219/255, 76/255, 164/255;  % Magenta  % Red
    255/255, 209/255, 102/255; % Yellow
    6/255, 214/255, 160/255;   % Green
    17/255, 138/255, 178/255;  % Blue
    255/255, 127/255, 80/255;  % Orange      
];

% Plot joint values wrapped from -pi to pi
figure;
hold on;
for i=1:1:6
    if i~=6
        plot(1:length(configFinal),wrapToPi(configFinal(:,i)),'Color',colors(i,:))
    else
        plot(1:length(configFinal),unwrap(configFinal(:,i)),'Color',colors(i,:))
    end    
end
%ylim([-pi pi])
legend({'Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6'}, ...
       'Location', 'northoutside', 'Orientation', 'horizontal');
ylabel('Original Joint Position [rad]');
xlabel('Position index')
xlim([0 length(configFinal)])
hold off

%% Calculate Trajectory waypoints

jointConfigurations = [];

% Unwrap joint configurations to prevent discontinuities (mainly for the 6th joint)
for i = 1:6
    jointConfigurations(:,i) = unwrap(configFinal(:,i));
end

% Wrap from -pi to pi as the real robot works
for i = 1:5
    jointConfigurations(:,i) = wrapToPi(jointConfigurations(:,i));
end

jointConfigurations_og = jointConfigurations;

%% Trajectory generation

% Adjust waypoints to avoid crossing the pi -pi boundary for all joints except for the 6th, which can rotate
adjustedConfs = adjustWaypointsForAllJoints_prev(jointConfigurations');
jointConfigurations = adjustedConfs';

% Calculate differences between consecutive configurations
numConfigurations = size(jointConfigurations, 1);
numJoints = size(jointConfigurations, 2);
differences = zeros(numConfigurations-1, numJoints);

for i = 1:numConfigurations - 1
    differences(i,:) = abs(jointConfigurations(i+1,:) - jointConfigurations(i,:));
end

% Define maximum joint speeds (radians/sec) - adjust based on robot specs
maxSpeeds = [pi, pi, pi, 2*pi, 2*pi, 2*pi].*0.9;  % Example values

% Define maximum joint accelerations (radians/sec^2) - adjust based on robot specs
maxAccelerations = [3, 3, 4, 5, 5, 5].*3;  % Example values

% Calculate minimum time required for each transition based on max speed
minTimesSpeed = max(differences ./ maxSpeeds, [], 2);

% Calculate minimum time required for each transition based on max acceleration
minTimesAccel = max(sqrt(2 * differences ./ maxAccelerations), [], 2);

% Combine the two time constraints (use the maximum of both)
minTimes = max(minTimesSpeed, minTimesAccel);

% Apply a safety factor to ensure feasible execution by the robot
safetyFactor = 1.5;
transitionTimes = safetyFactor * minTimes;

% Ensure all times are at least a certain minimum to avoid too fast transitions
minTime = 0.1;  % Minimum time for any transition
transitionTimes = max(transitionTimes, minTime);

% Construct the time vector
timeVector = [0; cumsum(transitionTimes)];

diff_timeVector = diff(timeVector)';
diff_timeVector_n = repmat(diff_timeVector, 6, 1);

fprintf('mean diff time vector: %0.4f\n', mean(diff_timeVector));
fprintf('std diff time vector: %0.4f\n', std(diff_timeVector));

%% Generate trajectory using quintic polynomial interpolation and check for collisions

[q, qd, qdd, ~] = trapveltraj(jointConfigurations', 2*length(timeVector),'EndTime',diff_timeVector_n);
%time_samples = linspace(0, timeVector(end), 2*length(timeVector));
%[q, qd, qdd,~] = quinticpolytraj(jointConfigurations', timeVector, time_samples);

for i = 1:length(q)
    
    configSoln = q(:,i);
    goalConfig = configSoln';
    [validState,~] = checkCollision(robotModel,goalConfig',env,"IgnoreSelfCollision","off","Exhaustive","on","SkippedSelfCollisions","parent");

    if any(validState)
        disp('Colision en quinticpolytraj')
        pause
    end
end    

%% Plot the trajectory for each joint

figure;
hold on
for i = 1:6
    if i~=6
        plot(linspace(0, timeVector(end), length(q)), wrapToPi(q(i, :)), 'Color', colors(i, :));
    else
        plot(linspace(0, timeVector(end), length(q)), q(i, :), 'Color', colors(i, :));
    end
end
xlabel('Time [s]');
ylabel('Joint Position [rad]');
xlim([0 timeVector(end)])
%ylim([-pi pi])
legend({'Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6'}, ...
       'Location', 'northoutside', 'Orientation', 'horizontal');
hold off


figure;
hold on
for i = 1:6
plot(linspace(0, timeVector(end), length(qd)), (qd(i, :)), 'Color', colors(i, :));
end
xlabel('Time [s]');
ylabel('Joint Velocity [rad/s]');
xlim([0 timeVector(end)])
legend({'Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6'}, ...
       'Location', 'northoutside', 'Orientation', 'horizontal');
hold off

figure;
hold on
for i = 1:6
plot(linspace(0, timeVector(end), length(qdd)), (qdd(i, :)), 'Color', colors(i, :));
end
xlabel('Time [s]');
ylabel('Joint Acceleration [rad/s^{2}]');
xlim([0 timeVector(end)])
legend({'Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6'}, ...
       'Location', 'northoutside', 'Orientation', 'horizontal');
hold off

%% Visualize Robot following the Trajectory
configFinal = q';
figure;
ROTEND2 = [];
END2 = [];
quatEND2 = [];
for i=1:1:length(configFinal)
    k= k+1
    configuraciones(1).JointPosition = configFinal(i,1);
    configuraciones(2).JointPosition = configFinal(i,2);
    configuraciones(3).JointPosition = configFinal(i,3);
    configuraciones(4).JointPosition = configFinal(i,4);
    configuraciones(5).JointPosition = configFinal(i,5);
    configuraciones(6).JointPosition = configFinal(i,6);
    zoom(gca,'on');
    show(robot,configuraciones,"Collisions","off");
    camzoom(1.7);
    hold on
    show(env{1});
    show(env{2});
    show(env{3});
    show(env{4});
    hold on
    PuntoEnd = getTransform(robot,configuraciones,'tool0','base_link');
    END2 = [END2;PuntoEnd(1,4),PuntoEnd(2,4),PuntoEnd(3,4)];
    Rot_matrix = [PuntoEnd(1,1),PuntoEnd(1,2),PuntoEnd(1,3);PuntoEnd(2,1),PuntoEnd(2,2),PuntoEnd(2,3);PuntoEnd(3,1),PuntoEnd(3,2),PuntoEnd(3,3)];
    ROTEND2 = [ROTEND2;Rot_matrix];
    EulerAnglesEF = rotm2eul(Rot_matrix);
    quat2 = eul2quat(EulerAnglesEF, 'ZYX');  % n2x4 matrix
    quatEND2 = [quatEND2;quat2];
    drawnow;
    pause(0.0001);
    hold off
end

%% DTW to measure if the Trajectory computed by quinticpolytraj allows to follow the goal points in position and orientation

Measure_Accuracy
