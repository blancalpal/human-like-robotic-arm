for jj = 1:1:6
    configSoln(jj).JointPosition =res(ii,jj);
    %configSoln2(jj).JointPosition =res(ii,jj);
end
% Check collision using the exact collision model
goalConfig = [configSoln(1).JointPosition configSoln(2).JointPosition configSoln(3).JointPosition configSoln(4).JointPosition configSoln(5).JointPosition configSoln(6).JointPosition];
[validState,~] = checkCollision(robotModel,goalConfig',env,"IgnoreSelfCollision","off","Exhaustive","on","SkippedSelfCollisions","parent");
if ~any(validState)

    % End efector for the specific configuration
    EfectorFinal = getTransform(robot,configSoln,'tool0','base_link');
    X_RA = [EfectorFinal(1,4) EfectorFinal(2,4) EfectorFinal(3,4)];
    Rot_EF =[EfectorFinal(1,1) EfectorFinal(1,2) EfectorFinal(1,3);
        EfectorFinal(2,1) EfectorFinal(2,2) EfectorFinal(2,3);
        EfectorFinal(3,1) EfectorFinal(3,2) EfectorFinal(3,3)];
    EulerAnglesEF = rotm2eul(Rot_EF);
    quat_r = eul2quat(EulerAnglesEF);

    % Detect if the point is in the limits'
    %                 pointEnd = [gola(k,1),gola(k,2),gola(k,3)];
    %                 checkConfig

    % Wrist 2
    Wrist2 = getTransform(robot,configSoln,'wrist_2_link','base_link');
    S_wrist2 = [Wrist2(1,4) Wrist2(2,4) Wrist2(3,4)];

    % Wrist 1
    Wrist1 = getTransform(robot,configSoln,'wrist_1_link','base_link');
    S_wrist1 = [Wrist1(1,4) Wrist1(2,4) Wrist1(3,4)];

    % Wrist 3
    Wrist3 = getTransform(robot,configSoln,'wrist_3_link','base_link');
    S_wrist3 = [Wrist3(1,4) Wrist3(2,4) Wrist3(3,4)];

    %Elbow
    Codo = getTransform(robot,configSoln,'forearm_link','base_link');
    S_codo = [Codo(1,4) Codo(2,4) Codo(3,4)];

    %Shoulder
    HombroSaliente = getTransform(robot,configSoln,'shoulder_link','base_link');
    S_hombro = [HombroSaliente(1,4) HombroSaliente(2,4) HombroSaliente(3,4)];

    %Calculate the diference in position
    d_RAx = distPosition(X_RA,X_RAGoal);
    % Calculate the diference in orientation
    d_RAo = distOrientation(quat_r,quat_h);
    % Calculate the diference between the human elbow and the
    % rest of the arm
    MDistancia = distanceMetric(VectorCodo, S_wrist2,S_wrist1,S_wrist3,S_codo,S_hombro,gola');

    %Wrist error estimation
    if i == 1 || ~exist('Wrist_old','var')
        ErroWrist = 0;
    else
        ErroWrist = variationWrist(configSoln(4).JointPosition,Wrist_old,configSoln(3).JointPosition,Elbow_old);
    end

    DistFinal = real(d_RAx + 100 * d_RAo + MDistancia);% + ErroWrist);

    CodoRobotrot = rotx(45)*S_codo';
    WristRobotRot = rotx(45)*S_wrist1';
    HombroRobotRot = rotx(45)*S_hombro';

    % Check if it is finished
    if d_RAx<=0.2 && d_RAo <= 1 && DistFinal < DistDif && wrapToPi(configSoln(4).JointPosition) >= Wrist4Lim(1) && wrapToPi(configSoln(4).JointPosition) <= Wrist4Lim(2) && configSoln(1).JointPosition >= HombroLim(1) && configSoln(1).JointPosition <= HombroLim(2) && configSoln(2).JointPosition >= HombroLim2(1) && configSoln(2).JointPosition <= HombroLim2(2)
        DistDif = DistFinal;
        MejorConfig = [configSoln(1).JointPosition configSoln(2).JointPosition configSoln(3).JointPosition configSoln(4).JointPosition configSoln(5).JointPosition configSoln(6).JointPosition];
        mejor_ii = ii;
        keep_best = 1;
    end
end