
%% Function to evaluate the distance between the human elbow and the rest of the robotic joints
function [D] = distanceMetric(CodoH,W2,W1,W3,CodoRobot,Hombro,gola)
    CodoH = (inv(rotx(45))*CodoH')'; 
    C_W2 = norm(CodoH-W2);
    C_W1 = norm(CodoH-W1);
    C_W3 = norm(CodoH-W3);
    C_CR = norm(CodoH-CodoRobot);
    C_HR = norm(CodoH-Hombro);
    CodoRobotrot = rotx(45)*CodoRobot';

    punto = (gola./norm(gola)).*(norm(gola)/2);

    DD = norm(CodoRobotrot-punto);
    
    Dist_C = (norm(CodoRobotrot-[0;-0.03;-0.15]));
    C_A = 0.2/Dist_C;
    
    D = 10*C_W1+10*C_W2+10*C_W3+300*C_CR+C_HR; % That cost function must be adapted for your problem 
end
