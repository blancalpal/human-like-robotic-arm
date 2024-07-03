%% Plot the values to compare
% Script that plot the errors to comapre the eficiency
% Position error
figure(10);
subplot(3,2,1);
plot(gola2(:,1),'Color','g');
hold on
plot(END(:,1),'Color',[0 0.4470 0.7410]);
legend("X Robot","X Human");
hold off
title('X');
% xlabel('Index');
xlim([0 length(END)])
ylabel('Position X [m]');

subplot(3,2,2);
plot(END(:,2),'Color','g');
hold on

plot(gola2(:,2),'Color',[0 0.4470 0.7410]);
legend("Y Robot","Y Human");
hold off
title('Y');
% xlabel('Index');
xlim([0 length(gola2)])
ylabel('Position Y [m]');

subplot(3,2,3);
plot(gola2(:,3),'Color','g');
hold on
plot(END(:,3),'Color',[0 0.4470 0.7410]);
legend("Z Robot","Z Human");
hold off
title('Z');
% xlabel('Index');
xlim([0 length(END)])
ylabel('Position Z [m]');

dataErrorPos = abs(gola2(:,:) - END(:,:));
subplot(3,2,4);
h = boxchart(dataErrorPos,'JitterOutliers','off','MarkerStyle','.');
set(gca,'XTickLabel',{'X Error', 'Y Error','Z Error'});
title('Position Error');
ylabel('Error [m]');


%% Orientation error
THETA = [];
for i=1:3:length(ROTEND)
    R1 = [ROTMAT(i,1),ROTMAT(i,2),ROTMAT(i,3);ROTMAT(i+1,1),ROTMAT(i+1,2),ROTMAT(i+1,3);ROTMAT(i+2,1),ROTMAT(i+2,2),ROTMAT(i+2,3)];
    R2 = [ROTEND(i,1),ROTEND(i,2),ROTEND(i,3);ROTEND(i+1,1),ROTEND(i+1,2),ROTEND(i+1,3);ROTEND(i+2,1),ROTEND(i+2,2),ROTEND(i+2,3)];
    R_1 = R1;
    R_2 = R2;

    EulerAnglesEF1 = rotm2eul(R1);
    quat1 = eul2quat(EulerAnglesEF1, 'ZYX');  % n2x4 matrix

    EulerAnglesEF2 = rotm2eul(R2);
    quat2 = eul2quat(EulerAnglesEF2, 'ZYX');  % n2x4 matrix
    
    quat_dif = distOrientation(quat1, quat2);
    THETA = [THETA;quat_dif];    
end

subplot(3,2,5);
hold on
plot(THETA,'Color',[0 0 0])
scatter(1:1:length(THETA), THETA, 10, 'filled', 'MarkerFaceColor', [0 0.4470 0.7410]);
title('Orientation Error along the Path');
% xlabel('Index');
xlim([0 length(THETA)])
ylabel('Error [rad]');
hold off

subplot(3,2,6);
boxchart(THETA,'JitterOutliers','off','MarkerStyle','.');
title('Orientation Error');
ylabel('Error [rad]');
set(gca,'XTickLabel',{});
hold off
