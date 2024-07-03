
% Example data for curves
curveA = END;
curveB = END2;

% Compute DTW distance and path
%[distance, pathA, pathB] = dtw3d(curveA, curveB);
[distance, std_distance, max_distance, pathA, pathB] = dtw3d_oneWay(curveA, curveB);

fprintf('DTW distance between the position curves: %.4f, %.4f, %.4f\n', [distance, std_distance, max_distance]);

% Plotting the curves and DTW path
figure;
plot3(curveA(:,1), curveA(:,2), curveA(:,3), 'ro-', 'LineWidth', 2, 'MarkerSize', 3);
hold on;
plot3(curveB(:,1), curveB(:,2), curveB(:,3), 'bo-', 'LineWidth', 2, 'MarkerSize', 3);

% Plot lines between matched points
for i = 1:length(pathA)
    plot3([curveA(pathA(i),1) curveB(pathB(i),1)], ...
          [curveA(pathA(i),2) curveB(pathB(i),2)], ...
          [curveA(pathA(i),3) curveB(pathB(i),3)], 'k--');
end

title('3D DTW Curve Distance Alignment');
xlabel('X');
ylabel('Y');
zlabel('Z');
legend('Curve A', 'Curve B', 'DTW Matching');
grid on;
axis equal;
hold off;



curveA = quatEND;
curveB = quatEND2;

% Calculate DTW distance for quaternion data
%[dist, pathA, pathB] = dtw_quat_2(curveA, curveB);
[dist, std_dist, max_dist, pathA, pathB] = dtw_quat_oneWay(curveA, curveB);

fprintf('DTW distance between the orientation curves: %.4f, %.4f, %.4f\n', [dist, std_dist, max_dist]);



% Plotting the curves and DTW path
figure;
plot3(curveA(:,1), curveA(:,2), curveA(:,3), 'ro-', 'LineWidth', 2, 'MarkerSize', 3);
hold on;
plot3(curveB(:,1), curveB(:,2), curveB(:,3), 'bo-', 'LineWidth', 2, 'MarkerSize', 3);

% Plot lines between matched points
for i = 1:length(pathA)
    plot3([curveA(pathA(i),1) curveB(pathB(i),1)], ...
          [curveA(pathA(i),2) curveB(pathB(i),2)], ...
          [curveA(pathA(i),3) curveB(pathB(i),3)], 'k--');
end

title('3D DTW Curve Orientation Alignment');
xlabel('X');
ylabel('Y');
zlabel('Z');
legend('Curve A', 'Curve B', 'DTW Matching');
grid on;
axis equal;
hold off;
