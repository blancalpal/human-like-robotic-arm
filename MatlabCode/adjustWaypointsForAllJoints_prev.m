function [adjustedWPs] = adjustWaypointsForAllJoints_prev(wps)
n = size(wps, 1);
adjustedWPs = wps;

adjustedWPs_deg = rad2deg(mod(adjustedWPs + 2*pi, 2*pi));

i = 1;
while i <= size(adjustedWPs, 2) - 1
    affected_indices = zeros(1,n);
    short_path_angles = zeros(1,n);
    for j = 1:6

        if j ~=6 %|| (j==6 && any(affected_indices))

            angle1 = adjustedWPs_deg(j, i);
            angle2 = adjustedWPs_deg(j, i+1);

            doesCross = crosses180Degrees(angle1,angle2);

            short_path_angle = (max(adjustedWPs_deg(j, i),adjustedWPs_deg(j, i+1)) - min(adjustedWPs_deg(j, i),adjustedWPs_deg(j, i+1)) );

            if sign(adjustedWPs(j, i+1)) ~= sign(adjustedWPs(j, i)) && (short_path_angle < (360 - short_path_angle))
                affected_indices(1,j) = 1;
                short_path_angles(1,j) = (max(adjustedWPs_deg(j, i),adjustedWPs_deg(j, i+1)) + min(adjustedWPs_deg(j, i),adjustedWPs_deg(j, i+1)) )/2;
            end

            if (doesCross && ~(sign(adjustedWPs(j, i+1)) ~= sign(adjustedWPs(j, i)) && (short_path_angle < (360 - short_path_angle)))) || ((sign(adjustedWPs(j, i+1)) ~= sign(adjustedWPs(j, i)) && (short_path_angle < (360 - short_path_angle)))&& ~doesCross)
                disp('fallo adjustWaypoints')
                pause
            end
        end
    end

    if any(affected_indices)
        newWPs = zeros(n,1);

        for j = 1:6
            if affected_indices(1,j)
                newWPs(j,1) = deg2rad(short_path_angles(1,j) - 180);
            else
                newWPs(j,1) = adjustedWPs(j, i); %(adjustedWPs(j, i+1) + adjustedWPs(j, i))/2;
            end


        end

        adjustedWPs = [adjustedWPs(:,1:i), newWPs, adjustedWPs(:,i+1:end)];
        adjustedWPs(1:5,:) = wrapToPi(adjustedWPs(1:5,:)); % we do not modify the 6th joint as it can rotate
        adjustedWPs_deg = rad2deg(mod(adjustedWPs + 2*pi, 2*pi));
        i = i + 1;  % Adjust index to account for new waypoint
    end

    i = i + 1;
end