function isCrossing180 = crosses180Degrees(angle1, angle2)
    % Normalize angles to range [-180, 180)
    angle1 = mod(angle1 + 180, 360) - 180;
    angle2 = mod(angle2 + 180, 360) - 180;

    % Calculate the direct distance between angles
    directDistance = abs(angle1 - angle2);

    % Check if the shortest path crosses -180/180 degrees
    isCrossing180 = false;
    if directDistance > 180
        isCrossing180 = true;
    end
end
