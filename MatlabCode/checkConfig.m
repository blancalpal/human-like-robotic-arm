%% Check if the configuration generates a position out of the limits
PuntoEnd = getTransform(robot2,configSoln2,'tool0','world');
dif = norm(PuntoEnd(1:3,4)'- ([matrixRead(i,4) matrixRead(i+1,4) matrixRead(i+2,4)]+[0.07 0.13 1.15]));

if dif > 0.05
    correct = false;
else
    correct = true;
end