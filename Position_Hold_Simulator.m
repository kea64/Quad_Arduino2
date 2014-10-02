timeDelay = 0.02;
runTime = 3;
runClock = 0.0;
gravityConstant = -9.81;

yaw = 90.0;
roll = 20.0;
pitch = 20.0;
iniXCoord = 0.0;
iniYCoord = 0.0;
iniRollVelocity = 0.0;
iniPitchVelocity = 0.0;
quadWeight = 1.25;
hoverThrust = quadWeight;

kpr = 500;
kir = 0;
kdr = 0;

kpp = 500;
kip = 0;
kdp = 0;

targetXCoord = 2;
targetYCoord = 2;

rollVelocity = iniRollVelocity;
pitchVelocity = iniPitchVelocity;
rollPosition = iniXCoord;
pitchPosition = iniYCoord;

quad = [-.25,.25,.25,0,-.25;-0.25,-0.25,.25,0.5,.25];

xCoord = iniXCoord;
yCoord = iniYCoord;

hold off
axis([-5,5,-5,5])
xlabel('Longitude')
ylabel('Latitude')
title('Position Hold')


while runClock < runTime
    %% Position Hold PID
    errorX = targetXCoord - xCoord;
    errorY = targetYCoord - yCoord;
    
    errorRoll = errorX * cosd(yaw) + errorY * sind(yaw);
    errorPitch = errorY * cosd(yaw) + errorX * sind(yaw);
    
    roll = kpr * errorRoll;
    pitch = kpp * errorPitch;
    
    
    %% Quad Physics
    rollThrust = hoverThrust * tand(roll); %Accounts for Thrust loss due to tilt
    pitchThrust = hoverThrust * tand(pitch);
    
    rollForce = rollThrust * g;
    pitchForce = pitchThrust * g;
    
    rollAccel = rollForce / quadWeight;
    pitchAccel = pitchForce / quadWeight;
    
    rollVelocity = rollVelocity + rollAccel * timeDelay;
    pitchVelocity = pitchVelocity + pitchAccel * timeDelay;
    
    rollPosition = rollPosition + rollVelocity * timeDelay;
    pitchPosition = pitchPosition + pitchVelocity * timeDelay;
    
    %% Quad Mapping
    xCoord = rollPosition * cosd(yaw) + pitchPosition * sind(yaw); %Maps Quad Motion to Coordinate System
    yCoord = pitchPosition * cosd(yaw) + rollPosition * sind(yaw);
    
    %% Plotting Code
    rotationFactorX = [cosd(-yaw),-sind(-yaw)];
    rotationFactorY = [sind(-yaw),cosd(-yaw)];
    rotationQuad = [rotationFactorX*quad;rotationFactorY*quad];
    h = fill(rotationQuad(1,:)+xCoord,rotationQuad(2,:)+yCoord,'b');
    axis([-5,5,-5,5])
    
    pause(timeDelay);
    runClock = runClock + timeDelay;
    
end