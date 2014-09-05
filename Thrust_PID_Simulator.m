timeDelay = 0.02;
gravityConstant = -9.81;
sensorNoise = 0.4;

runTime = 30;
runClock = 0.0;

iniVelocity = 0;
velocity = iniVelocity;
iniAlt = 1;
alt = iniAlt;
lastAlt = iniAlt;
sensorAlt = alt - sensorNoise + 2 * sensorNoise * rand(1);
targetAlt = 2;
MAX_ALT = 5;

quadcopterWeight = 1.4;
Fg = quadcopterWeight * gravityConstant;
iniMotorServo = 1318;
throttleOut = iniMotorServo;
%throttleOut = 1388;
ITermT = iniMotorServo;
MAX_THROTTLE = 1800;
MIN_THROTTLE = 1100;

kpt = 5;
kit = 0.25;
kdt = 5;

iniX = [-.25,.25,.25,-.25]; %Object Coordinates
iniY = [0,0,.5,.5];

hold off
axis([-3,3,0,15])
xlabel('X-Axis')
ylabel('Y-Axis')
title('Thrust PID')


while runClock < runTime
    %PID
    errorThrottle = targetAlt - sensorAlt;
    ITermT = ITermT + kit * timeDelay * errorThrottle;
    if ITermT > MAX_THROTTLE
        ITermT = MAX_THROTTLE;
    end
    if ITermT < MIN_THROTTLE
        ITermT = MIN_THROTTLE;
    end
    throttleOut = kpt * errorThrottle + ITermT - (kdt * (sensorAlt - lastAlt))/timeDelay;
    if throttleOut > MAX_THROTTLE
        throttleOut = MAX_THROTTLE;
    end
    if throttleOut < MIN_THROTTLE
        throttleOut = MIN_THROTTLE;
    end
    lastAlt = sensorAlt;
    
    motorThrust = 0.0044 * (throttleOut - 1000);
    netAccel = (motorThrust * -gravityConstant)/quadcopterWeight + gravityConstant;
    velocity = velocity + netAccel * timeDelay;
    %alt = alt + iniVelocity * timeDelay + netAccel * timeDelay * runClock;
    alt = alt + velocity * timeDelay; 
    sensorAlt = alt - sensorNoise + 2 * sensorNoise * rand(1);
    if alt <= 0 
        alt = 0;
        velocity = 0;
    end
    
    h = fill(0.5*iniX,0.5*iniY + alt,'g');
    axis([-3,3,0,10])
    
    pause(timeDelay);
    runClock = runClock + timeDelay;
end