timeDelay = 0.02;
gravityConstant = -9.81;
sensorNoise = 0.2;

runTime = 30;
runClock = 0.0;

iniVelocity = 0;
velocity = iniVelocity;
iniAlt = 5;
alt = iniAlt;
lastAlt = iniAlt;
sensorAlt = alt - sensorNoise + 2 * sensorNoise * rand(1);
targetAlt = 7;
MAX_ALT = 15;

quadcopterWeight = 1.4;
Fg = quadcopterWeight * gravityConstant;
iniMotorServo = 1318;
throttleOut = iniMotorServo;
%throttleOut = 1318;
ITermT = iniMotorServo;
MAX_THROTTLE = iniMotorServo + 25;
MIN_THROTTLE = iniMotorServo - 25;

kpt = 0;
kit = 5;
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
    sensorAlt = alt - (0.01 * sensorNoise * (randi(200,1,1)-100));
    if alt <= 0 
        alt = 0;
        velocity = 0;
    end
    
    h = fill(0.5*iniX,0.5*iniY + alt,'g');
    axis([-3,3,0,10])
    
    pause(timeDelay);
    runClock = runClock + timeDelay;
end