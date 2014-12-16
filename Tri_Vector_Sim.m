clear all;
clc;

timeDelay = 0.02;
runTime = 2;
runClock = 0.0;

arm1 = [-0.05,0.05,0.05,-0.05;0.125,0.125,1,1];
arm2 = [0.133,0.891,0.841,0.083;-0.0192,-0.4567,-0.5433,-0.1058];
arm3 = [-0.133,-0.891,-0.841,-0.083;-0.0192,-0.4567,-0.5433,-0.1058];

hold off;
axis([-5,5,-5,5]);

rotation = 0;
xCoord = 0;
yCoord = 0;

maxServo = 2000;
minServo = 1000;
midServo = 1500;

inRoll = 2000;
inPitch = 1500;
inYaw = 1600;

absRoll = (inRoll - midServo)/(midServo-minServo);
absPitch = (inPitch - midServo)/(midServo-minServo);
absYaw = (inYaw - midServo)/(midServo-minServo);

angle1 = (0.5 * absRoll + 1/3 * absYaw);
angle2 = (-0.577 * absPitch - 0.25 * absRoll + 1/3 * absYaw);
angle3 = (0.577 * absPitch - 0.25 * absRoll + 1/3 * absYaw);

angle1New = angle1 * 500 + 1500;
angle2New = angle2 * 500 + 1500;
angle3New = angle3 * 500 + 1500;

yawScalar = 10;
rollScalar = 0.1;
pitchScalar = 0.1;

while runClock < runTime
    rotation = yawScalar * (angle1 + angle2 + angle3) + rotation;
    roll = (angle1 - 0.5 * angle2 - 0.5 * angle3) * rollScalar;
    pitch = (0.577 * angle3 - 0.577 * angle2) * pitchScalar;
    
    xCoord = roll * cosd(rotation) - pitch * sind(rotation) + xCoord; 
    yCoord = pitch * cosd(rotation) - roll * sind(rotation) + yCoord;
    
    rotationFactorX1 = [cosd(-rotation),-sind(-rotation)];
    rotationFactorY1 = [sind(-rotation),cosd(-rotation)];
    rotationFactorX2 = [cosd(-rotation),-sind(-rotation)];
    rotationFactorY2 = [sind(-rotation),cosd(-rotation)];
    rotationFactorX3 = [cosd(-rotation),-sind(-rotation)];
    rotationFactorY3 = [sind(-rotation),cosd(-rotation)];
    
    rotationArm1 = [rotationFactorX1*arm1;rotationFactorY1*arm1];
    rotationArm2 = [rotationFactorX2*arm2;rotationFactorY2*arm2];
    rotationArm3 = [rotationFactorX3*arm3;rotationFactorY3*arm3];
    
    fill(rotationArm1(1,:)+xCoord,rotationArm1(2,:)+yCoord,'r')
    hold on;
    fill(rotationArm2(1,:)+xCoord,rotationArm2(2,:)+yCoord,'b')
    fill(rotationArm3(1,:)+xCoord,rotationArm3(2,:)+yCoord,'b')
    hold off;
    axis([-5,5,-5,5]);
    
    label = sprintf('Arm 1: %d  Arm 2: %d  Arm 3: %d', angle1, angle2, angle3);
    xlabel(label);
    pause(timeDelay);
    runClock = runClock + timeDelay;
end









