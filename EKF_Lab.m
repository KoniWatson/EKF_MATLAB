clear all
close all
%% Information 
maxSpeed = 20; %(mm/s)
leftSpeed = maxSpeed;
rightSpeed = maxSpeed;

wheelBase = 120; %(mm)
wheelRadius = 10; %(mm)

origin = [-250 -250 0]; %(mm)
xOrigin = origin(1);
yOrigin = origin(2);
thetaOrigin = 0; %(rad)
%% PART ONE - True Trajectory 
% MATLAB was used to program a robot to preform a raster scan of length
% 2000mm with spacing of 150mm, with 7 passings. This representing robots true
% trajectory(path). Such Trajectory is the ideal
% path the robot would take if the robot was to work perfectly and there
% was no deviation in the wheel speeds. To ilisutate this the assumtion
% that there are no errors within the system was made.
%
count = 1;
xPoints(count,:) = xOrigin;
yPoints(count,:) = yOrigin;
thetaPosition(count,:) = thetaOrigin;
wheelSpeeds(count,:) = [maxSpeed, maxSpeed];

currentX = xOrigin;
currentY = yOrigin;
currentTheta = thetaOrigin;

for passes = 1:7
    xOrigin = currentX;
    yOrigin = currentY;
    thetaOrigin = currentTheta;
    
    if passes < 7 
        if mod(passes,2) == 1
            while abs(currentX - xOrigin) < 2000
                count = count + 1;
                
                currentX = currentX + horizontalMove(maxSpeed,maxSpeed,currentTheta);
                
                xPoints(count,:) = currentX;
                yPoints(count,:) = currentY;
                thetaPosition(count,:) = currentTheta;
                wheelSpeeds(count,:) = [maxSpeed, maxSpeed];
            end
            
            while currentTheta - thetaOrigin < pi/2
                count = count + 1;
                
                currentTheta = currentTheta + rotation(maxSpeed,-maxSpeed);
                
                thetaPosition(count,:) = currentTheta;
                xPoints(count,:) = currentX;
                yPoints(count,:) = currentY;
                wheelSpeeds(count,:) = [maxSpeed, -maxSpeed];
            end
            
            while abs(currentY - yOrigin) < 150
                count = count + 1;
                
                currentY = currentY + verticalMove(maxSpeed,maxSpeed,currentTheta);
                
                yPoints(count,:) = currentY;
                xPoints(count,:) = currentX;
                thetaPosition(count,:) = currentTheta;
                wheelSpeeds(count,:) = [maxSpeed, maxSpeed];
            end
            
            while currentTheta - thetaOrigin < pi
                count = count + 1;
                
                currentTheta = currentTheta + rotation(maxSpeed,-maxSpeed);
                
                thetaPosition(count,:) = currentTheta;
                xPoints(count,:) = currentX;
                yPoints(count,:) = currentY;
                wheelSpeeds(count,:) = [maxSpeed, -maxSpeed];
            end
            
        else
            while abs(currentX - xOrigin) < 2000
                count = count + 1;
                
                currentX = currentX + horizontalMove(maxSpeed,maxSpeed,currentTheta);
                
                xPoints(count,:) = currentX;
                yPoints(count,:) = currentY;
                thetaPosition(count,:) = currentTheta;
                wheelSpeeds(count,:) = [maxSpeed, maxSpeed];
            end
            
            while currentTheta - thetaOrigin > -pi/2
                count = count + 1;
                
                currentTheta = currentTheta + rotation(-maxSpeed,maxSpeed);
                
                xPoints(count,:) = currentX;
                yPoints(count,:) = currentY;
                thetaPosition(count,:) = currentTheta;
                wheelSpeeds(count,:) = [-maxSpeed, maxSpeed];
            end
            
            while abs(currentY - yOrigin) < 150
                count = count + 1;
                
                currentY = currentY + verticalMove(maxSpeed,maxSpeed,currentTheta);
                
                yPoints(count,:) = currentY;
                xPoints(count,:) = currentX;
                thetaPosition(count,:) = currentTheta;
                wheelSpeeds(count,:) = [maxSpeed, maxSpeed];
            end
            
            while currentTheta - thetaOrigin > -pi
                count = count + 1;
                
                currentTheta = currentTheta + rotation(-maxSpeed,maxSpeed);
                
                xPoints(count,:) = currentX;
                yPoints(count,:) = currentY;
                thetaPosition(count,:) = currentTheta;
                wheelSpeeds(count,:) = [-maxSpeed, maxSpeed];
            end
        end
    else
        while abs(currentX - xOrigin) < 2000
            count = count + 1;
            
            currentX = currentX + horizontalMove(maxSpeed,maxSpeed,currentTheta);
            
            xPoints(count,:) = currentX;
            yPoints(count,:) = currentY;
            thetaPosition(count,:) = currentTheta;
            wheelSpeeds(count,:) = [maxSpeed, maxSpeed];
        end
    end
end
% figure(1);
% subplot(2,1,1);
% plot(wheelSpeeds(:,2),'r', 'LineWidth', 1);
% subplot(2,1,2);
% plot(wheelSpeeds(:,1),'b', 'LineWidth', 1);
% figure(2);
% plot(xPoints, yPoints, 'b', 'LineWidth', 3);
% hold on
%% PART TWO - Dead Reckoning  
% The assumption before are not assumed. The same trajectory was illistated
% but this time it incorporates the wheel encoder noise (speed estimate with
% variance of 0.1 (mm/s)^2 per measurement). 
encVary = 0.1; %(mm/s)^2

xDeadPoints(1) = origin(1);
yDeadPoints(1) = origin(2);
thetaDeadPosition(1) = 0;

wheelDeadSpeeds = wheelSpeeds + (sqrt(encVary)*randn(length(wheelSpeeds),2));

for i = 2:length(wheelDeadSpeeds)
    thetaDeadPosition(i,:) = thetaDeadPosition(i-1) + rotation(wheelDeadSpeeds(i,1), wheelDeadSpeeds(i,2));
    xDeadPoints(i,:) = xDeadPoints(i-1) + horizontalMove(wheelDeadSpeeds(i,1), wheelDeadSpeeds(i,2), thetaDeadPosition(i-1));
    yDeadPoints(i,:) = yDeadPoints(i-1) + verticalMove(wheelDeadSpeeds(i,1), wheelDeadSpeeds(i,2), thetaDeadPosition(i-1));
end
% figure(3);
% subplot(2,1,1);
% plot(wheelDeadSpeeds(:,2),'r', 'LineWidth', 1);
% subplot(2,1,2);
% plot(wheelDeadSpeeds(:,1),'b', 'LineWidth', 1);
% figure(2);
% hold on 
% plot(xPoints, yPoints, 'b', 'LineWidth', 3);
% plot3(xDeadPoints, yDeadPoints, zeros(length(xDeadPoints),1), 'r', 'LineWidth', 2);
%% PART THREE - Trilateration 
% From the inoframtion known there are three beacons used to idetify the
% robots postion. These beacons acting like 'GPS' of the robot and
% postioned at: B1 = (2500 2500 1000)T, B2 = (-2500 2500 1000)T, 
% B3 = (2500 -2500 1000)T.
%
% These beacons estimate the distance between them and the robot with a
% variance of 20mm^2. Taking into account this varience the new trajectory
% can be illistated.
B1 = [2500 2500 1000]; %(mm)
B2 = [-2500 2500 1000]; %(mm)
B3 = [2500 -2500 1000]; %(mm)
posVary = 20; %mm^2

for i = 1:length(xPoints)
    tri(i,:) = trilateration(xPoints(i),yPoints(i),1000,posVary,B1,B2,B3);
end
% figure(2)
% plot(tri(:,1),tri(:,2), 'm.', 'MarkerSize', 2);
% hold on
% plot(xPoints, yPoints, 'b', 'LineWidth', 3);
% hold on 
% plot(xDeadPoints, yDeadPoints, 'r', 'LineWidth', 2);
% plot3(B1(1), B1(2), B1(3), '*r')
% plot3(B2(1), B2(2), B2(3), '*b')
% plot3(B3(1), B3(2), B3(3), '*k')
%% PART FOUR - Extended Kalman Filter 
% A EKF can be used to combot these vareince to allow for the robot to be
% closer to the true tragectory and thus perform the raster scan as intended.
encFrequency = 10; %(Hz)
encTime = 1/encFrequency; %(s)

startP = zeros(4,4);
P = startP;

%Jacobian matrix of the partial derivatives of h (respect to z)
V = diag([1, 1, 1]);
%Measurement noise covariance 
R = diag([posVary, posVary, posVary]);

xHatPrev = [origin(1,1);origin(1,2);0;0];

for i = 1:length(xPoints)
    xHatMinus = [xHatPrev(1) + horizontalMove(wheelDeadSpeeds(i,1),wheelDeadSpeeds(i,2),xHatPrev(4));
                 xHatPrev(2) + verticalMove(wheelDeadSpeeds(i,1),wheelDeadSpeeds(i,2),xHatPrev(4));
                 xHatPrev(3);
                 xHatPrev(4) + rotation(wheelDeadSpeeds(i,1),wheelDeadSpeeds(i,2))];
    
    %Jacobian matrix of the partial derivatives of f (respect to x)
    Ak = [1 0 0 ((-1/2)*((wheelDeadSpeeds(i,1)+wheelDeadSpeeds(i,2))*sin(xHatPrev(4)))*encTime);
          0 1 0 ((1/2)*((wheelDeadSpeeds(i,1)+wheelDeadSpeeds(i,2))*cos(xHatPrev(4)))*encTime);
          0 0 1 0;
          0 0 0 1];
    
    %Jacobian matrix of the partial derivatives of f(respect to w - process noise)
    Wk = [((1/2)*cos(xHatPrev(4))*encTime) ((1/2)*sin(xHatPrev(4))*encTime);
          ((1/2)*cos(xHatPrev(4))*encTime) ((1/2)*sin(xHatPrev(4))*encTime);
          0 0;
          (1/wheelBase)*encTime (1/wheelBase)*encTime];
    
    %Process noise
    Qk = [encVary 0;
          0 encVary]; 
    
    %Error Prediction
    Pminus = (Ak * P * Ak') + (Wk * Qk * Wk');
    
    disB1 = sqrt((xPoints(i) - B1(1))^2 + (yPoints(i) - B1(2))^2 + (0 - B1(3))^2);
    disB2 = sqrt((xPoints(i) - B2(1))^2 + (yPoints(i) - B2(2))^2 + (0 - B2(3))^2);
    disB3 = sqrt((xPoints(i) - B3(1))^2 + (yPoints(i) - B3(2))^2 + (0 - B3(3))^2);
    
    L1 = sqrt((xHatMinus(1) - B1(1))^2 + (xHatMinus(2) - B1(2))^2 + (xHatMinus(3) - B1(3))^2);
    L2 = sqrt((xHatMinus(1) - B2(1))^2 + (xHatMinus(2) - B2(2))^2 + (xHatMinus(3) - B2(3))^2);
    L3 = sqrt((xHatMinus(1) - B3(1))^2 + (xHatMinus(2) - B3(2))^2 + (xHatMinus(3) - B3(3))^2);
    
    %Measurement noise
    z = [disB1 + (sqrt(encVary)*randn(1));
         disB2 + (sqrt(encVary)*randn(1));
         disB3 + (sqrt(encVary)*randn(1))];
    
    h = [L1;
         L2;
         L3];
    %Jacobian matrix of the partial derivatives of h (respect to x)
    H = [((xHatMinus(1)-B1(1))/L1) ((xHatMinus(2)-B1(2))/L1) ((xHatMinus(3)-B1(3))/L1) 0;
         ((xHatMinus(1)-B2(1))/L2) ((xHatMinus(2)-B2(2))/L2) ((xHatMinus(3)-B2(3))/L2) 0;
         ((xHatMinus(1)-B3(1))/L3) ((xHatMinus(2)-B3(2))/L3) ((xHatMinus(3)-B3(3))/L3) 0];
    
    %Kalman Gain Calculation
    K = Pminus * H'/((H * Pminus * H') + (V * R * V'));
    
    %Measurement Correction
    xHat = xHatMinus + K * (z - h);
    
    %Error Propagation
    P = (eye(4) - K * H) * Pminus;
    
    efk(i, 1:4) = xHat;
    xHatPrev = xHat;
end
% figure(2)
% plot(tri(:,1),tri(:,2), 'm.', 'MarkerSize', 2);
% hold on
% plot(xPoints, yPoints, 'b', 'LineWidth', 3);
% plot(xDeadPoints, yDeadPoints, 'r', 'LineWidth', 2);
% plot(efk(:,1),efk(:,2), 'g', 'LineWidth', 1.5);




