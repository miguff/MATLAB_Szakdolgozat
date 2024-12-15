function sdot2 = exampleHelperDerivative(t,s,model,desiredPosition,desiredYaw,kp,kd,ki,kyaw)
% Copyright 2022 The MathWorks, Inc.


% global prevError prevTime

DroneMass = model.Configuration.Mass;
Gravity = model.environment.Gravity;
yaw = s(7);

%use extra state for integral error and velocity difference for pd

currentPosition = [s(1);s(2);s(3)];
currentVelocity = [s(4);s(5);s(6)];
currentError = desiredPosition - currentPosition;
% intError = intError + currentError*dt;


% if abs(dt) > eps
%     diffError = ((currentError - prevError)/dt);
% else
%     diffError = 0;
% end

pTerm = kp*currentError;
iTerm = ki*s(14); % intError;
dTerm = -kd*currentVelocity;

acc = pTerm + iTerm + dTerm;

% prevError = currentError;
% prevTime = t;

[u.Roll,u.Pitch,u.Thrust] = exampleHelperAccControl(acc,yaw,DroneMass,Gravity);
u.YawRate = kyaw*angdiff(yaw,desiredYaw);
e = model.environment;

sdot = derivative(model,s(1:13),u,e);
sdot2 = [sdot;currentError];

