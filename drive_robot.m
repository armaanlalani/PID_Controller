% Edit this function to implement alternative control schemes
%  *** Add code in the switch statement starting at line 19 ***
function robot=drive_robot(robot,T,dT,ctrl_enable,Kp,Ki,Kd,Kc,f)

% Initializing
DC_ideal=getDC(robot.setSpeed,robot.r,robot.wMax); % Get duty cycle to initialize motors
robot.rMot = DC_ideal;
robot.lMot = DC_ideal;

% Working out timings
Tstep=T(2); % Step size of simulation
ctrl_interval=round(dT/Tstep);

% Control parameters *** Add parameters here as necessary ***


figure;

for i=2:length(T)
    
    % Compute change in position
    robot=newPos(robot,Tstep,i,f);
    
    if mod(i,ctrl_interval)< eps %If it is time to provide feedback
       switch ctrl_enable % Open loop control; no feedback
           case 0; % Open loop, do nothing
           case 1; % Proportional control
               
               % Get error signal (Computed as Left - Right encoder reading)
               enc_error = getError(robot,i,ctrl_interval);
               % Modify motor duty cycle accordingly
               robot.rMot = robot.rMot+Kp*enc_error;
               robot.lMot = robot.lMot-Kp*enc_error;
               
               % Make sure PWM is still between [0, 1]
               robot=checkDC(robot);
               
           case 2; % Full PID Control
               enc_error = getError(robot,i,ctrl_interval);
               dterm = (enc_error - robot.olderror)/dT;
               robot.error = robot.error + enc_error;
               
               robot.rMot = robot.rMot + Kp*enc_error + Ki*robot.error + Kd*dterm;
               robot.lMot = robot.lMot - Kp*enc_error - Ki*robot.error - Kd*dterm;
               
               robot.olderror = enc_error;
               
               robot.e(i) = enc_error;
              
           case 3  % Compass control
               enc_error = getError(robot,i,ctrl_interval);
               
               robot.rMot = robot.rMot + Kc * robot.heading;
               robot.lMot = robot.lMot - Kc * robot.heading;
               
               robot.e(i) = enc_error;

           case 4 % PID control with implemented compass
               enc_error = getError(robot,i,ctrl_interval);
               dterm = (enc_error - robot.olderror)/dT;
               robot.error = robot.error + enc_error;
               
               robot.rMot = robot.rMot + Kp*enc_error + Ki*robot.error + Kd*dterm + Kc * robot.heading;
               robot.lMot = robot.lMot - Kp*enc_error - Ki*robot.error - Kd*dterm - Kc * robot.heading;
               
               robot.olderror = enc_error;
               robot.e(i) = enc_error;

       end
       
           % Code to visualize robot as it steps
%     clf; hold on;
%     scatter(robot.path(1,i),robot.path(2,i),'b');
%     scatter(robot.lWheel(1,i),robot.lWheel(2,i),'k');
%     scatter(robot.rWheel(1,i),robot.rWheel(2,i),'k');
%     axis tight
%     plot(xlim, [0 0], '-r')
%     xlim([-0.1 1]); ylim([-.2 .2]);
       
    end

end
end
