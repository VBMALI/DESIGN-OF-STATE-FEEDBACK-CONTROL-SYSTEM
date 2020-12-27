%% Clean Up
clearvars;  % Clear the workspace
clc;  % Clear the command window
close('all');  % Close the open figures
Simulink.sdi.clear;  % Clear the simulink data inspector

%% Simulation Parameters

% Solver
simPrm.solTyp = 'Fixed-step';  % Solver type
simPrm.sol    = 'ode3';  % Solver type 2
simPrm.dt  = 0.001; % Integration step size
simPrm.tEnd = 10; % Simulation end time

% Input
Input.Ts = 1;  % Start time of input step
Input.U = 1;   % Amplitude of step input
Umax = 2.5;    % Max allowed input
Umin = -2.5;   % Min allowed input

% Controller update rate
Control.Z = 0.01;  % Sample time for discrete controller

%% Plant & sensor Paramters

% Plant
numG = 25;  % Numerator of plant transfer function
denG = [1 4 25];  % Denominator of plant transfer function
G = tf(numG,denG);  % Plant transfer function

% Sensor
Tau = 0.1;  % Time constant of sensor
numH = 1;  % Numerator of sensor transfer function
denH = [Tau, 1];  % Denominator of sensor transfer function
H = tf(numH,denH);  % Sensor transfer function

% Input filter
Tf = 0.06;  % Input filtering time constant
numFSF = 1;  % Numerator of filter transfer function
denFSF = [Tf 1]; % Denominator of filter transfer function
ctf = tf(numFSF,denFSF);  % Create a continuous transfer function of filter
dtf = c2d(ctf,Control.Z);  % Convert to discrete transfer function of filter
[numd,dend] = tfdata(dtf,'v');  % Extract the numerator and denominator of discrete transfer function

%% Open loop system stability
p = [0.1 1.4 6.5 25];  % define the coefficients
r = roots(p); % roots of the equation

%% Open loop system step response characteristics
TF_OL = G*H;  % Create the transfer function of open loop

S = stepinfo(TF_OL);  % create the step response information

%% PID Controller Design Parameters

% First Iteration
% Control.Ku = 2.64; 
% Control.Pu = 0.8;
% Control.Kp = 1.31*Control.Ku;   
% Control.Ki = 0.7*Control.Kp*Control.Pu;  
% Control.Kd = 1.5*Control.Kp*Control.Pu/8;

% Second Iteration
% Control.Ku = 2.64;  
% Control.Pu = 0.8;
% Control.Kp = 1.4*Control.Ku;      
% Control.Ki = 0.9*Control.Kp*Control.Pu;  
% Control.Kd = 2.1*Control.Kp*Control.Pu/8;   

% Third Iteration or Final design
Control.Ku = 2.64;   % Ultimate gain
Control.Pu = 0.8;    % Ultimate time period of oscillations
Control.Kp = 1.62*Control.Ku;   % Proportional gain ;   
Control.Ki = 0.95*Control.Kp*Control.Pu;  % Integral Gain   
Control.Kd = 2.35*Control.Kp*Control.Pu/8;   % Derivative gain 

%% SF Controller Design

% Create state space model
P.A = [0 1 0;-25 -4 0;1/Tau 0 -1/Tau]; % A matrix
P.B = [0;25;0];  % B matrix
P.C = [0 0 1];  % C Matrix
P.D = [0];  % D Matrix

P.ss = ss(P.A,P.B,P.C,P.D); % Create state space model

P.ssd = c2d(P.ss,Control.Z);  % Discretized version

%% Check Controllability
Mc = ctrb(P.A, P.B);  % Create controllability matrix
rho_C = rank(Mc);   % Calculate the rank of controllability matrix

%% Pole Placement

% desired closed loop poles
% First Stage
% cl.P=[-8; -3.2+2.4j; -3.2-2.4j];

% First Iteration
% cl.P=[-8; -4.05+1.96j; -4.05-1.96j];

% Second Iteration
% cl.P=[-8; -8+4.5j; -8-4.5j];

% Third Iteration
cl.P=[-8; -8+10j; -8-10j];

% find the SF gains
cl.K = place( P.A, P.B, cl.P );

% create the closed loop state space matrices
cl.A = P.A - P.B * cl.K;
cl.B = P.B;
cl.C = P.C;
cl.D = P.D;

% closed loop state space object
cl.ss = ss(cl.A, cl.B, cl.C, cl.D);
[cl.num, cl.den] = ss2tf(cl.A, cl.B, cl.C, cl.D);

cl.refCmdGain = cl.den(end) / cl.num(1,end);


%% Open and configure the simulink model

open_system('p2Sim.slx');  % Open the simulink model
set_param('p2Sim','SolverType',simPrm.solTyp);  % set this solver type in simulink parameters 
set_param('p2Sim','Solver',simPrm.sol);     % set this integration method in simulink solver

%% Simulate the model

SimOut = sim('p2Sim','SignalLoggingName','sdata');  % Simulate the model and save results in sdata

%% Extract the results for plotting and calculation
%Handy constants
UPID = 1;
YPID = 2;
Ref = 3; 
USF = 4;
YSF = 5;

simPrm.t = 0:Control.Z:simPrm.tEnd;  %control system update rate goes from 0 to 10

Results.Time = SimOut.tout;  % Grab the values of time
Results.YPID = SimOut.sdata{YPID}.Values.Data(:,1);  % Extract the results of actual position of PID controller
Results.YSF = SimOut.sdata{YSF}.Values.Data(:,1);  % Extract the results of actual position of SF controller
Results.Ref = SimOut.sdata{Ref}.Values.Data(:,1);  % Extract the results of reference position
Results.essPID = abs(Results.YPID(simPrm.tEnd/simPrm.dt,1)-Results.Ref(simPrm.tEnd/simPrm.dt,1))*100/Results.Ref(simPrm.tEnd/simPrm.dt,1);  % Steady state error of PID Controller
Results.essSF = abs(Results.YSF(simPrm.tEnd/simPrm.dt,1)-Results.Ref(simPrm.tEnd/simPrm.dt,1))*100/Results.Ref(simPrm.tEnd/simPrm.dt,1);  % Steady state error of SF Controller
Results.UPID = SimOut.sdata{UPID}.Values.Data(:,1); % Actuator force for PID Controller
Results.USF = SimOut.sdata{USF}.Values.Data(:,1);  % Actuator force for SF Controller

%% Calculate time domain characteristics

StepPID=stepinfo(Results.YPID,Results.Time,'SettlingTimethreshold',0.01);  % Step info of PID Controller
StepSF=stepinfo(Results.YSF,Results.Time,'SettlingTimethreshold',0.01); % Step info of SF Controller

%% Plot figures

figure(1)
myplot(simPrm.t, Results.UPID,'Actuator Response for finalized PID Controller Design',1,'r','Time(sec)','Force(N)')
xticks(0:1:10);

figure(2)
myplot(simPrm.t, Results.USF,'Actuator Response for finalized State Feedback Controller Design',1,'r','Time(sec)','Force(N)')
xticks(0:1:10);

figure(3)
myplot(Results.Time, Results.YPID,'Closed loop system Output vs Time',1,'r','Time(sec)','Position (Unit)')
hold on
yticks(0:0.1:1.2);
myplot(Results.Time, Results.YSF,'Closed loop system Output vs Time',1,'b','Time(sec)','Position (Unit)')
myplot(Results.Time, Results.Ref,'Closed loop system Output vs Time',1,'k','Time(sec)','Position (Unit)')
legend('Actual Position (PID)','Actual Position (SF controller)','Reference Position');

% Individual figures
% figure(1)
% myplot(Results.Time, Results.YSF,'Output of closed loop system for finalized State Feedback Controller Design',1,'b','Time(sec)','Position(unit)')
% hold on
% myplot(Results.Time, Results.Ref,'Output of closed loop system for finalized State Feedback Controller Design',1,'k','Time(sec)','Position(unit)')
% xticks(0:1:10);
% yticks(0:0.1:1.2);
% legend('Actual Position (SF controller)','Reference Position');

% figure(3)
% myplot(Results.Time, Results.YPID,'Output of closed loop system for finalized PID Controller Design',1,'b','Time(sec)','Position(unit)')
% hold on
% myplot(Results.Time, Results.Ref,'Output of closed loop system for finalized PID Controller Design',1,'k','Time(sec)','Position(unit)')
% xticks(0:1:10);
% yticks(0:0.1:1.2);
% legend('Actual Position (PID controller)','Reference Position');

%% Calculate Incentives

Inc.PID = incentives(StepPID.Overshoot,StepPID.SettlingTime,Results.essPID,Umax,Tau,Control.Z); % Incentive of PID Controller
Inc.SF = incentives(StepSF.Overshoot,StepSF.SettlingTime,Results.essSF,Umax,Tau,Control.Z);  % Incentive of SF Controller

%% Estimate good controller
goodctrl = max_incentive(Inc.PID,Inc.SF);  % Run the function of good controller

%% Anonymous functions
% 
% function for plotting the figures
function myplot(x,y,ttl,LW,color,xlbl,ylbl)
plot(x,y,color,'LineWidth',LW);
xticks(0:1:10);  % Give the x axis ticks
title(ttl); % Give the title.
xlabel(xlbl);
ylabel(ylbl);
end


% function for calculating the incentives
function incentives = incentives(os,ts,err,u,tau,tstepz)
if os <= 5
    os_in = round((5-os)*10);
else
    os_in = round((5-os)*10);
end

if ts <= 3
    ts_in = round((3-ts)*20);
else
    ts_in = round((3-ts)*20);
end

if err <= 5
    err_in = round((5-err)*20);
else
    err_in = round((5-err)*20);
end

if u <= 2.5
    u_penalty = 0;
elseif 2.5 < u <=4
    u_penalty = round((u - 2.5)*50);
else
    u_penalty = round((u - 2.5)*50);
end

if tau == 0.1
    tau_in = 0;
elseif 0.04 < tau < 0.1 
    tau_in = round((tau-0.1)*140);
elseif tau > 0.1
    tau_in = round((tau-0.1)*35);
else
    tau_in = 0;
end

if 0.001 <= tstepz < 0.01
    tstepz_incentive = -round((0.01-tstepz)*300);
elseif tstepz == 0.01
    tstepz_incentive = 0;
elseif tstepz < 0.001
    tstepz_incentive = 0;
else
    tstepz_incentive = -round((0.01-tstepz)*300);
end

total = os_in + ts_in + err_in - u_penalty + tstepz_incentive;
incentives = [total,os_in,ts_in,err_in,u_penalty,tau_in,tstepz_incentive];
end

% function for selecting best controller
% One which gives maximum incentives is the best controller

function goodctrl = max_incentive(incPID,incSF)

max_incentive = max(incPID(1),incSF(1));

if max_incentive == incPID(1)
    goodctrl = 'PID';
    fprintf('<strong>------------------------------------------------------------------------------</strong>\n');
    fprintf('<strong>Your total incentive for PID controller is $%f.</strong>\n',incPID(1));
    fprintf('<strong>------------------------------------------------------------------------------</strong>\n');
    fprintf('<strong>Your total incentive for SF controller is $%f.</strong>\n',incSF(1));
    fprintf('<strong>------------------------------------------------------------------------------</strong>\n');
    fprintf(2,'<strong>Based on maximum incentive criteria,your good controller of two designs is PID Controller.</strong>\n');
    fprintf('<strong>---------------------------------------------------------------------------</strong>\n');
    fprintf('<strong>Below are the details of individual incentives.</strong>\n');
    T = array2table([incPID(2),incSF(2);incPID(3),incSF(3);incPID(4),incSF(4);incPID(5),incSF(5);incPID(6),incSF(6);incPID(7),incSF(7);incPID(1),incSF(1)],...
        'VariableNames',{'PID','SF'},...
        'RowName',{'Overshoot Incentive ($)','Settling Time Incentive ($)','Steady State Error Incentive ($)','Actuator Incentive/penalty ($)',...
        'Sensor Time constant Incentive/penalty ($)','Controller update rate Incentive/penalty ($)','Total Incentive ($)'});
    disp(T)
    
else
    max_incentive == incSF(1);
    goodctrl = 'State Feedback';
    fprintf('<strong>------------------------------------------------------------------------------</strong>\n');
    fprintf('<strong>Your total incentive for PID controller is $%f.</strong>\n',incPID(1));
    fprintf('<strong>------------------------------------------------------------------------------</strong>\n');
    fprintf('<strong>Your total incentive for SF controller is $%f.</strong>\n',incSF(1));
    fprintf('<strong>------------------------------------------------------------------------------</strong>\n');
    fprintf(2,'<strong>Based on maximum incentive criteria,your good controller of two designs is State Feedback Controller.</strong>\n');
    fprintf('<strong>-----------------------------------------------------------------------------------------------------------</strong>\n');
    fprintf('<strong>Below are the details of individual incentives.</strong>\n');
    T = array2table([incPID(2),incSF(2);incPID(3),incSF(3);incPID(4),incSF(4);incPID(5),incSF(5);incPID(6),incSF(6);incPID(7),incSF(7);incPID(1),incSF(1)],...
        'VariableNames',{'PID','SF'},...
        'RowName',{'Overshoot Incentive ($)','Settling Time Incentive ($)','Steady State Error Incentive ($)','Actuator Incentive/penalty ($)',...
        'Sensor Time constant Incentive/penalty ($)','Controller update rate Incentive/penalty ($)','Total Incentive ($)'});
    disp(T)
    fprintf('<strong>---------------------------------------------------------------------------</strong>\n');

end

end
