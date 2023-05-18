close all; clear; clc;
%================== MPC Parameters ================
% Create a nonlinear MPC controller with one state, one output, and one input control.
nx = 7; %Psi, e_R, e_omega
nu = 3; % tau (M)
ny = 7; %Psi, e_R, e_omega
nlobj = nlmpc(nx,ny,nu);

% Names of the states and control input
nlobj.States(1).Name = 'Psi';
nlobj.States(2).Name = 'e_R1';
nlobj.States(3).Name = 'e_R2';
nlobj.States(4).Name = 'e_R3';
nlobj.States(5).Name = 'e_omega_1';
nlobj.States(6).Name = 'e_omega_2';
nlobj.States(7).Name = 'e_omega_3';

nlobj.MV(1).Name     = 'tau_1';
nlobj.MV(2).Name     = 'tau_2';
nlobj.MV(3).Name     = 'tau_3';

% Specify the sample time and horizons of the controller.
Ts = 0.05;
p  = 10;
nlobj.Ts                       = Ts;
nlobj.PredictionHorizon        = p;
nlobj.ControlHorizon           = 5;

%This is necesary to use more parameters in the StateFunction 
% R_d, R, R_d_dot, R_d_ddot, omega,J,J_inv
nlobj.Model.NumberOfParameters = 7;

%Weights of the objective function
nlobj.Weights.OutputVariables          = [10 10 10 10 10 10 10];
% nlobj.Weights.ManipulatedVariablesRate = [0.0 0.0 0.0];
% nlobj.ManipulatedVariables(1).Max  = 50;
% nlobj.ManipulatedVariables(2).Max  = 50;
% nlobj.ManipulatedVariables(3).Max  = 50;
% nlobj.ManipulatedVariables(1).RateMin  = -1;
% nlobj.ManipulatedVariables(1).RateMax  = 50;

% nlobj.States(1).Min = -1; %zdot min
% nlobj.States(1).Max = 1; %zdot max
% nlobj.States(2).Min = 0;   %gamma min
% nlobj.States(2).Max = 0.; %gamma max
% nlobj.ManipulatedVariables(1).Min = 0;
%nlobj.Weights.ManipulatedVariables     = [0.0 0.0 0.0];

%Specify the state function for the controller
nlobj.Model.StateFcn         = "StateFunction";
nlobj.Model.IsContinuousTime = true;

% Specify the output function for the controller.
nlobj.Model.OutputFcn = "OutputFunction";

%Speficy optimization paremeters.
nlobj.Optimization.CustomCostFcn         = "CostFunctionSimulink";
nlobj.Optimization.ReplaceStandardCost   = false;
%nlobj.Optimization.CustomIneqConFcn      = "IneqConFunction";
nlobj.Optimization.SolverOptions.MaxIter = 1000;
nlobj.Optimization.UseSuboptimalSolution = true;

%=========== Scale Factors ==============
% nlobj.OutputVariables(1).ScaleFactor = 1e-2;
% nlobj.ManipulatedVariables(1).ScaleFactor = 1e-2;
% nlobj.ManipulatedVariables(2).ScaleFactor = 1e-2;
% nlobj.ManipulatedVariables(3).ScaleFactor = 1e-2;
% nlobj.States(1).ScaleFactor = 1e0;
 

R_d      = eye(3);
R        = eye(3); 
R_d_dot  = eye(3); 
R_d_ddot = eye(3);
omega    = ones(3,1);
J        = diag([22, 20, 23]);
J_inv    = inv(J);
Mg       = ones(3,1);


% % Validate the prediction model functions for nominal states x0 and nominal inputs u0. 
% x0 = [1 1 1 1 1 1];
% u0 = [-5 5 5];
% validateFcns(nlobj, x0, u0, [],{R_d, R, R_d_dot, R_d_ddot, omega,J,J_inv,Mg});
%=================== MPC Parameters =================



% %================ Conexion with simulink ===========
mdl  = 'MPC_Satellite/Position Controlled Flight Mode ';
open_system(mdl);
createParameterBus(nlobj,[mdl '/Nonlinear MPC Controller'],'myBusObject',{R_d, R, R_d_dot, R_d_ddot, omega,J,J_inv});
