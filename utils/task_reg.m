function [Task] = task_reg(Model)
% task_reg
%
% Description: Register task
%
% Inputs:
%     Model:                    Structure of model parameters containing:
%                                 * name: model name (string)
%                                 * nq: number of qpos (int)
%                                 * nv: number of qvel (int)
%                                 * nu: number of ctrl (int)
%                                 * nsys: sum of nq and nv (int)
%                                 * file: model filename (string)
%                                 * uRange: control bounds (double)
%                                 * xInit: initial states (double)
%
% Outputs:
%     Task:                     Structure of task parameters containing:
%                                 * qx: number of outputs stacked (int)
%                                 * qu: number of inputs stacked (int)
%                                 * nSim: number of rollouts for sysid (int)
%                                 * ptb: sysid noise std (double)
%                                 * Ck: output matrix (nm by nq double)
%                                 * nm: number of outputs (int)
%                                 * alpha: line search parameter (double)
%                                 * horizon: number of steps to optimize (int)
%                                 * converge: convergence stopping criteria (double)
%                                 * maxIte: max iteration allowed (int)
%                                 * xTarget: target state (nq by stepNum double)
%                                 * R: control cost parameter (nu by nu double)
%                                 * Q: incremental state cost parameter ((qx*nm+(qu-1)*nu) by (qx*nm+(qu-1)*nu) double)
%                                 * QT: terminal state cost parameter ((qx*nm+(qu-1)*nu) by (qx*nm+(qu-1)*nu) double)
%
% Example:                      fishTask = task_reg(fishModel)
%
% $Revision: R2020b$
% $Author: Ran Wang$
% $Date: February 17, 2021$
%------------------------------------------------------------------------------------------------------------

%% pendulum
if strcmp(Model.name, 'pendulum')
    Task.qx = 1;
    Task.qu = 1;
    Task.nSim = 40;
    Task.Ck = eye(Model.nsys);%[1 0];%
    Task.nm = size(Task.Ck, 1);
    Task.alpha = 1;
    Task.horizon = 30;
    Task.converge = 0.0001;
    Task.maxIte = 100;
    Task.avg_num = 100;
    Task.ptb = 0.0001;
    Task.statePtb = 0.0001;
    Task.training_noise = 0.0;
    Task.measurement_noise = 0;
    Task.xTarget = [0;0];
    Task.R = 2*10^0 * eye(Model.nu);
    Task.Q = 0*eye(Task.nm*Task.qx+Model.nu*(Task.qu-1));
    Task.QT = 0*eye(Task.nm*Task.qx+Model.nu*(Task.qu-1));
    Task.Q(1:Task.nm,1:Task.nm) = 1*Task.Ck*[1 0;0 0.1]*Task.Ck';
    Task.QT(1:Task.nm,1:Task.nm) = 100*eye(Task.nm);
elseif strcmp(Model.name, 'cartpole')
    Task.qx = 2;
    Task.qu = 2;
    Task.nSim = 400;%50
    Task.Ck = [1 0 0 0;0 1 0 0];%eye(Model.nsys);%
    Task.nm = size(Task.Ck, 1);
    Task.alpha = 0.1;
    Task.horizon = 30;
    Task.converge = 0.0000001;
    Task.maxIte = 200;
    Task.avg_num = 1;%100
    Task.avg_num_forward = 1;
    Task.training_noise = 0.0001;
    Task.measurement_noise = 0.000;
    Task.ptb = 0.01; % 0.01
    Task.statePtb = 0.001;
    Task.A_kinematics = eye(2);
    Task.B_kinematics = [0.1 0;0 0.1];
    Task.xTarget = [0;pi;0;0];
    Task.R = 100*10^-2 * eye(Model.nu);
    cx = 1*10^-1; cv = 1*10^-1;
    ctx = 5000*10^0; ctv = 1*10^5;
    Task.Q = 0*eye(Task.nm*Task.qx+Model.nu*(Task.qu-1));
    Task.QT = 0*eye(Task.nm*Task.qx+Model.nu*(Task.qu-1));
%     Task.Q(1:Task.nm*2,1:Task.nm*2) = [(cx+cv)*eye(Task.nm),-cv*eye(Task.nm);-cv*eye(Task.nm),cv*eye(Task.nm)];
%     Task.QT(1:Task.nm*2,1:Task.nm*2) = [(ctx+ctv)*eye(Task.nm),-ctv*eye(Task.nm);-ctv*eye(Task.nm),ctv*eye(Task.nm)];
    Task.Q(1:Task.nm,1:Task.nm) = cx*eye(Task.nm);
    Task.QT(1:Task.nm,1:Task.nm) = ctx*eye(Task.nm);
elseif strcmp(Model.name, 's3')
    Task.qx = 1;
    Task.qu = 1;
    Task.nSim = 30;
    Task.ptb = 0.00001;
    Task.Ck = eye(Model.nsys);%[[eye(3);zeros(7,3)] zeros(10,1) [zeros(3,1);1;zeros(6,1)] zeros(10,1) [zeros(4,1);1;zeros(5,1)] zeros(10,1) [zeros(5,1);1;zeros(4,1)] zeros(10,1) [zeros(6,1);1;zeros(3,1)] zeros(10,1) [zeros(7,1);1;zeros(2,1)] zeros(10,1) [zeros(8,1);1;0] zeros(10,1) [zeros(9,1);1] zeros(10,17)];%[eye(POS_NUM) zeros(POS_NUM,VEL_NUM) ];%
    Task.nm = size(Task.Ck, 1);
    Task.alpha = 0.3;
    Task.horizon = 950;
    Task.converge = 0.0001;
    Task.maxIte = 100;
    Task.statePtb = 0.00001;
    Task.xTarget = [0.5;-0.5;0;zeros(Model.nsys-3,1)];
    Task.R = 5*10^-2 * eye(Model.nu);
    Task.Q = 0*eye(Task.nm*Task.qx+Model.nu*(Task.qu-1));
    Task.QT = 0*eye(Task.nm*Task.qx+Model.nu*(Task.qu-1));
    Task.Q(1:2,1:2) = 8*10^0*[1 0;0 1];
    Task.QT(1:2,1:2) = 60*10^2*[1 0;0 1];
elseif strcmp(Model.name, 's6')
    Task.qx = 1;
    Task.qu = 1;
    Task.nSim = 30;
    Task.ptb = 0.00001;
    Task.Ck = eye(Model.nsys);%[[eye(3);zeros(7,3)] zeros(10,1) [zeros(3,1);1;zeros(6,1)] zeros(10,1) [zeros(4,1);1;zeros(5,1)] zeros(10,1) [zeros(5,1);1;zeros(4,1)] zeros(10,1) [zeros(6,1);1;zeros(3,1)] zeros(10,1) [zeros(7,1);1;zeros(2,1)] zeros(10,1) [zeros(8,1);1;0] zeros(10,1) [zeros(9,1);1] zeros(10,17)];%[eye(POS_NUM) zeros(POS_NUM,VEL_NUM) ];%
    Task.nm = size(Task.Ck, 1);
    Task.alpha = 0.3;
    Task.horizon = 900;
    Task.converge = 0.0001;
    Task.maxIte = 100;
    Task.statePtb = 0.001;
    Task.xTarget = [0.5;-0.6;0;zeros(Model.nsys-3,1)];
    Task.R = 1*10^-3 * eye(Model.nu);
    Task.Q = 0*eye(Task.nm*Task.qx+Model.nu*(Task.qu-1));
    Task.QT = 0*eye(Task.nm*Task.qx+Model.nu*(Task.qu-1));
    Task.Q(1:2,1:2) = 8*10^0*[1 0;0 1];
    Task.QT(1:2,1:2) = 10*10^2*[1 0;0 1];
elseif strcmp(Model.name, 's15')
    Task.qx = 2;%5
    Task.qu = 2;
    Task.nSim = 300;
    Task.ptb = 0.0001;
    Task.Ck = eye(Model.nsys);%[[eye(3);zeros(7,3)] zeros(10,1) [zeros(3,1);1;zeros(6,1)] zeros(10,1) [zeros(4,1);1;zeros(5,1)] zeros(10,1) [zeros(5,1);1;zeros(4,1)] zeros(10,1) [zeros(6,1);1;zeros(3,1)] zeros(10,1) [zeros(7,1);1;zeros(2,1)] zeros(10,1) [zeros(8,1);1;0] zeros(10,1) [zeros(9,1);1] zeros(10,17)];%eye(SYS_NUM);%[eye(POS_NUM) zeros(POS_NUM,VEL_NUM) ];%
    Task.nm = size(Task.Ck, 1);
    Task.alpha = 0.3;
    Task.horizon = 2400;
    Task.converge = 0.01;
    Task.maxIte = 150;
    Task.xTarget = [0.7;0.7;0;zeros(Model.nsys-3,1)];
    Task.R = 1*10^-3 * eye(Model.nu);
    Task.Q = 0*eye(Task.nm*Task.qx+Model.nu*(Task.qu-1));
    Task.QT = 0*eye(Task.nm*Task.qx+Model.nu*(Task.qu-1));
    Task.Q(1:2,1:2) = 8*[1 0;0 1];
    Task.QT(1:2,1:2) = 10000*[1 0;0 1];
elseif strcmp(Model.name, 'fish')
    Task.qx = 1;
    Task.qu = 1;
    Task.nSim = 40;
    Task.ptb = 0.00001;
    Task.statePtb = 0.00001;
    Task.Ck = eye(Model.nsys);%[eye(4) zeros(4,23); zeros(7,7) eye(7) zeros(7,13)];
    Task.nm = size(Task.Ck, 1);
    Task.alpha = 0.3;
    Task.horizon = 1200;
    Task.converge = 0.01;
    Task.maxIte = 40;
    Task.xTarget = [0;0.4;0.2;1;zeros(Model.nsys-4,1)];
    Task.R = 0.06 * eye(Model.nu);
    Task.Q = 0*eye(Task.nm*Task.qx+Model.nu*(Task.qu-1));
    Task.QT = 0*eye(Task.nm*Task.qx+Model.nu*(Task.qu-1));
    Task.Q(1:4,1:4) = 10*[2 0 0 0;0 2 0 0;0 0 2 0;0 0 0 0.1];
    Task.QT(1:4,1:4) = 10000*[2 0 0 0;0 2 0 0;0 0 2 0;0 0 0 0.3];
elseif strcmp(Model.name, 'acrobot')
    Task.qx = 1;
    Task.qu = 1;
    Task.nSim = 60;
    Task.ptb = 0.00001;
    Task.Ck = eye(Model.nsys);
    Task.nm = size(Task.Ck, 1);
    Task.alpha = 0.3;
    Task.horizon = 500;
    Task.converge = 0.001;
    Task.maxIte = 1000;
    Task.statePtb = 0.00001;
    Task.xTarget = [0;0;0;0];
    Task.R = 10*10^-1 * eye(Model.nu);
    cx = 2*10^0; cv = 1*10^-1;
    ctx = 15*10^2; ctv = 1*10^5;
    Task.Q = 0*eye(Task.nm*Task.qx+Model.nu*(Task.qu-1));
    Task.QT = 0*eye(Task.nm*Task.qx+Model.nu*(Task.qu-1));
    Task.Q(1:Task.nm,1:Task.nm) = cx*diag([1,0.05,0.01,0.01]);
    Task.QT(1:Task.nm,1:Task.nm) = ctx*diag([2,0.5,0.1,0.1]);
elseif strcmp(Model.name, 'cheetah')
    Task.qx = 1;
    Task.qu = 1;
    Task.nSim = 60;
    Task.ptb = 0.00001;
    Task.Ck = eye(Model.nsys);
    Task.nm = size(Task.Ck, 1);
    Task.alpha = 0.3;
    Task.horizon = 300;
    Task.converge = 0.001;
    Task.maxIte = 100;
    Task.statePtb = 0.00001;
    Task.xTarget = [zeros(Model.nq,1);10;0;zeros(Model.nv-2,1)]; 
    Task.R = 10*10^-1 * eye(Model.nu);
    cx = 1*10^1; cv = 1*10^-1;
    ctx = 1*10^1; ctv = 1*10^5;
    Task.Q = 0*eye(Task.nm*Task.qx+Model.nu*(Task.qu-1));
    Task.QT = 0*eye(Task.nm*Task.qx+Model.nu*(Task.qu-1));
    Task.Q(10:11,10:11) = cx * [1 0;0 1];
    Task.QT(10:11,10:11) = ctx * [1 0;0 1];
else
    error('No task registered for this model')
end
