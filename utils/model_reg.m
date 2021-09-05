function [Model] = model_reg(modelName)
% model_reg
%
% Description: Register model info
%
% Inputs:
%     modelName:		        Model name (string), choose from:
%                                 * pendulum
%                                 * cartpole
%                                 * s3
%                                 * s6
%                                 * s15
%                                 * fish
%                                 * t2d1
%                                 * acrobot
%
% Outputs:
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
% Example:                      fish = model_reg('fish')
%
% $Revision: R2020b$ 
% $Author: Ran Wang$
% $Date: February 17, 2021$
%------------------------------------------------------------------------------------------------------------

%% check input
p = inputParser;
addRequired(p,'model_name',@(x)validateattributes(x,{'char'},{'nonempty'}))
parse(p,modelName)

%% pendulum
if strcmp(modelName, 'pendulum')
    Model.name = 'pendulum';
    Model.nq = 1;
    Model.nv = 1;
    Model.nu = 1;
    Model.file = 'pendulum.xml';
    Model.uRange = [-inf,inf];
    Model.nsys = Model.nq+Model.nv;
    Model.xInit = [pi;0];
%% cartpole
elseif strcmp(modelName, 'cartpole')
    Model.name = 'cartpole';
    Model.nq = 2;
    Model.nv = 2;
    Model.nu = 1;
    Model.file = 'cartpole.xml';
    Model.uRange = [-inf,inf];
    Model.nsys = Model.nq+Model.nv;
    Model.xInit = zeros(Model.nsys,1);
%% swimmer3
elseif strcmp(modelName, 's3')
    Model.name = 's3';
    Model.nq = 5;
    Model.nv = 5;
    Model.nu = 2;
    Model.file = 'swimmer3.xml';
    Model.uRange = [-10,10];
    Model.nsys = Model.nq+Model.nv;
    Model.xInit = zeros(Model.nsys,1);
%% swimmer6
elseif strcmp(modelName, 's6')
    Model.name = 's6';
    Model.nq = 8;
    Model.nv = 8;
    Model.nu = 5;
    Model.file = 'swimmer6.xml';
    Model.uRange = [-10,10];
    Model.nsys = Model.nq+Model.nv;
    Model.xInit = zeros(Model.nsys,1);
%% swimmer15
elseif strcmp(modelName, 's15')
    Model.name = 's15';
    Model.nq = 17;
    Model.nv = 17;
    Model.nu = 14;
    Model.file = 'swimmer15.xml';
    Model.uRange = [-inf,inf];
    Model.nsys = Model.nq+Model.nv;
    Model.xInit = zeros(Model.nsys,1);
%% fish
elseif strcmp(modelName, 'fish')
    Model.name = 'fish';
    Model.nq = 14;
    Model.nv = 13;
    Model.nu = 6;
    Model.file = 'fish.xml';
    Model.uRange = [-10,10];
    Model.nsys = Model.nq+Model.nv;
    Model.xInit = [0;0;0;1;0;0;0;zeros(Model.nsys-7,1)];
%% t2d1 3d
elseif strcmp(modelName, 't2d1')
    Model.name = 't2d1';
    Model.nq = 132;
    Model.nv = 99;
    Model.nu = 46;
    Model.nnode = 25;
    Model.file = 't2d1_3d.xml';
    Model.uRange = [-inf,-0.2];
    Model.nsys = Model.nq+Model.nv;
    Model.xInit = zeros(Model.nsys,1);
%% acrobot
elseif strcmp(modelName, 'acrobot')
    Model.name = 'acrobot';
    Model.nq = 2;
    Model.nv = 2;
    Model.nu = 1;
    Model.file = 'acrobot.xml';
    Model.uRange = [-10,10];
    Model.nsys = Model.nq+Model.nv;
    Model.xInit = [pi;0;0;0];
else
    error('Invalid or unregistered model name...');
end



% POS_NUM = 1;
% VEL_NUM = 1;
% SYS_NUM = 2;
% IN_NUM = 1;
% OUT_NUM= 2;
% MODEL = 'pendulum.xml';
% STEP_NUM = 30;
% X_INIT = [pi;0];
% X_TARGET = [0;0];
% Ck=eye(SYS_NUM);%[1 0];%
% % Ri = 10^-1 * eye(IN_NUM);
% % cx = 10^-1; cv = 1*10^-1;
% % ctx = 10^2; ctv = 10^2;

%% cartpole
% POS_NUM = 2;
% VEL_NUM = 2;
% SYS_NUM = 4;
% IN_NUM = 1;
% OUT_NUM = 2;
% STEP_NUM = 30;
% MODEL = 'cartpole.xml';
% X_INIT = [0;0;0;0];
% X_TARGET = [0;pi;0;0];
% Ck = [1 0 0 0;0 1 0 0];%eye(OUT_NUM);%

% ID_PERT_COEF = 0.01;
% q=qu=1
% Ri = 10^-3 * eye(IN_NUM);
% cx = 10^0; cv = 10^-1;
% ctx = 2*10^1; ctv = 10^2;
% Qi(1:OUT_NUM,1:OUT_NUM) = cx * [0.2 0 0 0;0 1 0 0;0 0 .2 0;0 0 0 .2];
% Qf(1:OUT_NUM,1:OUT_NUM) = ctx * [0.5 0 0 0;0 1 0 0;0 0 0.5 0;0 0 0 0.5];
% q=qu=2
% alpha = 0.2;
% TRIAL_NUM = 30;
% ITE_NUM = 40;
% STEP_DIV = 1; SKIPo = 1; SKIP = 300; 
% q = 2;
% qu = 2;
% Ri = 1*10^-2 * eye(IN_NUM);
% cx = 1*10^-1; cv = 1*10^-1;
% ctx = 100*10^0; ctv = 1*10^5;

%% swimmer3
% POS_NUM = 5;
% VEL_NUM = 5;
% SYS_NUM = 10;
% IN_NUM = 2;
% OUT_NUM = 10;
% MODEL = 'swimmer3.xml';
% STEP_NUM = 950;
% X_INIT = zeros(SYS_NUM,1);
% X_TARGET = [0.5;-0.5;pi/4;zeros(SYS_NUM-3,1)];
% Ck = eye(SYS_NUM);%[eye(POS_NUM) zeros(VEL_NUM,VEL_NUM) ];%

% q=qu=1
% ID_PERT_COEF = 0.1;
% Ri = 2*10^-2 * eye(IN_NUM);
% cx = 15*10^0; cv = 0;
% ctx = 18*10^2; ctv = 0;
% Qi(1:3,1:3) = cx * [1 0 0;0 1 0;0 0 0.0];
% Qf(1:3,1:3) = ctx * [1 0 0;0 1 0;0 0 0.0];
% alpha = 0.3; % 0.9 0.8 0.02
% q=qu=2
% ID_PERT_COEF = 0.01;
% Ri = 5*10^-2 * eye(IN_NUM);
% cx = 15*10^0; cv = 0;
% ctx = 18*10^2; ctv = 0;
% Qi(1:3,1:3) = cx * [1 0 0;0 1 0;0 0 0.0];
% Qf(1:3,1:3) = ctx * [1 0 0;0 1 0;0 0 0.0];
% alpha = 0.3; % 0.9 0.8 0.025

%% swimmer6
% POS_NUM = 8;
% VEL_NUM = 8;
% SYS_NUM = 16;
% IN_NUM = 5;
% OUT_NUM= 8;
% MODEL = 'swimmer6.xml';
% STEP_NUM = 900;
% X_INIT = zeros(SYS_NUM,1);
% X_TARGET = [0.5;-0.6;0;zeros(SYS_NUM-3,1)];
% Ck=[eye(POS_NUM) zeros(VEL_NUM,VEL_NUM) ];%eye(SYS_NUM);%[zeros(4,2) [1;0;0;0] zeros(4,1) [0;1;0;0] [0;0;1;0] zeros(4,1) [0;0;0;1] zeros(4,8)];%

% q=qu=1
% ID_PERT_COEF = 0.0001;
% TRIAL_NUM = 100;
% ITE_NUM = 100;
% STEP_DIV = 1; SKIPo = 50; SKIP = 300; 
% Ri = 2*10^-2 * eye(IN_NUM);
% cx = 8*10^0; cv = 2*10^0;
% ctx = 60*10^2; ctv = 1*10^3;
% Qi(1:3,1:3) = cx * [1 0 0;0 1 0;0 0 0.0];
% Qf(1:3,1:3) = ctx * [1 0 0;0 1 0;0 0 0.0];
% alpha = 0.3;
% q=qu=2 same

%% swimmer15
% POS_NUM = 17;
% VEL_NUM = 17;
% SYS_NUM = 34;
% IN_NUM = 14;
% OUT_NUM= 10;
% MODEL = 'swimmer15.xml';
% STEP_NUM = 2400;
% X_INIT = zeros(SYS_NUM,1);
% X_TARGET = [0.7;0.7;0;zeros(SYS_NUM-3,1)];
% Ck=[[eye(3);zeros(7,3)] zeros(10,1) [zeros(3,1);1;zeros(6,1)] zeros(10,1) [zeros(4,1);1;zeros(5,1)] zeros(10,1) [zeros(5,1);1;zeros(4,1)] zeros(10,1) [zeros(6,1);1;zeros(3,1)] zeros(10,1) [zeros(7,1);1;zeros(2,1)] zeros(10,1) [zeros(8,1);1;0] zeros(10,1) [zeros(9,1);1] zeros(10,17)];%eye(SYS_NUM);%[eye(POS_NUM) zeros(POS_NUM,VEL_NUM) ];%[zeros(8,2) [1;zeros(7,1)] zeros(8,1) [0;1;zeros(6,1)] zeros(8,1) [0;0;1;zeros(5,1)] zeros(8,1) [0;0;0;1;zeros(4,1)] zeros(8,1) [zeros(4,1);1;zeros(3,1)] zeros(8,1) [zeros(5,1);1;zeros(2,1)] zeros(8,1) [zeros(6,1);1;0] zeros(8,1) [zeros(7,1);1] zeros(8,17)];%[eye(POS_NUM) zeros(VEL_NUM,VEL_NUM) ];%[zeros(POS_NUM,POS_NUM) eye(VEL_NUM)];%

% q=qu=1,2
% ID_PERT_COEF = 0.0001;
% TRIAL_NUM = 200;
% ITE_NUM = 100;
% STEP_DIV = 1; SKIPo = 1; SKIP = 300; 
% Ri = 1*10^-3 * eye(IN_NUM);
% cx = 8*10^0; cv = 2*10^0;
% ctx = 100*10^2; ctv = 1*10^0;
% Qi(1:3,1:3) = cx * [1 0 0;0 1 0;0 0 0.0];
% Qf(1:3,1:3) = ctx * [1 0 0;0 1 0;0 0 0.0];
% alpha = 0.3;
% q=qu=5,not successful, need more ite
% ID_PERT_COEF = 0.01;
% TRIAL_NUM = 400;
% ITE_NUM = 300;
% Ri = 2*10^-2 * eye(IN_NUM);
% cx = 5*10^0; cv = 2*10^0;
% ctx = 120*10^2; ctv = 1*10^0;
% alpha=0.3 and start line search from 0.01 in training if alpha<0.01

%% fish
% POS_NUM = 14;
% VEL_NUM = 13;
% SYS_NUM = 27;
% IN_NUM = 6;
% OUT_NUM = 4;
% MODEL = 'fish.xml';
% STEP_NUM = 1200;
% X_INIT = [0;0;0;1;0;0;0;zeros(SYS_NUM-7,1)];
% X_TARGET = [0;0.4;0.2;1;zeros(SYS_NUM-4,1)];
% Ck=[eye(4) zeros(4,23)];%[eye(4) zeros(4,23); zeros(7,7) eye(7) zeros(7,13)];%[eye(POS_NUM) zeros(POS_NUM,VEL_NUM) ];%eye(SYS_NUM);%[zeros(7,7) eye(7) zeros(7,13)];%

% q=qu=1
% ID_PERT_COEF = 0.01;
% TRIAL_NUM = 400;
% ITE_NUM = 200;
% STEP_DIV = 1; SKIPo = 1; SKIP = 300; 
% Ri = 1*10^-2 * eye(IN_NUM);
% cx = 12*10^-1; cv = 2*10^0;
% ctx = 100*10^2; ctv = 1*10^0;
% Qi(1:4,1:4) = cx * [1 0 0 0;0 1 0 0;0 0 1 0;0 0 0 1];
% Qf(1:4,1:4) = ctx * [1 0 0 0;0 1 0 0;0 0 1 0;0 0 0 1];
% alpha=0.3;
% q=qu=2 pos only
% ID_PERT_COEF = 0.01;
% TRIAL_NUM = 400;
% ITE_NUM = 200;
% STEP_DIV = 1; SKIPo = 2; SKIP = 300; 
% Ri = 4*10^-2 * eye(IN_NUM);
% cx = 10*10^-1; cv = 2*10^0;
% ctx = 100*10^2; ctv = 1*10^0;
% Qi(1:4,1:4) = cx * [2 0 0 0;0 2 0 0;0 0 2 0;0 0 0 0.5];
% Qf(1:4,1:4) = ctx * [2 0 0 0;0 2 0 0;0 0 2 0;0 0 0 0.5];
% alpha=0.3;
% q=qu=2 xyz+q0+fin pos
% ID_PERT_COEF = 0.0001;
% TRIAL_NUM = 400;
% ITE_NUM = 300;
% STEP_DIV = 1; SKIPo = 2; SKIP = 300; 
% Ri = 4*10^-2 * eye(IN_NUM);
% cx = 2*10^0; cv = 2*10^0;
% ctx = 120*10^2; ctv = 1*10^0;
% Qi(1:4,1:4) = cx * [2 0 0 0;0 2 0 0;0 0 2 0;0 0 0 0.5];
% Qf(1:4,1:4) = ctx * [2 0 0 0;0 2 0 0;0 0 2 0;0 0 0 0.5];
% alpha=0.3 and start line search from 0.01 in training if alpha<0.01
% q=qu=7 xyz+q0
% ID_PERT_COEF = 0.0001;
% TRIAL_NUM = 400;
% ITE_NUM = 300;
% STEP_DIV = 1; SKIPo = 7; SKIP = 300; 
% Ri = 4*10^-2 * eye(IN_NUM);
% cx = 2*10^0; cv = 2*10^0;
% ctx = 120*10^2; ctv = 1*10^0;
% Qi(1:4,1:4) = cx * [2 0 0 0;0 2 0 0;0 0 2 0;0 0 0 0.5];
% Qf(1:4,1:4) = ctx * [2 0 0 0;0 2 0 0;0 0 2 0;0 0 0 0.5];
% alpha=0.3 and start line search from 0.01 in training if alpha<0.01
% Ck=[eye(4) zeros(4,23)];