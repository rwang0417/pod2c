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
    Model.uRange = [-inf,inf];
    Model.nsys = Model.nq+Model.nv;
    Model.xInit = [pi;0;0;0];
%% cheetah
elseif strcmp(modelName, 'cheetah')
    Model.name = 'cheetah';
    Model.nq = 9;
    Model.nv = 9;
    Model.nu = 6;
    Model.file = 'cheetah.xml';
    Model.uRange = [-inf,inf];
    Model.nsys = Model.nq+Model.nv;
    Model.xInit = zeros(Model.nsys,1);
else
    error('Invalid or unregistered model name...');
end