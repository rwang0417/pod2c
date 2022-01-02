function [x_traj,u_traj] = evolve_traj(Model,x0,u_nominal,x_ref,MTK,process_noise)
% evolve_traj
%
% Description: Generate state trajectory given the control trajectory.
%              Load model before calling this function.
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
%     x0:		                Initial state (nq by horizon double)
%     u_nominal:		        Control trajectory (nu by horizon double)
%     x_ref:		            Reference state trajectory for feedback (nsys by horizon+1 double), default 0
%     MTK:		                Feedback gain (nu by nsys by horizon double), default 0
%     process_noise:		    Std of process noise (double), default 0
%
% Outputs:
%     x_traj:                   State trajectory under given control (nsys by horizon+1 double)
%     u_traj:                   Control trajectory under given control (nu by horizon double)
%
% Example:                      x_nominal = evolve_traj(pendulum,pendulum.xInit,u_nominal)
%
% $Revision: R2020b$
% $Author: Ran Wang$
% $Date: May 13, 2021$
%------------------------------------------------------------------------------------------------------------

%% args
horizon=size(u_nominal,2);
if nargin == 3
    x_ref = zeros(Model.nsys,horizon+1);
    MTK = zeros(Model.nu,Model.nsys,horizon);
    process_noise = 0;
end

%% generate nominal state trajectory
x_traj = zeros(Model.nsys,horizon+1);x_traj(:,1)=x0;
u_traj = zeros(Model.nu,horizon);
mexstep('reset'); % reset all states and controls
mexstep('set','qpos',x0(1:Model.nq,1),Model.nq);
mexstep('set','qvel',x0(Model.nq+1:Model.nsys,1),Model.nv); % set initial states
mexstep('forward'); % update sites and sensors
for i = 1:1:horizon
    u_traj(:,i) = max(Model.uRange(1),min(Model.uRange(2),u_nominal(:,i)-MTK(:,:,i)*(x_traj(:,i)-x_ref(:,i))+process_noise*randn(Model.nu,1)));
    mexstep('set','ctrl',u_traj(:,i),Model.nu);
    mexstep('step',1);
    x_traj(1:Model.nq,i+1)=mexstep('get','qpos')';
    x_traj(Model.nq+1:Model.nsys,i+1)=mexstep('get','qvel')';
end
end