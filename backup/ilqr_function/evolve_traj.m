function x_traj = evolve_traj(Model,x0,u_traj)
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
%     u_traj:		            Control trajectory (nu by horizon double)
%
% Outputs:
%     x_traj:                   State trajectory under given control (nq by horizon double)
%
% Example:                      x_nominal = evolve_traj(pendulum,pendulum.xInit,u_nominal)
%
% $Revision: R2020b$
% $Author: Ran Wang$
% $Date: May 13, 2021$
%------------------------------------------------------------------------------------------------------------

%% generate nominal state trajectory
horizon=size(u_traj,2);
x_traj = zeros(Model.nsys,horizon+1);x_traj(:,1)=x0;
mexstep('reset'); % reset all states and controls
mexstep('set','qpos',x0(1:Model.nq,1),Model.nq);
mexstep('set','qvel',x0(Model.nq+1:Model.nsys,1),Model.nv); % set initial states
mexstep('forward'); % update sites and sensors
for i = 1:1:horizon
    mexstep('set','ctrl',u_traj(:,i),Model.nu);
    mexstep('step',1);
    x_traj(1:Model.nq,i+1)=mexstep('get','qpos')';
    x_traj(Model.nq+1:Model.nsys,i+1)=mexstep('get','qvel')';
end
end