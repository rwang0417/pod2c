function cost = episodic_cost(Model,Task,u_traj)
% episodic_cost
%
% Description: Calculate episodic cost given the control trajectory
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
%     u_traj:		            Control trajectory (nu by horizon double)
%
% Outputs:
%     cost:                     Episodic cost (double)
%
% Example:                      cost = episodic_cost(pendulum,pendulumTask,u_traj)
%
% $Revision: R2020b$
% $Author: Ran Wang$
% $Date: May 13, 2021$
%----------------

cost=0;
x_traj = evolve_traj(Model,Model.xInit,u_traj);

for i=1:1:Task.horizon
    cost=cost+0.5*(Task.Ck*(x_traj(:,i)-Task.xTarget))'*Task.Q(1:Task.nm,1:Task.nm)*(Task.Ck*(x_traj(:,i)-Task.xTarget))+0.5*u_traj(:,i)'*Task.R*u_traj(:,i);
end
cost=cost+0.5*(Task.Ck*(x_traj(:,Task.horizon+1)-Task.xTarget))'*Task.QT(1:Task.nm,1:Task.nm)*(Task.Ck*(x_traj(:,Task.horizon+1)-Task.xTarget));
