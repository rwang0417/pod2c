function xt_norm = angle_normalize(Model,xt)
% angle_normalize
%
% Description: Normalize the joint angle for acrobot
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
%     xt:		                Actual state (nq by 1 double)
%
% Outputs:
%     xt_norm:                  Normalized state (nq by 1 double)
%
% Example:                      x = angle_normalize(pendulum,xt)
%
% $Revision: R2020b$
% $Author: Ran Wang$
% $Date: May 27, 2021$
%------------------------------------------------------------------------------------------------------------

%% normalize angles
if strcmp(Model.name, 'acrobot')
    xt_norm=xt;
    xt_norm(1:2,1)=pi-mod(abs(pi-xt(1:2,1)),2*pi);
else
    xt_norm=xt;
end
end