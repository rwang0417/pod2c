function [error] = getStateError(...
    Model,...
    state1,...
    state2)
% getStateError
%
% Description: Calculate state error between current state and reference state
%
% Inputs:
%     Model:		            Model infomation (structure)
%     state1:                   State1 (nq x 1 double)
%     state2:                   State2 (nq x 1 double)
%
% Outputs:
%     error:                    Error = norm(state1-state2) (double)
%
% Example:                      [error] = getStateError(pend,pendTask,cur_state,pend.xTarget);
%
% $Revision: R2020b$ 
% $Author: Ran Wang$
% $Date: March 24, 2021$

if strcmp(Model.name, 's6') || strcmp(Model.name, 'pendulum') || strcmp(Model.name, 's3')
    error = norm(state1(1:2)-state2(1:2));
elseif strcmp(Model.name, 'cartpole')
    error = norm(state1(1:4)-state2(1:4));
elseif strcmp(Model.name, 'fish')
    error = norm(state1(1:3)-state2(1:3));
else
    DISP('Model not recognized by func getStateError');
end