function [u_nom,x_nom,cost,train_time] = ilqr_lls(...
    Model,...
    Task,...
    cur_state,...
    cur_u_traj,...
    horizon,...
    cost0)
% ilqr_ls
%
% Description: Run ls-ilqr algorithm on mujoco robotics examples
%
% Inputs:
%     Model:		            Model infomation (structure)
%     Task:		                Task parameters (structure)
%     cur_state:		        Model states at current step (nq by stepNum double)
%     cur_u_traj:               Current nominal control for the future horizon(initial control,nu x stepNum double,optional)
%     horizon:                  Number of steps to optimize (int)
%     cost0:                    Initial cost (double, optional)
%
% Outputs:
%     u_nom:                    Optimized control (nu x stepNum,mat)
%     x_nom:                    State trajectory under optimized control (nq by stepNum double)
%     cost:                     Cost during optimization (1 by nite double)
%     trian_time:               Time taken for training (double)
%
% Example:                      [u_nom,x_nom,cost,trian_time] = ilqr_lss(pend,pendTask,pend.xInit,[],pend.stepNum,cost0);
%
% $Revision: R2020b$ 
% $Author: Ran Wang$
% $Date: March 21, 2021$
%------------------------------------------------------------------------------------------------------------

%% preprocess
z = 1;u_nom = cur_u_traj;
if rank(Task.Ck) < max(size(Task.Ck))
    error('ERROR: LLS needs full observation.')
end

%% variables
x_nom = zeros(Model.nsys,horizon+1);x_nom(:,1) = cur_state(:,1);
Sk = zeros(Model.nsys,Model.nsys,horizon+1);Sk(:,:,horizon+1) = Task.QT;
vk = zeros(Model.nsys,horizon+1);
K = zeros(Model.nu,Model.nsys,horizon);
Kv = zeros(Model.nu,Model.nsys,horizon);
Ku = zeros(Model.nu,Model.nu,horizon);
Quu = zeros(Model.nu,Model.nu,horizon);
kt  = zeros(Model.nu,horizon);
Aid = zeros(Model.nsys,Model.nsys,horizon);
Bid = zeros(Model.nsys,Model.nu,horizon);
criteria = true;
conv_rate = ones(3,1);

%% ILQR
ite = 1;idx = 1;tic;
while ite <= Task.maxIte && criteria
%% forward pass
forward_flag = true;
while forward_flag
    mexstep('reset');
    mexstep('set','qpos',cur_state(1:Model.nq,1),Model.nq);
    mexstep('set','qvel',cur_state(Model.nq+1:Model.nsys,1),Model.nv);
    mexstep('forward');
    cost_new = 0;x_new(:,1) = cur_state;u_new = u_nom;
    for i = 1:1:horizon
        u_new(:,i) = max(Model.uRange(1),min(Model.uRange(2),u_nom(:,i) - 1*K(:,:,i)*(x_new(:,i)-x_nom(:,i)) + Task.alpha*kt(:,i)));  
%         feedback(1,i,ite) = K(:,:,i)*(x_new(:,i)-x_nom(:,i));
%         nominal(1,i,ite) = kt(:,i);
        cost_new = cost_new + 0.5*(x_new(:,i)-Task.xTarget)'*Task.Q*(x_new(:,i)-Task.xTarget)+0.5*u_new(:,i)'*Task.R*u_new(:,i);
        mexstep('set','ctrl',u_new(:,i),Model.nu);
        mexstep('step',1);
        x_new(1:Model.nq,i+1)=mexstep('get','qpos')';
        x_new(Model.nq+1:Model.nsys,i+1)=mexstep('get','qvel')';
        x_new(:,i+1)=angle_normalize(Model,x_new(:,i+1));
    end
    cost_new = cost_new + 0.5*(x_new(:,horizon+1)-Task.xTarget)'*Task.QT*(x_new(:,horizon+1)-Task.xTarget);
    if ite > 1
        z = (cost(ite-1) - cost_new)/delta_j;
    end
    
    if (z >= 0 || Task.alpha < 10^-5)
        forward_flag = false;
        cost(ite) = cost_new;
        x_nom = x_new;
        u_nom = u_new;
        vk(:,horizon+1) = Task.QT*(x_nom(:,horizon+1)-Task.xTarget);
%         alpha=Task.alpha
        if Task.alpha<0.05
            Task.alpha=0.05;
        end
    else
        Task.alpha = 0.99*Task.alpha;
    end
end
x_traj_ite_ilqr(:,:,ite) = x_nom;
u_traj_ite_ilqr(:,:,ite) = u_nom;
% state_error = getStateError(Model,x_nom(:,end),Task.xTarget)
% [state_error cost_new]

% lsid
for i = 1:1:horizon
    delta_x1 = Task.ptb*1*randn(Model.nsys,Task.nSim);
    delta_u = Task.ptb*1*randn(Model.nu,Task.nSim);  
    delta_x2 = zeros(Model.nsys,Task.nSim);
    for j = 1:1:Task.nSim
        % plus
        mexstep('set','qpos',x_nom(1:Model.nq,i)+delta_x1(1:Model.nq,j),Model.nq);
        mexstep('set','qvel',x_nom(Model.nq+1:Model.nsys,i)+delta_x1(Model.nq+1:Model.nsys,j),Model.nv);
        mexstep('forward');
        mexstep('set','ctrl',u_nom(:,i)+delta_u(:,j),Model.nu);
        mexstep('step',1);
        delta_x2(1:Model.nq,j)=mexstep('get','qpos')';
        delta_x2(Model.nq+1:Model.nsys,j)=mexstep('get','qvel')';
        % minus
        mexstep('set','qpos',x_nom(1:Model.nq,i)-delta_x1(1:Model.nq,j),Model.nq);
        mexstep('set','qvel',x_nom(Model.nq+1:Model.nsys,i)-delta_x1(Model.nq+1:Model.nsys,j),Model.nv);
        mexstep('forward');
        mexstep('set','ctrl',u_nom(:,i)-delta_u(:,j),Model.nu);
        mexstep('step',1);
        delta_x2(1:Model.nq,j)=(delta_x2(1:Model.nq,j)-mexstep('get','qpos')')/2;
        delta_x2(Model.nq+1:Model.nsys,j)=(delta_x2(Model.nq+1:Model.nsys,j)-mexstep('get','qvel')')/2;
    end
%     [Q,R]=qr([delta_x1;delta_u]');
%     AB=(R\Q'*delta_x2')';
    AB = delta_x2*[delta_x1;delta_u]'/([delta_x1;delta_u]*[delta_x1;delta_u]');
    Aid(:,:,i) = AB(:,1:Model.nsys);
    Bid(:,:,i) = AB(:,Model.nsys+1:end);
end

%% backpass
delta_j = 0; 
for i = horizon:-1:1
    Quu(:,:,i) = Bid(:,:,i)'*Sk(:,:,i+1)*Bid(:,:,i)+Task.R;
    if min(eig(Quu(:,:,i))) <= 0
        disp('Quu is not positive definite')
    end

    kpreinv = inv(Quu(:,:,i));
    K(:,:,i) = kpreinv*Bid(:,:,i)'*Sk(:,:,i+1)*Aid(:,:,i);
    Kv(:,:,i) = kpreinv*Bid(:,:,i)';
    Ku(:,:,i) = kpreinv*Task.R;
    Sk(:,:,i) = Aid(:,:,i)'*Sk(:,:,i+1)*(Aid(:,:,i)-Bid(:,:,i)*K(:,:,i))+Task.Q;
    vk(:,i) = (Aid(:,:,i)-Bid(:,:,i)*K(:,:,i))'*vk(:,i+1)-K(:,:,i)'*Task.R*u_nom(:,i)+Task.Q*(x_nom(:,i)-Task.xTarget);
    kt(:,i) = - Kv(:,:,i)*vk(:,i+1) - Ku(:,:,i)*u_nom(:,i); Qu = Task.R*u_nom(:,i)+Bid(:,:,i)'*vk(:,i+1);
    delta_j = delta_j - (Task.alpha*kt(:,i)'*Qu+Task.alpha^2/2*kt(:,i)'*Quu(:,:,i)*kt(:,i));
end
if isempty(cost0)
    cost0 = cost(1);
end
if ite >= 2
    conv_rate(idx) = abs((cost(ite-1)-cost(ite))/cost0);
    idx = idx + 1;
end
if idx > size(conv_rate,1)
    idx = 1;
end
ite = ite + 1; 
if strcmp(Model.name, 's6') || strcmp(Model.name, 's3')
    criteria = (norm(x_nom(1:2,end)-Task.xTarget(1:2))>=0.03); % s6
elseif strcmp(Model.name, 'cartpole') || strcmp(Model.name, 'acrobot')% || strcmp(Model.name, 'pendulum')
    criteria = (mean(conv_rate) > Task.converge); % cartpole
elseif strcmp(Model.name, 'fish')
    criteria = (norm(x_nom(1:3,end)-Task.xTarget(1:3))>=0.02); % fish
elseif strcmp(Model.name, 'pendulum')
%     criteria = (norm(x_nom(1:2,end)-Task.xTarget(1:2))>=0.02); % pendulum
    criteria = (ite < Task.maxIte); % pendulum
else
    disp('Criteria not specified...')
end
end
train_time = toc;
% save('ite_result_ilqr_cart_diffinit.mat','cost','x_traj_ite_ilqr','u_traj_ite_ilqr','nominal','feedback');
end
