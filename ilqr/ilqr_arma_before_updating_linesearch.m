function [u_nom,x_nom,cost,train_time] = ilqr_arma(...
    Model,...
    Task,...
    cur_state,...
    cur_u_traj,...
    horizon,...
    cost0)
% ilqr_arma
%
% Description: Run arma-ilqr algorithm on mujoco robotics examples
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
% Example:                      [u_nom,x_nom,cost,trian_time] = ilqr_arma(pend,pendTask,pend.xInit,[],pend.stepNum,cost0);
%
% $Revision: R2020b$ 
% $Author: Ran Wang$
% $Date: February 17, 2021$
%------------------------------------------------------------------------------------------------------------

%% preprocess
z = 1;skip = max(Task.qx,Task.qu);
u_nom = cur_u_traj;

%% variables
Z_NORM = zeros(Task.nm,horizon+1);Z_NORM(:,1) = Task.Ck*cur_state(:,1); 
dz_seq = zeros(Task.nm*(horizon+1),1);
du_seq = zeros(Model.nu*(horizon+1),1);
Sk = zeros(Task.nm*Task.qx+Model.nu*(Task.qu-1),Task.nm*Task.qx+Model.nu*(Task.qu-1),horizon+1);Sk(:,:,horizon+1) = Task.QT;
vk = zeros(Task.nm*Task.qx+Model.nu*(Task.qu-1),horizon+1);
K = zeros(Model.nu,Task.nm*Task.qx+Model.nu*(Task.qu-1),horizon);
Kv = zeros(Model.nu,Task.nm*Task.qx+Model.nu*(Task.qu-1),horizon);
Ku = zeros(Model.nu,Model.nu,horizon);
Quu = zeros(Model.nu,Model.nu,horizon);
Qu = zeros(Model.nu,horizon);
kt  = zeros(Model.nu,horizon);
fitcoef = zeros(Task.nm,Task.nm*Task.qx+Model.nu*Task.qu,horizon); 
A_aug = zeros(Task.nm*Task.qx+Model.nu*(Task.qu-1),Task.nm*Task.qx+Model.nu*(Task.qu-1),horizon);
B_aug = zeros(Task.nm*Task.qx+Model.nu*(Task.qu-1),Model.nu,horizon);
delta_z = zeros(Task.nm*(horizon+2),Task.nSim);
conv_rate = ones(2,1);
criteria = true;

ite = 1;idx = 1;tic;
while ite <= Task.maxIte && criteria
%% forward pass
forward_flag = true;
while forward_flag
    mexstep('reset');
    mexstep('set','qpos',cur_state(1:Model.nq,1),Model.nq);
    mexstep('set','qvel',cur_state(Model.nq+1:Model.nsys,1),Model.nv);
    mexstep('forward');
    cost_new = 0; x_new(:,1) = cur_state; u_new = u_nom;
    for i = 1:1:horizon
        if i >= skip
            du_seq(Model.nu*(horizon-i+1)+1:Model.nu*(horizon-i+2),1)=max(Model.uRange(1)-u_nom(:,i),min(Model.uRange(2)-u_nom(:,i),-K(:,:,i)*[dz_seq(Task.nm*(horizon-i+1)+1:Task.nm*(horizon-i+1+Task.qx),1);du_seq(Model.nu*(horizon-i+2)+1:Model.nu*(horizon-i+1+Task.qu),1)] + Task.alpha*kt(:,i)));
            u_new(:,i) = u_nom(:,i) + du_seq(Model.nu*(horizon-i+1)+1:Model.nu*(horizon-i+2),1);
            cost_new = cost_new + 0.5*[reshape(Task.Ck*(x_new(:,i:-1:i-Task.qx+1)-repmat(Task.xTarget,1,Task.qx)),Task.nm*Task.qx,1);reshape(u_new(:,i-1:-1:i-Task.qu+1),Model.nu*(Task.qu-1),1)]'*Task.Q*[reshape(Task.Ck*(x_new(:,i:-1:i-Task.qx+1)-repmat(Task.xTarget,1,Task.qx)),Task.nm*Task.qx,1);reshape(u_new(:,i-1:-1:i-Task.qu+1),Model.nu*(Task.qu-1),1)]+0.5*u_new(:,i)'*Task.R*u_new(:,i);
        end 
        mexstep('set','ctrl',u_new(:,i),Model.nu);
        mexstep('step',1);
        xt(1:Model.nq,1)=mexstep('get','qpos')';
        xt(Model.nq+1:Model.nsys,1)=mexstep('get','qvel')';
        dz_seq(Task.nm*(horizon-i)+1:Task.nm*(horizon-i+1),1)=Task.Ck*xt-Z_NORM(:,i+1);
        x_new(:,i+1) = xt;
    end
    cost_new = cost_new + 0.5*[reshape(Task.Ck*(x_new(:,horizon+1:-1:horizon+2-Task.qx)-repmat(Task.xTarget,1,Task.qx)),Task.nm*Task.qx,1);reshape(u_new(:,horizon:-1:horizon+2-Task.qu),Model.nu*(Task.qu-1),1)]'*Task.QT*[reshape(Task.Ck*(x_new(:,horizon+1:-1:horizon+2-Task.qx)-repmat(Task.xTarget,1,Task.qx)),Task.nm*Task.qx,1);reshape(u_new(:,horizon:-1:horizon+2-Task.qu),Model.nu*(Task.qu-1),1)];
    if ite > 1
        z = (cost(ite-1) - cost_new)/delta_j;
    end
    
    if (z >= -0.6 || Task.alpha < 10^-5)
        forward_flag = false;
        cost(ite) = cost_new;
        Z_NORM = Task.Ck*x_new;
        x_nom = x_new;
        u_nom = u_new;
        vk(:,horizon+1) = Task.QT*[reshape(Z_NORM(:,horizon+1:-1:horizon+2-Task.qx)-Task.Ck*repmat(Task.xTarget,1,Task.qx),Task.nm*Task.qx,1);reshape(u_nom(:,horizon:-1:horizon+2-Task.qu),Model.nu*(Task.qu-1),1)];
%         alpha=Task.alpha
        if Task.alpha<0.05
            Task.alpha=0.05;
        end
    else
        Task.alpha = 0.99*Task.alpha;
    end
end
x_terminal=x_nom;
state_error = norm(x_terminal(1:2,end)-Task.xTarget(1:2));
% [state_error cost_new]

% sysid - arma
delta_u=Task.ptb*1*randn(Model.nu*(horizon+1),Task.nSim);
delta_x0=Task.statePtb*randn(Model.nsys,Task.nSim);
if strcmp(Model.name, 'fish')
    dx1 = delta_x0(4:7,:)+x_nom(4:7,1);
    for j=1:1:Task.nSim
        dx1(:,j) = dx1(:,j)./norm(dx1(:,j));
        delta_x0(4:7,j) = dx1(:,j)-x_nom(4:7,1);
    end
end

delta_z(Task.nm*horizon+1:Task.nm*(horizon+1),:)=Task.Ck*delta_x0;
for j=1:1:Task.nSim
    mexstep('reset');
    mexstep('set','qpos',x_nom(1:Model.nq,1)+delta_x0(1:Model.nq,j),Model.nq);
    mexstep('set','qvel',x_nom(Model.nq+1:Model.nsys,1)+delta_x0(Model.nq+1:Model.nsys,j),Model.nv);
    mexstep('forward');
    for i=1:1:horizon
        mexstep('set','ctrl',u_nom(:,i)+delta_u(Model.nu*(horizon-i+1)+1:Model.nu*(horizon-i+2),j),Model.nu);
        mexstep('step',1);
        x2(1:Model.nq,1)=mexstep('get','qpos')';
        x2(Model.nq+1:Model.nsys,1)=mexstep('get','qvel')';
        delta_z(Task.nm*(horizon-i)+1:Task.nm*(horizon-i+1),j)=Task.Ck*x2-Z_NORM(:,i+1);
    end
end 

% arma fitting - least square: M1 * fitcoef = delta_y
for i=skip:1:horizon % skip the first few steps to wait for enough data
    M1=[delta_z(Task.nm*(horizon-i+1)+1:Task.nm*(horizon-i+1+Task.qx),:);delta_u(Model.nu*(horizon-i+1)+1:Model.nu*(horizon-i+Task.qu+1),:)];
    [Q,R]=qr(M1');
    fitcoef(:,:,i)=(R\Q'*delta_z(Task.nm*(horizon-i)+1:Task.nm*(horizon-i+1),:)')';
%         r(i)=sqrt(mean(mean((delta_z(Task.nm*(horizon-i)+1:Task.nm*(horizon-i+1),:)-fitcoef(:,:,i)*M1).^2,1))); % residual
end

% construct augmented Ak, Bk
for i=skip:1:horizon 
    A_aug(:,:,i)=[fitcoef(:,1:Task.nm*Task.qx,i),fitcoef(:,Task.nm*Task.qx+Model.nu+1:end,i);
      eye((Task.qx-1)*Task.nm),zeros((Task.qx-1)*Task.nm,Task.nm+Model.nu*(Task.qu-1));
      zeros(Model.nu*(Task.qu-1),Task.nm*Task.qx),[zeros(Model.nu,Model.nu*(Task.qu-1));eye(Model.nu*(Task.qu-2)) zeros(Model.nu*(Task.qu-2),Model.nu)]];
    B_aug(:,:,i)=[fitcoef(:,Task.nm*Task.qx+1:Task.nm*Task.qx+Model.nu,i);zeros(Task.nm*(Task.qx-1),Model.nu);eye(Model.nu*min(Task.qu-1,1));zeros(Model.nu*(Task.qu-2),Model.nu)];
end

%% backpass
delta_j = 0; 
for i = horizon:-1:skip
    Quu(:,:,i) = B_aug(:,:,i)'*Sk(:,:,i+1)*B_aug(:,:,i)+Task.R;
    if min(eig(Quu(:,:,i))) <= 0
        disp('Quu is not positive definite')
    end

    kpreinv = inv(Quu(:,:,i));
    K(:,:,i) = kpreinv*B_aug(:,:,i)'*Sk(:,:,i+1)*A_aug(:,:,i);
    Kv(:,:,i) = kpreinv*B_aug(:,:,i)';
    Ku(:,:,i) = kpreinv*Task.R;
    Sk(:,:,i) = A_aug(:,:,i)'*Sk(:,:,i+1)*(A_aug(:,:,i)-B_aug(:,:,i)*K(:,:,i))+Task.Q;
    vk(:,i) = (A_aug(:,:,i)-B_aug(:,:,i)*K(:,:,i))'*vk(:,i+1)-K(:,:,i)'*Task.R*u_nom(:,i)+Task.Q*[reshape(Z_NORM(:,i:-1:i-Task.qx+1)-Task.Ck*repmat(Task.xTarget,1,Task.qx),Task.nm*Task.qx,1);reshape(u_nom(:,i-1:-1:i-Task.qu+1),Model.nu*(Task.qu-1),1)];
    kt(:,i) = - Kv(:,:,i)*vk(:,i+1) - Ku(:,:,i)*u_nom(:,i); 
    Qu(:,i) = Task.R*u_nom(:,i)+B_aug(:,:,i)'*vk(:,i+1);
    delta_j = delta_j - (Task.alpha*kt(:,i)'*Qu(:,i)+Task.alpha^2/2*kt(:,i)'*Quu(:,:,i)*kt(:,i));
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
end
