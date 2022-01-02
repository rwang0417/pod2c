function [u_nom,x_nom,cost,train_time] = ilqr_arma_stochastic(...
    Model,...
    Task,...
    cur_state,...
    cur_u_traj,...
    horizon,...
    cost0)
% ilqr_arma_stochastic
%
% Description: Run arma-ilqr algorithm on mujoco robotics examples under noise
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
% $Date: December 7, 2021$
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

ite = 1;idx = 1;tic;noise_switch = 0;
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
        z = (cost_new-cost(ite-1))/delta_j;
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
state_error = norm(x_nom(1:2,end)-Task.xTarget(1:2));
[state_error cost_new]

% sysid - arma
delta_u=Task.ptb*1*randn(Model.nu*(horizon+1),Task.nSim);
delta_uact = zeros(Model.nu*(horizon+1),Task.nSim);
delta_x0=Task.statePtb*randn(Model.nsys,Task.nSim);

if strcmp(Model.name, 'fish')
    dx1 = delta_x0(4:7,:)+x_nom(4:7,1);
    for j=1:1:Task.nSim
        dx1(:,j) = dx1(:,j)./norm(dx1(:,j));
        delta_x0(4:7,j) = dx1(:,j)-x_nom(4:7,1);
    end
end

delta_z(Task.nm*horizon+1:Task.nm*(horizon+1),:)=Task.Ck*delta_x0;
e = zeros(Task.nm,Task.nSim,horizon+1);
dz_est = zeros(Task.nm,Task.nSim,horizon+1);
V_vio = Task.measurement_noise^2*eye(2);
V_camera = Task.measurement_noise^2*eye(Task.nm);
sum = 0;
for j=1:1:Task.nSim
    V_est = Task.measurement_noise^2*eye(Task.nm);
    x1 = x_nom(:,1)+delta_x0(:,j);
    z_vio = null(Task.Ck)'*x1+noise_switch*Task.measurement_noise*randn(Task.nm,1);
    mexstep('reset');
    mexstep('set','qpos',x1(1:Model.nq,1),Model.nq);
    mexstep('set','qvel',x1(Model.nq+1:Model.nsys,1),Model.nv);
    mexstep('forward');
    for i=1:1:horizon
        delta_uact(Model.nu*(horizon-i+1)+1:Model.nu*(horizon-i+2),j)=delta_u(Model.nu*(horizon-i+1)+1:Model.nu*(horizon-i+2),j);
        if i >= skip
            delta_uact(Model.nu*(horizon-i+1)+1:Model.nu*(horizon-i+2),j)=delta_u(Model.nu*(horizon-i+1)+1:Model.nu*(horizon-i+2),j)-1*K(:,:,i)*[delta_z(Task.nm*(horizon-i+1)+1:Task.nm*(horizon-i+1+Task.qx),1);delta_u(Model.nu*(horizon-i+2)+1:Model.nu*(horizon-i+1+Task.qu),j)];%
        end
        u_act = u_nom(:,i)+delta_uact(Model.nu*(horizon-i+1)+1:Model.nu*(horizon-i+2),j);
        mexstep('set','ctrl',u_act,Model.nu);
        mexstep('step',1);
        x2(1:Model.nq,1) = mexstep('get','qpos')';
        x2(Model.nq+1:Model.nsys,1) = mexstep('get','qvel')';
        x2 = x2+noise_switch*Task.training_noise*randn(Model.nsys,1);
        sum = sum + sumsqr(x2-x_nom(:,i+1));
        mexstep('set','qpos',x2(1:Model.nq,1),Model.nq);
        mexstep('set','qvel',x2(Model.nq+1:Model.nsys,1),Model.nv);
        mexstep('forward');
        z_camera = Task.Ck*x2+1*noise_switch*Task.measurement_noise*randn(Task.nm,1);
        z_predict = Task.A_kinematics*(delta_z(Task.nm*(horizon-i+1)+1:Task.nm*(horizon-i+2),j)+Z_NORM(:,i))+Task.B_kinematics*z_vio;
        L = (Task.A_kinematics*V_est*Task.A_kinematics'+Task.B_kinematics*V_vio*Task.B_kinematics')/(Task.A_kinematics*V_est*Task.A_kinematics'+Task.B_kinematics*V_vio*Task.B_kinematics'+V_camera);
        delta_z(Task.nm*(horizon-i)+1:Task.nm*(horizon-i+1),j)=z_camera-Z_NORM(:,i+1);%z_predict+L*(z_camera-z_predict)-Z_NORM(:,i+1);%
        z_vio = null(Task.Ck)'*x2+noise_switch*Task.measurement_noise*randn(Task.nm,1);
        V_est = (eye(size(L,1))-L)*(Task.A_kinematics*V_est*Task.A_kinematics'+Task.B_kinematics*V_vio*Task.B_kinematics');
        dz_est(:,j,i+1) = z_predict+L*(z_camera-z_predict)-Z_NORM(:,i+1);
        e(:,j,i+1)=z_camera-z_predict;
    end
end 

% arma fitting - least square: M1 * fitcoef = delta_y
for i=skip:1:horizon % skip the first few steps to wait for enough data
    e(:,:,i+1)*dz_est(:,:,i)';
    M1=[delta_z(Task.nm*(horizon-i+1)+1:Task.nm*(horizon-i+1+Task.qx),:);delta_uact(Model.nu*(horizon-i+1)+1:Model.nu*(horizon-i+Task.qu+1),:)];
    [Q,R]=qr(M1');
    fitcoef(:,:,i)=(R\Q'*delta_z(Task.nm*(horizon-i)+1:Task.nm*(horizon-i+1),:)')';
%         r(i)=sqrt(mean(mean((delta_z(Task.nm*(horizon-i)+1:Task.nm*(horizon-i+1),:)-fitcoef(:,:,i)*M1).^2,1))); % residual
end

% construct augmented Ak, Bk
for i=skip:1:horizon 
    bk = fitcoef(:,Task.nm*Task.qx+1:Task.nm*Task.qx+Model.nu,i)*K(:,:,i);
    A_r1 = [fitcoef(:,1:Task.nm*Task.qx,i),fitcoef(:,Task.nm*Task.qx+Model.nu+1:end,i)];%+bk;
    A_aug(:,:,i)=[A_r1;
      eye((Task.qx-1)*Task.nm),zeros((Task.qx-1)*Task.nm,Task.nm+Model.nu*(Task.qu-1));
      zeros(Model.nu*(Task.qu-1),Task.nm*Task.qx),[zeros(Model.nu,Model.nu*(Task.qu-1));eye(Model.nu*(Task.qu-2)) zeros(Model.nu*(Task.qu-2),Model.nu)]];
    B_aug(:,:,i)=[fitcoef(:,Task.nm*Task.qx+1:Task.nm*Task.qx+Model.nu,i);zeros(Task.nm*(Task.qx-1),Model.nu);eye(Model.nu*min(Task.qu-1,1));zeros(Model.nu*(Task.qu-2),Model.nu)];
end
traj_norm = sqrt(sum/horizon/Task.avg_num/Task.nSim)
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
    delta_j = delta_j + (Task.alpha*kt(:,i)'*Qu(:,i)+Task.alpha^2/2*kt(:,i)'*Quu(:,:,i)*kt(:,i));
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
if ite > 1
    noise_switch = 1;
end
if strcmp(Model.name, 's6') || strcmp(Model.name, 's3')
    criteria = (norm(x_nom(1:2,end)-Task.xTarget(1:2))>=0.03); % s6
elseif strcmp(Model.name, 'cartpole') || strcmp(Model.name, 'acrobot') || strcmp(Model.name, 'cheetah')
    criteria = (mean(conv_rate) > Task.converge); % cartpole
elseif strcmp(Model.name, 'fish')
    criteria = (norm(x_nom(1:3,end)-Task.xTarget(1:3))>=0.02); % fish
elseif strcmp(Model.name, 'pendulum')
%     criteria = (norm(x_nom(1:2,end)-Task.xTarget(1:2))>=0.02); % pendulum
    criteria = (ite < Task.maxIte); % pendulum
else
%     disp('Criteria not specified...')
    criteria = (ite < Task.maxIte);
end
end
train_time = toc;
end
