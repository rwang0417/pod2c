% function ilqr_linear_case_stochastic
clear variables;clc;close all;tic;
%% model setup and variables
Model.nu=1;
Task.nm=1;
Model.uRange=[-inf,inf];
Model.xInit=[pi;0];
Model.nsys=2;
Model.name='linear';
Task.qx=2;
Task.qu=2;
Task.Ck=[1 0];%eye(Model.nsys);
Task.converge=0.0001;
Task.maxIte=20;%20
Task.horizon=30;
Task.xTarget=[0;0];
Task.statePtb = 0.01;
Task.ptb = 0.01;
Task.nSim = 1000;%200
Task.avg_num_id = 1;
Task.avg_num_forward = 1;
Task.training_noise = 0.001;
Task.measurement_noise = 0.005;
Task.R = 1*10^0 * eye(Model.nu);
Task.Q = 0*eye(Task.nm*Task.qx+Model.nu*(Task.qu-1));
Task.QT = 0*eye(Task.nm*Task.qx+Model.nu*(Task.qu-1));
Task.Q(1:Task.nm,1:Task.nm) = Task.Ck*[1 0;0 0.1]*Task.Ck';
Task.QT(1:Task.nm,1:Task.nm) = 100*eye(Task.nm);
Task.alpha = 0.3;
Ak = [1.002641490000000   0.099209440000000;0.026414880000000   0.992094400000000];
Bk = [0.08;0.8];
A = [1 0.1];
C_orth=[0 1];
u_init = 0*randn(Model.nu,Task.horizon);
x_traj = Model.xInit;cost0 = [];
z = 1;u_nom = u_init;skip = max(Task.qx,Task.qu);

%% variables
horizon = Task.horizon;
Z_NORM = zeros(Task.nm,horizon+1);Z_NORM(:,1) = Task.Ck*Model.xInit; 
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
A_aug_cl = zeros(Task.nm*Task.qx+Model.nu*(Task.qu-1),Task.nm*Task.qx+Model.nu*(Task.qu-1),horizon);
B_aug = zeros(Task.nm*Task.qx+Model.nu*(Task.qu-1),Model.nu,horizon);
delta_z = zeros(Task.nm*(horizon+2),Task.nSim);
conv_rate = ones(2,1);
criteria = true;
% x_est_id = zeros(Model.nsys,horizon+1,Task.avg_num_id); 

%% ILQR
ite = 1;idx = 1;tic;noise_switch = 0;
while ite <= Task.maxIte && criteria
%% forward pass
forward_flag = true;
while forward_flag
    cost_new = 0; x_new(:,1) = Model.xInit; u_new = u_nom;
    for i = 1:1:horizon
        if i >= skip
            du_seq(Model.nu*(horizon-i+1)+1:Model.nu*(horizon-i+2),1)=max(Model.uRange(1)-u_nom(:,i),min(Model.uRange(2)-u_nom(:,i),-K(:,:,i)*[dz_seq(Task.nm*(horizon-i+1)+1:Task.nm*(horizon-i+1+Task.qx),1);du_seq(Model.nu*(horizon-i+2)+1:Model.nu*(horizon-i+1+Task.qu),1)] + Task.alpha*kt(:,i)));
            u_new(:,i)=u_nom(:,i)+du_seq(Model.nu*(horizon-i+1)+1:Model.nu*(horizon-i+2),1);
            cost_new=cost_new+0.5*[reshape(Task.Ck*(x_new(:,i:-1:i-Task.qx+1)-repmat(Task.xTarget,1,Task.qx)),Task.nm*Task.qx,1);reshape(u_new(:,i-1:-1:i-Task.qu+1),Model.nu*(Task.qu-1),1)]'*Task.Q*[reshape(Task.Ck*(x_new(:,i:-1:i-Task.qx+1)-repmat(Task.xTarget,1,Task.qx)),Task.nm*Task.qx,1);reshape(u_new(:,i-1:-1:i-Task.qu+1),Model.nu*(Task.qu-1),1)]+0.5*u_new(:,i)'*Task.R*u_new(:,i);
        end 
        x_new(:,i+1)=Ak*x_new(:,i)+Bk*u_new(:,i);
        dz_seq(Task.nm*(horizon-i)+1:Task.nm*(horizon-i+1),1)=Task.Ck*x_new(:,i+1)-Z_NORM(:,i+1);
    end
    cost_new = cost_new + 0.5*[reshape(Task.Ck*(x_new(:,horizon+1:-1:horizon+2-Task.qx)-repmat(Task.xTarget,1,Task.qx)),Task.nm*Task.qx,1);reshape(u_new(:,horizon:-1:horizon+2-Task.qu),Model.nu*(Task.qu-1),1)]'*Task.QT*[reshape(Task.Ck*(x_new(:,horizon+1:-1:horizon+2-Task.qx)-repmat(Task.xTarget,1,Task.qx)),Task.nm*Task.qx,1);reshape(u_new(:,horizon:-1:horizon+2-Task.qu),Model.nu*(Task.qu-1),1)];
    if ite > 1
        z = -(cost(ite-1) - cost_new)/delta_j;
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
x_traj_ite_ilqr(:,:,ite) = x_nom;
u_traj_ite_ilqr(:,:,ite) = u_nom;
% state_error = getStateError(Model,x_nom(:,end),Task.xTarget)
% [state_error cost_new]

% arma-id
% for i = 1:1:horizon
%     A_aug(:,:,i) = Ak;
%     B_aug(:,:,i) = Bk;
% end

du_fb = zeros(Model.nu*(horizon+1),1);
delta_u=Task.ptb*randn(Model.nu*(horizon+1),Task.nSim);
delta_x0=Task.statePtb*randn(Model.nsys,Task.nSim);  
delta_z(Task.nm*horizon+1:Task.nm*(horizon+1),:) = Task.Ck*delta_x0+0*Task.measurement_noise*randn(Task.nm,1);
delta_x1_avg = zeros(Model.nsys,Task.nSim,horizon+1);
delta_x2_avg = zeros(Model.nsys,Task.nSim,horizon+1);
e = zeros(Model.nsys,Task.nSim,horizon+1);
zt = zeros(Model.nsys,Task.nSim,horizon+1);
delta_u_avg = zeros(Model.nu,Task.nSim,horizon);
sum = 0;
L = (A*Task.measurement_noise^2*A')/((A*Task.measurement_noise^2*A')+4*Task.measurement_noise^2);
for j=1:1:Task.nSim
    x1 = x_nom(:,1)+delta_x0(:,j)+noise_switch*Task.training_noise*randn(Model.nsys,1);
    z_vio = C_orth*x1+noise_switch*Task.measurement_noise*randn(Task.nm,1);
    for i=1:1:horizon
        if i >= skip
            du_fb(Model.nu*(horizon-i+1)+1:Model.nu*(horizon-i+2),1)=-K(:,:,i)*[delta_z(Task.nm*(horizon-i+1)+1:Task.nm*(horizon-i+1+Task.qx),1);du_fb(Model.nu*(horizon-i+2)+1:Model.nu*(horizon-i+1+Task.qu),1)];
        end
        u_act = u_nom(:,i)+delta_u(Model.nu*(horizon-i+1)+1:Model.nu*(horizon-i+2),j)+du_fb(Model.nu*(horizon-i+1)+1:Model.nu*(horizon-i+2),1);
        x2=Ak*x1+Bk*u_act+noise_switch*Task.training_noise*randn(Model.nsys,1);
        z_camera = Task.Ck*x2+2*noise_switch*Task.measurement_noise*randn(Task.nm,1);
        z_predict = A*[delta_z(Task.nm*(horizon-i+1)+1:Task.nm*(horizon-i+2),j)+Z_NORM(:,i);z_vio];
        delta_z(Task.nm*(horizon-i)+1:Task.nm*(horizon-i+1),j)=z_predict+L*(z_camera-z_predict)-Z_NORM(:,i+1);%z_camera-Z_NORM(:,i+1);%
%         delta_z(Task.nm*(horizon-i)+1:Task.nm*(horizon-i+1),j)=Task.Ck*x2-Z_NORM(:,i+1)+0*Task.measurement_noise*randn(Task.nm,1);
        z_vio = C_orth*x2+noise_switch*Task.measurement_noise*randn(Task.nm,1);
        x1=x2;
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
    bk = fitcoef(:,Task.nm*Task.qx+1:Task.nm*Task.qx+Model.nu,i)*K(:,:,i);
    A_r1 = [fitcoef(:,1:Task.nm*Task.qx,i),fitcoef(:,Task.nm*Task.qx+Model.nu+1:end,i)]+bk;
    A_aug(:,:,i)=[A_r1;
      eye((Task.qx-1)*Task.nm),zeros((Task.qx-1)*Task.nm,Task.nm+Model.nu*(Task.qu-1));
      zeros(Model.nu*(Task.qu-1),Task.nm*Task.qx),[zeros(Model.nu,Model.nu*(Task.qu-1));eye(Model.nu*(Task.qu-2)) zeros(Model.nu*(Task.qu-2),Model.nu)]];
    B_aug(:,:,i)=[fitcoef(:,Task.nm*Task.qx+1:Task.nm*Task.qx+Model.nu,i);zeros(Task.nm*(Task.qx-1),Model.nu);eye(Model.nu*min(Task.qu-1,1));zeros(Model.nu*(Task.qu-2),Model.nu)];
end
% for i=skip:1:horizon 
%     A_aug(:,:,i) = A_aug_cl(:,:,i)-B_aug(:,:,i)*K(:,:,i);
% end
%-
% for j = 1:1:Task.nSim
%     delta_x2 = zeros(Model.nsys,horizon,Task.avg_num_id);
%     delta_x1 = zeros(Model.nsys,horizon,Task.avg_num_id);
%     delta_uact = zeros(Model.nu,horizon,Task.avg_num_id);
%     for r = 1:1:Task.avg_num_id
%         x1 = x_nom(:,1)+delta_x0(:,j)+noise_switch*Task.training_noise*randn(Model.nsys,1);
%         delta_x1(:,1,r)=x1-x_nom(:,1)+noise_switch*Task.measurement_noise*randn(Model.nsys,1);
%         x_est_id(:,1,r)=x_nom(:,1);
%         for i = 1:1:horizon
%             if i >= 2
%                 e(:,j,i)=zm-Aid(:,:,i-1)*x_est_id(:,i-1,r)-Bid(:,:,i-1)*u_act;
%                 x_est_id(:,i,r) = Aid(:,:,i-1)*x_est_id(:,i-1,r)+Bid(:,:,i-1)*u_act+LK(:,:,i)*(zm-Aid(:,:,i-1)*x_est_id(:,i-1,r)-Bid(:,:,i-1)*u_act);
%                 delta_x2(:,i,r)=x_est_id(:,i,r)-x_nom(:,i);
%                 if noise_switch == 0 % >=
%                     delta_x2(:,i,r)=zm-x_nom(:,i);
%                 end
%                 delta_x1(:,i,r)=delta_x2(:,i,r);
%             end
%             u_act = u_nom(:,i)+delta_u(:,j,i)-K(:,:,i)*(x_est_id(:,i,r)-x_nom(:,i)); % with kalman filter
%             if noise_switch == 0 % >=
%                 u_act = u_nom(:,i)+delta_u(:,j,i)-K(:,:,i)*delta_x1(:,i,r);
%             end
%             delta_uact(:,i,r)=u_act-u_nom(:,i);
%             x2=Ak*x1+Bk*u_act+noise_switch*Task.training_noise*randn(Model.nsys,1);
%             zm=x2+noise_switch*Task.measurement_noise*randn(Model.nsys,1);
%             x1=x2;
%         end
%         x_est_id(:,horizon+1,r) = Aid(:,:,horizon)*x_est_id(:,horizon,r)+Bid(:,:,horizon)*u_act+LK(:,:,horizon+1)*(zm-Aid(:,:,horizon)*x_est_id(:,horizon,r)-Bid(:,:,horizon)*u_act);       
%         delta_x2(:,horizon+1,r)=x_est_id(:,horizon+1,r)-x_nom(:,horizon+1);%
%         if noise_switch == 0 % >=
%             delta_x2(:,horizon+1,r)=zm-x_nom(:,horizon+1);
%         end
%         delta_x1(:,horizon+1,r)=delta_x2(:,horizon+1,r);
%     end
%     sum=sum+sumsqr(delta_x1);
%     delta_x1_avg(:,j,:)=mean(delta_x1,3);
%     delta_x2_avg(:,j,:)=mean(delta_x2,3);
%     delta_u_avg(:,j,:)=mean(delta_uact,3);
% end
% 
% for i = 1:1:horizon
%     if i > 2
%         e(:,:,i+1)*delta_x1_avg(:,:,i)'
%     end
%     AB = delta_x2_avg(:,:,i+1)*[delta_x1_avg(:,:,i);delta_u(:,:,i)]'/([delta_x1_avg(:,:,i);delta_u(:,:,i)]*[delta_x1_avg(:,:,i);delta_u(:,:,i)]');
%     Bid(:,:,i) = AB(:,Model.nsys+1:end);
%     Aid(:,:,i) = AB(:,1:Model.nsys)+1*Bid(:,:,i)*K(:,:,i);
% %     AB = delta_x2_avg(:,:,i+1)*[delta_x1_avg(:,:,i);delta_u_avg(:,:,i)]'/([delta_x1_avg(:,:,i);delta_u_avg(:,:,i)]*[delta_x1_avg(:,:,i);delta_u_avg(:,:,i)]');
% %     Bid(:,:,i) = AB(:,Model.nsys+1:end);
% %     Aid(:,:,i) = AB(:,1:Model.nsys);   
% end
% traj_norm = sqrt(sum/Model.nsys/horizon/Task.avg_num_id/Task.nSim)

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
% Wi = 1e0*eye(Model.nsys);
% Vi = 1e-2*eye(Task.nm);
% for i= 1:1:horizon
%     PS(:, :, i+1) = Aid(:,:,i) * (PS(:, :, i) - PS(:, :, i) / (PS(:, :, i) + Vi) * PS(:, :, i)) * Aid(:,:,i)' + Wi;
%     LK(:, :, i) = PS(:, :, i)/ (PS(:, :, i) + Vi);
% end
% LK(:, :, horizon+1) = PS(:, :, horizon+1)/(PS(:, :, horizon+1) + Vi);
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
if strcmp(Model.name, 'linear')
    criteria = (ite < Task.maxIte); 
end
end
toc;
terminal_cost = cost(end)
% save('ite_result_ilqr_linear.mat','cost','x_traj_ite_ilqr','u_traj_ite_ilqr','nominal','feedback');

%% plot
figure;
subplot(1,3,1)
plot([x_nom(1,:)', x_nom(2,:)'])
xlabel('step')
ylabel('state')

subplot(1,3,2)
plot(0:1:size(u_nom,2)-1, u_nom(1,:))
xlabel('step')
ylabel('u')

subplot(1,3,3)
plot(0:1:size(cost,2)-1, cost)
xlabel('iteration')
ylabel('cost')

