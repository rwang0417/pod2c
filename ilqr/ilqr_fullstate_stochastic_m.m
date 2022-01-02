clear variables;clc;tic;
%% model setup and variables
Model = model_reg('cartpole');
Task = task_reg(Model);
mexstep('load',['./model/' Model.file]);
u_init = 0.0*rand(Model.nu,Task.horizon);
x_traj = Model.xInit;cost0 = [];
z = 1;u_nom = u_init;

%% variables
x_nom = zeros(Model.nsys,Task.horizon+1);x_nom(:,1) = x_traj(:,1);
Sk = zeros(Model.nsys,Model.nsys,Task.horizon+1);Sk(:,:,Task.horizon+1) = Task.QT;
vk = zeros(Model.nsys,Task.horizon+1);
K = zeros(Model.nu,Model.nsys,Task.horizon);
Kv = zeros(Model.nu,Model.nsys,Task.horizon);
Ku = zeros(Model.nu,Model.nu,Task.horizon);
Quu = zeros(Model.nu,Model.nu,Task.horizon);
kt  = zeros(Model.nu,Task.horizon);
Aid = zeros(Model.nsys,Model.nsys,Task.horizon);
Bid = zeros(Model.nsys,Model.nu,Task.horizon);
PS = zeros(Task.nm, Model.nsys, Task.horizon+1);
LK = zeros(Task.nm, Model.nsys, Task.horizon+1);
criteria = true;
conv_rate = ones(3,1);
horizon = Task.horizon;
x_new = zeros(Model.nsys,horizon+1,Task.avg_num_forward);
u_new = zeros(Model.nu,horizon,Task.avg_num_forward);
x_est_forward = zeros(Model.nsys,horizon+1,Task.avg_num_forward);
x_est_id = zeros(Model.nsys,horizon+1,Task.avg_num); 
z_forward = zeros(Model.nsys,horizon+1,Task.avg_num_forward);
system_error = zeros(1,Task.maxIte);
AB_ = zeros(Model.nsys,Model.nsys+Model.nu,horizon+1);

%% ILQR
ite = 1;idx = 1;tic;noise_switch = 0;
while ite <= Task.maxIte && criteria
    if ite == Task.maxIte
        Task.training_noise=0;
    end
%% forward pass
forward_flag = true;
while forward_flag
    for r = 1:1:Task.avg_num_forward
        x_new(:,1,r) = Model.xInit+0*Task.training_noise*randn(Model.nsys,1);
        z_forward(:,1,r) = x_new(:,1,r)+0*Task.measurement_noise*randn(Model.nsys,1);
        u_new(:,:,r) = u_nom;
        x_est_forward(:,1,r) = Model.xInit;
        mexstep('reset');
        mexstep('set','qpos',x_new(1:Model.nq,1,r),Model.nq);
        mexstep('set','qvel',x_new(Model.nq+1:Model.nsys,1,r),Model.nv);
        mexstep('forward');
        for i = 1:1:horizon
            if i >= 2
                x_est_forward(:,i,r) = Aid(:,:,i-1)*x_est_forward(:,i-1,r)+Bid(:,:,i-1)*u_new(:,i-1,r)+LK(:,:,i)*(z_forward(:,i,r)-Aid(:,:,i-1)*x_est_forward(:,i-1,r)-Bid(:,:,i-1)*u_new(:,i-1,r));
            end
            u_new(:,i,r) = max(Model.uRange(1),min(Model.uRange(2),u_nom(:,i) - K(:,:,i)*(x_est_forward(:,i,r)-x_nom(:,i)) + Task.alpha*kt(:,i)));
            if noise_switch >= 0
                u_new(:,i,r) = max(Model.uRange(1),min(Model.uRange(2),u_nom(:,i) - K(:,:,i)*(z_forward(:,i,r)-x_nom(:,i)) + Task.alpha*kt(:,i)));
            end
%             u_new(:,i,r) = max(Model.uRange(1),min(Model.uRange(2),u_nom(:,i) - K(:,:,i)*(z_forward(:,i,r)-x_nom(:,i)) + Task.alpha*kt(:,i)));
            mexstep('set','ctrl',u_new(:,i,r),Model.nu);
            mexstep('step',1);
            xt(1:Model.nq,1) = mexstep('get','qpos')';
            xt(Model.nq+1:Model.nsys,1) = mexstep('get','qvel')';
            x_new(:,i+1,r) = xt+0*Task.training_noise*randn(Model.nsys,1);
            z_forward(:,i+1,r) = x_new(:,i+1,r)+0*Task.measurement_noise*randn(Model.nsys,1);
            mexstep('set','qpos',x_new(1:Model.nq,i+1,r),Model.nq);
            mexstep('set','qvel',x_new(Model.nq+1:Model.nsys,i+1,r),Model.nv);
            mexstep('forward');
        end
        x_est_forward(:,horizon+1,r) = Aid(:,:,horizon)*x_est_forward(:,horizon,r)+Bid(:,:,horizon)*u_new(:,horizon,r)+LK(:,:,horizon+1)*(z_forward(:,horizon+1,r)-Aid(:,:,horizon)*x_est_forward(:,horizon,r)-Bid(:,:,horizon)*u_new(:,horizon,r));
    end
    x_avg = mean(z_forward,3);
    u_avg = mean(u_new,3);
    cost_new = 0;
    for i = 1:1:horizon
        cost_new = cost_new + 0.5*(x_avg(:,i)-Task.xTarget)'*Task.Q*(x_avg(:,i)-Task.xTarget)+0.5*u_avg(:,i)'*Task.R*u_avg(:,i);
    end
    cost_new = cost_new + 0.5*(x_avg(:,horizon+1)-Task.xTarget)'*Task.QT*(x_avg(:,horizon+1)-Task.xTarget);
    if ite > 1
        z = (cost(ite-1) - cost_new)/delta_j;
    end
    
    if (z >= -0.6 || Task.alpha < 10^-5)
        forward_flag = false;
        cost(ite) = cost_new;
        x_nom = x_avg;
        u_nom = u_avg;
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
state_error(ite) = getStateError(Model,x_nom(:,end),Task.xTarget);
% [state_error cost_new]

% lsid
%%%%%%%%%%%%%%%%%%%%%%%%% self converge (0 correlation)
% delta_x0 = Task.statePtb*1*randn(Model.nsys,Task.nSim);
% delta_u = Task.ptb*1*randn(Model.nu,Task.nSim,horizon);  
% delta_x1_avg = zeros(Model.nsys,Task.nSim,horizon+1);
% delta_x2_avg = zeros(Model.nsys,Task.nSim,horizon+1);
% delta_u_avg = zeros(Model.nu,Task.nSim,horizon);
% sum = 0;
% for j = 1:1:Task.nSim
%     delta_x2 = zeros(Model.nsys,horizon,Task.avg_num);
%     delta_x1 = zeros(Model.nsys,horizon,Task.avg_num);
%     delta_uact = zeros(Model.nu,horizon,Task.avg_num);
%     for r = 1:1:Task.avg_num
%         x1 = x_nom(:,1)+delta_x0(:,j)+noise_switch*Task.training_noise*randn(Model.nsys,1);
%         delta_x1(:,1,r)=x1-x_nom(:,1)+noise_switch*Task.measurement_noise*randn(Model.nsys,1);
%         x_est_id(:,1,r)=x_nom(:,1);
%         mexstep('reset');
%         mexstep('set','qpos',x1(1:Model.nq,1),Model.nq);
%         mexstep('set','qvel',x1(Model.nq+1:Model.nsys,1),Model.nv);
%         mexstep('forward');
%         for i = 1:1:horizon
%             if i >= 2
%                 x_est_id(:,i,r) = Aid(:,:,i-1)*x_est_id(:,i-1,r)+Bid(:,:,i-1)*u_act+LK(:,:,i)*(zm-Aid(:,:,i-1)*x_est_id(:,i-1,r)-Bid(:,:,i-1)*u_act);
%                 delta_x2(:,i,r)=x_est_id(:,i,r)-x_nom(:,i);
%                 if noise_switch >= 0 % >=
%                     delta_x2(:,i,r)=zm-x_nom(:,i);
%                 end
%                 delta_x1(:,i,r)=delta_x2(:,i,r);
%             end
%             u_act = u_nom(:,i)+delta_u(:,j,i)-K(:,:,i)*(x_est_id(:,i,r)-x_nom(:,i)); % with kalman filter
%             if noise_switch >= 0 % >=
%                 u_act = u_nom(:,i)+delta_u(:,j,i)-K(:,:,i)*delta_x1(:,i,r);
%             end
%             delta_uact(:,i,r)=u_act-u_nom(:,i);
%             mexstep('set','ctrl',u_act,Model.nu);
%             mexstep('step',1);
%             x2(1:Model.nq,1)=mexstep('get','qpos')';
%             x2(Model.nq+1:Model.nsys,1)=mexstep('get','qvel')';
%             x2=x2+noise_switch*Task.training_noise*randn(Model.nsys,1);
%             zm=x2+noise_switch*Task.measurement_noise*randn(Model.nsys,1);
%             mexstep('set','qpos',x2(1:Model.nq,1),Model.nq);
%             mexstep('set','qvel',x2(Model.nq+1:Model.nsys,1),Model.nv);
%             mexstep('forward');
% %             delta_x2(:,i+1,r)=x_est_id(:,i+1,r)-x_nom(:,i+1);
% %             if noise_switch >= 0 % >=
% %                 delta_x2(:,i+1,r)=zm-x_nom(:,i+1);
% %             end
% %             delta_x1(:,i+1,r)=delta_x2(:,i+1,r);
%         end
%         x_est_id(:,horizon+1,r) = Aid(:,:,horizon)*x_est_id(:,horizon,r)+Bid(:,:,horizon)*u_act+LK(:,:,horizon+1)*(zm-Aid(:,:,horizon)*x_est_id(:,horizon,r)-Bid(:,:,horizon)*u_act);       
%         delta_x2(:,horizon+1,r)=x_est_id(:,horizon+1,r)-x_nom(:,horizon+1);%
%         if noise_switch >= 0 % >=
%             delta_x2(:,horizon+1,r)=zm-x_nom(:,horizon+1);
%         end
%         delta_x1(:,horizon+1,r)=delta_x2(:,horizon+1,r);
%     end
%     sum=sum+sumsqr(delta_x1);
%     delta_x1_avg(:,j,:)=mean(delta_x1,3);
%     delta_x2_avg(:,j,:)=mean(delta_x2,3);
%     delta_u_avg(:,j,:)=mean(delta_uact,3);
% end
% for i = 1:1:horizon
%     AB = delta_x2_avg(:,:,i+1)*[delta_x1_avg(:,:,i);delta_u(:,:,i)]'/([delta_x1_avg(:,:,i);delta_u(:,:,i)]*[delta_x1_avg(:,:,i);delta_u(:,:,i)]');
%     if ite > 1
%         system_error(ite) = system_error(ite) + norm(AB-AB_(:,:,i))/norm(AB_(:,:,i));
%     end
%     AB_(:,:,i)=AB;
%     Bid(:,:,i) = AB(:,Model.nsys+1:end);
%     Aid(:,:,i) = AB(:,1:Model.nsys)+1*Bid(:,:,i)*K(:,:,i);
%     
% %     if noise_switch == 1
% %         AB = delta_x2_avg(:,:,i+1)*[delta_x1_avg(:,:,i);delta_u_avg(:,:,i)]'/([delta_x1_avg(:,:,i);delta_u_avg(:,:,i)]*[delta_x1_avg(:,:,i);delta_u_avg(:,:,i)]');
% %         if ite > 1
% %             system_error(ite) = system_error(ite) + norm(AB-AB_(:,:,i))/norm(AB_(:,:,i));
% %         end
% %         AB_(:,:,i)=AB;
% %         Bid(:,:,i) = AB(:,Model.nsys+1:end);
% %         Aid(:,:,i) = AB(:,1:Model.nsys);   
% %     else
% %         AB = delta_x2_avg(:,:,i+1)*[delta_x1_avg(:,:,i);delta_u(:,:,i)]'/([delta_x1_avg(:,:,i);delta_u(:,:,i)]*[delta_x1_avg(:,:,i);delta_u(:,:,i)]');
% %         if ite > 1
% %             system_error(ite) = system_error(ite) + norm(AB-AB_(:,:,i))/norm(AB_(:,:,i));
% %         end
% %         AB_(:,:,i)=AB;
% %         Bid(:,:,i) = AB(:,Model.nsys+1:end);
% %         Aid(:,:,i) = AB(:,1:Model.nsys)+1*Bid(:,:,i)*K(:,:,i);
% %     end
%         
% end
% system_error(ite) = system_error(ite) / horizon * 100;
% traj_norm = sqrt(sum/Model.nsys/horizon/Task.avg_num/Task.nSim)
% iteration = ite
%%%%%%%%%%%%%%%%%%%%%% multirollouts average
delta_x0 = Task.statePtb*1*randn(Model.nsys,Task.nSim);
delta_u = Task.ptb*1*randn(Model.nu,Task.nSim,horizon);
delta_x1_avg = zeros(Model.nsys,Task.nSim,horizon+1);
delta_x2_avg = zeros(Model.nsys,Task.nSim,horizon+1);
e = zeros(Model.nsys,Task.nSim,horizon+1);
delta_u_avg = zeros(Model.nu,Task.nSim,horizon);
sum = 0;
for j = 1:1:Task.nSim
    delta_x2 = zeros(Model.nsys,horizon,Task.avg_num);
    delta_x1 = zeros(Model.nsys,horizon,Task.avg_num);
    delta_uact = zeros(Model.nu,horizon,Task.avg_num);
    for r = 1:1:Task.avg_num
        x1 = x_nom(:,1)+delta_x0(:,j)+noise_switch*Task.training_noise*randn(Model.nsys,1);
        delta_x1(:,1,r)=x1-x_nom(:,1)+noise_switch*Task.measurement_noise*randn(Model.nsys,1);
        x_est_id(:,1,r)=x_nom(:,1);
        mexstep('reset');
        mexstep('set','qpos',x1(1:Model.nq,1),Model.nq);
        mexstep('set','qvel',x1(Model.nq+1:Model.nsys,1),Model.nv);
        mexstep('forward');
        for i = 1:1:horizon
            if i >= 2
                e(:,j,i)=zm-Aid(:,:,i-1)*x_est_id(:,i-1,r)-Bid(:,:,i-1)*u_act;
                x_est_id(:,i,r) = Aid(:,:,i-1)*x_est_id(:,i-1,r)+Bid(:,:,i-1)*u_act+LK(:,:,i)*(zm-Aid(:,:,i-1)*x_est_id(:,i-1,r)-Bid(:,:,i-1)*u_act);
                delta_x2(:,i,r)=x_est_id(:,i,r)-x_nom(:,i);
                if noise_switch == 0 % >=
                    delta_x2(:,i,r)=zm-x_nom(:,i);
                end
                delta_x1(:,i,r)=delta_x2(:,i,r);
            end
            u_act = u_nom(:,i)+delta_u(:,j,i)-K(:,:,i)*(x_est_id(:,i,r)-x_nom(:,i)); % with kalman filter
            delta_uact(:,i,r) = u_act-u_nom(:,i);
            if noise_switch == 0
                u_act = u_nom(:,i)+delta_u(:,j,i)-K(:,:,i)*delta_x1(:,i,r);
            end
%             u_act = u_nom(:,i)+delta_u(:,j,i)-K(:,:,i)*delta_x1(:,i,r); % no kalman filter
            mexstep('set','ctrl',u_act,Model.nu);
            mexstep('step',1);
            x2(1:Model.nq,1)=mexstep('get','qpos')';
            x2(Model.nq+1:Model.nsys,1)=mexstep('get','qvel')';
            x2=x2+noise_switch*Task.training_noise*randn(Model.nsys,1);
            zm=x2+noise_switch*Task.measurement_noise*randn(Model.nsys,1);
            mexstep('set','qpos',x2(1:Model.nq,1),Model.nq);
            mexstep('set','qvel',x2(Model.nq+1:Model.nsys,1),Model.nv);
            mexstep('forward');
        end
        x_est_id(:,horizon+1,r) = Aid(:,:,horizon)*x_est_id(:,horizon,r)+Bid(:,:,horizon)*u_act+LK(:,:,horizon+1)*(zm-Aid(:,:,horizon)*x_est_id(:,horizon,r)-Bid(:,:,horizon)*u_act);       
        delta_x2(:,horizon+1,r)=x_est_id(:,horizon+1,r)-x_nom(:,horizon+1);%
        if noise_switch == 0 % >=
            delta_x2(:,horizon+1,r)=zm-x_nom(:,horizon+1);
        end
        delta_x1(:,horizon+1,r)=delta_x2(:,horizon+1,r);
    end
    sum=sum+sumsqr(delta_x1);
    delta_x1_avg(:,j,:)=mean(delta_x1,3);
    delta_x2_avg(:,j,:)=mean(delta_x2,3);
    delta_u_avg(:,j,:)=mean(delta_uact,3);
end
for i = 1:1:horizon
% with kalman filter
%     if noise_switch == 1
%         AB = delta_x2_avg(:,:,i+1)*[delta_x1_avg(:,:,i);delta_u_avg(:,:,i)]'/([delta_x1_avg(:,:,i);delta_u_avg(:,:,i)]*[delta_x1_avg(:,:,i);delta_u_avg(:,:,i)]');
%         if ite > 1
%             system_error(ite) = system_error(ite) + norm(AB-AB_(:,:,i))/norm(AB_(:,:,i));
%         end
%         AB_(:,:,i)=AB;
%         Bid(:,:,i) = AB(:,Model.nsys+1:end);
%         Aid(:,:,i) = AB(:,1:Model.nsys);
%     else
%         AB = delta_x2_avg(:,:,i+1)*[delta_x1_avg(:,:,i);delta_u(:,:,i)]'/([delta_x1_avg(:,:,i);delta_u(:,:,i)]*[delta_x1_avg(:,:,i);delta_u(:,:,i)]');
%         if ite > 1
%             system_error(ite) = system_error(ite) + norm(AB-AB_(:,:,i))/norm(AB_(:,:,i));
%         end
%         AB_(:,:,i)=AB;
%         Bid(:,:,i) = AB(:,Model.nsys+1:end);
%         Aid(:,:,i) = AB(:,1:Model.nsys)+1*Bid(:,:,i)*K(:,:,i);
%     end
% no kalman filter
    if i > 2
        e(:,:,i+1)*delta_x1_avg(:,:,i)'
    end
    AB = delta_x2_avg(:,:,i+1)*[delta_x1_avg(:,:,i);delta_u(:,:,i)]'/([delta_x1_avg(:,:,i);delta_u(:,:,i)]*[delta_x1_avg(:,:,i);delta_u(:,:,i)]');
    Bid(:,:,i) = AB(:,Model.nsys+1:end);
    Aid(:,:,i) = AB(:,1:Model.nsys)+1*Bid(:,:,i)*K(:,:,i);
end
system_error(ite) = system_error(ite) / horizon * 100;
traj_norm = sqrt(sum/Model.nsys/horizon/Task.avg_num/Task.nSim)
iteration = ite
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% for i = 1:1:horizon
%     delta_x1 = Task.statePtb*1*randn(Model.nsys,Task.nSim);
%     delta_u = Task.statePtb*1*randn(Model.nu,Task.nSim);  
%     delta_x2 = zeros(Model.nsys,Task.nSim,Task.avg_num);
%     delta_x2_avg = zeros(Model.nsys,Task.nSim);
%     for j = 1:1:Task.nSim
%         for r = 1:1:Task.avg_num
%             % plus
%             delta_x2(:,j,r)=Ak*(x_nom(:,i)+delta_x1(:,j))+Bk*(u_nom(:,i)+delta_u(:,j))+1*Task.training_noise*rand(Model.nsys,1);
%             % minus
%             delta_x2(:,j,r)=(delta_x2(:,j,r)-(Ak*(x_nom(:,i)-delta_x1(:,j))+Bk*(u_nom(:,i)-delta_u(:,j))+1*Task.training_noise*rand(Model.nsys,1)))/2;
%         end
%         delta_x2_avg=mean(delta_x2,3);
%     end
% %     [Q,R]=qr([delta_x1;delta_u]');
% %     AB=(R\Q'*delta_x2')';
%     AB = delta_x2_avg*[delta_x1;delta_u]'/([delta_x1;delta_u]*[delta_x1;delta_u]');
%     Aid(:,:,i) = AB(:,1:Model.nsys);
%     Bid(:,:,i) = AB(:,Model.nsys+1:end);
% end

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
Wi = 1e0*eye(Model.nsys);
Vi = 1e-2*eye(Task.nm);
for i= 1:1:horizon
    PS(:, :, i+1) = Aid(:,:,i) * (PS(:, :, i) - PS(:, :, i) / (PS(:, :, i) + Vi) * PS(:, :, i)) * Aid(:,:,i)' + Wi;
    LK(:, :, i) = PS(:, :, i)/ (PS(:, :, i) + Vi);
end
LK(:, :, horizon+1) = PS(:, :, horizon+1)/(PS(:, :, horizon+1) + Vi);
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
    criteria = (mean(conv_rate) > Task.converge); 
% elseif strcmp(Model.name, 'linear')
%     criteria = (ite < Task.maxIte); 
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

figure;
subplot(1,1,1)
plot(0:1:size(state_error,2)-1, state_error)
xlabel('iteration')
ylabel('terminal state error')

figure;
subplot(1,1,1)
plot(1:1:size(system_error,2)-1, system_error(1,2:end))
xlabel('iteration')
ylabel('diff in system matrices (%)')
