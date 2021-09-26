function ilqr_linear_case_stochastic
clear variables;clc;close all;tic;
%% model setup and variables
Model.nu=1;
Model.uRange=[-inf,inf];
Model.xInit=[pi;0];
Model.nsys=2;
Model.name='linear';
Task.converge=0.0001;
Task.maxIte=10;
Task.horizon=30;
Task.training_noise=0.05;
Task.avg_num=100;
Task.xTarget=[0;0];
Task.statePtb = 0.01;
Task.ptb = 0.01;
Task.nSim = 100;
Task.R = 1*10^0 * eye(Model.nu);
Task.Q = 0*eye(Model.nsys);
Task.QT = 0*eye(Model.nsys);
Task.Q(1:Model.nsys,1:Model.nsys) = 1*[1 0;0 0.1];
Task.QT(1:Model.nsys,1:Model.nsys) = 100*eye(Model.nsys);
Task.alpha = 1;
Ak = [1.002641490000000   0.099209440000000;0.026414880000000   0.992094400000000];
Bk = [0.08;0.8];
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
criteria = true;
conv_rate = ones(3,1);
horizon = Task.horizon;

%% ILQR
ite = 1;idx = 1;tic;
while ite <= Task.maxIte && criteria
%% forward pass
forward_flag = true;
while forward_flag
    for r = 1:1:Task.avg_num
        x_new(:,1,r) = Model.xInit;
        for i = 1:1:horizon
            u_new(:,i,r) = max(Model.uRange(1),min(Model.uRange(2),u_nom(:,i) - K(:,:,i)*(x_new(:,i,r)-x_nom(:,i)) + Task.alpha*kt(:,i)));
            feedback(1,i,ite) = K(:,:,i)*(x_new(:,i,r)-x_nom(:,i));
            nominal(1,i,ite) = kt(:,i);
            x_new(:,i+1,r)=Ak*x_new(:,i,r)+Bk*u_new(:,i,r)+Task.training_noise*randn(Model.nsys,1);
        end
    end
    x_avg = mean(x_new,3);
    u_avg = mean(u_new,3);
    cost_new = 0;
    for i = 1:1:horizon
        cost_new = cost_new + 0.5*(x_avg(:,i)-Task.xTarget)'*Task.Q*(x_avg(:,i)-Task.xTarget)+0.5*u_avg(:,i)'*Task.R*u_avg(:,i);
    end
    cost_new = cost_new + 0.5*(x_avg(:,horizon+1)-Task.xTarget)'*Task.QT*(x_avg(:,horizon+1)-Task.xTarget);
    if ite > 1
        z = (cost(ite-1) - cost_new)/delta_j;
    end
    
    if (z >= 0 || Task.alpha < 10^-5)
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
% state_error = getStateError(Model,x_nom(:,end),Task.xTarget)
% [state_error cost_new]

% lsid
% for i = 1:1:horizon
%     Aid(:,:,i) = Ak;
%     Bid(:,:,i) = Bk;
% end

delta_x0 = Task.statePtb*1*randn(Model.nsys,Task.nSim);
delta_u = Task.ptb*1*randn(Model.nu,Task.nSim,horizon);  
delta_x1_avg = zeros(Model.nsys,Task.nSim,horizon+1);
delta_x2_avg = zeros(Model.nsys,Task.nSim,horizon+1);
sum = 0;
for j = 1:1:Task.nSim
    delta_x2 = zeros(Model.nsys,horizon,Task.avg_num);
    delta_x1 = zeros(Model.nsys,horizon,Task.avg_num);
    for r = 1:1:Task.avg_num
        x1 = x_nom(:,1)+delta_x0(:,j);
        delta_x1(:,1,r)=delta_x0(:,j);
        for i = 1:1:horizon
            x2=Ak*x1+Bk*(u_nom(:,i)+delta_u(:,j,i)-1*K(:,:,i)*delta_x1(:,i,r))+Task.training_noise*randn(Model.nsys,1);
            delta_x2(:,i+1,r)=x2-x_nom(:,i+1);
            delta_x1(:,i+1,r)=delta_x2(:,i+1,r);
            x1=x2;
        end
    end
    sum=sum+sumsqr(delta_x1);
    delta_x1_avg(:,j,:)=mean(delta_x1,3);
    delta_x2_avg(:,j,:)=mean(delta_x2,3);
end
for i = 1:1:horizon
    AB = delta_x2_avg(:,:,i+1)*[delta_x1_avg(:,:,i);delta_u(:,:,i)]'/([delta_x1_avg(:,:,i);delta_u(:,:,i)]*[delta_x1_avg(:,:,i);delta_u(:,:,i)]');
    Bid(:,:,i) = AB(:,Model.nsys+1:end);
    Aid(:,:,i) = AB(:,1:Model.nsys)+1*Bid(:,:,i)*K(:,:,i);
end
traj_norm = sqrt(sum/Model.nsys/horizon/Task.avg_num/Task.nSim)

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
if strcmp(Model.name, 'linear')
    criteria = (mean(conv_rate) > Task.converge); 
% elseif strcmp(Model.name, 'linear')
%     criteria = (ite < Task.maxIte); 
end
end
toc;
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
subplot(1,3,1)
plot([x_avg(1,:)', x_det(1,:)'])
xlabel('step')
ylabel('state')
legend('avg', 'det')

subplot(1,3,2)
plot([u_avg(1,:)', u_det(1,:)'])
xlabel('step')
ylabel('u')
legend('avg', 'det')
