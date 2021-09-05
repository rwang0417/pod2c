function sqp_linear_case
clear variables;clc;close all;tic;
%% model setup and variables
Model.nu=1;
Model.uRange=[-10,10];
Model.xInit=[pi;0];
Model.nsys=2;
Task.horizon=30;
Task.xTarget=[0;0];
Task.R = 1*10^0 * eye(Model.nu);
Task.Q = 0*eye(Model.nsys);
Task.QT = 0*eye(Model.nsys);
Task.Q(1:Model.nsys,1:Model.nsys) = 1*[1 0;0 0.1];
Task.QT(1:Model.nsys,1:Model.nsys) = 100*eye(Model.nsys);
Ak=[1.002641490000000   0.099209440000000;0.026414880000000   0.992094400000000];
Bk=[0.08;0.8];
u_init = 0*rand(Model.nu,Task.horizon);
cost_sqp = [];
u_traj_ite = [];

%% OpenLoop Nominal
% Controller constraints
for i = 1 : Task.horizon
    u_min(1:Model.nu, i) = Model.uRange(1);
    u_max(1:Model.nu, i) = Model.uRange(2);
end

% solve the NLP using fmincon function
% use opt to increse the number of maximum iterations
% options = optimset('outputfcn',@outfun,'Tolx',1e-10,'MaxFunEvals',40000,'Algorithm','sqp-legacy','MaxIter',1500);
options = optimoptions(@fmincon,'Algorithm','sqp','MaxIterations',1500,'outputfcn',@outfun,'Tolx',1e-10,'MaxFunEvals',40000);

u_nom = fmincon(@(uk)episodic_cost_linear(Model,Task,uk), u_init, [],[],[],[], u_min, u_max,[], options);
x_nom = evolve_traj_linear(Model,Model.xInit,u_nom);

%% save result
iter_num = size(cost_sqp,2);
u_traj_ite_sqp = reshape(u_traj_ite',[Model.nu,Task.horizon,iter_num]);
for i=1:1:iter_num
    x_traj_ite_sqp(:,:,i) = evolve_traj_linear(Model,Model.xInit,u_traj_ite_sqp(:,:,i));
end
% save('ite_result_sqp_linear.mat','cost_sqp','x_traj_ite_sqp','u_traj_ite_sqp');
toc;

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
plot(0:1:size(cost_sqp,2)-1, cost_sqp)
xlabel('iteration')
ylabel('cost')

function stop = outfun(x,optimValues,~)
    stop = false;

    % Concatenate current point and objective function
    % value with history.x must be a row vector.
    cost_sqp = [cost_sqp,optimValues.fval];
    u_traj_ite = [u_traj_ite;x];
end

function x_traj = evolve_traj_linear(Model,x0,u_traj)
    horizon=size(u_traj,2);
    x_traj = zeros(Model.nsys,horizon+1);x_traj(:,1)=x0;
    for j = 1:1:horizon
        x_traj(:,j+1)=Ak*x_traj(:,j)+Bk*u_traj(:,j);
    end
end

function cost = episodic_cost_linear(Model,Task,u_traj)
    cost=0;
    x_traj = evolve_traj_linear(Model,Model.xInit,u_traj);

    for k=1:1:Task.horizon
        cost=cost+0.5*(x_traj(:,k)-Task.xTarget)'*Task.Q*(x_traj(:,k)-Task.xTarget)+0.5*u_traj(:,k)'*Task.R*u_traj(:,k);
    end
    cost=cost+0.5*(x_traj(:,Task.horizon+1)-Task.xTarget)'*Task.QT*(x_traj(:,Task.horizon+1)-Task.xTarget);
end
end

% %% iLQG
% full_DDP = false;
% 
% DYNCST  = @(b,u,i) beliefDynCost_swingup(b,u,xf,nDT,full_DDP,mm,om);
% %Control constraints
% Op.lims  = [-5.0 5.0; -5.0 5.0; -5.0 5.0; -5.0 5.0];         % Vx limits (m/s)
% Op.plot = 1; % plot the derivatives as well
% 
% [bs,us_opt,Ls_opt,Paral]= iLQG(DYNCST, b0, u0, Op);
% %us_opt: optimal control solved by iLQG
% %bs: optimal trajectory generated using us_opt
% 
% %%
% %Check solutions:
% %first four states +/- 2pi:
% for i = 1 : nDT
%     bs(1: 4, i) = mod(bs(1:4, i), 2 * pi);
%     X_nor(1:4, i) = mod(X_nor(1:4, i), 2*pi);
% end
% 
% figure;
% plot(bs(1, :));
% hold on;
% plot(X_nor(1, :), '--r');
% legend('iLQG','OpenLoop Nominal');
% title('\theta_1');
% 
% %%  After swing up, the task is to balance: 
% b1 = bs(:, end); % new initial belief state for iLQG
% x1 = X_nor(:, end); %new initial state for NLP
% %Target time :
% Tf1 = 0.5;        
% nDT1 = Tf1/dt;
% %% Setup controls
% u1 = zeros(1, nDT1);    %new initial guess for iLQG controller, does not need to change
% for i = 1 : nDT1
%     u1(1:4, i) =us_opt(1:4, nDT);
% end
% u2= zeros(1, nDT1);      %new intiial guess for NLP controller, does not need to change
% for i = 1 : nDT1
%     u2(1:4, i) = u_nor(1:4, nDT);
% end
% for i = 1 : nDT1
%     u_min1(1:4, i) = -5;
%     u_max1(1:4, i) = 5;
% end
% 
% %%  NLP open loop nominal
% opt = optimset('Tolx', 1e-10, 'MaxFunEvals', 50000);
% 
% [u_nor1, fval1] = fmincon(@(uk)belief_opt_balance(uk,x1,xf,nDT1, mm, om), u2, [],[],[],[], u_min1, u_max1,[],opt);
% 
% 
% X_nor1 = evolve_all(x1, u_nor1, mm);
% %% iLQG 
% 
% DYNCST1  = @(b,u,i) beliefDynCost_balance(b,u,xf,nDT1,full_DDP,mm,om);
% 
% [bs1,us_opt1,Ls_opt1,Paral1]= iLQG(DYNCST1, b1, u1, Op);
% 
% %% Check solutions
% 
% for i = 1 : nDT1
%     bs1(1: 4, i) = mod(bs1(1:4, i), 2 * pi);
%     X_nor1(1:4, i) = mod(X_nor1(1:4, i), 2*pi);
% end
% 
% figure;
% plot(bs1(1, :));
% hold on;
% plot(X_nor1(1, :), '--r');
% legend('iLQG','OpenLoop Nominal');
% title('\theta_1');
% 
% %% Swing up + Balance Check
% uf =[u_nor u_nor1];
% us = [us_opt us_opt1];
% 
% Xf = evolve_all(b0, uf, mm);
% Xs = evolve_all(b0, us, mm);
% 
% for i = 1 : nDT + nDT1
%     Xf(1:4, i) = mod(Xf(1:4, i), 2*pi);
%     Xs(1:4, i) = mod(Xs(1:4, i), 2*pi);
% end
% 
% 
% figure;
% plot(Xs(1, :));
% hold on;
% plot(Xf(1, :), '--r');
% legend('iLQG','OpenLoop Nominal');
% title('\theta_1');
% 
% %Animation
% figure;
% for i = 1 : nDT + nDT1 + 1
% drawPendulum_4D(Xf,i, draw_para);
% drawnow;
% pause(0.05);
% end  

