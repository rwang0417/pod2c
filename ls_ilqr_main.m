clear variables;clc;warning off;
%% model and task setup
Model = model_reg('pendulum');
Task = task_reg(Model);
u_init = 5.0*rand(Model.nu,Task.horizon);
x_traj = Model.xInit;ite_per_step=[];time_per_step=[];cost0=[];

%% init
% fid = fopen('result_arma_11.txt','r');
% U = fscanf(fid, '%f');
% fclose(fid);
% u_init = reshape(U(1:Model.nu*Task.horizon), Model.nu, Task.horizon);

%% ilqr training
mexstep('load',['./model/' Model.file]);
[u_nom,x_nom,cost,train_time] = ilqr_lls(Model,Task,x_traj(:,1),u_init,Task.horizon,cost0);
umax = max(max(abs(u_init)));
% [MTK] = ls_lqg(Model,Task,Model.xInit,u_nom,0,1);

%% nominal traj
x_traj = evolve_traj(Model,Model.xInit,u_nom);
terminal_state_error = norm(x_traj(1:2,end)-Task.xTarget(1:2))
mexstep('exit');

%% output result
% fid = fopen('result_arma_11.txt','wt');
% for i = 1 : Task.horizon
%     for j = 1 : Model.nu
%         fprintf(fid,'%.10f ',u_nom(j,i));
%     end
% end
% fclose(fid);
% umax=max(max(abs(u_traj)));

%% plot
figure;
subplot(1,3,1)
plot([x_traj(1,:)', x_traj(2,:)'])
xlabel('step')
ylabel('state')

subplot(1,3,2)
plot(0:1:size(cost,2)-1, cost);
xlabel('iteration')
ylabel('cost')

subplot(1,3,3)
plot(0:1:Task.horizon-1, u_nom(1,:));
xlabel('iteration')
ylabel('u')

% subplot(2,2,3)
% plot(0:1:size(ite_per_step,2)-1, ite_per_step);
% xlabel('step')
% ylabel('ite to converge')
% 
% subplot(2,2,4)
% plot(0:1:size(time_per_step,2)-1, time_per_step);
% xlabel('step')
% ylabel('time to converge')