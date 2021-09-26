clear all;clc;warning off;
addpath(genpath('./ilqr'), genpath('./utils'))

%% model and task setup
Model = model_reg('cartpole');
Task = task_reg(Model);

u_traj = zeros(Model.nu,Task.horizon);
x_traj = Model.xInit;ite_per_step=[];time_per_step=[];cost0=[];

%% run arma-ilqr
mexstep('load',['./model/' Model.file]);
[u_nom,x_nom,cost,train_time] = ilqr_arma(Model,Task,x_traj(:,1),u_traj,Task.horizon,cost0);
total_time = train_time
cost = cost

%% nominal traj
% x_traj_check = Model.xInit;
% mexstep('reset');
% mexstep('set','qpos',Model.xInit(1:Model.nq,1),Model.nq);
% mexstep('set','qvel',Model.xInit(Model.nq+1:Model.nsys,1),Model.nv);
% mexstep('forward');
% for i=1:1:Task.horizon
%     mexstep('set','ctrl',u_traj(:,i),Model.nu);
%     mexstep('step',1);
%     x_traj_check(1:Model.nq,i) = mexstep('get','qpos')';
%     x_traj_check(Model.nq+1:Model.nsys,i) = mexstep('get','qvel')';
% end
% mexstep('exit');

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
plot(0:1:size(cost,2)-1, cost);
xlabel('iteration')
ylabel('cost')
