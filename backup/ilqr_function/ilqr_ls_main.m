clear all;clc;warning off;
%% model and task setup
Model = model_reg('pendulum');
Task = ilqr_task_reg(Model);
u_init = 0.1*rand(Model.nu,Task.horizon);
x_traj = Model.xInit;ite_per_step=[];time_per_step=[];cost0=[];
x_nom_traj = zeros(Model.nsys,Task.horizon+1);x_nom_traj(:,1) = Model.xInit;

%% ilqr training
mexstep('load',Model.file);
[u_nom,x_nom,cost,train_time] = ilqr_ls_full_state(Model,Task,x_traj(:,1),u_init,Task.horizon,cost0);
umax = max(max(abs(u_init)));
% [MTK] = ls_lqg(Model,Task,Model.xInit,u_init,0,1);

%% save results

%% nominal traj
% mexstep('load',Model.file);
mexstep('reset');
mexstep('set','qpos',Model.xInit(1:Model.nq,1),Model.nq);
mexstep('set','qvel',Model.xInit(Model.nq+1:Model.nsys,1),Model.nv);
mexstep('forward');
for i=1:1:Task.horizon
%     if i==1000
        u = max(Model.uRange(1),min(Model.uRange(2),u_nom(:,i)+0.0*umax*randn(Model.nu,1)));
%     else
%         u = u_init(:,i);
%     end
    mexstep('set','ctrl',u,Model.nu);
    mexstep('step',1);
    x_traj(1:Model.nq,i) = mexstep('get','qpos')';
    x_traj(Model.nq+1:Model.nsys,i) = mexstep('get','qvel')';
end
terminal_state_error = norm(x_traj(1:2,end)-Task.xTarget(1:2))
mexstep('exit');

%% output result
% fid = fopen('result_arma_.txt','wt');
% for i = 1 : Task.horizon
%     for j = 1 : Model.nu
%         fprintf(fid,'%.10f ',u_traj(j,i));
%     end
% end
% fclose(fid);
% umax=max(max(abs(u_traj)));

%% plot
figure;
subplot(2,2,1)
plot([x_traj(1,:)', x_traj(2,:)'])
xlabel('step')
ylabel('state')

subplot(2,2,2)
plot(0:1:size(cost,2)-1, cost);
xlabel('iteration')
ylabel('cost')

subplot(2,2,3)
plot(0:1:size(ite_per_step,2)-1, ite_per_step);
xlabel('step')
ylabel('ite to converge')

subplot(2,2,4)
plot(0:1:size(time_per_step,2)-1, time_per_step);
xlabel('step')
ylabel('time to converge')