clear variables;
clc;warning off;
addpath(genpath('./ilqr'), genpath('./utils'))
%% model and task setup
Model = model_reg('s3');
Task = task_reg(Model);
u_init = 0.*randn(Model.nu,Task.horizon);
ite_per_step=[];time_per_step=[];cost0=[];

%% init
fid = fopen('./results/result_arma_s3.txt','r');
U = fscanf(fid, '%f');
fclose(fid);
u_nom = reshape(U(1:Model.nu*Task.horizon), Model.nu, Task.horizon);

%% ilqr training
tic;
mexstep('load',['./model/' Model.file]);
% [u_nom,x_nom,cost,train_time] = ilqr_lls(Model,Task,Model.xInit,u_init,Task.horizon,cost0);
u_max = max(max(abs(u_nom)));
% terminal_state_error = getStateError(Model,x_nom(:,end),Task.xTarget)
[MTK] = ls_lqg(Model,Task,Model.xInit,u_nom,0,1);
toc;

%% nominal traj
x_nom = evolve_traj(Model,Model.xInit,u_nom);
terminal_state_error = getStateError(Model,x_nom(:,end),Task.xTarget)

%% performance check
NSAMPLE=200;
noise=(0:0.1:1)';
terminal_state_error = zeros(size(noise,1),NSAMPLE);
energy = zeros(size(noise,1),NSAMPLE);
perfdata = zeros(size(noise,1),5);

for p=1:1:size(noise,1)
    for s=1:1:NSAMPLE
       [x_traj, u_traj] = evolve_traj(Model,Model.xInit,u_nom,x_nom,MTK,noise(p)*u_max);
       terminal_state_error(p,s) = getStateError(Model,x_traj(:,end),Task.xTarget);
       energy(p,s) = sum(u_traj.^2,'all');
    end
    perfdata(p,:) = [noise(p) mean(terminal_state_error(p,:),2) std(terminal_state_error(p,:),0,2) mean(energy(p,:),2) std(energy(p,:),0,2)]
end

mexstep('exit');

%% output result
fid = fopen('energy.txt','wt');
for p = 1 : size(noise,1)
    for c = 1 : 5
        fprintf(fid,'%.10f ',perfdata(p,c));
    end
    fprintf(fid,'\n');
end
fclose(fid);
% fid = fopen('result0_lls_pend.txt','wt');
% for i = 1 : Task.horizon
%     for j = 1 : Model.nu
%         fprintf(fid,'%.10f ',u_nom(j,i));
%     end
% end
% fclose(fid);
% fid = fopen('cost0.txt','wt');
% for i = 1 : size(cost,2)
%     fprintf(fid,'%f ',cost(i));
% end
% fclose(fid);
% fid = fopen('./nominal_d2c/state04.txt','wt');
% for i = 1 : Task.horizon+1
%     for j = 1 : Model.nsys
%         fprintf(fid,'%.10f ',x_nom(j,i));
%     end
%     fprintf(fid,'\n');
% end
% fclose(fid);

%% plot
figure;
subplot(1,3,1)
plot([x_nom(1,:)', x_nom(2,:)'])
xlabel('step')
ylabel('state')

% subplot(1,3,2)
% plot(0:1:size(cost,2)-1, cost);
% xlabel('iteration')
% ylabel('cost')

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