clear all;clc;warning off;
%% model and task setup
Model = model_reg('s3');
Task = ilqr_task_reg(Model);
u_traj = 0.1*rand(Model.nu,Task.horizon);
x_traj = Model.xInit;ite_per_step=[];time_per_step=[];cost0=[];
x_nom_traj = zeros(Model.nsys,Task.horizon+1);x_nom_traj(:,1) = Model.xInit;

%% Monte-Carlo test params
NSAMPLE=5;
noise=(0:0.0001:0.001)';
data = zeros(size(noise,1),NSAMPLE);

%% initialize with ilqr nominal control sequence
fid = fopen('result_arma_s3.txt','r');
U = fscanf(fid, '%f');
fclose(fid);
u_init = reshape(U(1:Model.nu*Task.horizon), Model.nu, Task.horizon);
umax = max(max(abs(u_init)));

% mexstep('load',Model.file);
% [u_nom,x_nom,cost,train_time] = ilqr_ls_full_state(Model,Task,x_traj(:,1),u_init,Task.horizon,cost0);
% [MTK] = ls_lqg(Model,Task,Model.xInit,u_init,0,1);

%% run arma-ilqr-mpc
for p=1:1:size(noise,1)
    mexstep('load',Model.file);
    nite=zeros(1,NSAMPLE);tite=zeros(1,NSAMPLE);nreplan=zeros(1,NSAMPLE);
    for s=1:1:NSAMPLE
        [u_nom,x_nom,cost,train_time] = ilqr_ls_full_state(Model,Task,x_traj(:,1),u_init,Task.horizon,cost0);
        x_nom_traj = x_nom;
        u_traj = u_nom;
        [MTK] = ls_lqg(Model,Task,Model.xInit,u_nom,0,0);
        for t=1:1:Task.horizon 
            mexstep('reset');
            mexstep('set','qpos',x_traj(1:Model.nq,t),Model.nq);
            mexstep('set','qvel',x_traj(Model.nq+1:Model.nsys,t),Model.nv);
            mexstep('forward');
            dx=x_traj(:,t)-x_nom_traj(:,t);
            u = max(Model.uRange(1),min(Model.uRange(2),u_traj(:,t)-MTK(:,:,t)*dx+noise(p)*umax*randn(Model.nu,1)));
            mexstep('set','ctrl',u,Model.nu);
            mexstep('step',1);
            x_traj(1:Model.nq,t+1) = mexstep('get','qpos')';
            x_traj(Model.nq+1:Model.nsys,t+1) = mexstep('get','qvel')';
            x_traj(:,t+1) = x_traj(:,t+1) + noise(p)*umax*randn(Model.nsys,1); % process noise..........

            state_error = getStateError(Model,x_traj(:,t+1),x_nom_traj(:,t+1))
            if state_error > 0.03 && t < Task.horizon % fish 0.02, s6 0.03 cartpole 0.1 pendulum 0.1
                tic;
                [u_nom,x_nom,cost,train_time] = ilqr_ls_full_state(Model,Task,x_traj(:,t+1),u_traj(:,t+1:end),Task.horizon-t,cost0);
                tite(s)=tite(s)+toc;
                u_traj(:,t+1:end) = u_nom;
                x_nom_traj(:,t+1:end) = x_nom;
                [MTKtp] = ls_lqg(Model,Task,x_traj(:,t+1),u_traj(:,t+1:end),0,0);
                MTK(:,:,t+1:end)=MTKtp;
                nite(s)=nite(s)+size(cost,2);
                nreplan(s)=nreplan(s)+1;
            end
        %     ite_per_step = [ite_per_step,size(cost,2)];
        %     time_per_step = [time_per_step,train_time]; 
        %     if t == 1
        %         cost0 = cost(1);
        %     end
            step = t
        %     current_cost = cost(end)
        end

        % total_time = sum(time_per_step)
        terminal_state_error = getStateError(Model,x_traj(:,t+1),Task.xTarget)
        noise_level=noise(p)
        data(p,s) = terminal_state_error;
    end
    mexstep('exit');
end

%% save results
perfcheck = [noise,mean(data,2),std(data,0,2)];
fid = fopen('perfchecksu.txt','wt');
for i = 1:1:size(noise,1)
    for j = 1:1:3
        fprintf(fid,'%.5f ',perfcheck(i,j));
    end
    fprintf(fid,'\n');
end
fclose(fid);

% fid = fopen('rawperfdata.txt','wt');
% for i = 1:1:size(noise,1)
%     for j = 1:1:size(data,2)
%         fprintf(fid,'%.5f ',data(i,j));
%     end
%     fprintf(fid,'\n');
% end
% fclose(fid);

% fid = fopen('rawperfdata.txt','r');
% rawdata = fscanf(fid, '%f');
% fclose(fid);
% rawdata = reshape(rawdata, 10, 11)';
% noise=[0:0.1:1]';
% perfcheck = [noise,mean(rawdata,2),std(rawdata,0,2)];

%% nominal traj
% mexstep('load',Model.file);
% mexstep('reset');
% mexstep('set','qpos',Model.xInit(1:Model.nq,1),Model.nq);
% mexstep('set','qvel',Model.xInit(Model.nq+1:Model.nsys,1),Model.nv);
% mexstep('forward');
% for i=1:1:Task.horizon
%     if i==1000
%         u = max(Model.uRange(1),min(Model.uRange(2),u_init(:,i)+0.0*umax*randn(Model.nu,1)));
%     else
%         u = u_init(:,i);
%     end
%     mexstep('set','ctrl',u,Model.nu);
%     mexstep('step',1);
%     x_traj(1:Model.nq,i) = mexstep('get','qpos')';
%     x_traj(Model.nq+1:Model.nsys,i) = mexstep('get','qvel')';
% end
% terminal_state_error = norm(x_traj(1:3,end)-Task.xTarget(1:3))
% mexstep('exit');

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
% figure;
% subplot(2,2,1)
% plot([x_traj(1,:)', x_traj(2,:)'])
% xlabel('step')
% ylabel('state')
% 
% subplot(2,2,2)
% plot(0:1:size(cost,2)-1, cost);
% xlabel('iteration')
% ylabel('cost')
% 
% subplot(2,2,3)
% plot(0:1:size(ite_per_step,2)-1, ite_per_step);
% xlabel('step')
% ylabel('ite to converge')
% 
% subplot(2,2,4)
% plot(0:1:size(time_per_step,2)-1, time_per_step);
% xlabel('step')
% ylabel('time to converge')