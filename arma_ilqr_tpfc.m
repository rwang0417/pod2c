clear all;clc;warning off;
%% model and task setup
Model = model_reg('fish');
Task = task_reg(Model);
u_traj = rand(Model.nu,Task.horizon);
x_traj = Model.xInit;ite_per_step=[];time_per_step=[];cost0=[];
x_nom_traj = zeros(Model.nsys,Task.horizon+1);x_nom_traj(:,1) = Model.xInit;
y_closed=zeros(Task.nm*(Task.horizon+1),1);
y_est = zeros(Task.nm*Task.qx+Model.nu*(Task.qu-1),Task.horizon+1);
u_feedback=zeros(Model.nu*(Task.horizon+1),1);

%% Monte-Carlo test params
NSAMPLE=1;
noise=0.3;
data = zeros(size(noise,1),NSAMPLE);

%% initialize with ilqr nominal control sequence
fid = fopen('./u_init/result_arma_fish.txt','r');
U = fscanf(fid, '%f');
fclose(fid);
u_init = reshape(U(1:Model.nu*Task.horizon), Model.nu, Task.horizon);
umax = max(max(abs(u_init)));

%% run arma-ilqr-mpc
for p=1:1:size(noise,1)
    mexstep('load',['./model/' Model.file]);
    for s=1:1:NSAMPLE
        [u_nom,x_nom,cost,train_time] = ilqr_arma(Model,Task,x_traj(:,1),u_init,Task.horizon,cost0);
        x_nom_traj = x_nom;
        u_traj = u_nom;
        terminal_state_error = norm(x_nom_traj(1:3,end)-Task.xTarget(1:3));
        [MTK,MYE,MU1,MYM] = arma_lqg(Model,Task,Model.xInit,u_nom,0,0);
        for t=1:1:Task.horizon 
            mexstep('reset');
            mexstep('set','qpos',x_traj(1:Model.nq,t),Model.nq);
            mexstep('set','qvel',x_traj(Model.nq+1:Model.nsys,t),Model.nv);
            mexstep('forward');
            y_closed(Task.nm*(Task.horizon-t+1)+1:Task.nm*(Task.horizon-t+2),1)=Task.Ck*(x_traj(:,t)-x_nom_traj(:,t));
            if t >= max(Task.qx,Task.qu)+1
                y_est(:,t)=	MYE(:,:,t-1)*y_est(:,t-1)+MU1(:,:,t-1)*u_feedback(Model.nu*(Task.horizon-t+2)+1:Model.nu*(Task.horizon-t+3),1)+MYM(:,:,t-1)*y_closed(Task.nm*(Task.horizon-t+1)+1:Task.nm*(Task.horizon-t+2),1);
                u_feedback(Model.nu*(Task.horizon-t+1)+1:Model.nu*(Task.horizon-t+2),1)=-MTK(:,:,t)*y_est(:,t);
                u = max(Model.uRange(1),min(Model.uRange(2),u_traj(:,t)+u_feedback(Model.nu*(Task.horizon-t+1)+1:Model.nu*(Task.horizon-t+2),1)+noise(p)*umax*randn(Model.nu,1)));
            else
                u = max(Model.uRange(1),min(Model.uRange(2),u_traj(:,t)+noise(p)*umax*randn(Model.nu,1)));
            end
            mexstep('set','ctrl',u,Model.nu);
            mexstep('step',1);
            x_traj(1:Model.nq,t+1) = mexstep('get','qpos')';
            x_traj(Model.nq+1:Model.nsys,t+1) = mexstep('get','qvel')';
            x_traj(:,t+1) = x_traj(:,t+1) + 0.0*randn(Model.nsys,1); % process noise..........

            state_error = norm(x_traj(1:3,t+1)-x_nom_traj(1:3,t+1));
            if state_error > 0.005 && t < Task.horizon 
                [u_nom,x_nom,cost,train_time] = ilqr_arma(Model,Task,x_traj(:,t+1),u_traj(:,t+1:end),Task.horizon-t,cost0);
                u_traj(:,t+1:end) = u_nom;
                x_nom_traj(:,t+1:end) = x_nom;
                [MTKtp,MYEtp,MU1tp,MYMtp] = arma_lqg(Model,Task,x_traj(:,t+1),u_traj(:,t+1:end),0,0);
                MTK(:,:,t+1:end)=MTKtp;MYE(:,:,t+1:end)=MYEtp;MU1(:,:,t+1:end)=MU1tp;MYM(:,:,t+1:end)=MYMtp;
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
        % % cartpole 
        % terminal_state_error = norm(x_traj(:,Task.horizon+1)-Task.xTarget)
        % % swimmer6
%         terminal_state_error = norm(x_traj(1:2,end)-Task.xTarget(1:2))
        % % fish
        data(p,s) = norm(x_traj(1:3,end)-Task.xTarget(1:3))
    end
    mexstep('exit');
    noise_level=noise(p)
end

%% save results
% perfcheck = [noise,mean(data,2),std(data,0,2)];
% fid = fopen('perfcheck.txt','wt');
% for i = 1:1:size(noise,1)
%     for j = 1:1:3
%         fprintf(fid,'%.5f ',perfcheck(i,j));
%     end
%     fprintf(fid,'\n');
% end
% fclose(fid);

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
% fid = fopen('result_arma_s6.txt','wt');
% for i = 1 : Task.horizon
%     for j = 1 : Model.nu
%         fprintf(fid,'%.10f ',u_traj(j,i));
%     end
% end
% fclose(fid);
% umax=max(max(abs(u_traj)));

%% plot
figure;
% subplot(2,2,1)
plot([x_traj(1,:)', x_traj(2,:)'])
xlabel('step')
ylabel('state')
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