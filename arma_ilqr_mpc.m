clear all;clc;warning off;

%% model and task setup
Model = model_reg('cartpole');
Task = task_reg(Model);
u_traj = 0.1*rand(Model.nu,Task.horizon);
x_traj = Model.xInit;ite_per_step=[];time_per_step=[];cost0=[];

%% Monte-Carlo test params
NSAMPLE=1;
noise=(1:0.1:1)';
data = zeros(size(noise,1),NSAMPLE);

%% initialize with ilqr nominal control sequence
fid = fopen('./u_init/result_arma_cart.txt','r');
U = fscanf(fid, '%f');
fclose(fid);
u_init = reshape(U(1:Model.nu*Task.horizon), Model.nu, Task.horizon);
u_max = max(max(abs(u_init)));
% u_max = abs(Model.uRange(2));

%% run arma-ilqr-mpc
for p=1:1:size(noise,1)
    mexstep('load',['./model/' Model.file]);
    for s=1:1:NSAMPLE
%         u_traj = 0.1*rand(Model.nu,Task.horizon);
        u_traj=u_init;
        for t=1:1:Task.horizon
            [u_nom,x_nom,cost,train_time] = ilqr_arma(Model,Task,x_traj(:,t),u_traj(:,t:end),Task.horizon-t+1,cost0);
            
            u_traj(:,t:end) = u_nom;
            mexstep('reset');
            mexstep('set','qpos',x_traj(1:Model.nq,t),Model.nq);
            mexstep('set','qvel',x_traj(Model.nq+1:Model.nsys,t),Model.nv);
            mexstep('forward');
            u = max(Model.uRange(1),min(Model.uRange(2),u_traj(:,t)+noise(p)*u_max*randn(Model.nu,1)));
            mexstep('set','ctrl',u,Model.nu);
            mexstep('step',1);
            x_traj(1:Model.nq,t+1) = mexstep('get','qpos')';
            x_traj(Model.nq+1:Model.nsys,t+1) = mexstep('get','qvel')';
            x_traj(:,t+1) = x_traj(:,t+1) + 0.0*randn(Model.nsys,1); % process noise..........
        %     ite_per_step = [ite_per_step,size(cost,2)];
        %     time_per_step = [time_per_step,train_time]; 
            if t == 1
                cost0 = cost(1);
            end
        end
        % % cartpole 
        data(p,s) = norm(x_traj(:,Task.horizon+1)-Task.xTarget);
        % % swimmer6
%         data(p,s) = norm(x_traj(1:2,Task.horizon+1)-Task.xTarget(1:2));
        % total_time = sum(time_per_step)
    end
    mexstep('exit');
    noise_level=noise(p)
end

%% save results
perfcheck = [noise,mean(data,2),std(data,0,2)];
% fid = fopen('perfcheck.txt','wt');
% for i = 1:1:size(noise,1)
%     for j = 1:1:3
%         fprintf(fid,'%.5f ',perfcheck(i,j));
%     end
%     fprintf(fid,'\n');
% end
% fclose(fid);

% fid = fopen('result_arma_cart.txt','wt');
% for i = 1 : Task.horizon
%     for j = 1 : Model.nu
%         fprintf(fid,'%.10f ',u_traj(j,i));
%     end
% end
% fclose(fid);

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