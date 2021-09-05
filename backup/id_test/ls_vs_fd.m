clear variables;clc;warning off;close all;
%% model and task setup
Model = model_reg('s15');
Task = ilqr_task_reg(Model);

%% init
fid = fopen('result0_s15.txt','r');
U = fscanf(fid, '%f');
fclose(fid);
u_nom = reshape(U(1:Model.nu*Task.horizon), Model.nu, Task.horizon);

%% nominal traj
mexstep('load',Model.file);
x_nom = evolve_traj(Model,Model.xInit,u_nom);

%% sysid
% lsid
Task.nSim=(Model.nsys+Model.nu)*100;
ABls = zeros(Model.nsys,Model.nsys+Model.nu,Task.horizon);
for i = 1:1:Task.horizon
    delta_x1 = Task.ptb*1*randn(Model.nsys,Task.nSim);
    delta_u = Task.ptb*1*randn(Model.nu,Task.nSim);  
    delta_x2 = zeros(Model.nsys,Task.nSim);
    for j = 1:1:Task.nSim
        mexstep('set','qpos',x_nom(1:Model.nq,i)+delta_x1(1:Model.nq,j),Model.nq);
        mexstep('set','qvel',x_nom(Model.nq+1:Model.nsys,i)+delta_x1(Model.nq+1:Model.nsys,j),Model.nv);
        mexstep('forward');
        mexstep('set','ctrl',u_nom(:,i)+delta_u(:,j),Model.nu);
        mexstep('step',1);
        delta_x2(1:Model.nq,j)=mexstep('get','qpos')'-x_nom(1:Model.nq,i+1);
        delta_x2(Model.nq+1:Model.nsys,j)=mexstep('get','qvel')'-x_nom(Model.nq+1:Model.nsys,i+1);
    end
%     [Q,R]=qr([delta_x1;delta_u]');
%     ABls(:,:,i)=(R\Q'*delta_x2')';
    ABls(:,:,i) = delta_x2*[delta_x1;delta_u]'/([delta_x1;delta_u]*[delta_x1;delta_u]');
    rls(i)=sqrt(mean(mean((delta_x2-ABls(:,:,i)*[delta_x1;delta_u]).^2,1))); % residual
end

% fd
nTrial=100;
ABfd = zeros(Model.nsys,Model.nsys+Model.nu,Task.horizon);
for i = 1:1:Task.horizon
    for k=1:1:nTrial
        for j=1:1:(Model.nsys+Model.nu)
            delta_xu = zeros(Model.nsys+Model.nu,1);delta_xu(j,1)=Task.ptb*1*randn(1,1);
            mexstep('set','qpos',x_nom(1:Model.nq,i)+delta_xu(1:Model.nq,1),Model.nq);
            mexstep('set','qvel',x_nom(Model.nq+1:Model.nsys,i)+delta_xu(Model.nq+1:Model.nsys,1),Model.nv);
            mexstep('forward');
            mexstep('set','ctrl',u_nom(:,i)+delta_xu(Model.nsys+1:end,1),Model.nu);
            mexstep('step',1);
            ABfd(:,j,i)=ABfd(:,j,i)+[(mexstep('get','qpos')'-x_nom(1:Model.nq,i+1))./delta_xu(j,1);(mexstep('get','qvel')'-x_nom(Model.nq+1:Model.nsys,i+1))./delta_xu(j,1)];
        end
    end
end
ABfd=ABfd/nTrial;

%% verify result
testNum=100;
for i = 1:1:Task.horizon
    delta_x1 = Task.ptb*1*randn(Model.nsys,testNum);
    delta_u = Task.ptb*1*randn(Model.nu,testNum);  
    delta_x2 = zeros(Model.nsys,testNum);
    for j = 1:1:testNum
        mexstep('set','qpos',x_nom(1:Model.nq,i)+delta_x1(1:Model.nq,j),Model.nq);
        mexstep('set','qvel',x_nom(Model.nq+1:Model.nsys,i)+delta_x1(Model.nq+1:Model.nsys,j),Model.nv);
        mexstep('forward');
        mexstep('set','ctrl',u_nom(:,i)+delta_u(:,j),Model.nu);
        mexstep('step',1);
        delta_x2(1:Model.nq,j)=mexstep('get','qpos')'-x_nom(1:Model.nq,i+1);
        delta_x2(Model.nq+1:Model.nsys,j)=mexstep('get','qvel')'-x_nom(Model.nq+1:Model.nsys,i+1);
    end
    
    error_ls(i)=sqrt(mean(mean((delta_x2-ABls(:,:,i)*[delta_x1;delta_u]).^2,1)))./Task.ptb;
    error_fd(i)=sqrt(mean(mean((delta_x2-ABfd(:,:,i)*[delta_x1;delta_u]).^2,1)))./Task.ptb; % normalized rms error
end
mexstep('exit');

rollout_num_ls=Task.nSim
rollout_num_ls=nTrial*(Model.nsys+Model.nu)
normalized_rms_error_ls=mean(error_ls)
normalized_rms_error_fd=mean(error_fd)

%% plot
figure;
plot([error_ls' error_fd'])
xlabel('step')
ylabel('rms error')
legend('LLS','FD')
title(['Sysid LLS vs FD tested over ' num2str(testNum) ' rollouts'])