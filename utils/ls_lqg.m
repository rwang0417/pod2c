function [MTK] = ls_lqg(...
    Model,...
    Task,...
    cur_state,...
    u_nom,...
    save2file,...
    checkResult)
% ls_lqg
%
% Description: Run ls-lqg algorithm to find LQG feedback along u_nom
%              Load model before calling this function.
%
% Inputs:
%     Model:		            Model infomation (structure)
%     Task:		                Task parameters (structure)
%     cur_state:                Current state (nq x 1 double)
%     u_nom:                    Nominal control trajectory (nu x stepNum double)
%     save2file:                True: save results to file, False: don't save (bool)
%     checkResult:              True: plot closed-loop performance result, False: don't plot (bool)
%
% Outputs:
%     MTK,MYE,MU1,MYM:          Feedback gains (mat)
%
% Example:                      [MTK,MYE,MU1,MYM] = arma_lqg(pend,pendTask,cur_state,u_nom,0,1);
%
% $Revision: R2020b$ 
% $Author: Ran Wang$
% $Date: March 18, 2021$

%% params
% lqr cost parameters
sig_q = 10^2;
sig_f = 10^8;%8fish
sig_r = 10^0;

u_max = max(max(abs(u_nom)));
horizon = size(u_nom,2);
nSim = 10*Task.nSim;

%% generate nominal state trajectory
x_nom = evolve_traj(Model,Model.xInit,u_nom);

%% lsid
Aid = zeros(Model.nsys,Model.nsys,horizon);
Bid = zeros(Model.nsys,Model.nu,horizon);
if strcmp(Model.name, 'fish')
for i = 1:1:horizon
    delta_x1 = Task.ptb*1*randn(Model.nsys,nSim);
    delta_u = Task.ptb*1*randn(Model.nu,nSim);  
    delta_x2 = zeros(Model.nsys,nSim);
    
    dx1 = delta_x1(4:7,:)+x_nom(4:7,i);
    for j=1:1:Task.nSim
        dx1(:,j) = dx1(:,j)./norm(dx1(:,j));
        delta_x1(4:7,j) = dx1(:,j)-x_nom(4:7,i);
    end
    
    for j = 1:1:nSim
        % plus
        mexstep('set','qpos',x_nom(1:Model.nq,i)+delta_x1(1:Model.nq,j),Model.nq);
        mexstep('set','qvel',x_nom(Model.nq+1:Model.nsys,i)+delta_x1(Model.nq+1:Model.nsys,j),Model.nv);
        mexstep('forward');
        mexstep('set','ctrl',u_nom(:,i)+delta_u(:,j),Model.nu);
        mexstep('step',1);
        delta_x2(1:Model.nq,j)=mexstep('get','qpos')'-x_nom(1:Model.nq,i+1);
        delta_x2(Model.nq+1:Model.nsys,j)=mexstep('get','qvel')'-x_nom(Model.nq+1:Model.nsys,i+1);
    end
    [Q,R]=qr([delta_x1;delta_u]');
    AB=(R\Q'*delta_x2')';
    Aid(:,:,i) = AB(:,1:Model.nsys);
    Bid(:,:,i) = AB(:,Model.nsys+1:end);
end
else
for i = 1:1:horizon
    delta_x1 = Task.ptb*1*randn(Model.nsys,nSim);
    delta_u = Task.ptb*1*randn(Model.nu,nSim);  
    delta_x2 = zeros(Model.nsys,nSim);
    for j = 1:1:nSim
        % plus
        mexstep('set','qpos',x_nom(1:Model.nq,i)+delta_x1(1:Model.nq,j),Model.nq);
        mexstep('set','qvel',x_nom(Model.nq+1:Model.nsys,i)+delta_x1(Model.nq+1:Model.nsys,j),Model.nv);
        mexstep('forward');
        mexstep('set','ctrl',u_nom(:,i)+delta_u(:,j),Model.nu);
        mexstep('step',1);
        delta_x2(1:Model.nq,j)=mexstep('get','qpos')';
        delta_x2(Model.nq+1:Model.nsys,j)=mexstep('get','qvel')';
        % minus
        mexstep('set','qpos',x_nom(1:Model.nq,i)-delta_x1(1:Model.nq,j),Model.nq);
        mexstep('set','qvel',x_nom(Model.nq+1:Model.nsys,i)-delta_x1(Model.nq+1:Model.nsys,j),Model.nv);
        mexstep('forward');
        mexstep('set','ctrl',u_nom(:,i)-delta_u(:,j),Model.nu);
        mexstep('step',1);
        delta_x2(1:Model.nq,j)=(delta_x2(1:Model.nq,j)-mexstep('get','qpos')')/2;
        delta_x2(Model.nq+1:Model.nsys,j)=(delta_x2(Model.nq+1:Model.nsys,j)-mexstep('get','qvel')')/2;
    end
    [Q,R]=qr([delta_x1;delta_u]');
    AB=(R\Q'*delta_x2')';
    Aid(:,:,i) = AB(:,1:Model.nsys);
    Bid(:,:,i) = AB(:,Model.nsys+1:end);
end
end

%% prediction check with rolling window
if checkResult == true
TEST_NUM=1; % number of monte-carlo runs to verify the fitting result
ucheck=0.01*u_max*randn(Model.nu,horizon,TEST_NUM); % input used for checking
x_sim=zeros(Model.nsys,horizon+1,TEST_NUM); % output from real system
x_pred=zeros(Model.nsys,horizon+1,TEST_NUM); % output from arma model
for j=1:1:TEST_NUM
    mexstep('reset');
    mexstep('set','qpos',cur_state(1:Model.nq,1),Model.nq);
    mexstep('set','qvel',cur_state(Model.nq+1:Model.nsys,1),Model.nv);
    mexstep('forward');
    for i=1:1:horizon
        mexstep('set','ctrl',u_nom(:,i)+ucheck(:,i,j),Model.nu);
        mexstep('step',1);
        x2(1:Model.nq,1)=mexstep('get','qpos');
        x2(Model.nq+1:Model.nsys,1)=mexstep('get','qvel');
        x_sim(:,i+1,j)=x2-x_nom(:,i+1);
    end
end

x_pred(:,1,:)=x_sim(:,1,:); % manually match the first few steps
for i=1:1:horizon % start to apply input after having enough data
    M2=[x_pred(:,i,:);ucheck(:,i,:)];
    x_pred(:,i+1,:)=[Aid(:,:,i) Bid(:,:,i)]*M2;
end

% plot y_sim and y_pred to check if they match
figure()
plot([x_pred(1,:,1)' x_sim(1,:,1)']); % plot only the first state
legend('pred1','sim1');
end

% LQR cost matrices
Ri = sig_r*1*eye(Model.nu);
Qi = sig_q * eye(Model.nsys);
OS = zeros(Model.nsys, Model.nsys, horizon+1);
TK = zeros(Model.nu, Model.nsys, horizon);
OS(:, :, horizon+1) = sig_f * eye(Model.nsys);
for i= horizon:-1:1
    OS(:, :, i) = Aid(:, :, i)' * (OS(:, :, i+1) - OS(:, :, i + 1) * Bid(:, :, i) / (Bid(:, :, i)' * OS(:, :, i +1) * Bid(:, :, i) + Ri) * Bid(:, :, i)' * OS(:, :, i + 1)) * Aid(:, :, i) + Qi;
end
for i = 1:1:horizon
    TK(:, :, i) = (Ri + Bid(:, :, i)' * OS(:, :, i+1) * Bid(:, :, i)) \ Bid(:, :, i)' * OS(:, :, i+1) * Aid(:, :, i);
end

% %% LQG Addition
% Wi = 1e1*eye(Model.nu);
% Vi = 1e-2*eye(Task.nm);
% 
% D_aug = zeros(Task.nm*Task.qx+Model.nu*(Task.qu-1),Model.nu);
% PS = zeros(Task.nm*Task.qx+Model.nu*(Task.qu-1), Task.nm*Task.qx+Model.nu*(Task.qu-1), horizon+1);
% LK = zeros(Task.nm*Task.qx+Model.nu*(Task.qu-1), Task.nm, horizon+1);
% for i=max(Task.qx,Task.qu):1:horizon
%     C_aug = [eye(Task.nm) zeros(Task.nm,Task.nm*(Task.qx-1)+Model.nu*(Task.qu-1))];
%     D_aug1 = zeros(Task.nm,Model.nu);
%     for j = 1:Task.qu
%         D_aug1 = D_aug1 + fitcoef(:,Task.nm*Task.qx+Model.nu*(Task.qu-1)+1:Task.nm*Task.qx+Model.nu*Task.qu,i);
%     end
%     D_aug(:,:,i) = [D_aug1;zeros(Task.nm*(Task.qx-1)+Model.nu*(Task.qu-1),Model.nu)];
% end
% 
% for i= 1:1:horizon
%     PS(:, :, i+1) = Aid(:,:,i) * (PS(:, :, i) - PS(:, :, i) * C_aug' / (C_aug * PS(:, :, i) * C_aug' + Vi) * C_aug * PS(:, :, i)) * Aid(:,:,i)' + D_aug(:,:,i)*Wi*D_aug(:,:,i)';
%     LK(:, :, i) = PS(:, :, i)*C_aug'/ (C_aug * PS(:, :, i) * C_aug' + Vi);
% end
% LK(:, :, horizon+1) = PS(:, :, horizon+1)*C_aug'/ (C_aug * PS(:, :, horizon+1) * C_aug' + Vi);
% 
% %% save data to .mat file for closed-loop testing and creating animation in MuJoCo C++ software
% MYE=zeros(Task.nm*Task.qx+Model.nu*(Task.qu-1),Task.nm*Task.qx+Model.nu*(Task.qu-1),horizon);
% MU1=zeros(Task.nm*Task.qx+Model.nu*(Task.qu-1),Model.nu,horizon);
% MYM=zeros(Task.nm*Task.qx+Model.nu*(Task.qu-1),Task.nm,horizon);
MTK=TK;
% MCK=Task.Ck;
% MQ=Task.qx;
% MQU=Task.qu;
% for i= 1:1:horizon
%     MYE(:, :, i) = Aid(:,:,i)-LK(:,:,i+1)*C_aug*Aid(:,:,i);
%     MU1(:,:,i) = Bid(:,:,i)-LK(:,:,i+1)*C_aug*Bid(:,:,i);
%     MYM(:, :, i) = LK(:,:,i+1);
% end
% if save2file == true
%     save('feedbackiolqg.mat','MTK','MCK','MQ','MQU','MYE','MU1','MYM');
% end

%% closed-loop performance test
if checkResult == true
delta_u_test=0.1*u_max*randn(Model.nu,horizon,TEST_NUM);
x_open=zeros(Model.nsys,horizon+1,TEST_NUM); % open-loop output, no feedback
x_closed=zeros(Model.nsys,horizon+1,TEST_NUM); % closed-loop output, with the above feedback
% y_est = zeros(Task.nm*Task.qx+Model.nu*(Task.qu-1),horizon+1,TEST_NUM);
u_feedback=zeros(Model.nu,horizon,TEST_NUM); % input feedback term
for j=1:1:TEST_NUM
    % open-loop
    mexstep('reset');
    mexstep('set','qpos',cur_state(1:Model.nq,1),Model.nq);
    mexstep('set','qvel',cur_state(Model.nq+1:Model.nsys,1),Model.nv);
    mexstep('forward');
    for i=1:1:horizon
        mexstep('set','ctrl',u_nom(:,i)+delta_u_test(:,i,j),Model.nu);
        mexstep('step',1);
        x2(1:Model.nq,1)=mexstep('get','qpos');
        x2(Model.nq+1:Model.nsys,1)=mexstep('get','qvel');
        x_open(:,i+1,j)=x2-x_nom(:,i+1);
    end
    % closed-loop
    mexstep('reset');
    mexstep('set','qpos',cur_state(1:Model.nq,1),Model.nq);
    mexstep('set','qvel',cur_state(Model.nq+1:Model.nsys,1),Model.nv);
    mexstep('forward');
    for i=1:1:horizon
        x1(1:Model.nq,1)=mexstep('get','qpos');
        x1(Model.nq+1:Model.nsys,1)=mexstep('get','qvel');
        x_closed(:,i,j)=x1-x_nom(:,i);
            % LQG
%             y_est(:,i,j)=Aid(:,:,i-1)*y_est(:,i-1,j)+Bid(:,:,i-1)*u_feedback(Model.nu*(horizon-i+2)+1:Model.nu*(horizon-i+3),j)+LK(:,:,i)*(x_closed(Task.nm*(horizon-i+1)+1:Task.nm*(horizon-i+2),j)-C_aug*(Aid(:,:,i-1)*y_est(:,i-1,j)+Bid(:,:,i-1)*u_feedback(Model.nu*(horizon-i+2)+1:Model.nu*(horizon-i+3),j)));
%             u_feedback(Model.nu*(horizon-i+1)+1:Model.nu*(horizon-i+2),j)=-TK(:,:,i)*y_est(:,i,j);            
  
            % LQR
        u_feedback(:,i,j)=-TK(:,:,i)*x_closed(:,i,j);

        mexstep('set','ctrl',u_nom(:,i)+delta_u_test(:,i,j)+u_feedback(:,i,j),Model.nu);
        mexstep('step',1);
    end
    x1(1:Model.nq,1)=mexstep('get','qpos');
    x1(Model.nq+1:Model.nsys,1)=mexstep('get','qvel');
    x_closed(:,horizon+1,j)=x1-x_nom(:,horizon+1);
end
% avg_measurement = mean((abs(y_closed)),1)
% max_measurement = max(abs(y_closed))

% plot open-loop closed-loop comparison
y_closed_avg=mean((x_closed),3);
y_open_avg=mean((x_open),3);
figure()
plot([y_open_avg(1,:)' y_closed_avg(1,:)']); % plot only the first state
legend('openloop1','closedloop1');
% closedloop=mean(abs(y_closed_avg))
end
