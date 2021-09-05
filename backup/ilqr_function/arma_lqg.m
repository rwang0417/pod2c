function [MTK,MYE,MU1,MYM] = arma_lqg(...
    Model,...
    Task,...
    cur_state,...
    u_nom,...
    save2file,...
    checkResult)
% arma_lqg
%
% Description: Run arma-lqg algorithm to find LQG feedback along u_nom
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
sig_f = 10^10;
sig_r = 10^0;

u_max = max(max(abs(u_nom)));
horizon = size(u_nom,2);
nSim = Task.nSim;

%% generate nominal state trajectory
Y_NOM = zeros(Task.nm,horizon+1);
x2 = zeros(Model.nsys,1);
mexstep('reset'); % reset all states and controls
mexstep('set','qpos',cur_state(1:Model.nq,1),Model.nq); % set initial states
mexstep('set','qvel',cur_state(Model.nq+1:Model.nsys,1),Model.nv);
mexstep('forward'); % update sites and sensors
Y_NOM(:,1)=Task.Ck*cur_state(:,1);
for i = 1:1:horizon
    mexstep('set','ctrl',u_nom(:,i),Model.nu);
    mexstep('step',1);
    x2(1:Model.nq,1)=mexstep('get','qpos')';
    x2(Model.nq+1:Model.nsys,1)=mexstep('get','qvel')';
    Y_NOM(:,i+1)=Task.Ck*x2;
end

%% collect data for arma model fitting
delta_u=Task.ptb*u_max*randn(Model.nu*(horizon+1),nSim); % generate random inputs du
delta_y=zeros(Task.nm*(horizon+1),nSim);
delta_y(Task.nm*horizon+1:Task.nm*(horizon+1),:)=Task.statePtb*randn(Task.nm,nSim);
if strcmp(Model.name, 'fish')
    dz1 = delta_y(Task.nm*horizon+4:Task.nm*horizon+7,:)+cur_state(4:7,1);
    for j=1:1:Task.nSim
        dz1(:,j) = dz1(:,j)./norm(dz1(:,j));
        delta_y(Task.nm*horizon+4:Task.nm*horizon+7,j) = dz1(:,j)-cur_state(4:7,1);
    end
end
for j=1:1:nSim
    mexstep('reset');
    mexstep('set','qpos',cur_state(1:Model.nq,1)+delta_y(Task.nm*horizon+1:Task.nm*horizon+Model.nq,j),Model.nq);
    mexstep('set','qvel',cur_state(Model.nq+1:Model.nsys,1)+delta_y(Task.nm*horizon+Model.nq+1:Task.nm*(horizon+1),j),Model.nv);
    mexstep('forward');
    for i=1:1:horizon
        mexstep('set','ctrl',u_nom(:,i)+delta_u(Model.nu*(horizon-i+1)+1:Model.nu*(horizon-i+2),j),Model.nu);
        mexstep('step',1);
        x2(1:Model.nq,1)=mexstep('get','qpos');
        x2(Model.nq+1:Model.nsys,1)=mexstep('get','qvel');
        delta_y(Task.nm*(horizon-i)+1:Task.nm*(horizon-i+1),j)=Task.Ck*x2-Y_NOM(:,i+1); % record output dy
    end
end 

%% least square fitting for the arma model
fitcoef=zeros(Task.nm,Task.nm*Task.qx+Model.nu*Task.qu,horizon); % M1 * fitcoef = delta_y
for i=max(Task.qx,Task.qu):1:horizon % skip the first few steps to wait for enough data
    M1=[delta_y(Task.nm*(horizon-i+1)+1:Task.nm*(horizon-i+1+Task.qx),:);delta_u(Model.nu*(horizon-i+1)+1:Model.nu*(horizon-i+Task.qu+1),:)];
    [Q,R]=qr(M1');
    fitcoef(:,:,i)=(R\Q'*delta_y(Task.nm*(horizon-i)+1:Task.nm*(horizon-i+1),:)')';
%     r(i)=sqrt(mean(mean((delta_y(Task.nm*(horizon-i)+1:Task.nm*(horizon-i+1),:)-fitcoef(:,:,i)*M1).^2,1))); % residual
end
% max_residual =  max(r)

%% prediction check with rolling window
if checkResult == true
TEST_NUM=1; % number of monte-carlo runs to verify the fitting result
ucheck=0.01*u_max*randn(Model.nu*(horizon+1),TEST_NUM); % input used for checking
y_sim=zeros(Task.nm*(horizon+1),TEST_NUM); % output from real system
y_pred=zeros(Task.nm*(horizon+1),TEST_NUM); % output from arma model
for j=1:1:TEST_NUM
    mexstep('reset');
    mexstep('set','qpos',cur_state(1:Model.nq,1),Model.nq);
    mexstep('set','qvel',cur_state(Model.nq+1:Model.nsys,1),Model.nv);
    mexstep('forward');
    for i=1:1:horizon
        mexstep('set','ctrl',u_nom(:,i)+ucheck(Model.nu*(horizon-i+1)+1:Model.nu*(horizon-i+2),j),Model.nu);
        mexstep('step',1);
        x2(1:Model.nq,1)=mexstep('get','qpos');
        x2(Model.nq+1:Model.nsys,1)=mexstep('get','qvel');
        y_sim(Task.nm*(horizon-i)+1:Task.nm*(horizon-i+1),j)=Task.Ck*x2-Y_NOM(:,i+1);
    end
end

y_pred(Task.nm*(horizon-Task.qx-1)+1:Task.nm*(horizon+1),:)=y_sim(Task.nm*(horizon-Task.qx-1)+1:Task.nm*(horizon+1),:); % manually match the first few steps
for i=max(Task.qx,Task.qu):1:horizon % start to apply input after having enough data
    M2=[y_pred(Task.nm*(horizon-i+1)+1:Task.nm*(horizon-i+1+Task.qx),:);ucheck(Model.nu*(horizon-i+1)+1:Model.nu*(horizon-i+Task.qu+1),:)];
    y_pred(Task.nm*(horizon-i)+1:Task.nm*(horizon-i+1),:)=fitcoef(:,:,i)*M2;
end

% plot y_sim and y_pred to check if they match
rpred=reshape(y_pred,Task.nm,horizon+1,TEST_NUM);
rsim=reshape(y_sim,Task.nm,horizon+1,TEST_NUM);
figure()
plot([fliplr(rpred(1,:,1))' fliplr(rsim(1,:,1))']); % plot only the first state
legend('pred1','sim1');
end

%% time-varying LQR
% construct augmented Ak, Bk
A_aug = zeros(Task.nm*Task.qx+Model.nu*(Task.qu-1),Task.nm*Task.qx+Model.nu*(Task.qu-1),horizon);
B_aug = zeros(Task.nm*Task.qx+Model.nu*(Task.qu-1),Model.nu,horizon);
for i=max(Task.qx,Task.qu):1:horizon 
    A_aug(:,:,i)=[fitcoef(:,1:Task.nm*Task.qx,i),fitcoef(:,Task.nm*Task.qx+Model.nu+1:end,i);
      eye((Task.qx-1)*Task.nm),zeros((Task.qx-1)*Task.nm,Task.nm+Model.nu*(Task.qu-1));
      zeros(Model.nu*(Task.qu-1),Task.nm*Task.qx),[zeros(Model.nu,Model.nu*(Task.qu-1));eye(Model.nu*(Task.qu-2)) zeros(Model.nu*(Task.qu-2),Model.nu)]];
    B_aug(:,:,i)=[fitcoef(:,Task.nm*Task.qx+1:Task.nm*Task.qx+Model.nu,i);zeros(Task.nm*(Task.qx-1),Model.nu);eye(Model.nu*min(Task.qu-1,1));zeros(Model.nu*(Task.qu-2),Model.nu)];
end

% LQR cost matrices
Ri = sig_r *1* eye(Model.nu);
Qi = sig_q * eye(Task.nm*Task.qx+Model.nu*(Task.qu-1));Qi(Task.nm+1:end,Task.nm+1:end)=0*Qi(Task.nm+1:end,Task.nm+1:end);
OS = zeros(Task.nm*Task.qx+Model.nu*(Task.qu-1), Task.nm*Task.qx+Model.nu*(Task.qu-1), horizon+1);
TK = zeros(Model.nu, Task.nm*Task.qx+Model.nu*(Task.qu-1), horizon);
OS(:, :, horizon+1) = sig_f * eye(Task.nm*Task.qx+Model.nu*(Task.qu-1));OS(Task.nm+1:end,Task.nm+1:end)=0*OS(Task.nm+1:end,Task.nm+1:end);
for i= horizon:-1:max(Task.qx,Task.qu)
    OS(:, :, i) = A_aug(:, :, i)' * (OS(:, :, i+1) - OS(:, :, i + 1) * B_aug(:, :, i) / (B_aug(:, :, i)' * OS(:, :, i +1) * B_aug(:, :, i) + Ri) * B_aug(:, :, i)' * OS(:, :, i + 1)) * A_aug(:, :, i) + Qi;
end
for i = max(Task.qx,Task.qu):1:horizon
    TK(:, :, i) = (Ri + B_aug(:, :, i)' * OS(:, :, i+1) * B_aug(:, :, i)) \ B_aug(:, :, i)' * OS(:, :, i+1) * A_aug(:, :, i);
end

%% LQG Addition
Wi = 1e1*eye(Model.nu);
Vi = 1e-2*eye(Task.nm);

D_aug = zeros(Task.nm*Task.qx+Model.nu*(Task.qu-1),Model.nu);
PS = zeros(Task.nm*Task.qx+Model.nu*(Task.qu-1), Task.nm*Task.qx+Model.nu*(Task.qu-1), horizon+1);
LK = zeros(Task.nm*Task.qx+Model.nu*(Task.qu-1), Task.nm, horizon+1);
for i=max(Task.qx,Task.qu):1:horizon
    C_aug = [eye(Task.nm) zeros(Task.nm,Task.nm*(Task.qx-1)+Model.nu*(Task.qu-1))];
    D_aug1 = zeros(Task.nm,Model.nu);
    for j = 1:Task.qu
        D_aug1 = D_aug1 + fitcoef(:,Task.nm*Task.qx+Model.nu*(Task.qu-1)+1:Task.nm*Task.qx+Model.nu*Task.qu,i);
    end
    D_aug(:,:,i) = [D_aug1;zeros(Task.nm*(Task.qx-1)+Model.nu*(Task.qu-1),Model.nu)];
end

for i= 1:1:horizon
    PS(:, :, i+1) = A_aug(:,:,i) * (PS(:, :, i) - PS(:, :, i) * C_aug' / (C_aug * PS(:, :, i) * C_aug' + Vi) * C_aug * PS(:, :, i)) * A_aug(:,:,i)' + D_aug(:,:,i)*Wi*D_aug(:,:,i)';
    LK(:, :, i) = PS(:, :, i)*C_aug'/ (C_aug * PS(:, :, i) * C_aug' + Vi);
end
LK(:, :, horizon+1) = PS(:, :, horizon+1)*C_aug'/ (C_aug * PS(:, :, horizon+1) * C_aug' + Vi);

%% save data to .mat file for closed-loop testing and creating animation in MuJoCo C++ software
MYE=zeros(Task.nm*Task.qx+Model.nu*(Task.qu-1),Task.nm*Task.qx+Model.nu*(Task.qu-1),horizon);
MU1=zeros(Task.nm*Task.qx+Model.nu*(Task.qu-1),Model.nu,horizon);
MYM=zeros(Task.nm*Task.qx+Model.nu*(Task.qu-1),Task.nm,horizon);
MTK=TK;
MCK=Task.Ck;
MQ=Task.qx;
MQU=Task.qu;
for i= 1:1:horizon
    MYE(:, :, i) = A_aug(:,:,i)-LK(:,:,i+1)*C_aug*A_aug(:,:,i);
    MU1(:,:,i) = B_aug(:,:,i)-LK(:,:,i+1)*C_aug*B_aug(:,:,i);
    MYM(:, :, i) = LK(:,:,i+1);
end
if save2file == true
    save('feedbackiolqg.mat','MTK','MCK','MQ','MQU','MYE','MU1','MYM');
end

%% closed-loop performance test
if checkResult == true
vk = 0.00*randn(Task.nm,horizon+1,TEST_NUM);
delta_u_test=0.1*u_max*randn(Model.nu*(horizon+1),TEST_NUM);
y_open=zeros(Task.nm*(horizon+1),TEST_NUM); % open-loop output, no feedback
y_closed=zeros(Task.nm*(horizon+1),TEST_NUM); % closed-loop output, with the above feedback
y_est = zeros(Task.nm*Task.qx+Model.nu*(Task.qu-1),horizon+1,TEST_NUM);
u_feedback=zeros(Model.nu*(horizon+1),TEST_NUM); % input feedback term
for j=1:1:TEST_NUM
    % open-loop
    mexstep('reset');
    mexstep('set','qpos',cur_state(1:Model.nq,1),Model.nq);
    mexstep('set','qvel',cur_state(Model.nq+1:Model.nsys,1),Model.nv);
    mexstep('forward');
    for i=1:1:horizon
        if i >= max(Task.qx,Task.qu) % start to apply perturbation after certain steps
            mexstep('set','ctrl',u_nom(:,i)+delta_u_test(Model.nu*(horizon-i+1)+1:Model.nu*(horizon-i+2),j),Model.nu);
        else
            mexstep('set','ctrl',u_nom(:,i),Model.nu);
        end
        mexstep('step',1);
        x2(1:Model.nq,1)=mexstep('get','qpos');
        x2(Model.nq+1:Model.nsys,1)=mexstep('get','qvel');
        y_open(Task.nm*(horizon-i)+1:Task.nm*(horizon-i+1),j)=Task.Ck*x2-Y_NOM(:,i+1)+vk(:,i+1,j);
    end
    % closed-loop
    mexstep('reset');
    mexstep('set','qpos',cur_state(1:Model.nq,1),Model.nq);
    mexstep('set','qvel',cur_state(Model.nq+1:Model.nsys,1),Model.nv);
    mexstep('forward');
    for i=1:1:horizon
        x1(1:Model.nq,1)=mexstep('get','qpos');
        x1(Model.nq+1:Model.nsys,1)=mexstep('get','qvel');
        y_closed(Task.nm*(horizon-i+1)+1:Task.nm*(horizon-i+2),j)=Task.Ck*x1-Y_NOM(:,i)+vk(:,i,j);
        if i >= max(Task.qx,Task.qu)+1 % start to apply perturbation after certain steps
            % LQG
            y_est(:,i,j)=A_aug(:,:,i-1)*y_est(:,i-1,j)+B_aug(:,:,i-1)*u_feedback(Model.nu*(horizon-i+2)+1:Model.nu*(horizon-i+3),j)+LK(:,:,i)*(y_closed(Task.nm*(horizon-i+1)+1:Task.nm*(horizon-i+2),j)-C_aug*(A_aug(:,:,i-1)*y_est(:,i-1,j)+B_aug(:,:,i-1)*u_feedback(Model.nu*(horizon-i+2)+1:Model.nu*(horizon-i+3),j)));
            u_feedback(Model.nu*(horizon-i+1)+1:Model.nu*(horizon-i+2),j)=-TK(:,:,i)*y_est(:,i,j);            
  
            % LQR
%             u_feedback(Model.nu*(horizon-i+1)+1:Model.nu*(horizon-i+2),j)=-TK(:,:,i)*[y_closed(Task.nm*(horizon-i+1)+1:Task.nm*(horizon-i+1+Task.qx),j);u_feedback(Model.nu*(horizon-i+2)+1:Model.nu*(horizon-i+1+Task.qu),j)];

            mexstep('set','ctrl',u_nom(:,i)+delta_u_test(Model.nu*(horizon-i+1)+1:Model.nu*(horizon-i+2),j)+u_feedback(Model.nu*(horizon-i+1)+1:Model.nu*(horizon-i+2),j),Model.nu);
        else
            mexstep('set','ctrl',u_nom(:,i),Model.nu);
        end
        mexstep('step',1);
    end
    x1(1:Model.nq,1)=mexstep('get','qpos');
    x1(Model.nq+1:Model.nsys,1)=mexstep('get','qvel');
    y_closed(1:Task.nm,j)=Task.Ck*x1-Y_NOM(:,horizon+1)+vk(:,horizon+1,j);
end
% avg_measurement = mean((abs(y_closed)),1)
% max_measurement = max(abs(y_closed))

% plot open-loop closed-loop comparison
y_closed_avg=mean((y_closed),2);
y_open_avg=mean((y_open),2);
ropenavg=reshape(y_open_avg,Task.nm,horizon+1);
rclosedavg=reshape(y_closed_avg,Task.nm,horizon+1);
figure()
plot([fliplr(ropenavg(1,1:horizon-max(Task.qx,Task.qu)))' fliplr(rclosedavg(1,1:horizon-max(Task.qx,Task.qu)))']); % plot only the first state
legend('openloop1','closedloop1');
% closedloop=mean(abs(y_closed_avg))
end
