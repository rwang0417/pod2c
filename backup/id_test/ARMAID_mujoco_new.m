clear all;clc;warning off;tic;
%% cartpole
% POS_NUM = 2;
% VEL_NUM = 2;
% SYS_NUM = 4;
% IN_NUM = 1;
% OUT_NUM = 4;
% STEP_NUM = 30;
% MODEL = 'cartpole.xml';
% X_INIT = [0;0;0;0];
% Ck = eye(OUT_NUM);%[1 0 0 0;0 1 0 0];%

%% cheetah
% POS_NUM = 9;
% VEL_NUM = 9;
% SYS_NUM = 18;
% IN_NUM = 6;
% OUT_NUM = 18;
% STEP_NUM = 300;
% MODEL = 'cheetah.xml';
% X_INIT = zeros(SYS_NUM,1);
% X_TARGET = [zeros(POS_NUM,1);3;0;zeros(VEL_NUM-2,1)]; % rootx, rootz
% Ck = eye(OUT_NUM);%[1 0 0 0;0 1 0 0];%

%% slip
POS_NUM = 7;
VEL_NUM = 7;
SYS_NUM = 14;
IN_NUM = 4;
OUT_NUM = 14;
STEP_NUM = 2000;
MODEL = 'slip.xml';
X_INIT = zeros(SYS_NUM,1);
Ck = eye(OUT_NUM);%[1 0 0 0;0 1 0 0];%

%% tuning
ID_PERT_COEF = 0.0001;
TRIAL_NUM = 200;
STEP_DIV = 1; SKIPo = 2; SKIP = 10; 
q=1;
qu=1;

%% variables
Aid = zeros(SYS_NUM,SYS_NUM,STEP_NUM);
Bid = zeros(SYS_NUM,IN_NUM,STEP_NUM);

%% read nominal control sequence
fid = fopen('result0_slip.txt','r');
U = fscanf(fid, '%f');
fclose(fid);
u_norm = reshape(U(1:IN_NUM*STEP_NUM), IN_NUM, STEP_NUM);
umax = max(max(abs(u_norm)));

%% or generate random sequence
% u_norm = .1*randn(IN_NUM, STEP_NUM);
% umax = max(max(abs(u_norm)));

%% nominal trajectory
Y_NORM = zeros(OUT_NUM,STEP_NUM+1);
X_NORM = zeros(OUT_NUM,STEP_NUM+1);
x2 = zeros(SYS_NUM, 1); X_NORM(:,1) = X_INIT;
mexstep('load',MODEL); % load model
mexstep('set','qpos',X_INIT(1:POS_NUM,1),POS_NUM); % set initial states
mexstep('set','qvel',X_INIT(POS_NUM+1:SYS_NUM,1),VEL_NUM);
mexstep('forward'); % update sites and sensors
Y_NORM(:,1)=Ck*X_INIT(:,1);
for i = 1 : 1 : STEP_NUM
    mexstep('set','ctrl',u_norm(:,i),IN_NUM);
    mexstep('step',1);
    x2(1:POS_NUM,1)=mexstep('get','qpos')';
    x2(POS_NUM+1:SYS_NUM,1)=mexstep('get','qvel')';
    X_NORM(:,i+1) = x2;
    Y_NORM(:,i+1)=Ck*x2;
end

%% sysid
% arma
ids = floor(linspace(1,STEP_NUM,STEP_DIV+1)); ID_START = ids(1:end-1); ID_END = [ID_START(2:end)+SKIP,STEP_NUM];
delta_ybatch=zeros(OUT_NUM*(STEP_NUM+1),TRIAL_NUM);
delta_ysingle=zeros(OUT_NUM*(STEP_NUM+1),TRIAL_NUM);
fitcoefbatch=zeros(OUT_NUM,OUT_NUM*q+IN_NUM*qu,STEP_NUM);
fitcoefsingle=zeros(OUT_NUM,OUT_NUM*q+IN_NUM*qu,STEP_NUM);
skipu = 0;
delta_u=[ID_PERT_COEF*1*randn(IN_NUM*(STEP_NUM+1-skipu),TRIAL_NUM);0.01*randn(IN_NUM*skipu,TRIAL_NUM)];
for k=1:1:STEP_DIV
    for j=1:1:TRIAL_NUM
        mexstep('reset');
        mexstep('set','qpos',X_NORM(1:POS_NUM,ID_START(k)),POS_NUM);
        mexstep('set','qvel',X_NORM(POS_NUM+1:SYS_NUM,ID_START(k)),VEL_NUM);
        mexstep('forward');
        for i=ID_START(k):1:ID_END(k)
            mexstep('set','ctrl',u_norm(:,i)+delta_u(IN_NUM*(STEP_NUM-i+1)+1:IN_NUM*(STEP_NUM-i+2),j),IN_NUM);
            mexstep('step',1);
            x2(1:POS_NUM,1)=mexstep('get','qpos')';
            x2(POS_NUM+1:SYS_NUM,1)=mexstep('get','qvel')';
            delta_ybatch(OUT_NUM*(STEP_NUM-i)+1:OUT_NUM*(STEP_NUM-i+1),j)=Ck*x2-Y_NORM(:,i+1);
        end
    end 

    % arma fitting - least square: M1 * fitcoef = delta_y
    for i=ID_START(k)+SKIPo:1:ID_END(k) % skip the first few steps to wait for enough data
        M1=[delta_ybatch(OUT_NUM*(STEP_NUM-i+1)+1:OUT_NUM*(STEP_NUM-i+1+q),:);delta_u(IN_NUM*(STEP_NUM-i+1)+1:IN_NUM*(STEP_NUM-i+qu+1),:)];
        [Q,R]=qr(M1');
        fitcoefbatch(:,:,i)=(R\Q'*delta_ybatch(OUT_NUM*(STEP_NUM-i)+1:OUT_NUM*(STEP_NUM-i+1),:)')';
%         fitcoef(:,:,i)=delta_z(OUT_NUM*(STEP_NUM-i)+1:OUT_NUM*(STEP_NUM-i+1),:)*M1'/(M1*M1');
        rb(i)=sqrt(mean(mean((delta_ybatch(OUT_NUM*(STEP_NUM-i)+1:OUT_NUM*(STEP_NUM-i+1),:)-fitcoefbatch(:,:,i)*M1).^2,1))); % residual
    end
end
rmaxb = max(rb)

% lsid
for i = 1:1:STEP_NUM
    delta_x1 = ID_PERT_COEF*1*randn(SYS_NUM,TRIAL_NUM);
    delta_u1 = ID_PERT_COEF*1*randn(IN_NUM,TRIAL_NUM);  
    delta_x2 = zeros(SYS_NUM,TRIAL_NUM);
    for j = 1:1:TRIAL_NUM
        % plus
        mexstep('set','qpos',X_NORM(1:POS_NUM,i)+delta_x1(1:POS_NUM,j),POS_NUM);
        mexstep('set','qvel',X_NORM(POS_NUM+1:SYS_NUM,i)+delta_x1(POS_NUM+1:SYS_NUM,j),VEL_NUM);
        mexstep('forward');
        mexstep('set','ctrl',u_norm(:,i)+delta_u1(:,j),IN_NUM);
        mexstep('step',1);
        delta_x2(1:POS_NUM,j)=mexstep('get','qpos')';
        delta_x2(POS_NUM+1:SYS_NUM,j)=mexstep('get','qvel')';
        % minus
        mexstep('set','qpos',X_NORM(1:POS_NUM,i)-delta_x1(1:POS_NUM,j),POS_NUM);
        mexstep('set','qvel',X_NORM(POS_NUM+1:SYS_NUM,i)-delta_x1(POS_NUM+1:SYS_NUM,j),VEL_NUM);
        mexstep('forward');
        mexstep('set','ctrl',u_norm(:,i)-delta_u1(:,j),IN_NUM);
        mexstep('step',1);
        delta_x2(1:POS_NUM,j)=(delta_x2(1:POS_NUM,j)-mexstep('get','qpos')')/2;
        delta_x2(POS_NUM+1:SYS_NUM,j)=(delta_x2(POS_NUM+1:SYS_NUM,j)-mexstep('get','qvel')')/2;
    end
%     AB = delta_x2*[delta_x1;delta_u1]'/([delta_x1;delta_u1]*[delta_x1;delta_u1]');
    [Q,R]=qr([delta_x1;delta_u1]');
    AB=(R\Q'*delta_x2')';
    Aid(:,:,i) = AB(:,1:SYS_NUM);
    Bid(:,:,i) = AB(:,SYS_NUM+1:end);
    rls(i) = sqrt(mean(mean((delta_x2-AB*[delta_x1;delta_u1]).^2,1)));
end
rmaxls=max(rls)
mexstep('exit');

%% prediction check with rolling window
TEST_NUM=100; % number of monte-carlo runs to verify the fitting result
ucheck=0.01*1*randn(IN_NUM*(STEP_NUM+1),TEST_NUM); % input used for checking
y_sim=zeros(OUT_NUM*(STEP_NUM+1),TEST_NUM); % output from real system
y_predb=zeros(OUT_NUM*(STEP_NUM+1),TEST_NUM); % output from arma model
x_predls=zeros(SYS_NUM,STEP_NUM+1,TEST_NUM);
x_rpredls=zeros(SYS_NUM*(STEP_NUM+1),TEST_NUM);
mexstep('load',MODEL); % load model
for j=1:1:TEST_NUM
    mexstep('reset');
    mexstep('set','qpos',X_INIT(1:POS_NUM,1),POS_NUM);
    mexstep('set','qvel',X_INIT(POS_NUM+1:SYS_NUM,1),VEL_NUM);
    mexstep('forward');
    for i=1:1:STEP_NUM
        mexstep('set','ctrl',u_norm(:,i)+ucheck(IN_NUM*(STEP_NUM-i+1)+1:IN_NUM*(STEP_NUM-i+2),j),IN_NUM);
        mexstep('step',1);
        x2(1:POS_NUM,1)=mexstep('get','qpos');
        x2(POS_NUM+1:SYS_NUM,1)=mexstep('get','qvel');
        y_sim(OUT_NUM*(STEP_NUM-i)+1:OUT_NUM*(STEP_NUM-i+1),j)=Ck*x2-Y_NORM(:,i+1);
    end
end

% batch
y_predb(OUT_NUM*(STEP_NUM-SKIP(1))+1:OUT_NUM*(STEP_NUM+1),:)=y_sim(OUT_NUM*(STEP_NUM-SKIP(1))+1:OUT_NUM*(STEP_NUM+1),:); % manually match the first few steps
for i=SKIP(1)+1:1:STEP_NUM % start to apply input after having enough data
    M2=[y_predb(OUT_NUM*(STEP_NUM-i+1)+1:OUT_NUM*(STEP_NUM-i+1+q),:);ucheck(IN_NUM*(STEP_NUM-i+1)+1:IN_NUM*(STEP_NUM-i+qu+1),:)];
    y_predb(OUT_NUM*(STEP_NUM-i)+1:OUT_NUM*(STEP_NUM-i+1),:)=fitcoefbatch(:,:,i)*M2;
end

% ls, first SKIP(1) steps set to zero
for j=1:1:TEST_NUM
    x_predls(:,SKIP(1)+1,j)=y_sim(OUT_NUM*(STEP_NUM-SKIP(1))+1:OUT_NUM*(STEP_NUM-SKIP(1)+1),j); 
    for i=SKIP(1)+1:1:STEP_NUM
        x_predls(:,i+1,j)=Aid(:,:,i)*x_predls(:,i,j)+Bid(:,:,i)*ucheck(IN_NUM*(STEP_NUM-i+1)+1:IN_NUM*(STEP_NUM-i+2),j);
    end
    x_rpredls(:,j) = reshape(fliplr(x_predls(:,:,j)),SYS_NUM*(STEP_NUM+1),1);
end

tra_diff_batch=norm(y_predb(1:OUT_NUM*(STEP_NUM-SKIP(end)),:)-y_sim(1:OUT_NUM*(STEP_NUM-SKIP(end)),:))
tra_diff_ls=norm(x_rpredls(1:OUT_NUM*(STEP_NUM-SKIP(end)),:)-y_sim(1:OUT_NUM*(STEP_NUM-SKIP(end)),:))

% plot y_sim and y_pred to check if they match
rpredb=mean(reshape(y_predb,OUT_NUM,STEP_NUM+1,TEST_NUM),3);
rsim=mean(reshape(y_sim,OUT_NUM,STEP_NUM+1,TEST_NUM),3);
figure()
% plot([fliplr(rpredb(1,:,1))' fliplr(rsim(1,:,1))' fliplr(rpredb(2,:,1))' fliplr(rsim(2,:,1))' fliplr(rpreds(1,:,1))' fliplr(rpreds(2,:,1))']); % plot only the first state
% legend('pred1b','sim1','pred2b','sim2','pred1s','pred2s');
plot([fliplr(rpredb(1,:))' fliplr(rsim(1,:))' mean(x_predls(1,:,:),3)']); % plot only the first state
legend('pred1b','sim1','ls1');
figure()
plot([fliplr(rsim(1,:))' fliplr(rpredb(1,:))']); % plot only the first state
legend('sim1','pred1b');
xlabel('step')
% ylim([0,0.1])
% figure()
% plot(rb)
% xlabel('step')
% ylabel('residual')

mexstep('exit');