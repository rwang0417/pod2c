clear all;clc;warning off;tic;
%% pendulum
% POS_NUM = 1;
% VEL_NUM = 1;
% SYS_NUM = 2;
% IN_NUM = 1;
% OUT_NUM = 2;
% STEP_NUM = 30;
% MODEL = 'pendulum.xml';
% X_INIT = [pi;0];
% X_TARGET = [0;0];
% Ck=eye(OUT_NUM);
% Dk=0;

%% cartpole
% POS_NUM = 2;
% VEL_NUM = 2;
% SYS_NUM = 4;
% IN_NUM = 1;
% OUT_NUM = 4;
% STEP_NUM = 30;
% MODEL = 'cartpole.xml';
% X_INIT = [0;0;0;0];
% X_TARGET = [0;pi;0;0];
% Ck=eye(OUT_NUM);
% Dk=0;

%% cart
% POS_NUM = 1;
% VEL_NUM = 1;
% SYS_NUM = 2;
% IN_NUM = 1;
% OUT_NUM = 2;
% STEP_NUM = 40;
% MODEL = 'cart.xml';
% X_INIT = [1;0];
% X_TARGET = [0;0];
% Ck=eye(OUT_NUM);
% Dk=0;

%% swimmer3
POS_NUM = 5;
VEL_NUM = 5;
SYS_NUM = 10;
IN_NUM = 2;
OUT_NUM = 5;
MODEL = 'swimmer3.xml';
STEP_NUM = 950;
X_INIT = zeros(SYS_NUM,1);
X_TARGET = [0.5;-0.5;pi/4;zeros(SYS_NUM-3,1)];
Ck = eye(SYS_NUM);%[eye(POS_NUM) zeros(VEL_NUM,VEL_NUM) ];%

%% swimmer6
% POS_NUM = 8;
% VEL_NUM = 8;
% SYS_NUM = 16;
% IN_NUM = 5;
% OUT_NUM= 16;
% MODEL = 'swimmer6.xml';
% STEP_NUM = 1500;
% X_INIT = zeros(SYS_NUM,1);
% X_TARGET = [0.5;-0.5;pi/4;zeros(SYS_NUM-3,1)];
% Ck=eye(SYS_NUM);%[eye(POS_NUM) zeros(VEL_NUM,VEL_NUM)];%

%% tuning
LS_PERT_COEF = 0.01;
TRIAL_NUM = 100;

%% variables
X_NORM = zeros(SYS_NUM,STEP_NUM+1);
Aid = zeros(SYS_NUM,SYS_NUM,STEP_NUM);
Bid = zeros(SYS_NUM,IN_NUM,STEP_NUM);

%% read nominal control sequence
fid = fopen('result0.txt','r');
U = fscanf(fid, '%f');
fclose(fid);
u_norm = reshape(U, IN_NUM, STEP_NUM);
umax = max(max(abs(u_norm)));

%% read time varying Ak, Bk
Ak=zeros(SYS_NUM,SYS_NUM,STEP_NUM);
Bk=zeros(SYS_NUM,IN_NUM,STEP_NUM);
fid = fopen('lnr.txt','r');
Ua  = fscanf(fid, '%f %f %f');
fclose(fid);
La = reshape(Ua, SYS_NUM + IN_NUM, SYS_NUM * STEP_NUM);
for i = 1 : STEP_NUM
    Ak(:, :, i) = La(1: SYS_NUM, (i-1)*SYS_NUM + 1: i* SYS_NUM)';
    Bk(:, :, i) = La(SYS_NUM + 1 : SYS_NUM + IN_NUM, (i-1)*SYS_NUM + 1 : i * SYS_NUM)';
end

%% nominal trajectory
mexstep('load',MODEL);
mexstep('set','qpos',X_INIT(1:POS_NUM,1),POS_NUM);
mexstep('set','qvel',X_INIT(POS_NUM+1:SYS_NUM,1),VEL_NUM);
mexstep('forward');
X_NORM(:,1) = X_INIT;
for i = 1 : 1 : STEP_NUM  
    mexstep('set','ctrl',u_norm(:,i),IN_NUM);
    mexstep('step',1);
    X_NORM(1:POS_NUM,i+1)=mexstep('get','qpos')';
    X_NORM(POS_NUM+1:SYS_NUM,i+1)=mexstep('get','qvel')';
end
        
%% lsid
for i = 1:1:STEP_NUM
    delta_x1 = LS_PERT_COEF*1*randn(SYS_NUM,TRIAL_NUM);
    delta_u = LS_PERT_COEF*1*randn(IN_NUM,TRIAL_NUM);  
    delta_x2 = zeros(SYS_NUM,TRIAL_NUM);
    for j = 1:1:TRIAL_NUM
        % plus
        mexstep('set','qpos',X_NORM(1:POS_NUM,i)+delta_x1(1:POS_NUM,j),POS_NUM);
        mexstep('set','qvel',X_NORM(POS_NUM+1:SYS_NUM,i)+delta_x1(POS_NUM+1:SYS_NUM,j),VEL_NUM);
        mexstep('forward');
        mexstep('set','ctrl',u_norm(:,i)+delta_u(:,j),IN_NUM);
        mexstep('step',1);
        delta_x2(1:POS_NUM,j)=mexstep('get','qpos')';
        delta_x2(POS_NUM+1:SYS_NUM,j)=mexstep('get','qvel')';
        % minus
        mexstep('set','qpos',X_NORM(1:POS_NUM,i)-delta_x1(1:POS_NUM,j),POS_NUM);
        mexstep('set','qvel',X_NORM(POS_NUM+1:SYS_NUM,i)-delta_x1(POS_NUM+1:SYS_NUM,j),VEL_NUM);
        mexstep('forward');
        mexstep('set','ctrl',u_norm(:,i)-delta_u(:,j),IN_NUM);
        mexstep('step',1);
        delta_x2(1:POS_NUM,j)=(delta_x2(1:POS_NUM,j)-mexstep('get','qpos')')/2;
        delta_x2(POS_NUM+1:SYS_NUM,j)=(delta_x2(POS_NUM+1:SYS_NUM,j)-mexstep('get','qvel')')/2;
    end
    AB = delta_x2*[delta_x1;delta_u]'/([delta_x1;delta_u]*[delta_x1;delta_u]');
    Aid(:,:,i) = AB(:,1:SYS_NUM);
    Bid(:,:,i) = AB(:,SYS_NUM+1:end);
end
mexstep('exit');

save('LSID.mat','Aid','Bid')