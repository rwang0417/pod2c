clear all;clc;warning off;tic;
%% pendulum
% POS_NUM = 1;
% VEL_NUM = 1;
% SYS_NUM = 2;
% IN_NUM = 1;
% OUT_NUM= 2;
% MODEL = 'pendulum.xml';
% STEP_NUM = 30;
% X_INIT = [pi;0];
% X_TARGET = [0;0];
% Ck=eye(SYS_NUM);%[1 0];%

% full obsv
% ID_PERT_COEF = 0.001;%0.0001
% TRIAL_NUM = 60;
% ITE_NUM = 100;
% STEP_DIV = 1; SKIPo = 3; SKIP = 300; 
% q = 1;
% qu = 1;
% Ri = 5*10^-1 * eye(IN_NUM);
% cx = 1*10^-1; cv = 1*10^-2;
% ctx = 2*10^2; ctv = 1*10^2;
% pos only
% ID_PERT_COEF = 0.001;%0.0001
% TRIAL_NUM = 60;
% ITE_NUM = 60;
% STEP_DIV = 1; SKIPo = 3; SKIP = 300; 
% q = 2;
% qu = 2;
% Ri = 2*10^-1 * eye(IN_NUM);
% cx = 1*10^-1; cv = 1*10^-2;
% ctx = 1*10^2; ctv = 1*10^0;

%% cartpole
% POS_NUM = 2;
% VEL_NUM = 2;
% SYS_NUM = 4;
% IN_NUM = 1;
% OUT_NUM = 2;
% STEP_NUM = 30;
% MODEL = 'cartpole.xml';
% X_INIT = [0;0;0;0];
% X_TARGET = [0;pi;0;0];
% Ck = [1 0 0 0;0 1 0 0];%eye(OUT_NUM);%

% q=qu=1
% Ri = 10^-3 * eye(IN_NUM);
% cx = 10^0; cv = 10^-1;
% ctx = 2*10^1; ctv = 10^2;
% Qi(1:OUT_NUM,1:OUT_NUM) = cx * [0.2 0 0 0;0 1 0 0;0 0 .2 0;0 0 0 .2];
% Qf(1:OUT_NUM,1:OUT_NUM) = ctx * [0.5 0 0 0;0 1 0 0;0 0 0.5 0;0 0 0 0.5];
% q=qu=2
% alpha = 0.2;
% TRIAL_NUM = 80;
% ITE_NUM = 60;
% STEP_DIV = 1; SKIPo = 2; SKIP = 300; 
% q = 2;
% qu = 2;
% Ri = 10^-2 * eye(IN_NUM);
% cx = 1*10^-1; cv = 1*10^-1;
% ctx = 5*10^2; ctv = 5*10^4;

%% swimmer3
POS_NUM = 5;
VEL_NUM = 5;
SYS_NUM = 10;
IN_NUM = 2;
OUT_NUM = 10;
MODEL = 'swimmer3.xml';
STEP_NUM = 950;
X_INIT = zeros(SYS_NUM,1);
X_TARGET = [0.5;-0.5;pi/4;zeros(SYS_NUM-3,1)];
Ck = eye(SYS_NUM);%[eye(POS_NUM) zeros(VEL_NUM,VEL_NUM) ];%

% q=qu=1
% ID_PERT_COEF = 0.1;
% Ri = 2*10^-2 * eye(IN_NUM);
% cx = 15*10^0; cv = 0;
% ctx = 18*10^2; ctv = 0;
% Qi(1:3,1:3) = cx * [1 0 0;0 1 0;0 0 0.0];
% Qf(1:3,1:3) = ctx * [1 0 0;0 1 0;0 0 0.0];
% alpha = 0.3; % 0.9 0.8 0.02
% q=qu=1 05/09
% ID_PERT_COEF = 0.001;%0.0001
% TRIAL_NUM = 150;
% ITE_NUM = 500;
% STEP_DIV = 1; SKIPo = 1; SKIP = 300; 
% q = 1;
% qu = 1;
% Ri = 5*10^-2 * eye(IN_NUM);
% cx = 8*10^0; cv = 1*10^-2;
% ctx = 60*10^2; ctv = 1*10^2;
% q=qu=2
% ID_PERT_COEF = 0.01;
% Ri = 5*10^-2 * eye(IN_NUM);
% cx = 15*10^0; cv = 0;
% ctx = 18*10^2; ctv = 0;
% Qi(1:3,1:3) = cx * [1 0 0;0 1 0;0 0 0.0];
% Qf(1:3,1:3) = ctx * [1 0 0;0 1 0;0 0 0.0];
% alpha = 0.3; % 0.9 0.8 0.025

%% swimmer6
% POS_NUM = 8;
% VEL_NUM = 8;
% SYS_NUM = 16;
% IN_NUM = 5;
% OUT_NUM= 8;
% MODEL = 'swimmer6.xml';
% STEP_NUM = 900;
% X_INIT = zeros(SYS_NUM,1);
% X_TARGET = [0.5;-0.6;0;zeros(SYS_NUM-3,1)];
% Ck=[eye(POS_NUM) zeros(VEL_NUM,VEL_NUM) ];%eye(SYS_NUM);%[zeros(4,2) [1;0;0;0] zeros(4,1) [0;1;0;0] [0;0;1;0] zeros(4,1) [0;0;0;1] zeros(4,8)];%

% q=qu=1
% ID_PERT_COEF = 0.0001;
% TRIAL_NUM = 100;
% ITE_NUM = 100;
% STEP_DIV = 1; SKIPo = 50; SKIP = 300; 
% Ri = 2*10^-2 * eye(IN_NUM);
% cx = 8*10^0; cv = 2*10^0;
% ctx = 60*10^2; ctv = 1*10^3;
% Qi(1:3,1:3) = cx * [1 0 0;0 1 0;0 0 0.0];
% Qf(1:3,1:3) = ctx * [1 0 0;0 1 0;0 0 0.0];
% alpha = 0.3;
% q=qu=2 same

%% swimmer15
% POS_NUM = 17;
% VEL_NUM = 17;
% SYS_NUM = 34;
% IN_NUM = 14;
% OUT_NUM= 10;
% MODEL = 'swimmer15.xml';
% STEP_NUM = 2400;
% X_INIT = zeros(SYS_NUM,1);
% X_TARGET = [0.7;0.7;0;zeros(SYS_NUM-3,1)];
% Ck=[[eye(3);zeros(7,3)] zeros(10,1) [zeros(3,1);1;zeros(6,1)] zeros(10,1) [zeros(4,1);1;zeros(5,1)] zeros(10,1) [zeros(5,1);1;zeros(4,1)] zeros(10,1) [zeros(6,1);1;zeros(3,1)] zeros(10,1) [zeros(7,1);1;zeros(2,1)] zeros(10,1) [zeros(8,1);1;0] zeros(10,1) [zeros(9,1);1] zeros(10,17)];%eye(SYS_NUM);%[eye(POS_NUM) zeros(POS_NUM,VEL_NUM) ];%[zeros(8,2) [1;zeros(7,1)] zeros(8,1) [0;1;zeros(6,1)] zeros(8,1) [0;0;1;zeros(5,1)] zeros(8,1) [0;0;0;1;zeros(4,1)] zeros(8,1) [zeros(4,1);1;zeros(3,1)] zeros(8,1) [zeros(5,1);1;zeros(2,1)] zeros(8,1) [zeros(6,1);1;0] zeros(8,1) [zeros(7,1);1] zeros(8,17)];%[eye(POS_NUM) zeros(VEL_NUM,VEL_NUM) ];%[zeros(POS_NUM,POS_NUM) eye(VEL_NUM)];%

% q=qu=1,2
% ID_PERT_COEF = 0.0001;
% TRIAL_NUM = 200;
% ITE_NUM = 100;
% STEP_DIV = 1; SKIPo = 1; SKIP = 300; 
% Ri = 1*10^-3 * eye(IN_NUM);
% cx = 8*10^0; cv = 2*10^0;
% ctx = 100*10^2; ctv = 1*10^0;
% Qi(1:3,1:3) = cx * [1 0 0;0 1 0;0 0 0.0];
% Qf(1:3,1:3) = ctx * [1 0 0;0 1 0;0 0 0.0];
% alpha = 0.3;
% q=qu=5,not successful, need more ite
% ID_PERT_COEF = 0.01;
% TRIAL_NUM = 400;
% ITE_NUM = 300;
% Ri = 2*10^-2 * eye(IN_NUM);
% cx = 5*10^0; cv = 2*10^0;
% ctx = 120*10^2; ctv = 1*10^0;
% alpha=0.3 and start line search from 0.01 in training if alpha<0.01

%% fish
% POS_NUM = 14;
% VEL_NUM = 13;
% SYS_NUM = 27;
% IN_NUM = 6;
% OUT_NUM = 11;
% MODEL = 'fish.xml';
% STEP_NUM = 1200;
% X_INIT = [0;0;0;1;0;0;0;zeros(SYS_NUM-7,1)];
% X_TARGET = [0;0.4;0.2;1;zeros(SYS_NUM-4,1)];
% Ck=[eye(4) zeros(4,23); zeros(7,7) eye(7) zeros(7,13)];%eye(SYS_NUM);%[eye(4) zeros(4,23)];%[eye(POS_NUM) zeros(POS_NUM,VEL_NUM) ];%[zeros(7,7) eye(7) zeros(7,13)];%

% VAR COMPARE
% ID_PERT_COEF = 0.001;%0.0001
% TRIAL_NUM = 400;
% ITE_NUM = 280;
% STEP_DIV = 1; SKIPo = 3; SKIP = 300; 
% q = 2;
% qu = 2;
% Ri = 12*10^-2 * eye(IN_NUM);
% cx = 2*10^0; cv = 1*10^-2;
% ctx = 110*10^2; ctv = 1*10^2;

% q=qu=1
% ID_PERT_COEF = 0.01;
% TRIAL_NUM = 400;
% ITE_NUM = 200;
% STEP_DIV = 1; SKIPo = 1; SKIP = 300; 
% Ri = 1*10^-2 * eye(IN_NUM);
% cx = 12*10^-1; cv = 2*10^0;
% ctx = 100*10^2; ctv = 1*10^0;
% Qi(1:4,1:4) = cx * [1 0 0 0;0 1 0 0;0 0 1 0;0 0 0 1];
% Qf(1:4,1:4) = ctx * [1 0 0 0;0 1 0 0;0 0 1 0;0 0 0 1];
% alpha=0.3;
% q=qu=2 pos only
% ID_PERT_COEF = 0.01;
% TRIAL_NUM = 400;
% ITE_NUM = 200;
% STEP_DIV = 1; SKIPo = 2; SKIP = 300; 
% Ri = 4*10^-2 * eye(IN_NUM);
% cx = 10*10^-1; cv = 2*10^0;
% ctx = 100*10^2; ctv = 1*10^0;
% Qi(1:4,1:4) = cx * [2 0 0 0;0 2 0 0;0 0 2 0;0 0 0 0.5];
% Qf(1:4,1:4) = ctx * [2 0 0 0;0 2 0 0;0 0 2 0;0 0 0 0.5];
% alpha=0.3;
% q=qu=2 xyz+q0+fin pos
% ID_PERT_COEF = 0.0001;
% TRIAL_NUM = 400;
% ITE_NUM = 300;
% STEP_DIV = 1; SKIPo = 2; SKIP = 300; 
% Ri = 4*10^-2 * eye(IN_NUM);
% cx = 2*10^0; cv = 2*10^0;
% ctx = 120*10^2; ctv = 1*10^0;
% Qi(1:4,1:4) = cx * [2 0 0 0;0 2 0 0;0 0 2 0;0 0 0 0.5];
% Qf(1:4,1:4) = ctx * [2 0 0 0;0 2 0 0;0 0 2 0;0 0 0 0.5];
% alpha=0.3 and start line search from 0.01 in training if alpha<0.01
% q=qu=7 xyz+q0
% ID_PERT_COEF = 0.0001;
% TRIAL_NUM = 400;
% ITE_NUM = 300;
% STEP_DIV = 1; SKIPo = 7; SKIP = 300; 
% Ri = 4*10^-2 * eye(IN_NUM);
% cx = 2*10^0; cv = 2*10^0;
% ctx = 120*10^2; ctv = 1*10^0;
% Qi(1:4,1:4) = cx * [2 0 0 0;0 2 0 0;0 0 2 0;0 0 0 0.5];
% Qf(1:4,1:4) = ctx * [2 0 0 0;0 2 0 0;0 0 2 0;0 0 0 0.5];
% alpha=0.3 and start line search from 0.01 in training if alpha<0.01

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
% POS_NUM = 7;
% VEL_NUM = 7;
% SYS_NUM = 14;
% IN_NUM = 4;
% OUT_NUM = 14;
% MODEL = 'slip.xml';
% STEP_NUM = 2000;
% PERT_COEF = .1;
% X_INIT = zeros(SYS_NUM,1);
% X_TARGET = [8;zeros(POS_NUM-1,1);6;0;zeros(VEL_NUM-2,1)];
% Ck = eye(OUT_NUM);%[1 0 0 0;0 1 0 0];%

%% tuning 
ID_PERT_COEF = 0.001;%0.0001
TRIAL_NUM = 150;
ITE_NUM = 500;
STEP_DIV = 1; SKIPo = 1; SKIP = 300; 
q = 1;
qu = 1;
Ri = 5*10^-2 * eye(IN_NUM);
cx = 8*10^0; cv = 1*10^-2;
ctx = 60*10^2; ctv = 1*10^2;
Qi = zeros(OUT_NUM*q+IN_NUM*(qu-1),OUT_NUM*q+IN_NUM*(qu-1));
Qf = zeros(OUT_NUM*q+IN_NUM*(qu-1),OUT_NUM*q+IN_NUM*(qu-1));

% q=qu=2 cartpole + pendulum pos only
% Qi(1:OUT_NUM*2,1:OUT_NUM*2)=[(cx+cv)*eye(OUT_NUM),-cv*eye(OUT_NUM);-cv*eye(OUT_NUM),cv*eye(OUT_NUM)];
% Qf(1:OUT_NUM*2,1:OUT_NUM*2)=[(ctx+ctv)*eye(OUT_NUM),-ctv*eye(OUT_NUM);-ctv*eye(OUT_NUM),ctv*eye(OUT_NUM)];
% fish
% Qi(1:4,1:4) = cx * [2 0 0 0;0 2 0 0;0 0 2 0;0 0 0 0.1];
% Qf(1:4,1:4) = ctx * [2 0 0 0;0 2 0 0;0 0 2 0;0 0 0 0.3];
% swimmers
Qi(1:3,1:3) = cx * [1 0 0;0 1 0;0 0 0.0];
Qf(1:3,1:3) = ctx * [1 0 0;0 1 0;0 0 0.0];
% cheetah
% Qi(10:11,10:11) = cx * [1 0;0 1];
% Qf(10:11,10:11) = ctx * [1 0;0 1];
% pendulum q=qu=1
% Qi(1:OUT_NUM,1:OUT_NUM) = cx * [1 0;0 0.1];
% Qf(1:OUT_NUM,1:OUT_NUM) = ctx * eye(OUT_NUM);
% general
% Qi(1:OUT_NUM,1:OUT_NUM) = cx * eye(OUT_NUM);
% Qf(1:OUT_NUM,1:OUT_NUM) = ctx * eye(OUT_NUM);
% slip
% Qi(8,8) = cv * 1;
% Qf(8,8) = ctv * 1;
% Qi(1:3,1:3) = cx * [.1 0 0;0 0 0;0 0 1.0];
% Qf(1:3,1:3) = ctx * [.1 0 0;0 0 0;0 0 1.0];

%% variables
Z_NORM = zeros(OUT_NUM,STEP_NUM+1); Z_NORM(:,1) = Ck*X_INIT(:,1); 
dz_seq = zeros(OUT_NUM*(STEP_NUM+1),1);
dzt_seq = zeros(OUT_NUM*(STEP_NUM+1),1);
u_norm = 0.01*randn(IN_NUM,STEP_NUM);%zeros(IN_NUM,STEP_NUM);%
du_seq = zeros(IN_NUM*(STEP_NUM+1),1);
Sk = zeros(OUT_NUM*q+IN_NUM*(qu-1),OUT_NUM*q+IN_NUM*(qu-1),STEP_NUM+1); 
Sk(:,:,STEP_NUM+1) = Qf;
vk = zeros(OUT_NUM*q+IN_NUM*(qu-1),STEP_NUM+1);
K = zeros(IN_NUM,OUT_NUM*q+IN_NUM*(qu-1),STEP_NUM);
Kv = zeros(IN_NUM,OUT_NUM*q+IN_NUM*(qu-1),STEP_NUM);
Ku = zeros(IN_NUM,IN_NUM,STEP_NUM);
Quu = zeros(IN_NUM,IN_NUM,STEP_NUM);
kt  = zeros(IN_NUM,STEP_NUM);
cost = zeros(ITE_NUM,1);
fitcoef=zeros(OUT_NUM,OUT_NUM*q+IN_NUM*qu,STEP_NUM); 
A_aug = zeros(OUT_NUM*q+IN_NUM*(qu-1),OUT_NUM*q+IN_NUM*(qu-1),STEP_NUM);
B_aug = zeros(OUT_NUM*q+IN_NUM*(qu-1),IN_NUM,STEP_NUM);
delta_z=zeros(OUT_NUM*(STEP_NUM+1),TRIAL_NUM);
delta_u=zeros(IN_NUM*(STEP_NUM+1),TRIAL_NUM);
ids = floor(linspace(1,STEP_NUM,STEP_DIV+1)); ID_START = ids(1:end-1); ID_END = [ID_START(2:end)+SKIP,STEP_NUM];

%% forward pass
tic; alpha = 0.3; z = 1;
mexstep('load',MODEL);
for ite=1:1:ITE_NUM
% nominal
forward_flag = true; ti = 0; %alpha = .3; 
while forward_flag
    mexstep('reset');
    mexstep('set','qpos',X_INIT(1:POS_NUM,1),POS_NUM);
    mexstep('set','qvel',X_INIT(POS_NUM+1:SYS_NUM,1),VEL_NUM);
    mexstep('forward');
    cost_new = 0; x_new(:,1) = X_INIT; u_new = u_norm; %delta_j = 0; 
    for i = 1 : 1 : STEP_NUM
        if i >= SKIPo+1%max(q,qu)+2
%             kt = - Kv(:,:,i)*vk(:,i+1) - Ku(:,:,i)*u_norm(:,i); Qu = Ri*u_norm(:,i)+B_aug(:,:,i)'*vk(:,i+1);
%             delta_j = delta_j + alpha*kt'*Qu+alpha^2/2*kt'*Quu(:,:,i)*kt;
            du_seq(IN_NUM*(STEP_NUM-i+1)+1:IN_NUM*(STEP_NUM-i+2),1)=-K(:,:,i)*[dz_seq(OUT_NUM*(STEP_NUM-i+1)+1:OUT_NUM*(STEP_NUM-i+1+q),1);du_seq(IN_NUM*(STEP_NUM-i+2)+1:IN_NUM*(STEP_NUM-i+1+qu),1)] + alpha*kt(:,i);
            u_new(:,i) = u_norm(:,i) + du_seq(IN_NUM*(STEP_NUM-i+1)+1:IN_NUM*(STEP_NUM-i+2),1);
            cost_new = cost_new + 0.5*[reshape(Ck*(x_new(:,i:-1:i-q+1)-repmat(X_TARGET,1,q)),OUT_NUM*q,1);reshape(u_new(:,i-1:-1:i-qu+1),IN_NUM*(qu-1),1)]'*Qi*[reshape(Ck*(x_new(:,i:-1:i-q+1)-repmat(X_TARGET,1,q)),OUT_NUM*q,1);reshape(u_new(:,i-1:-1:i-qu+1),IN_NUM*(qu-1),1)]+0.5*u_new(:,i)'*Ri*u_new(:,i);
        end        
        mexstep('set','ctrl',u_new(:,i),IN_NUM);
        mexstep('step',1);
        xt(1:POS_NUM,1)=mexstep('get','qpos')';
        xt(POS_NUM+1:SYS_NUM,1)=mexstep('get','qvel')';
        dz_seq(OUT_NUM*(STEP_NUM-i)+1:OUT_NUM*(STEP_NUM-i+1),1)=Ck*xt-Z_NORM(:,i+1);
        x_new(:,i+1) = xt;
    end
    cost_new = cost_new + 0.5*[reshape(Ck*(x_new(:,STEP_NUM+1:-1:STEP_NUM+2-q)-repmat(X_TARGET,1,q)),OUT_NUM*q,1);reshape(u_new(:,STEP_NUM:-1:STEP_NUM+2-qu),IN_NUM*(qu-1),1)]'*Qf*[reshape(Ck*(x_new(:,STEP_NUM+1:-1:STEP_NUM+2-q)-repmat(X_TARGET,1,q)),OUT_NUM*q,1);reshape(u_new(:,STEP_NUM:-1:STEP_NUM+2-qu),IN_NUM*(qu-1),1)];
    if ite > 1
        z = (cost(ite-1) - cost_new)/delta_j;
    end
    
    if z >= -0.6 || alpha < 10^-5
        forward_flag = false;
        cost(ite) = cost_new
        alpha
        Z_NORM = Ck*x_new;
        X_NORM = x_new;
        u_norm = u_new;
        vk(:,STEP_NUM+1) = Qf*[reshape(Z_NORM(:,STEP_NUM+1:-1:STEP_NUM+2-q)-Ck*repmat(X_TARGET,1,q),OUT_NUM*q,1);reshape(u_norm(:,STEP_NUM:-1:STEP_NUM+2-qu),IN_NUM*(qu-1),1)];
        if alpha < 0.01
            alpha=0.01;
        end
    else
        alpha = 0.99*alpha;
    end
    ti = ti + 1;
end

umax = max(1,max(max(abs(u_norm))))

% sysid - arma
% collect data
% batch
% delta_u=ID_PERT_COEF*1*randn(IN_NUM*(STEP_NUM+1),TRIAL_NUM);
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
            delta_z(OUT_NUM*(STEP_NUM-i)+1:OUT_NUM*(STEP_NUM-i+1),j)=Ck*x2-Z_NORM(:,i+1);
        end
    end 

    % arma fitting - least square: M1 * fitcoef = delta_y
    for i=ID_START(k)+SKIPo:1:ID_END(k) % skip the first few steps to wait for enough data
        M1=[delta_z(OUT_NUM*(STEP_NUM-i+1)+1:OUT_NUM*(STEP_NUM-i+1+q),:);delta_u(IN_NUM*(STEP_NUM-i+1)+1:IN_NUM*(STEP_NUM-i+qu+1),:)];
        [Q,R]=qr(M1');
        fitcoef(:,:,i)=(R\Q'*delta_z(OUT_NUM*(STEP_NUM-i)+1:OUT_NUM*(STEP_NUM-i+1),:)')';
%         fitcoef(:,:,i)=delta_z(OUT_NUM*(STEP_NUM-i)+1:OUT_NUM*(STEP_NUM-i+1),:)*M1'/(M1*M1');
        r(i)=sqrt(mean(mean((delta_z(OUT_NUM*(STEP_NUM-i)+1:OUT_NUM*(STEP_NUM-i+1),:)-fitcoef(:,:,i)*M1).^2,1))); % residual
    end
end

% construct augmented Ak, Bk
for i=SKIPo+1:1:STEP_NUM 
    A_aug(:,:,i)=[fitcoef(:,1:OUT_NUM*q,i),fitcoef(:,OUT_NUM*q+IN_NUM+1:end,i);
      eye((q-1)*OUT_NUM),zeros((q-1)*OUT_NUM,OUT_NUM+IN_NUM*(qu-1));
      zeros(IN_NUM*(qu-1),OUT_NUM*q),[zeros(IN_NUM,IN_NUM*(qu-1));eye(IN_NUM*(qu-2)) zeros(IN_NUM*(qu-2),IN_NUM)]];
    B_aug(:,:,i)=[fitcoef(:,OUT_NUM*q+1:OUT_NUM*q+IN_NUM,i);zeros(OUT_NUM*(q-1),IN_NUM);eye(IN_NUM*min(qu-1,1));zeros(IN_NUM*(qu-2),IN_NUM)];
end

%% backpass
delta_j = 0; 
for i = STEP_NUM:-1:SKIPo+1%max(q,qu)+2
    Quu(:,:,i) = B_aug(:,:,i)'*Sk(:,:,i+1)*B_aug(:,:,i)+Ri; miu = 0; delta = 0;
    if min(eig(Quu(:,:,i))) <= 0
        disp('Quu is not positive definite')
    end
%     while min(eig(Quu(:,:,i))) <= 0
%         disp('Quu is not positive definite')
%         delta = max(2, 2*delta);
%         miu = min(max(10^-3, miu*delta),10^8);
%         Quu(:,:,i) = B_aug(:,:,i)'*(Sk(:,:,i+1)+miu*eye(OUT_NUM*q+IN_NUM*(qu-1)))*B_aug(:,:,i)+Ri;
%     end
    kpreinv = inv(Quu(:,:,i));
    K(:,:,i) = kpreinv*B_aug(:,:,i)'*Sk(:,:,i+1)*A_aug(:,:,i);
    Kv(:,:,i) = kpreinv*B_aug(:,:,i)';
    Ku(:,:,i) = kpreinv*Ri;
    Sk(:,:,i) = A_aug(:,:,i)'*Sk(:,:,i+1)*(A_aug(:,:,i)-B_aug(:,:,i)*K(:,:,i))+Qi;
    vk(:,i) = (A_aug(:,:,i)-B_aug(:,:,i)*K(:,:,i))'*vk(:,i+1)-K(:,:,i)'*Ri*u_norm(:,i)+Qi*[reshape(Z_NORM(:,i:-1:i-q+1)-Ck*repmat(X_TARGET,1,q),OUT_NUM*q,1);reshape(u_norm(:,i-1:-1:i-qu+1),IN_NUM*(qu-1),1)];
    kt(:,i) = - Kv(:,:,i)*vk(:,i+1) - Ku(:,:,i)*u_norm(:,i); Qu = Ri*u_norm(:,i)+B_aug(:,:,i)'*vk(:,i+1);
    delta_j = delta_j - (alpha*kt(:,i)'*Qu+alpha^2/2*kt(:,i)'*Quu(:,:,i)*kt(:,i));
end
end
mexstep('exit');
training_time=toc
TSE=norm(X_NORM(1:2,end)-X_TARGET(1:2))

%% output result
% fid = fopen('./nominal_d2c/state07.txt','wt');
% for i = 1 : STEP_NUM+1
%     for j = 1 : SYS_NUM
%         fprintf(fid,'%.10f ',X_NORM(j,i));
%     end
%     fprintf(fid,'\n');
% end
% fclose(fid);

fid = fopen('result0.txt','wt');
for i = 1 : STEP_NUM
    for j = 1 : IN_NUM
        fprintf(fid,'%.10f ',u_norm(j,i));
    end
end
fclose(fid);

save('results.mat','cost','u_norm','training_time');

%% plot
figure;
plot([X_NORM(1,:)', X_NORM(2,:)'])
% legend('theta','thetad')
% plot([X_NORM(1,:)' X_NORM(2,:)' X_NORM(3,:)' X_NORM(4,:)'])
% legend('x','theta','xd','thetad')
% legend('x','y','theta1','theta2')
% plot([X_NORM(1,:)' X_NORM(2,:)' X_NORM(10,:)'])
% legend('x','z','xd')
xlabel('step')

figure;
plot(0:1:size(cost)-1, cost);
xlabel('iteration')
ylabel('cost')

% figure;
% plot(u_norm)
% legend('u')
% xlabel('step')