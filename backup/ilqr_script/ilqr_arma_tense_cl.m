clear all;clc;warning off;tic;
%% dbar3d
% NODE_NUM = 4;
% POS_NUM = 3 * NODE_NUM;
% VEL_NUM = 3 * NODE_NUM;
% SYS_NUM = POS_NUM + VEL_NUM;
% IN_NUM = 7;
% OUT_NUM= 24;
% MODEL = 'dbar3d.xml';
% STEP_NUM = 200;
% lb=-inf;ub=0;
% X_TARGET = [zeros(3,1);0;0;2.5;zeros(SYS_NUM-6,1)];
% Ck=eye(SYS_NUM);%[eye(6) zeros(6,18)];%[eye(POS_NUM) zeros(VEL_NUM,VEL_NUM)];%
% q=qu=1,2,3
% alpha=0.5;
% ID_PERT_COEF = 0.01;
% TRIAL_NUM = 60;
% ITE_NUM = 20;
% STEP_DIV = 1; SKIPo = 4; SKIP = 300; 
% Ri = 1*10^-2 * eye(IN_NUM);
% cx = 8*10^0; cv = 2*10^0;
% ctx = 10*10^2; ctv = 1*10^3;
% Qi(4:6,4:6) = cx * [1 0 0;0 1 0;0 0 1];
% Qf(4:6,4:6) = ctx * [1 0 0;0 1 0;0 0 1];

%% t1d1_3d
% NODE_NUM = 11;
% POS_NUM = 3 * NODE_NUM;
% VEL_NUM = 3 * NODE_NUM;
% SYS_NUM = POS_NUM + VEL_NUM;
% IN_NUM = 20;
% OUT_NUM = 66;
% MODEL = 't1d1_3dsmall.xml';
% STEP_NUM = 200;
% lb=-inf;ub=0;
% X_TARGET = [zeros(12,1);0;0;5;zeros(SYS_NUM-15,1)];
% Ck=eye(SYS_NUM);%[eye(POS_NUM) zeros(VEL_NUM,VEL_NUM) ];%[eye(12) zeros(12,54)];%
% q=qu=1
% ID_PERT_COEF = 0.01;
% TRIAL_NUM = 150;
% ITE_NUM = 20;
% STEP_DIV = 1; SKIPo = 5; SKIP = 300; 
% Ri = 2*10^-2 * eye(IN_NUM);
% cx = 1*10^0; cv = 2*10^0;
% ctx = 10*10^2; ctv = 1*10^3;
% Qi(13:15,13:15) = cx * [1 0 0;0 2 0;0 0 3];
% Qf(13:15,13:15) = ctx * [1 0 0;0 2 0;0 0 3];
% q=qu=2
% ID_PERT_COEF = 0.01;
% TRIAL_NUM = 150;
% ITE_NUM = 20;
% STEP_DIV = 1; SKIPo = 5; SKIP = 300; 
% Ri = 2*10^-2 * eye(IN_NUM);
% cx = 4*10^0; cv = 2*10^0;
% ctx = 1*10^2; ctv = 1*10^3;
% Qi(13:15,13:15) = cx * [1 0 0;0 2 0;0 0 3];
% Qf(13:15,13:15) = ctx * [1 0 0;0 2 0;0 0 3];

%% t2d1_3d stiffness must be 0 in the model file
NODE_NUM = 25;
POS_NUM = 3 * NODE_NUM;
VEL_NUM = 3 * NODE_NUM;
SYS_NUM = POS_NUM + VEL_NUM;
IN_NUM = 46;
OUT_NUM = 24;
MODEL = 't2d1_3d.xml';
STEP_NUM = 300;
lb=-inf;ub=-0.1; % set according to prestress tension
% large motion reach,8 node pos, q=3
X_TARGET = [zeros(48,1);3;1;4.5;zeros(SYS_NUM-51,1)];
Ck=[eye(6) zeros(6,144);zeros(6,24) eye(6) zeros(6,120);zeros(6,48) eye(6) zeros(6,96);zeros(6,69) eye(6) zeros(6,75)];
% all pos
% X_TARGET = zeros(SYS_NUM,1);
% 4 center nodes
% X_TARGET = [zeros(3,1);0;0;1;zeros(12,1);0;0;2;zeros(12,1);0;0;3;zeros(12,1);0;0;5;zeros(SYS_NUM-51,1)];%[zeros(48,1);2.5;1.5;6;zeros(SYS_NUM-51,1)];%
% X_TARGET(9,1)=1;X_TARGET(12,1)=1;X_TARGET(15,1)=1;X_TARGET(24,1)=2;X_TARGET(27,1)=2;X_TARGET(30,1)=2;X_TARGET(39,1)=3;X_TARGET(42,1)=3;X_TARGET(45,1)=3;
% Ck=[zeros(3,48) eye(3) zeros(3,99)];
% Ck=[eye(6) zeros(6,144);zeros(6,48) eye(6) zeros(6,96)];
% Ck=[eye(6) zeros(6,144);zeros(6,24) eye(6) zeros(6,120);zeros(6,48) eye(6) zeros(6,96);zeros(6,69) eye(6) zeros(6,75)];
% 4 center node pos+z of balance bars OUT_NUM = 21;
% Ck=[zeros(3,3) eye(3) zeros(3,144);zeros(1,8) 1 zeros(1,141);zeros(1,11) 1 zeros(1,138);zeros(1,14) 1 zeros(1,135);
%     zeros(3,18) eye(3) zeros(3,129);zeros(1,23) 1 zeros(1,126);zeros(1,26) 1 zeros(1,123);zeros(1,29) 1 zeros(1,120);
%     zeros(3,33) eye(3) zeros(3,114);zeros(1,38) 1 zeros(1,111);zeros(1,41) 1 zeros(1,108);zeros(1,44) 1 zeros(1,105);
%    zeros(3,48) eye(3) zeros(3,99)];
% 4 center node pos+balance bar end pos OUT_NUM = 39;
% Ck=[zeros(12,3) eye(12) zeros(12,135);zeros(12,18) eye(12) zeros(12,120);
%     zeros(12,33) eye(12) zeros(12,105);zeros(3,48) eye(3) zeros(3,99)];
% Ck=[eye(POS_NUM) zeros(VEL_NUM,VEL_NUM)];%eye(SYS_NUM);%
% q=qu=1
% ID_PERT_COEF = 0.01;
% TRIAL_NUM = 300;
% ITE_NUM = 15;
% STEP_DIV = 1; SKIPo = 5; SKIP = 300; 
% Ri = 4*10^-2 * eye(IN_NUM);
% cx = 1*10^0; cv = 1*10^-2;
% ctx = 20*10^2; ctv = 1*10^-1;
% Qi(49:51,49:51) = cx * [1 0 0;0 1 0;0 0 1];
% Qf(49:51,49:51) = ctx * [1 0 0;0 1 0;0 0 1];
% Qi(76:end,76:end) = cv * eye(75);
% Qf(76:end,76:end) = ctv * eye(75);
% q=qu=2
% ID_PERT_COEF = 0.01;
% TRIAL_NUM = 300;
% ITE_NUM = 6;
% STEP_DIV = 1; SKIPo = 5; SKIP = 300; 
% Ri = 1*10^-1 * eye(IN_NUM);
% cx = 1*10^0; cv = 1*10^2;
% ctx = 80*10^2; ctv = 1*10^4;
% Qi(49:51,49:51) = cx * [1 0 0;0 1 0;0 0 2];
% Qf(49:51,49:51) = ctx * [1 0 0;0 1 0;0 0 2];
% q=qu=3,8 node pos
% ID_PERT_COEF = 0.01;
% TRIAL_NUM = 300;
% ITE_NUM = 15;
% STEP_DIV = 1; SKIPo = 3; SKIP = 300; 
% small motion X_TARGET = [zeros(48,1);.2;.2;7;zeros(SYS_NUM-51,1)];
% Ri = 4*10^-2 * eye(IN_NUM);
% cx = 1*10^0; cv = 1*10^-2;
% ctx = 20*10^2; ctv = 1*10^-1;
% large motion X_TARGET = [zeros(48,1);2;2;5;zeros(SYS_NUM-51,1)];
% Ri = 4*10^-2 * eye(IN_NUM);
% cx = 1*10^-1; cv = 1*10^-2;
% ctx = 20*10^1; ctv = 1*10^-1;
% Qi(13:15,13:15) = cx * [1 0 0;0 1 0;0 0 2];
% Qf(13:15,13:15) = ctx * [1 0 0;0 1 0;0 0 2];
% alpha=0.5
% Ck=[eye(6) zeros(6,144);zeros(6,24) eye(6) zeros(6,120);zeros(6,48) eye(6) zeros(6,96);zeros(6,69) eye(6) zeros(6,75)];
% q=qu=4,6 both work, 4 node pos
% ID_PERT_COEF = 0.01;
% TRIAL_NUM = 300;
% ITE_NUM = 13;
% STEP_DIV = 1; SKIPo = 4 or 6; SKIP = 300; 
% Ri = 4*10^-2 * eye(IN_NUM);
% cx = 1*10^0; cv = 1*10^-2;
% ctx = 20*10^2; ctv = 1*10^-1;
% Qi(7:9,7:9) = cx * [1 0 0;0 1 0;0 0 2];
% Qf(7:9,7:9) = ctx * [1 0 0;0 1 0;0 0 2];
% alpha=0.5
% Ck=[eye(6) zeros(6,144);zeros(6,48) eye(6) zeros(6,96)];
% q=qu=2, 1 node pos
% ID_PERT_COEF = 0.01;
% TRIAL_NUM = 300;
% ITE_NUM = 13;
% STEP_DIV = 1; SKIPo = 2; SKIP = 300; 
% Ri = 4*10^-2 * eye(IN_NUM);
% cx = 1*10^0; cv = 1*10^-2;
% ctx = 20*10^2; ctv = 1*10^-1;
% Qi(1:3,1:3) = cx * [1 0 0;0 1 0;0 0 2];
% Qf(1:3,1:3) = ctx * [1 0 0;0 1 0;0 0 2];
% alpha=0.5
% Ck=[zeros(3,48) eye(3) zeros(3,99)];
% q=qu=3, 8 node pos
% ID_PERT_COEF = 0.01;
% TRIAL_NUM = 400;
% ITE_NUM = 18;
% STEP_DIV = 1; SKIPo = 3; SKIP = 300; 
% Ri = 5*10^-2 * eye(IN_NUM);
% cx = 1.2*10^-1; cv = 1*10^-2;
% ctx = 25*10^1; ctv = 1*10^-1;
% Qi(13:15,13:15) = cx * [1 0 0;0 1 0;0 0 0.8];
% Qf(13:15,13:15) = ctx * [1 0 0;0 1 0;0 0 0.8];
% alpha=0.5
% Ck=[eye(6) zeros(6,144);zeros(6,24) eye(6) zeros(6,120);zeros(6,48) eye(6) zeros(6,96);zeros(6,69) eye(6) zeros(6,75)];
% X_TARGET = [zeros(48,1);2.5;1.5;6;zeros(SYS_NUM-51,1)];
% all pos q=qu=2, works for folding to groung, ite = 7
% alpha=0.5
% ID_PERT_COEF = 0.01;
% TRIAL_NUM = 400;
% ITE_NUM = 10;
% STEP_DIV = 1; SKIPo = 3; SKIP = 300; 
% Ri = 6*10^-1 * eye(IN_NUM);
% cx = 0.4*10^-1; cv = 2*10^3;
% ctx = 8*10^1; ctv = 50*10^4;
% Ck=[eye(POS_NUM) zeros(VEL_NUM,VEL_NUM)];
% Qi(1:OUT_NUM,1:OUT_NUM) = cx * eye(OUT_NUM);
% Qf(1:OUT_NUM,1:OUT_NUM) = ctx * eye(OUT_NUM);
% 8 node pos, q=qu=3,bend
% ID_PERT_COEF = 0.01;
% TRIAL_NUM = 300; 
% ITE_NUM = 40;
% STEP_DIV = 1; SKIPo = 3; SKIP = 300; 
% Ri = 5*10^-2 * eye(IN_NUM);
% cx = 1*10^-1; cv = 2*10^-1;
% ctx = 35*10^1; ctv = 100*10^4;
% Qi(13:15,13:15) = (cx+0) * [2 0 0;0 2 0;0 0 2];
% Qf(13:15,13:15) = (ctx+0) * [1 0 0;0 1 0;0 0 3];
% X_TARGET = [zeros(48,1);3;1;4.5;zeros(SYS_NUM-51,1)];
% Ck=[eye(6) zeros(6,144);zeros(6,24) eye(6) zeros(6,120);zeros(6,48) eye(6) zeros(6,96);zeros(6,69) eye(6) zeros(6,75)];
% lb=-inf;ub=-0.1;

%% tuning 
ID_PERT_COEF = 0.01;
TRIAL_NUM = 300; 
ITE_NUM = 40;
STEP_DIV = 1; SKIPo = 3; SKIP = 300; 
q = 3;
qu = 3;
Ri = 5*10^-2 * eye(IN_NUM);
cx = 1*10^-1; cv = 2*10^-1;
ctx = 35*10^1; ctv = 100*10^4;
Qi = zeros(OUT_NUM*q+IN_NUM*(qu-1),OUT_NUM*q+IN_NUM*(qu-1));
Qf = zeros(OUT_NUM*q+IN_NUM*(qu-1),OUT_NUM*q+IN_NUM*(qu-1));
% dbar3d
% Qi(4:6,4:6) = cx * [1 0 0;0 1 0;0 0 1];
% Qf(4:6,4:6) = ctx * [1 0 0;0 1 0;0 0 1];
% t1d13d
% Qi(13:15,13:15) = cx * [1 0 0;0 2 0;0 0 3];
% Qf(13:15,13:15) = ctx * [1 0 0;0 2 0;0 0 3];
% t2d13d
% q=1,q=2
% Qi(49:51,49:51) = cx * [1 0 0;0 1 0;0 0 2];
% Qf(49:51,49:51) = ctx * [1 0 0;0 1 0;0 0 2];
% q=1
% Qi(76:end,76:end) = cv * eye(75);
% Qf(76:end,76:end) = ctv * eye(75);
% 8 node pos
Qi(13:15,13:15) = (cx+0) * [2 0 0;0 2 0;0 0 2];
% Qi(13:15,37:39) = -cv * [2 0 0;0 2 0;0 0 2];
% Qi(37:39,13:15) = -cv * [2 0 0;0 2 0;0 0 2];
% Qi(37:39,37:39) = cv * [2 0 0;0 2 0;0 0 2];
Qf(13:15,13:15) = (ctx+0) * [1 0 0;0 1 0;0 0 3];
% Qf(13:15,37:39) = -ctv * [1 0 0;0 1 0;0 0 3];
% Qf(37:39,13:15) = -ctv * [1 0 0;0 1 0;0 0 3];
% Qf(37:39,37:39) = ctv * [1 0 0;0 1 0;0 0 3];
% 4 center node pos + z of balance bar
% Qi(1:6,1:6) = cx * eye(6);
% Qi(7:12,7:12) = 1*cx * eye(6);
% Qi(13:18,13:18) = 0.5*cx * eye(6);
% Qi(19:21,19:21) = 0.01*cx *[2 0 0;0 2 0;0 0 0.6];%0.02?0.05?
% Qf(1:6,1:6) = ctx * eye(6);
% Qf(7:12,7:12) = 1*ctx * eye(6);
% Qf(13:18,13:18) = 0.5*ctx * eye(6);
% Qf(19:21,19:21) = 0.01*ctx * [2 0 0;0 2 0;0 0 0.6];
% with vel,wrong,should be x(t)-x(t-1)
% Qi(1:6,1:6) = [(cx+cv) * eye(3) -cv*eye(3);-cv*eye(3) cv*eye(3)];
% Qi(7:12,7:12) = 0.7*[(cx+cv) * eye(3) -cv*eye(3);-cv*eye(3) cv*eye(3)];
% Qi(13:18,13:18) = 0.3*[(cx+cv) * eye(3) -cv*eye(3);-cv*eye(3) cv*eye(3)];
% %Qi(19:21,19:21) = 0.01*cx *[2 0 0;0 2 0;0 0 0.5];%0.02?0.05?
% Qf(1:6,1:6) = [(ctx+ctv) * eye(3) -ctv*eye(3);-ctv*eye(3) ctv*eye(3)];
% Qf(7:12,7:12) = 0.7*[(ctx+ctv) * eye(3) -ctv*eye(3);-ctv*eye(3) ctv*eye(3)];
% Qf(13:18,13:18) = 0.3*[(ctx+ctv) * eye(3) -ctv*eye(3);-ctv*eye(3) ctv*eye(3)];
% %Qf(19:21,19:21) = 0.01*ctx * [2 0 0;0 2 0;0 0 0.5];
% 4 node pos + balance bar end node pos
% Qi(1:12,1:12) = cx * eye(12);
% Qi(13:24,13:24) = 1*cx * eye(12);
% Qi(25:36,25:36) = 1*cx * eye(12);
% Qi(37:39,37:39) = 0.01*cx *[1 0 0;0 1 0;0 0 0.6];
% Qf(1:12,1:12) = ctx * eye(12);
% Qf(13:24,13:24) = 1*ctx * eye(12);
% Qf(25:36,25:36) = 1*ctx * eye(12);
% Qf(37:39,37:39) = 0.01*ctx *[1 0 0;0 1 0;0 0 0.6];
% 4 node pos
% Qi(7:9,7:9) = cx * [1 0 0;0 1 0;0 0 2];
% Qf(7:9,7:9) = ctx * [1 0 0;0 1 0;0 0 2];
% 1 node pos
% Qi(1:3,1:3) = cx * [1 0 0;0 1 0;0 0 2];
% Qf(1:3,1:3) = ctx * [1 0 0;0 1 0;0 0 2];
% all node pos
% Qi(1:OUT_NUM,1:OUT_NUM) = cx * eye(OUT_NUM);
% Qf(1:OUT_NUM,1:OUT_NUM) = ctx * eye(OUT_NUM);
% all node pos with vel
% Qi(1:OUT_NUM,1:OUT_NUM) = (cx+cv) * eye(OUT_NUM);
% Qi(OUT_NUM+1:2*OUT_NUM,1:OUT_NUM) = -cv * eye(OUT_NUM);
% Qi(1:OUT_NUM,OUT_NUM+1:2*OUT_NUM) = -cv * eye(OUT_NUM);
% Qi(OUT_NUM+1:2*OUT_NUM,OUT_NUM+1:2*OUT_NUM) = cv * eye(OUT_NUM);
% Qf(1:OUT_NUM,1:OUT_NUM) = (ctx+ctv) * eye(OUT_NUM);
% Qf(OUT_NUM+1:2*OUT_NUM,1:OUT_NUM) = -ctv * eye(OUT_NUM);
% Qf(1:OUT_NUM,OUT_NUM+1:2*OUT_NUM) = -ctv * eye(OUT_NUM);
% Qf(OUT_NUM+1:2*OUT_NUM,OUT_NUM+1:2*OUT_NUM) = ctv * eye(OUT_NUM);
%% variables
dz_seq=zeros(OUT_NUM*(STEP_NUM+1),1);
dzt_seq=zeros(OUT_NUM*(STEP_NUM+1),1);
u_norm = [zeros(IN_NUM,SKIPo) 0.1*randn(IN_NUM,STEP_NUM-SKIPo)];%zeros(IN_NUM,STEP_NUM);
u_init = zeros(IN_NUM,STEP_NUM);
du_seq = zeros(IN_NUM*(STEP_NUM+1),1);
Sk = zeros(OUT_NUM*q+IN_NUM*(qu-1),OUT_NUM*q+IN_NUM*(qu-1),STEP_NUM+1); 
Sk(:,:,STEP_NUM+1) = Qf;
vk = zeros(OUT_NUM*q+IN_NUM*(qu-1),STEP_NUM+1);
K = zeros(IN_NUM,OUT_NUM*q+IN_NUM*(qu-1),STEP_NUM);
Kv = zeros(IN_NUM,OUT_NUM*q+IN_NUM*(qu-1),STEP_NUM);
Ku = zeros(IN_NUM,IN_NUM,STEP_NUM);
Quu = zeros(IN_NUM,IN_NUM,STEP_NUM);
dufc = zeros(IN_NUM,STEP_NUM);
uf = zeros(IN_NUM,STEP_NUM);
kt  = zeros(IN_NUM,STEP_NUM);
cost = zeros(ITE_NUM,1);
fitcoef = zeros(OUT_NUM,OUT_NUM*q+IN_NUM*qu,STEP_NUM); 
A_aug = zeros(OUT_NUM*q+IN_NUM*(qu-1),OUT_NUM*q+IN_NUM*(qu-1),STEP_NUM);
B_aug = zeros(OUT_NUM*q+IN_NUM*(qu-1),IN_NUM,STEP_NUM);
delta_z = zeros(OUT_NUM*(STEP_NUM+1),TRIAL_NUM);
delta_u = zeros(IN_NUM*(STEP_NUM+1),TRIAL_NUM);
ids = floor(linspace(1,STEP_NUM,STEP_DIV+1)); ID_START = ids(1:end-1); ID_END = [ID_START(2:end)+SKIP,STEP_NUM];

%% prestress & initial guess (need stiffness)
% fid = fopen('init0.txt','r');
% U = fscanf(fid, '%f');
% fclose(fid);
% u_norm = reshape(U, IN_NUM, STEP_NUM);
% u_init = u_norm;
% umax = max(max(abs(u_norm)));
% u_norm = u_norm + [zeros(IN_NUM,SKIPo) 0.1*randn(IN_NUM,STEP_NUM-SKIPo)];

%% forward pass
tic; alpha = 0.5; z = 1;
mexstep('load',MODEL);
mexstep('reset');
mexstep('forward');
N_array=mexstep('get','site_xpos');
Nd_array=mexstep('get','sensordata');
N_temp = N_array(:,1:NODE_NUM);
% all pos only unfolding
% N_temp(1:2,:) = N_temp(1:2,:)*0.816;
% N_temp(1:2,3:5) = N_array(1:2,3:5);N_temp(1:2,8:10) = N_array(1:2,8:10);N_temp(1:2,13:15) = N_array(1:2,13:15); % balance bar
% N_temp(3,:) = [1,2,2,2,2,3,4,4,4,4,5,6,6,6,6,7,10,1,3,5,7,1,3,5,7]; % z pos
% X_TARGET(1:3*NODE_NUM,1) = reshape(N_temp,1,3*NODE_NUM);
% all pos only folding
% N_temp(1:2,:) = N_temp(1:2,:)*1.225;
% N_temp(1:2,3:5) = N_array(1:2,3:5);N_temp(1:2,8:10) = N_array(1:2,8:10);N_temp(1:2,13:15) = N_array(1:2,13:15); % balance bar
% N_temp(3,:) = 0; % z pos
% X_TARGET(1:3*NODE_NUM,1) = reshape(N_temp,1,3*NODE_NUM);
% 4 center nodes + balance bar end pos
% X_TARGET(7:15) = reshape(N_array(:,3:5),1,9);X_TARGET(22:30) = reshape(N_array(:,8:10),1,9);X_TARGET(37:45) = reshape(N_array(:,13:15),1,9);
% X_TARGET(9,1)=1;X_TARGET(12,1)=1;X_TARGET(15,1)=1;X_TARGET(24,1)=2;X_TARGET(27,1)=2;X_TARGET(30,1)=2;X_TARGET(39,1)=3;X_TARGET(42,1)=3;X_TARGET(45,1)=3;
% N=N_array(:,1:end-1);
% save('.\Tensegrity_Files\N.mat','N');
x_new(1:POS_NUM,1)=reshape(N_array(:,1:NODE_NUM),1,3*NODE_NUM); % the site_xpos data read from MuJoCo is 3 by NODE_NUM
x_new(POS_NUM+1:SYS_NUM,1)=Nd_array(1:3*NODE_NUM);
Z_NORM=repmat(Ck*x_new(:,1),1,STEP_NUM+1);
for ite=1:1:ITE_NUM
% nominal
forward_flag = true; ti = 0;%alpha = .3; 
while forward_flag
    mexstep('reset');
    mexstep('forward');
    cost_new = 0; u_new = u_norm; %delta_j = 0; 
    for i = 1 : 1 : STEP_NUM
        if i >= SKIPo+1%max(q,qu)+2
            % no control limit
%             du_seq(IN_NUM*(STEP_NUM-i+1)+1:IN_NUM*(STEP_NUM-i+2),1)=-K(:,:,i)*[dz_seq(OUT_NUM*(STEP_NUM-i+1)+1:OUT_NUM*(STEP_NUM-i+1+q),1);du_seq(IN_NUM*(STEP_NUM-i+2)+1:IN_NUM*(STEP_NUM-i+1+qu),1)] + alpha*kt(:,i);
%             u_new(:,i) = u_norm(:,i) + du_seq(IN_NUM*(STEP_NUM-i+1)+1:IN_NUM*(STEP_NUM-i+2),1);
            % control limit
            du_seq(IN_NUM*(STEP_NUM-i+1)+1:IN_NUM*(STEP_NUM-i+2),1)=max(lb-u_norm(:,i),min(ub-u_norm(:,i),-K(:,:,i)*[dz_seq(OUT_NUM*(STEP_NUM-i+1)+1:OUT_NUM*(STEP_NUM-i+1+q),1);du_seq(IN_NUM*(STEP_NUM-i+2)+1:IN_NUM*(STEP_NUM-i+1+qu),1)] + alpha*(kt(:,i)+dufc(:,i)-uf(:,i))));  
            u_new(:,i) = u_norm(:,i) + du_seq(IN_NUM*(STEP_NUM-i+1)+1:IN_NUM*(STEP_NUM-i+2),1);
            cost_new = cost_new + 0.5*[reshape(Ck*(x_new(:,i:-1:i-q+1)-repmat(X_TARGET,1,q)),OUT_NUM*q,1);reshape(u_new(:,i-1:-1:i-qu+1),IN_NUM*(qu-1),1)]'*Qi*[reshape(Ck*(x_new(:,i:-1:i-q+1)-repmat(X_TARGET,1,q)),OUT_NUM*q,1);reshape(u_new(:,i-1:-1:i-qu+1),IN_NUM*(qu-1),1)]+0.5*(u_new(:,i)-u_init(:,i))'*Ri*(u_new(:,i)-u_init(:,i));
        end        
        mexstep('set','ctrl',u_new(:,i),IN_NUM);
        mexstep('step',1);
        N_array=mexstep('get','site_xpos');
        Nd_array=mexstep('get','sensordata');
        xt(1:POS_NUM,1)=reshape(N_array(:,1:NODE_NUM),1,3*NODE_NUM);
        xt(POS_NUM+1:SYS_NUM,1)=Nd_array(1:3*NODE_NUM);
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
%         if alpha < 0.01
%             alpha=0.01;
%         end
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
        mexstep('forward');
        for i=ID_START(k):1:ID_END(k)
            % control limit
            delta_u(IN_NUM*(STEP_NUM-i+1)+1:IN_NUM*(STEP_NUM-i+2),j)=max(lb-u_norm(:,i),min(ub-u_norm(:,i),delta_u(IN_NUM*(STEP_NUM-i+1)+1:IN_NUM*(STEP_NUM-i+2),j)));
            mexstep('set','ctrl',u_norm(:,i)+delta_u(IN_NUM*(STEP_NUM-i+1)+1:IN_NUM*(STEP_NUM-i+2),j)+0.00001*randn(IN_NUM,1),IN_NUM);
            mexstep('step',1);
            N_array=mexstep('get','site_xpos');
            Nd_array=mexstep('get','sensordata');
            x2(1:POS_NUM,1)=reshape(N_array(:,1:NODE_NUM),1,3*NODE_NUM);
            x2(POS_NUM+1:SYS_NUM,1)=Nd_array(1:3*NODE_NUM);
            delta_z(OUT_NUM*(STEP_NUM-i)+1:OUT_NUM*(STEP_NUM-i+1),j)=Ck*x2-Z_NORM(:,i+1)+0.00001*randn(OUT_NUM,1);
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

% single
% for j=1:1:TRIAL_NUM
%     mexstep('reset');
%     mexstep('set','qpos',X_INIT(1:POS_NUM,1),POS_NUM);
%     mexstep('set','qvel',X_INIT(POS_NUM+1:SYS_NUM,1),VEL_NUM);
%     mexstep('forward');
%     for i=1:1:STEP_NUM
%         mexstep('set','ctrl',u_norm(:,i)+delta_u(IN_NUM*(STEP_NUM-i+1)+1:IN_NUM*(STEP_NUM-i+2),j),IN_NUM);
%         mexstep('step',1);
%         x2(1:POS_NUM,1)=mexstep('get','qpos')';
%         x2(POS_NUM+1:SYS_NUM,1)=mexstep('get','qvel')';
%         delta_z(OUT_NUM*(STEP_NUM-i)+1:OUT_NUM*(STEP_NUM-i+1),j)=Ck*x2-Z_NORM(:,i+1);
%     end
% end 
% 
% % arma fitting - least square: M1 * fitcoef = delta_y
% for i=SKIPo+1:1:STEP_NUM % skip the first few steps to wait for enough data
%     M1=[delta_z(OUT_NUM*(STEP_NUM-i+1)+1:OUT_NUM*(STEP_NUM-i+1+q),:);delta_u(IN_NUM*(STEP_NUM-i+1)+1:IN_NUM*(STEP_NUM-i+qu+1),:)];
%     fitcoef(:,:,i)=delta_z(OUT_NUM*(STEP_NUM-i)+1:OUT_NUM*(STEP_NUM-i+1),:)*M1'/(M1*M1');
%     r(i)=sqrt(mean(mean((delta_z(OUT_NUM*(STEP_NUM-i)+1:OUT_NUM*(STEP_NUM-i+1),:)-fitcoef(:,:,i)*M1).^2,1))); % residual
% end
% rmax = max(r)

% construct augmented Ak, Bk
for i=SKIPo+1:1:STEP_NUM 
    A_aug(:,:,i)=[fitcoef(:,1:OUT_NUM*q,i),fitcoef(:,OUT_NUM*q+IN_NUM+1:end,i);
      eye((q-1)*OUT_NUM),zeros((q-1)*OUT_NUM,OUT_NUM+IN_NUM*(qu-1));
      zeros(IN_NUM*(qu-1),OUT_NUM*q),[zeros(IN_NUM,IN_NUM*(qu-1));eye(IN_NUM*(qu-2)),zeros(IN_NUM*(qu-2),IN_NUM)]];
    B_aug(:,:,i)=[fitcoef(:,OUT_NUM*q+1:OUT_NUM*q+IN_NUM,i);zeros(OUT_NUM*(q-1),IN_NUM);eye(IN_NUM*min(qu-1,1));zeros(IN_NUM*(qu-2),IN_NUM)];
end

%% backpass
delta_j = 0; uf = zeros(IN_NUM,STEP_NUM); kt  = zeros(IN_NUM,STEP_NUM); K = zeros(IN_NUM,OUT_NUM*q+IN_NUM*(qu-1),STEP_NUM); dufc = zeros(IN_NUM,STEP_NUM);
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
    Qux=B_aug(:,:,i)'*Sk(:,:,i+1)*A_aug(:,:,i);
    K(:,:,i) = kpreinv*Qux;
    Kv(:,:,i) = kpreinv*B_aug(:,:,i)';
    Ku(:,:,i) = kpreinv*Ri;
    Sk(:,:,i) = A_aug(:,:,i)'*Sk(:,:,i+1)*(A_aug(:,:,i)-B_aug(:,:,i)*K(:,:,i))+Qi;
    vk(:,i) = (A_aug(:,:,i)-B_aug(:,:,i)*K(:,:,i))'*vk(:,i+1)-K(:,:,i)'*Ri*(u_norm(:,i)-u_init(:,i))+Qi*[reshape(Z_NORM(:,i:-1:i-q+1)-Ck*repmat(X_TARGET,1,q),OUT_NUM*q,1);reshape(u_norm(:,i-1:-1:i-qu+1)-u_init(:,i-1:-1:i-qu+1),IN_NUM*(qu-1),1)];
%     kt(:,i) = - Kv(:,:,i)*vk(:,i+1) - Ku(:,:,i)*u_norm(:,i); 
    Qu = Ri*(u_norm(:,i)-u_init(:,i))+B_aug(:,:,i)'*vk(:,i+1); kt(:,i) = -kpreinv*Qu;
    delta_j = delta_j - (alpha*kt(:,i)'*Qu+alpha^2/2*kt(:,i)'*Quu(:,:,i)*kt(:,i));
    
    % IROS12
%     fidx=[]; cidx=[];
%     for j=1:1:IN_NUM
%         if u_norm(j,i) < ub && u_norm(j,i) > lb
%             fidx=[fidx,j];
%         else
%             cidx=[cidx,j];
%         end
%     end
%     uf(fidx,i) = u_norm(fidx,i);
%     kpreinv = inv(Quu(fidx,fidx,i));
%     Qux=B_aug(:,:,i)'*Sk(:,:,i+1)*A_aug(:,:,i);
%     K(fidx,:,i) = kpreinv*Qux(fidx,:);
% %     Kv(:,:,i) = kpreinv*B_aug(:,:,i)';
% %     Ku(:,:,i) = kpreinv*Ri;
%     Sk(:,:,i) = A_aug(:,:,i)'*Sk(:,:,i+1)*(A_aug(:,:,i)-B_aug(:,:,i)*K(:,:,i))+Qi;
%     vk(:,i) = (A_aug(:,:,i)-B_aug(:,:,i)*K(:,:,i))'*vk(:,i+1)-K(:,:,i)'*Ri*u_norm(:,i)+Qi*[reshape(Z_NORM(:,i:-1:i-q+1)-Ck*repmat(X_TARGET,1,q),OUT_NUM*q,1);reshape(u_norm(:,i-1:-1:i-qu+1),IN_NUM*(qu-1),1)];
% %     kt(:,i) = - Kv(:,:,i)*vk(:,i+1) - Ku(:,:,i)*u_norm(:,i); 
%     dufc(fidx,i) = -kpreinv*Quu(fidx,cidx,i)*u_norm(cidx,i);
%     Qu = Ri*(u_norm(:,i)-u_init(:,i))+B_aug(:,:,i)'*vk(:,i+1); kt(fidx,i) = -kpreinv*Qu(fidx,1);
%     delta_j = delta_j - (alpha*kt(:,i)'*Qu+alpha^2/2*kt(:,i)'*Quu(:,:,i)*kt(:,i));
end
end
mexstep('exit');
training_time=toc

%% output result
fid = fopen('result0.txt','wt');
for i = 1 : STEP_NUM
    for j = 1 : IN_NUM
        fprintf(fid,'%.10f ',u_norm(j,i));
    end
end
fclose(fid);

fid = fopen('statetarget.txt','wt');
for i = 1 : OUT_NUM
    fprintf(fid,'%.10f ',X_TARGET(i,1));
end
fclose(fid);

save('results.mat','cost','u_norm','training_time');

%% plot
figure;
% plot([X_NORM(4,:)', X_NORM(5,:)', X_NORM(6,:)'])
% plot([X_NORM(13,:)', X_NORM(14,:)', X_NORM(15,:)'])
plot([X_NORM(49,:)', X_NORM(50,:)', X_NORM(51,:)'])
legend('x','y','z')
xlabel('step')

figure;
plot(0:1:size(cost)-1, cost);
xlabel('iteration')
ylabel('cost')

% figure;
% plot(u_norm)
% legend('u')
% xlabel('step')