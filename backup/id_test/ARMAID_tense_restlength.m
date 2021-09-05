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
% lb=0;ub=inf;
% Ck=eye(SYS_NUM);%[eye(6) zeros(6,18)];%[eye(POS_NUM) zeros(VEL_NUM,VEL_NUM) ];%

%% t1d13d
NODE_NUM = 11;
POS_NUM = 3 * NODE_NUM;
VEL_NUM = 3 * NODE_NUM;
SYS_NUM = POS_NUM + VEL_NUM;
IN_NUM = 20;
OUT_NUM = 66;
MODEL = 't1d1_3dsmall.xml';
STEP_NUM = 200;
lb=0;ub=inf;
X_TARGET = [zeros(12,1);0;0;5;zeros(SYS_NUM-15,1)];
Ck=eye(SYS_NUM);%[eye(12) zeros(12,54) ];%[eye(POS_NUM) zeros(VEL_NUM,VEL_NUM) ];%

%% t2d13d
% NODE_NUM = 25;
% POS_NUM = 3 * NODE_NUM;
% VEL_NUM = 3 * NODE_NUM;
% SYS_NUM = POS_NUM + VEL_NUM;
% IN_NUM = 46;
% OUT_NUM = 150;
% MODEL = 't2d1_3d.xml';
% STEP_NUM = 200;
% X_TARGET = [zeros(48,1);0.2;0.2;7;zeros(SYS_NUM-51,1)];
% Ck=eye(SYS_NUM);%[eye(6) zeros(6,144);zeros(6,50) eye(6) zeros(6,94);zeros(6,100) eye(6) zeros(6,44);zeros(6,144) eye(6)];%[eye(POS_NUM) zeros(VEL_NUM,VEL_NUM)];%

%% tuning
ID_PERT_COEF = 0.001;
TRIAL_NUM = 150;
q = 1;
qu = 1;

%% variables
Aid = zeros(SYS_NUM,SYS_NUM,STEP_NUM);
Bid = zeros(SYS_NUM,IN_NUM,STEP_NUM);

%% read nominal control sequence
fid = fopen('length0.txt','r');
U = fscanf(fid, '%f');
fclose(fid);
u_norm = reshape(U, IN_NUM, STEP_NUM);
umax = max(max(abs(u_norm)));

%% nominal trajectory
Y_NORM = zeros(OUT_NUM,STEP_NUM+1);
X_NORM = zeros(OUT_NUM,STEP_NUM+1);
mexstep('load',MODEL); % load model
mexstep('reset');
mexstep('forward'); % update sites and sensors
N_array=mexstep('get','site_xpos');
Nd_array=mexstep('get','sensordata');
X_NORM(1:POS_NUM,1)=reshape(N_array(:,1:NODE_NUM),1,3*NODE_NUM); % the site_xpos data read from MuJoCo is 3 by NODE_NUM
X_NORM(POS_NUM+1:SYS_NUM,1)=Nd_array(1:3*NODE_NUM);
Y_NORM(:,1)=Ck*X_NORM(:,1);
for i = 1 : 1 : STEP_NUM
    mexstep('set','tendon_lengthspring',u_norm(:,i)+0.0*randn(IN_NUM,1),IN_NUM);
    mexstep('step',1);
    N_array=mexstep('get','site_xpos');
    Nd_array=mexstep('get','sensordata');
    X_NORM(1:POS_NUM,i+1)=reshape(N_array(:,1:NODE_NUM),1,3*NODE_NUM);
    X_NORM(POS_NUM+1:SYS_NUM,i+1)=Nd_array(1:3*NODE_NUM);
    Y_NORM(:,i+1)=Ck*X_NORM(:,i+1);
end
% mexstep('exit');
%% sysid
% arma
STEP_DIV = 3; SKIP = [3,repmat(20,1,STEP_DIV-1)]; ids = floor(linspace(1,STEP_NUM,STEP_DIV+1)); ID_START = ids(1:end-1); ID_END = [ID_START(2:end)+SKIP(end),STEP_NUM];
% delta_u=ID_PERT_COEF*umax*randn(IN_NUM*(STEP_NUM+1),TRIAL_NUM); % generate perturbation du
skipu = 0; % add large noise for the first few steps
delta_u=[ID_PERT_COEF*1*randn(IN_NUM*(STEP_NUM+1-skipu),TRIAL_NUM);0.1*randn(IN_NUM*skipu,TRIAL_NUM)];
delta_ybatch=zeros(OUT_NUM*(STEP_NUM+1),TRIAL_NUM);
delta_ysingle=zeros(OUT_NUM*(STEP_NUM+1),TRIAL_NUM);
fitcoefbatch=zeros(OUT_NUM,OUT_NUM*q+IN_NUM*qu,STEP_NUM);
fitcoefsingle=zeros(OUT_NUM,OUT_NUM*q+IN_NUM*qu,STEP_NUM);
% batch
for k=1:1:STEP_DIV
    for j=1:1:TRIAL_NUM
        mexstep('reset');
        mexstep('forward');
        for i=ID_START(k):1:ID_END(k)
            delta_u(IN_NUM*(STEP_NUM-i+1)+1:IN_NUM*(STEP_NUM-i+2),j)=max(lb-u_norm(:,i),min(ub-u_norm(:,i),delta_u(IN_NUM*(STEP_NUM-i+1)+1:IN_NUM*(STEP_NUM-i+2),j)));
            mexstep('set','tendon_lengthspring',u_norm(:,i)+delta_u(IN_NUM*(STEP_NUM-i+1)+1:IN_NUM*(STEP_NUM-i+2),j),IN_NUM);
            mexstep('step',1);
            N_array=mexstep('get','site_xpos');
            Nd_array=mexstep('get','sensordata');
            x2(1:POS_NUM,1)=reshape(N_array(:,1:NODE_NUM),1,3*NODE_NUM);
            x2(POS_NUM+1:SYS_NUM,1)=Nd_array(1:3*NODE_NUM);
            delta_ybatch(OUT_NUM*(STEP_NUM-i)+1:OUT_NUM*(STEP_NUM-i+1),j)=Ck*x2-Y_NORM(:,i+1);
        end
    end 

    %% least square fitting for the arma model
    for i=max(ID_START(k)+SKIP(k),max(q,qu)+2):1:ID_END(k) % skip the first few steps to wait for enough data
        M1=[delta_ybatch(OUT_NUM*(STEP_NUM-i+1)+1:OUT_NUM*(STEP_NUM-i+1+q),:);delta_u(IN_NUM*(STEP_NUM-i+1)+1:IN_NUM*(STEP_NUM-i+qu+1),:)];
%         fitcoefbatch(:,:,i)=delta_ybatch(OUT_NUM*(STEP_NUM-i)+1:OUT_NUM*(STEP_NUM-i+1),:)*M1'/(M1*M1');
        [Q,R]=qr(M1');
        fitcoefbatch(:,:,i)=(R\Q'*delta_ybatch(OUT_NUM*(STEP_NUM-i)+1:OUT_NUM*(STEP_NUM-i+1),:)')';
        rb(i)=sqrt(mean(mean((delta_ybatch(OUT_NUM*(STEP_NUM-i)+1:OUT_NUM*(STEP_NUM-i+1),:)-fitcoefbatch(:,:,i)*M1).^2,1))); % residual
    end
end
rmaxb = max(rb)

% single
for j=1:1:TRIAL_NUM
    mexstep('reset');
    mexstep('forward');
    for i=1:1:STEP_NUM
        delta_u(IN_NUM*(STEP_NUM-i+1)+1:IN_NUM*(STEP_NUM-i+2),j)=max(lb-u_norm(:,i),min(ub-u_norm(:,i),delta_u(IN_NUM*(STEP_NUM-i+1)+1:IN_NUM*(STEP_NUM-i+2),j)));
        mexstep('set','tendon_lengthspring',u_norm(:,i)+delta_u(IN_NUM*(STEP_NUM-i+1)+1:IN_NUM*(STEP_NUM-i+2),j),IN_NUM);
        mexstep('step',1);
        N_array=mexstep('get','site_xpos');
        Nd_array=mexstep('get','sensordata');
        x2(1:POS_NUM,1)=reshape(N_array(:,1:NODE_NUM),1,3*NODE_NUM);
        x2(POS_NUM+1:SYS_NUM,1)=Nd_array(1:3*NODE_NUM);
        delta_ysingle(OUT_NUM*(STEP_NUM-i)+1:OUT_NUM*(STEP_NUM-i+1),j)=Ck*x2-Y_NORM(:,i+1);
    end
end 
% dy=sqrt(mean(delta_ysingle(2:16:end,:).^2,2));
% dyd=sqrt(mean(delta_ysingle(10:16:end,:).^2,2));

% arma fitting - least square: M1 * fitcoef = delta_y
for i=SKIP(1)+1:1:STEP_NUM % skip the first few steps to wait for enough data
    M1=[delta_ysingle(OUT_NUM*(STEP_NUM-i+1)+1:OUT_NUM*(STEP_NUM-i+1+q),:);delta_u(IN_NUM*(STEP_NUM-i+1)+1:IN_NUM*(STEP_NUM-i+qu+1),:)];
%     fitcoefsingle(:,:,i)=delta_ysingle(OUT_NUM*(STEP_NUM-i)+1:OUT_NUM*(STEP_NUM-i+1),:)*M1'/(M1*M1');
%     aa=delta_u(IN_NUM*(STEP_NUM-i+1)+1:IN_NUM*(STEP_NUM-i+qu+1),:)*delta_u(IN_NUM*(STEP_NUM-i+1)+1:IN_NUM*(STEP_NUM-i+qu+1),:)';
    aa=delta_ysingle(OUT_NUM*(STEP_NUM-i+1)+1:OUT_NUM*(STEP_NUM-i+1+q),:)*delta_ysingle(OUT_NUM*(STEP_NUM-i+1)+1:OUT_NUM*(STEP_NUM-i+1+q),:)';
    min(eig(aa));
    [Q,R]=qr(M1');
    fitcoefsingle(:,:,i)=(R\Q'*delta_ysingle(OUT_NUM*(STEP_NUM-i)+1:OUT_NUM*(STEP_NUM-i+1),:)')';
    rs(i)=sqrt(mean(mean((delta_ysingle(OUT_NUM*(STEP_NUM-i)+1:OUT_NUM*(STEP_NUM-i+1),:)-fitcoefsingle(:,:,i)*M1).^2,1))); % residual
end
rmaxs = max(rs)
mexstep('exit');

% % construct augmented Ak, Bk
% for i=SKIP(1)+1:1:STEP_NUM 
%     A_aug(:,:,i)=[fitcoefsingle(:,1:OUT_NUM*q,i),fitcoefsingle(:,OUT_NUM*q+IN_NUM+1:end,i);
%       eye((q-1)*OUT_NUM),zeros((q-1)*OUT_NUM,OUT_NUM+IN_NUM*(qu-1));
%       zeros(IN_NUM*(qu-1),OUT_NUM*q),[zeros(IN_NUM,IN_NUM*(qu-1));eye(IN_NUM*(qu-2)),zeros(IN_NUM*(qu-2),IN_NUM)]];
%     B_aug(:,:,i)=[fitcoefsingle(:,OUT_NUM*q+1:OUT_NUM*q+IN_NUM,i);zeros(OUT_NUM*(q-1),IN_NUM);eye(IN_NUM*min(qu-1,1));zeros(IN_NUM*(qu-2),IN_NUM)];
% end
% for i = SKIP(1)+1:1:STEP_NUM-size(B_aug(:,:,i),1)+1
%     tvcg=B_aug(:,:,i);
%     tp=B_aug(:,:,i);
%     for j=1:1:size(B_aug(:,:,i),1)-1
%         tp=A_aug(:,:,i+j)*tp;
%         tvcg=[tvcg tp];
%     end
% ctmr(i)=rank(tvcg);
% end
% ctmr

%% prediction check with rolling window
TEST_NUM=100; % number of monte-carlo runs to verify the fitting result
ucheck=0.001*1*randn(IN_NUM*(STEP_NUM+1),TEST_NUM); % input used for checking
y_sim=zeros(OUT_NUM*(STEP_NUM+1),TEST_NUM); % output from real system
y_predb=zeros(OUT_NUM*(STEP_NUM+1),TEST_NUM); % output from arma model
y_preds=zeros(OUT_NUM*(STEP_NUM+1),TEST_NUM);
mexstep('load',MODEL); % load model
for j=1:1:TEST_NUM
    mexstep('reset');
    mexstep('forward');
    for i=1:1:STEP_NUM
        ucheck(IN_NUM*(STEP_NUM-i+1)+1:IN_NUM*(STEP_NUM-i+2),j)=max(lb-u_norm(:,i),min(ub-u_norm(:,i),ucheck(IN_NUM*(STEP_NUM-i+1)+1:IN_NUM*(STEP_NUM-i+2),j)));
        mexstep('set','tendon_lengthspring',u_norm(:,i)+ucheck(IN_NUM*(STEP_NUM-i+1)+1:IN_NUM*(STEP_NUM-i+2),j),IN_NUM);
        mexstep('step',1);
        N_array=mexstep('get','site_xpos');
        Nd_array=mexstep('get','sensordata');
        x2(1:POS_NUM,1)=reshape(N_array(:,1:NODE_NUM),1,3*NODE_NUM);
        x2(POS_NUM+1:SYS_NUM,1)=Nd_array(1:3*NODE_NUM);
        y_sim(OUT_NUM*(STEP_NUM-i)+1:OUT_NUM*(STEP_NUM-i+1),j)=Ck*x2-Y_NORM(:,i+1);
    end
end

% batch
y_predb(OUT_NUM*(STEP_NUM-SKIP(1))+1:OUT_NUM*(STEP_NUM+1),:)=y_sim(OUT_NUM*(STEP_NUM-SKIP(1))+1:OUT_NUM*(STEP_NUM+1),:); % manually match the first few steps
for i=SKIP(1)+1:1:STEP_NUM % start to apply input after having enough data
    M2=[y_predb(OUT_NUM*(STEP_NUM-i+1)+1:OUT_NUM*(STEP_NUM-i+1+q),:);ucheck(IN_NUM*(STEP_NUM-i+1)+1:IN_NUM*(STEP_NUM-i+qu+1),:)];
    y_predb(OUT_NUM*(STEP_NUM-i)+1:OUT_NUM*(STEP_NUM-i+1),:)=fitcoefbatch(:,:,i)*M2;
end

% single
y_preds(OUT_NUM*(STEP_NUM-SKIP(1))+1:OUT_NUM*(STEP_NUM+1),:)=y_sim(OUT_NUM*(STEP_NUM-SKIP(1))+1:OUT_NUM*(STEP_NUM+1),:); % manually match the first few steps
for i=SKIP(1)+1:1:STEP_NUM % start to apply input after having enough data
    M2=[y_preds(OUT_NUM*(STEP_NUM-i+1)+1:OUT_NUM*(STEP_NUM-i+1+q),:);ucheck(IN_NUM*(STEP_NUM-i+1)+1:IN_NUM*(STEP_NUM-i+qu+1),:)];
    y_preds(OUT_NUM*(STEP_NUM-i)+1:OUT_NUM*(STEP_NUM-i+1),:)=fitcoefsingle(:,:,i)*M2;
end

tra_diff_batch=norm(y_predb(1:OUT_NUM*(STEP_NUM-SKIP(end)),:)-y_sim(1:OUT_NUM*(STEP_NUM-SKIP(end)),:))
tra_diff_single=norm(y_preds(1:OUT_NUM*(STEP_NUM-SKIP(end)),:)-y_sim(1:OUT_NUM*(STEP_NUM-SKIP(end)),:))

% plot y_sim and y_pred to check if they match
rpredb=mean(reshape(y_predb,OUT_NUM,STEP_NUM+1,TEST_NUM),3);
rpreds=mean(reshape(y_preds,OUT_NUM,STEP_NUM+1,TEST_NUM),3);
rsim=mean(reshape(y_sim,OUT_NUM,STEP_NUM+1,TEST_NUM),3);
figure()
% plot([fliplr(rpredb(1,:,1))' fliplr(rsim(1,:,1))' fliplr(rpredb(2,:,1))' fliplr(rsim(2,:,1))' fliplr(rpreds(1,:,1))' fliplr(rpreds(2,:,1))']); % plot only the first state
% legend('pred1b','sim1','pred2b','sim2','pred1s','pred2s');
plot([fliplr(rsim(1,:))' fliplr(rpreds(1,:))']); % plot only the first state
legend('sim1','pred1s');
xlabel('step')
% ylim([0,0.1])
% figure()
% plot(rb)
% xlabel('step')
% ylabel('residual')

% figure()
% plot([flip(dy),flip(dyd)]);
% legend('dy','dydot')
% xlabel('step')
% xlim([1,40])

mexstep('exit');