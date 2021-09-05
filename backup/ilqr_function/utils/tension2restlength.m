%% t2d1_3d
NODE_NUM = 25;
POS_NUM = 3 * NODE_NUM;
VEL_NUM = 3 * NODE_NUM;
SYS_NUM = POS_NUM + VEL_NUM;
IN_NUM = 46;
OUT_NUM = 150;
MODEL = 't2d1_3d.xml';
STEP_NUM = 200;
STIFFNESS = 100;
C_s_in = [26 2;22 1;18 1;22 18;2 7;6 23;19 6;23 19;3 26;26 5;26 4;3 7;5 7;4 7;
          3 4;3 5;4 5;8 10;8 9;9 10;26 8;26 9;26 10;8 17;9 17;10 17;7 13;7 14;
          7 15;11 20;24 11;24 20;13 14;13 15;14 15;13 17;14 17;15 17;7 12;
          12 17;16 25;16 21;21 25;17 27;17 29;17 28];

%% read nominal control sequence
fid = fopen('result0_t2d13d.txt','r');
U = fscanf(fid, '%f');
fclose(fid);
u_norm = reshape(U, IN_NUM, STEP_NUM);
delta_x = u_norm./STIFFNESS;
u_max = max(max(abs(u_norm)));

%% nominal restlength trajectory
restlength = zeros(IN_NUM, STEP_NUM);
mexstep('load',MODEL); % load model
mexstep('reset'); % reset all states and controls
mexstep('forward');
for i = 1 : 1 : STEP_NUM
    N_array=mexstep('get','site_xpos');
    C_s = tenseg_ind2C(C_s_in,N_array(:,1:end-1));
    S = N_array(:,1:end-1)*C_s';
    len_str = diag(sqrt(S'*S));
    restlength(:,i) = len_str - delta_x(:,i);
    mexstep('set','ctrl',u_norm(:,i),IN_NUM);
    mexstep('step',1);
end

%% output rest length
fid = fopen('length0.txt','wt');
for i = 1 : STEP_NUM
    for j = 1 : IN_NUM
        fprintf(fid,'%.10f ',restlength(j,i));
    end
end
fclose(fid);