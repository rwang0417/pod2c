% EXAMPLE:
%    - Manual node matrix specification
%    - Manual connectivity matrix generation
%    - Prestress equilibrium solver
%    - Pinned_nodes = [n1 1 0 1; n2 1 0 0]; n1 node fixed in x and z direction, n2 node fixed in x direction 

clear all; clc; close all

%% dbar
% N = [1 1 1;0 0 2;-1 1 1;0 -1.414 1;0 0 0;1 1 0;-1 1 0;0 -1.414 0]';
% C_b_in = [1 2;1 5;2 3;2 4;3 5;4 5];    % This is indicating that bar 1 is the vector from node 1 to node 2 and so on
% C_s_in = [2 5;1 3;3 4;1 4;2 6;2 7;2 8];  % Similarly, this is saying string 1 connects node 2 to node 5 and so on
% Pinned_nodes = [5;6;7;8]; % dbar
% StepNum = 200;
% Stiffness = 100;
% force_density_min = 1;

%% t1d1 3d
% MODEL = '.\model\t1d1_3dsmall.xml';
% % % read N from mujoco
% mexstep('load',MODEL);
% mexstep('reset');
% mexstep('forward');
% N_array=mexstep('get','site_xpos');
% Nd_array=mexstep('get','sensordata');
% N=N_array(:,1:end-1);
% mexstep('exit');
% % % t1d1_3d axis symm & large angle
% C_b_in = [1 12;1 2;2 3;2 4;4 5;8 5;8 2;2 6;2 7;6 12;12 9;2 9;2 10;2 11;5 11]; 
% C_s_in = [2 12;1 9;1 6;9 6;2 5;4 11;4 8;8 11;3 12;10 12;7 12;3 5;5 10;5 7;3 10;3 7;10 7;5 13;5 15;5 14];
% Pinned_nodes = [12;13;14;15]; % t1d1_3d
% StepNum = 200;
% Stiffness = 100;
% force_density_min = 1;

%% t2d1 3d
MODEL = '.\model\t2d1_3d_folded.xml';
% read N from mujoco
mexstep('load',MODEL);
mexstep('reset');
mexstep('forward');
N_array=mexstep('get','site_xpos');
Nd_array=mexstep('get','sensordata');
N=N_array(:,1:end-1);
mexstep('exit');
% % t1d1_3d axis symm & large angle
C_b_in = [26 1;1 2;2 3;2 6;6 7;7 8;7 11;11 12;12 16;12 13;16 17;18 26;18 2;
          2 4;2 5;2 19;19 7;7 9;7 10;7 20;20 12;12 14;12 15;12 21;21 17;
          22 26;22 2;2 23;23 7;7 24;24 12;12 25;25 17]; 
C_s_in = [26 2;22 1;18 1;22 18;2 7;6 23;19 6;23 19;3 26;26 5;26 4;3 7;5 7;4 7;
          3 4;3 5;4 5;8 10;8 9;9 10;26 8;26 9;26 10;8 17;9 17;10 17;7 13;7 14;
          7 15;11 20;24 11;24 20;13 14;13 15;14 15;13 17;14 17;15 17;7 12;
          12 17;16 25;16 21;21 25;17 27;17 29;17 28];
Pinned_nodes = [26;27;28;29];
StepNum = 100;
Stiffness = 100;
force_density_min = 1;

%%
% Manually specify node positions (in 3D).
% n1 = [-1 1/2 0]';
% n2 = [-1/2 -1 0]';
% n3 = [1 1/2 0]';
% n4 = [-1/2 1 0]';

% Put node vectors in node matrix. Node matrix has to be 3xn for n nodes.
% N = [n1 n2 n3 n4];
% % t1d1_3d
% load('N.mat');

% Manually specify connectivity indices.
% % t1d1_3d previous model
% C_b_in = [1 12;1 2;2 3;2 4;4 5;9 5;9 2;2 6;2 8;8 12;12 10;2 10;2 7;2 11;5 11]; 
% C_s_in = [2 12;1 10;1 8;10 8;2 5;4 11;4 9;9 11;3 12;7 12;6 12;3 5;5 7;5 6;3 7;3 6;6 7;5 13;5 15;5 14];

% Convert the above matrices into full connectivity matrices.
C_b = tenseg_ind2C(C_b_in,N);
C_s = tenseg_ind2C(C_s_in,N);

% Plot the structure to make sure it looks right
Dbar.Bradius = 0.02*ones(size(C_b,1),1); % Radius of bars [# bars x 1]
Dbar.Sradius = 0.01*ones(size(C_s,1),1); % Radius of strings [# strings x 1]
Dbar.Nradius = 0.005*ones(size(N,2),1); % Radius of node spheres [# nodes x 1]
% tenseg_plot(N,C_b,C_s)
tenseg_plot(N,C_b,C_s,[],[],[],[],Dbar)


% Define the Force Matrix
W = zeros(size(N));
% W(:,1) = [0 -1 0];
% W(:,3) = [0 1 0];
% W(:,2) = [-1 0 0];
% W(:,4) = [1 0 0];

% Pinned_nodes
Pinned_nodes = [Pinned_nodes ones(size(Pinned_nodes,1),3)];
% Pinned_nodes = [6 1 1 1;7 1 1 1;8 1 1 1];

% Define a tensegrity data structure, 'Dbar'
Dbar.N = N;
Dbar.C_b = C_b;
Dbar.C_s = C_s;
Dbar.W = W;
Dbar.Pinned_nodes = Pinned_nodes;
Dbar.bar_material = 'Aluminum';
Dbar.string_material = 'Aluminum';

% Solve for an equilibrium condition 
[Force_den,Support_Load] = tenseg_equilibrium(Dbar, force_density_min);

S = N*C_s';
len_str = diag(sqrt(S'*S));
Tension = Force_den(1:size(S,2)).*len_str;
rest_len = len_str - Tension / Stiffness;
if min(rest_len) < 0
    disp('Stiffness is too small')
end

%% save nominal control to .txt file
% rest length
PreStressMat=repmat(rest_len',1,StepNum);
fid = fopen('length0.txt','wt');
fprintf(fid,'%f ',PreStressMat);
fprintf(fid,'\n');
fclose(fid);

% % tension
% PreStressMat=repmat(-Tension',1,StepNum);
% fid = fopen('init0.txt','wt');
% fprintf(fid,'%f ',PreStressMat);
% fprintf(fid,'\n');
% fclose(fid);

