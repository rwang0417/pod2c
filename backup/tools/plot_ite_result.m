clear variables;clc;close all;tic;

load('ite_result_ilqr_cart_0init.mat');
load('ite_result_sqp_cart_0init.mat');

iter_num_ilqr = size(cost,2);
iter_num_sqp = size(cost_sqp,2);

% cost plot
figure;
plot(0:1:iter_num_ilqr-1,cost)
hold on
plot(0:1:iter_num_sqp-1,cost_sqp)
xlabel('iteration')
ylabel('episodic cost')
legend('ILQR','SQP')

% terminal state comparison
figure;
plot([x_traj_ite_ilqr(1,:,end)' x_traj_ite_ilqr(2,:,end)'])
hold on
plot([x_traj_ite_sqp(1,:,end)' x_traj_ite_sqp(2,:,end)'])
xlabel('step')
ylabel('state')
legend('ILQR','ILQR','SQP','SQP')

% terminal control comparison
figure;
plot([u_traj_ite_ilqr(1,:,end)' u_traj_ite_sqp(1,:,end)'])
xlabel('step')
ylabel('u')
legend('ILQR','SQP')

% state trajactory iteration
% ilqr
% skip_ilqr=3;plot_num_ilqr=floor((iter_num_ilqr)/skip_ilqr);
% figure;
% for i=1:1:plot_num_ilqr
%     iter_ilqr=1+skip_ilqr*(i-1);
%     subplot(5,ceil(plot_num_ilqr/5),i)
%     plot([x_traj_ite_ilqr(1,:,iter_ilqr)' x_traj_ite_ilqr(2,:,iter_ilqr)'])
%     xlabel('step')
%     ylabel('state')
%     title(['ite ' num2str(iter_ilqr)])
% end

% sqp
% skip_sqp=20;plot_num_sqp=floor(iter_num_sqp/skip_sqp);
% figure;
% for i=1:1:plot_num_sqp
%     iter_sqp=1+skip_sqp*(i-1);
%     subplot(5,ceil(plot_num_sqp/5),i)
%     plot([x_traj_ite_sqp(1,:,iter_sqp)' x_traj_ite_sqp(2,:,iter_sqp)'])
%     xlabel('step')
%     ylabel('state')
%     title(['ite ' num2str(iter_sqp)])
% end

% control trajactory iteration
% ilqr
% skip_ilqr=3;plot_num_ilqr=floor(iter_num_ilqr/skip_ilqr);
% horiozn=size(u_traj_ite_ilqr,2);
% figure;
% for i=1:1:plot_num_ilqr
%     iter_ilqr=1+skip_ilqr*(i-1);
%     subplot(5,ceil(plot_num_ilqr/5),i)
%     plot(0:1:horiozn-1, u_traj_ite_ilqr(1,:,iter_ilqr))
%     xlabel('step')
%     ylabel('u')
%     title(['ite ' num2str(iter_ilqr)])
% end

% sqp
% skip_sqp=20;plot_num_sqp=floor(iter_num_sqp/skip_sqp);
% horiozn=size(u_traj_ite_sqp,2);
% figure;
% for i=1:1:plot_num_sqp
%     iter_sqp=1+skip_sqp*(i-1);
%     subplot(5,ceil(plot_num_sqp/5),i)
%     plot(0:1:horiozn-1, u_traj_ite_sqp(1,:,iter_sqp))
%     xlabel('step')
%     ylabel('u')
%     title(['ite ' num2str(iter_sqp)])
% end

% feedback and nominal terms iteration
% ilqr
% skip_ilqr=1;plot_num_ilqr=floor(iter_num_ilqr/skip_ilqr);
% horiozn=size(u_traj_ite_ilqr,2);
% figure;
% for i=1:1:plot_num_ilqr
%     iter_ilqr=1+skip_ilqr*(i-1);
%     subplot(1,plot_num_ilqr,i)
%     plot([nominal(1,:,iter_ilqr)' feedback(1,:,iter_ilqr)'])
%     xlabel('step')
%     ylabel('delta u')
%     legend('nom','fb')
%     title(['ite ' num2str(iter_ilqr)])
% end