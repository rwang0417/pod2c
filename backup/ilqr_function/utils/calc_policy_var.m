clear all;
filename=["result01.txt","result02.txt","result03.txt","result04.txt"];

for i=1:1:size(filename,2)
    fid = fopen('./pend_nominal_action_d2c/'+filename(i),'r');
    U_d2c(i,:) = fscanf(fid, '%f');
    fclose(fid);
end

for i=1:1:size(filename,2)
    fid = fopen('./pend_nominal_action_ddpg/'+filename(i),'r');
    U_ddpg(i,:) = fscanf(fid, '%f');
    fclose(fid);
end

d2c_avg_std_action = mean(std(U_d2c,0,1))
ddpg_avg_std_action = mean(std(U_ddpg,0,1))

figure()
plot([0,1],[d2c_avg_std_action,d2c_avg_std_action])
hold on;
plot(0,ddpg_avg_std_action,'ro')
legend('d2c','ddpg')
xlabel('process noise level')
ylabel('std of nominal action')