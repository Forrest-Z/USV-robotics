clear;
close all;

% read csv file
path='/home/skloe/Coding/CPP1X/USV/DPfloatover/QT/build/dataprocess/';
name_first='First.csv';
path_first=strcat(path,name_first);
totaldata_first=csvread(path_first,1,0);
% generate timestamp in seconds
timestamp0_first=totaldata_first(1,3);
timestamp_first=(totaldata_first(:,3)-timestamp0_first)*86400.0;  % seconds
Position_first=totaldata_first(:,4:9);
State_first=totaldata_first(:,10:15);
Tau_first=totaldata_first(:,16:18);
est_first=totaldata_first(:,19:21);
alpha_first=totaldata_first(:,22:24);
rpm_first=totaldata_first(:,25:27);

name_second='Second.csv';
path_second=strcat(path,name_second);
totaldata_second=csvread(path_second,1,0);
% generate timestamp in seconds
timestamp0_second=totaldata_second(1,3);
timestamp_second=(totaldata_second(:,3)-timestamp0_second)*86400.0;  % seconds
Position_second=totaldata_second(:,4:9);
State_second=totaldata_second(:,10:15);
Tau_second=totaldata_second(:,16:18);
est_second=totaldata_second(:,19:21);
alpha_second=totaldata_second(:,22:24);
rpm_second=totaldata_second(:,25:27);


%% figure 1 of II vessel
s_xmin=0000;
s_xmax=700;
desiredposition_second=[0.6 -2 0];
figure(1)
subplot(311)
plot(timestamp_second,Position_second(:,1),'-r','linewidth',2);hold on;
plot(timestamp_second,State_second(:,1),':k','linewidth',2); hold on;
plot([0,timestamp_second(end)],[desiredposition_second(1),desiredposition_second(1)],'--b');
xlim([s_xmin s_xmax]);
legend('position','State');
xlabel('time(s)');
ylabel('surge(m)');
title('Position comparison --- II vessel')


subplot(312)
plot(timestamp_second,Position_second(:,2),'-r','linewidth',2); hold on;
plot(timestamp_second,State_second(:,2),':k','linewidth',2); hold on;
plot([0,timestamp_second(end)],[desiredposition_second(2),desiredposition_second(2)],'--b');
xlim([s_xmin s_xmax]);
legend('position','State');
xlabel('time(s)');
ylabel('sway(m)');

subplot(313)
plot(timestamp_second,Position_second(:,6),'-r','linewidth',2);hold on;
plot(timestamp_second,State_second(:,3)*180/pi,':k','linewidth',2); hold on;
plot([0,timestamp_second(end)],[desiredposition_second(3),desiredposition_second(3)],'--b');
xlim([s_xmin s_xmax]);
legend('position','State');
xlabel('time(s)');
ylabel('Yaw(rad)');

%% figure 2 
figure(2)
subplot(311)
plot(timestamp_second,State_second(:,4),'-r','linewidth',2);
xlim([s_xmin s_xmax]);
xlabel('time(s)');
ylabel('surge(m/s)');
title('Velocity --- II vessel')

subplot(312)
plot(timestamp_second,State_second(:,5),'-r','linewidth',2);
xlim([s_xmin s_xmax]);
xlabel('time(s)');
ylabel('sway(m/s)');

subplot(313)
plot(timestamp_second,State_second(:,6),'-r','linewidth',2);
xlim([s_xmin s_xmax]);
xlabel('time(s)');
ylabel('yaw(rad/s)');



%% figure 3
figure(3)
title('Trajectory --- II vessel')
plot(Position_second(:,2),Position_second(:,1),'ro','MarkerSize',2); 


%% figure 4
figure(4)
subplot(311)
plot(timestamp_second, Tau_second(:,1),'-r','linewidth',2); hold on; 
plot(timestamp_second, est_second(:,1),':k','linewidth',2);
xlim([s_xmin s_xmax]);
xlabel('time(s)');
ylabel('surge(N)');
legend('Desired force','Estimated force');
title('PID force --- II vessel')
subplot(312)
plot(timestamp_second, Tau_second(:,2),'-r','linewidth',2); hold on; 
plot(timestamp_second, est_second(:,2),':k','linewidth',2);
xlim([s_xmin s_xmax]);
xlabel('time(s)');
ylabel('sway(N)');
legend('Desired force','Estimated force');
subplot(313)
plot(timestamp_second, Tau_second(:,3),'-r','linewidth',2); hold on; 
plot(timestamp_second, est_second(:,3),':k','linewidth',2);
xlim([s_xmin s_xmax]);
xlabel('time(s)');
ylabel('Yaw(N*m)');
legend('Desired force','Estimated force');


%% figure 1 of I vessel
desiredposition_first=[0.6 2 0];
figure(5)
subplot(311)
plot(timestamp_first,Position_first(:,1),'-r','linewidth',2);hold on;
plot(timestamp_first,State_first(:,1),':k','linewidth',2); hold on;
plot([0,timestamp_first(end)],[desiredposition_first(1),desiredposition_first(1)],'--b');
xlim([s_xmin s_xmax]);
legend('position','State');
xlabel('time(s)');
ylabel('surge(m)');
title('Position comparison --- I vessel')

subplot(312)
plot(timestamp_first,Position_first(:,2),'-r','linewidth',2); hold on;
plot(timestamp_first,State_first(:,2),':k','linewidth',2); hold on;
plot([0,timestamp_first(end)],[desiredposition_first(2),desiredposition_first(2)],'--b');
xlim([s_xmin s_xmax]);
legend('position','State');
xlabel('time(s)');
ylabel('sway(m)');

subplot(313)
plot(timestamp_first,Position_first(:,6),'-r','linewidth',2);hold on;
plot(timestamp_first,State_first(:,3)*180/pi,':k','linewidth',2); hold on;
plot([0,timestamp_first(end)],[desiredposition_first(3),desiredposition_first(3)],'--b');
xlim([s_xmin s_xmax]);
legend('position','State');
xlabel('time(s)');
ylabel('Yaw(rad)');

%% figure 2 
figure(6)
subplot(311)
plot(timestamp_first,State_first(:,4),'-r','linewidth',2);
xlim([s_xmin s_xmax]);
xlabel('time(s)');
ylabel('surge(m/s)');
title('Velocity --- I vessel')

subplot(312)
plot(timestamp_first,State_first(:,5),'-r','linewidth',2);
xlim([s_xmin s_xmax]);
xlabel('time(s)');
ylabel('sway(m/s)');

subplot(313)
plot(timestamp_first,State_first(:,6),'-r','linewidth',2);
xlim([s_xmin s_xmax]);
xlabel('time(s)');
ylabel('yaw(rad/s)');



%% figure 3
figure(7)
title('Trajectory --- I vessel')
plot(Position_first(:,2),Position_first(:,1),'ro','MarkerSize',2); 


%% figure 4
figure(8)
subplot(311)
plot(timestamp_first, Tau_first(:,1),'-r','linewidth',2); hold on; 
plot(timestamp_first, est_first(:,1),':k','linewidth',2);
xlim([s_xmin s_xmax]);
xlabel('time(s)');
ylabel('surge(N)');
legend('Desired force','Estimated force');
title('PID force --- I vessel')
subplot(312)
plot(timestamp_first, Tau_first(:,2),'-r','linewidth',2); hold on; 
plot(timestamp_first, est_first(:,2),':k','linewidth',2);
xlim([s_xmin s_xmax]);
xlabel('time(s)');
ylabel('sway(N)');
legend('Desired force','Estimated force');
subplot(313)
plot(timestamp_first, Tau_first(:,3),'-r','linewidth',2); hold on; 
plot(timestamp_first, est_first(:,3),':k','linewidth',2);
xlim([s_xmin s_xmax]);
xlabel('time(s)');
ylabel('Yaw(N*m)');
legend('Desired force','Estimated force');


