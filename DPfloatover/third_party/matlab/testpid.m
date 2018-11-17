clear;
close all;

% read csv file
path='/home/skloe/Coding/CPP1X/USV/DPfloatover/QT/build/dataprocess/';

sampletime=0.1;
name_second='Second.csv';
s_xmin=2100;
s_xmax=2400;
path_second=strcat(path,name_second);
totaldata_second=csvread(path_second,1,0);
% generate timestamp in seconds
timestamp0_second=totaldata_second(1,3);
timestamp_second=(totaldata_second(:,3)-timestamp0_second)*86400.0;  % seconds


index2view=find(timestamp_second>s_xmin & timestamp_second<s_xmax);
timestamp_second=timestamp_second(index2view);
Position_second=totaldata_second(index2view,4:9);
State_second=totaldata_second(index2view,10:15);
Tau_second=totaldata_second(index2view,16:18);
est_second=totaldata_second(index2view,19:21);
alpha_second=totaldata_second(index2view,22:24);
rpm_second=totaldata_second(index2view,25:27);
State4control_second=totaldata_second(index2view,28:29);


desiredposition_second=[0;-3;pi/4];
Tg2b=[sqrt(2)/2 sqrt(2)/2 0; 
     -sqrt(2)/2 sqrt(2)/2 0;
     0 0 1];
desiredposition_second=Tg2b*desiredposition_second;

totalnum=size(State4control_second,1);

% calculate the state for control
position_error_integral=zeros(3,totalnum);
for i=1:totalnum-1
    tempvector=zeros(3,1);
    tempvector(1,1)=State4control_second(i,1)-desiredposition_second(1);
    tempvector(2,1)=State4control_second(i,2)-desiredposition_second(2);
    tempvector(3,1)=State_second(i,3)-desiredposition_second(3);
    position_error_integral(:,i+1)= position_error_integral(:,i)+tempvector*sampletime;
end

%% figure 1 of II vessel
figure(1)
subplot(311)
plot(timestamp_second,State4control_second(:,1)-desiredposition_second(1),'-r','linewidth',2);hold on;
% ylim([0 0.1]);
legend('position','State');
xlabel('time(s)');
ylabel('surge(m)');
title('Position comparison --- II vessel')

subplot(312)
plot(timestamp_second,State4control_second(:,2)-desiredposition_second(2),'-r','linewidth',2); hold on;
legend('position','State');
xlabel('time(s)');
ylabel('sway(m)');

subplot(313)
plot(timestamp_second,State_second(:,3)*180/pi-45,'-r','linewidth',2);hold on;
xlim([s_xmin s_xmax]);
legend('position','State');
xlabel('time(s)');
ylabel('Yaw(rad)');

%% figure 2 
figure(2)
subplot(311)
plot(timestamp_second,position_error_integral(1,:),'-r','linewidth',2);
xlabel('time(s)');
ylabel('surge(m/s)');
title('Integral --- II vessel')

subplot(312)
plot(timestamp_second,position_error_integral(2,:),'-r','linewidth',2);
xlabel('time(s)');
ylabel('sway(m/s)');

subplot(313)
plot(timestamp_second,position_error_integral(3,:),'-r','linewidth',2);
xlabel('time(s)');
ylabel('yaw(rad/s)');





