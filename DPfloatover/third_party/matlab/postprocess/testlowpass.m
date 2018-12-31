clear;
close all;

Frequency=50;
sampletime=1/Frequency;
max_positive_velocity_surge=1;
max_negative_velocity_surge=-0.9;
max_positive_velocity_sway=0.5;
max_negative_velocity_sway=-0.4;
max_positive_velocity_yaw=1; % deg/s
max_negative_velocity_yaw=-1; % deg/s

% read csv file
path='/home/skloe/Coding/CPP1X/USV/DPfloatover/QT/build/dataprocess/';
name_first='First.csv';
path_first=strcat(path,name_first);
totaldata_first=csvread(path_first,1,0);
% generate timestamp in seconds
timestamp0_first=totaldata_first(1,3);
timestamp_first=(totaldata_first(:,3)-timestamp0_first)*86400.0;  % seconds

% resample all data
totalnum=floor(Frequency*max(timestamp_first));
r_timestamp_first=sampletime*(1:totalnum);
resample_totaldata_first=ones(totalnum,24);
for i=1:24
    resample_totaldata_first(:,i)=interp1(timestamp_first,totaldata_first(:,i+3),r_timestamp_first,'linear');
end
Position_first=resample_totaldata_first(:,1:6);
State_first=resample_totaldata_first(:,7:12);
Tau_first=resample_totaldata_first(:,13:15);
est_first=resample_totaldata_first(:,16:18);
alpha_first=resample_totaldata_first(:,19:21);
rpm_first=resample_totaldata_first(:,22:24);


name_second='Second.csv';
path_second=strcat(path,name_second);
totaldata_second=csvread(path_second,1,0);
% generate timestamp in seconds
timestamp0_second=totaldata_second(1,3);
timestamp_second=(totaldata_second(:,3)-timestamp0_second)*86400.0;  % seconds

% resample all data
totalnum=floor(Frequency*max(timestamp_second));
r_timestamp_second=sampletime*(1:totalnum);
resample_totaldata_second=ones(totalnum,24);
for i=1:24
    resample_totaldata_second(:,i)=interp1(timestamp_second,totaldata_second(:,i+3),r_timestamp_second,'linear');
end
Position_second=resample_totaldata_second(:,1:6);
Position_second_remove=Position_second;
State_second=resample_totaldata_second(:,7:12);
Tau_second=resample_totaldata_second(:,13:15);
est_second=resample_totaldata_second(:,16:18);
alpha_second=resample_totaldata_second(:,19:21);
rpm_second=resample_totaldata_second(:,22:24);

% low pass and bad data removal
average_num_surge=50;
average_num_sway=50;
average_num_yaw=50;

v_average_surge=zeros(average_num_surge,1);
v_average_sway=zeros(average_num_sway,1);
v_average_yaw=zeros(average_num_yaw,1);

lowpassmotion_second=zeros(totalnum,3);
lowpassvelocity_second=zeros(totalnum,3);

for i=2:totalnum
    % copy avergae vector
    tv_average_surge=zeros(average_num_surge,1);
    tv_average_sway=zeros(average_num_sway,1);
    tv_average_yaw=zeros(average_num_yaw,1);
    tv_average_surge(1:(average_num_surge-1))=v_average_surge(2:average_num_surge);
    tv_average_sway(1:(average_num_sway-1))=v_average_sway(2:average_num_sway);
    tv_average_yaw(1:(average_num_yaw-1))=v_average_yaw(2:average_num_yaw);

    tv_average_surge(average_num_surge)=Position_second(i,1);
    tv_average_sway(average_num_sway)=Position_second(i,2);
    
    temp_v_yaw=(Position_second(i,6)-Position_second(i-1,6))/sampletime;
    if(temp_v_yaw>max_positive_velocity_yaw)
        Position_second_remove(i,6)=Position_second(i-1,6)+max_positive_velocity_yaw*sampletime;
    elseif (temp_v_yaw<max_negative_velocity_yaw)
        Position_second_remove(i,6)=Position_second(i-1,6)+max_negative_velocity_yaw*sampletime;
    end
    tv_average_yaw(average_num_yaw)=Position_second(i,6);

    v_average_surge=tv_average_surge;
    v_average_sway=tv_average_sway;
    v_average_yaw=tv_average_yaw;
    lowpassmotion_second(i,1)=mean(v_average_surge);
    lowpassmotion_second(i,2)=mean(v_average_sway);
    lowpassmotion_second(i,3)=mean(v_average_yaw);
   

end


% calcuate the velocity
for i=1:3
    lowpassvelocity_second(2:totalnum,i)=diff(lowpassmotion_second(:,i))/sampletime;
end

% low pass and bad data removal
average_num_yaw_v=30;
v_average_yaw_v=zeros(average_num_yaw_v,1);

lowpassv_second_yaw=zeros(totalnum,1);


for i=1:totalnum-1
    % copy avergae vector
    tv_average_yaw=zeros(average_num_yaw_v,1);
    tv_average_yaw(1:(average_num_yaw_v-1))=v_average_yaw_v(2:average_num_yaw_v);
    tv_average_yaw(average_num_yaw_v)=lowpassvelocity_second(i,3);
    v_average_yaw_v=tv_average_yaw;
    lowpassv_second_yaw(i)=mean(v_average_yaw_v);
end


figure(1)
subplot(211)
plot(r_timestamp_second,lowpassvelocity_second(:,2),'-r','linewidth',2);hold on;
plot(r_timestamp_second,State_second(:,5),':k','linewidth',2); hold on;
% plot(r_timestamp_second,lowpassv_second_yaw,'--g','linewidth',2); hold on;
xlim([10,1200])
subplot(212)
plot(r_timestamp_second,Position_second(:,2),'-r','linewidth',2);hold on;
% plot(r_timestamp_second,Position_second_remove(:,6),':k','linewidth',2);hold on;
xlim([10,1200])

s_xmin=70;
s_xmax=100;
index2view=find(timestamp_second>s_xmin & timestamp_second<s_xmax);
timestamp_second=timestamp_second(index2view);
desiredposition_second=[0 0 45];

figure(1)
% subplot(311)
plot(timestamp_second,State_second(index2view,6),'-r','linewidth',2);hold on;
plot(timestamp_second(1:end-1),diff(State_second(index2view,3))/0.02,':k','linewidth',2); hold on;
