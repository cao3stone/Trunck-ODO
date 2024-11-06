% close all;clear;clc;

%load('data.mat');
% load('m.mat');

m_front=m_f;%从工作区读取数据
m_back=m_b;%从工作区读取数据

first=m_front(:,590:4100)';
second=m_back(:,590:4100)';

figure
subplot(3,1,1)
plot(-first(:,1));
hold on
plot(second(:,1));
subplot(3,1,2)
plot(first(:,2));
hold on
plot(second(:,2));
subplot(3,1,3)
plot(first(:,3));
hold on
plot(second(:,3));

% first=oMFP';
% second=rMFP';
pointsize=size(first,1);
w=10;
[dist,DTW]=modify_DTW(first(:,3),pointsize,second(:,3),pointsize,w);
[dtwpath]=modify_DTW_path(first(:,3),pointsize,second(:,3),pointsize,w);
[m,n]=find(dtwpath==1);   


     figure         
       for j=1:size(m,1)          
         line([w+m(j) w+n(j)],[first(w+m(j),3) second(w+n(j),3)]);% line([i+w+m(j) i+w+n(j)],[magnfirst(i+w+m(j),3) magnsecond(i+w+n(j),3)]);
          hold on
       end
         plot(first(:,3),'b');%magnfirst(:,3)
         hold on
         plot(second(:,3),'r');%magnsecond(:,3)
         legend('匹配线','first','second');
         xlabel('采样点数');
         ylabel('磁场强度（uT）');
       
