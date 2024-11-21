    % close all;
    % clear;
    % clc;
function v=gradient2(m_f,m_b,row_number)    

    % for i=w+1:size(magnfirst,1)-w
    %     gradient11(i,:)=((magnfirst(i,:)-magnfirst(i-w,:))+(magnfirst(i+w,:)-magnfirst(i-w,:))/2)/2;%s1(i+w,:)-s1(i,:)
    % end
    % 
    % for j=w+1:size(magnsecond,1)-w
    %     gradient12(j,:)=((magnsecond(j,:)-magnsecond(j-w,:))+(magnsecond(j+w,:)-magnsecond(j-w,:))/2)/2;%s2(j+w,:)-s2(j,:)
    % end

    % load('m.mat');

    m_front=m_f;%从工作区读取数据
    m_back=m_b;%从工作区读取数据
    magnfirst=m_front';
    magnsecond=m_back';
    %  magnfirst=vecnorm(m_front,1)';
    % magnsecond=vecnorm(m_back,1)';

    pointsize1=size(magnfirst,1);
    pointsize2=size(magnsecond,1);

    % window=1000;
    % w=200;
    window=500;
    w=50;
    w1=0.3;
    w2=1-w1;
    p=1;
    q=1;
    v=[];
    id=[];
    j=1;

    for i=1:pointsize1-window
        if m_front(row_number,i)~=0
            % [dist,DTW]=modify_DTW(magnfirst(i:i+window,row_number),window+1,magnsecond(i:i+window,row_number),window+1,w);%magnfirst(i:i+window,3),window+1,magnsecond(i:i+window,3),window+1,w
            % [dtwpath]=modify_DTW_path(magnfirst(i:i+window,row_number),window+1,magnsecond(i:i+window,row_number),window+1,w);
            [dtwpath]=modify_DTW_path(magnfirst(i:i+window),window+1,magnsecond(i:i+window),window+1,w,w1,w2);
        
            [m,n]=find(dtwpath==1); 
        
            idm=find(m==min(m));
            idn=find(n==min(n));
            
            id1=max(max(idm),max(idn));
            id=[id;id1];
            v1=0.26/((abs(m(id(end))-n(id(end))))/100);  
            % 
            % if v1>2
            %     v1=2;
            % end
            % 
            v=[v;v1];
        end
    end
    % plot(v,'o')
    % plot(v)
%      a=1;
%      if (m(a)~=m(a+1)) && (n(a)~=n(a+1))
%         id(p,:)=[m(a+1) n(a+1)];
%         p=p+1;
%         a=a+1;
%         v1=0.25/((abs(id(q,1)-id(q,2)))/100); 
%         q=q+1;
%         v=[v;v1];
%      end
    

     
if i==300  
figure   
for i=1:pointsize1-window
if mod(i,200)==0

       for j=1:100:size(m,1)          
         line([i+w+m(j) i+w+n(j)],[magnfirst(i+w+m(j)) magnsecond(i+w+n(j))]);% line([i+w+m(j) i+w+n(j)],[magnfirst(i+w+m(j),3) magnsecond(i+w+n(j),3)]);
          hold on
       end

         plot(magnfirst,'b');%magnfirst(:,3)
         hold on
         plot(magnsecond,'r');%magnsecond(:,3)
         legend('匹配线','magnfirst','magnsecond');
         xlabel('采样点数');
         ylabel('磁场强度（uT）');
end
end


%          for k=1:size(m,1)-1
%              if (m(k)~=m(k+1)) && (n(k)~=n(k+1))
%                  id(p,:)=[m(k+1) n(k+1)];
%                  p=p+1;
%              elseif (m(k)==m(k+1)) && (n(k)~=n(k+1))
%                  id(p,:)=[m(k+1) n(k+1)];
%                  p=p+1;
%              elseif (m(k)~=m(k+1)) && (n(k)==n(k+1))
%                  id(p,:)=[m(k+1) n(k+1)];
%                  p=p+1;                 
%            end 
              

%         v1=0.25/((abs(id(q,1)-id(q,2)))/100);  
%         q=q+1;
% 
%          v=[v;v1];
    % end
    % f=1;
    % for i=1:size(v)
    %     if v(i)<2
    %       vfinal(f,1)=v(i);
    %       f=f+1;
    %     end
    % end
    % 
    % 
    % 
    %  realv=54*0.8/((4100-590)*0.01)*ones(size(vfinal,1)); %5100-600
    % 
    %  rmse=sqrt(abs(sum((realv(:,1)-vfinal(:,1).^2))/size(vfinal,1)));
    % 
    % 
    % figure
    % plot(v(:,1),'-');
    % hold on
    % plot(realv(:,1),'-');
    % title('v图');
    % xlabel('采样点数');
    % ylabel('速度（m/s）');   
    % legend('计算速度','真实平均速度');
    % 
    % figure
    % plot(vfinal(:,1),'-');
    % hold on
    % plot(realv(:,1),'-');
    % title('v图');
    % xlabel('采样点数');
    % ylabel('速度（m/s）');   
    % legend('计算速度','真实平均速度');
end