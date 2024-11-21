%%
function [dist,DTW]=modify_DTW(s1,I,s2,J,w,w1,w2)%,J1

for p=1:I-2*w
    gradient11(p,:)=((s1(w+p,:)-s1(p,:))+(s1(p+2*w,:)-s1(p,:))/2)/2;%s1(i+w,:)-s1(i,:)，第w+p个点的梯度
end

for q=1:J-2*w
    gradient12(q,:)=((s2(w+q,:)-s2(q,:))+(s2(q+2*w,:)-s2(q,:))/2)/2;%s2(j+w,:)-s2(j,:)
end

    DTW=zeros(I-2*w,J-2*w);
    d1=0;
    d2=0;
    for p=1:I-2*w
       d1 =d1+ sqrt((s1(w+p)-s2(w+1))^2);
       DTW(p,1)=d1;
    end
    
    for q=1:J-2*w
      d2 = d2+sqrt((s1(w+1)-s2(w+q))^2);
      DTW(1, q)=d2;
    end  
    
    for p=2:I-2*w
        for q=2:J-2*w 
%             w1=10*sqrt(((gradient11(p))-(gradient12(q)))^2);
%             w2=20*sqrt((s1(w+p)-s2(w+q))^2);
            dist =w1*sqrt((s1(w+p)-s2(w+q))^2)+w2*sqrt(((gradient11(p))-(gradient12(q)))^2);%b系作差求模匹配法：行走1—3—5:30,50; 7—9：30-50；%投影法
            DTW(p, q) = dist + min(min(DTW(p-1, q),DTW(p, q-1)), DTW(p-1, q-1));
        end
    end
    dist=DTW(end, end);
       
   %[~,J1]=find(DTW(end,1:end)==min(DTW(end,1:end)));
    
%     dist=DTW(I-w,J-w);
%     DTW1=DTW(w+1:I-w,w+1:J-w);
%     [~,J1]=find(DTW1(end,1:end-1)==min(DTW1(end,1:end-1)));
     %J1=min(J1);
end

