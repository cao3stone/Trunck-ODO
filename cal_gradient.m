function cal_gradient(oMFP,rMFP,t_m,t_ins)

oMFP=movmean(oMFP,100,2);
rMFP=movmean(rMFP,100,2);

% row_number=3;
% figure
% hold on
% plot(t_m,oMFP(row_number,:))
% plot(t_ins,rMFP(row_number,:))
% legend('oMFP','rMFP')

n_o=length(oMFP);
n_r=length(rMFP);

lever=0.5;
window=10;
doMFP=zeros(3,n_o-2*window);
number=1;
for k=window+1:n_o-window
    doMFP(:,number)=lever*(oMFP(:,k)-oMFP(:,k-window))+(1-lever)*((oMFP(:,k+window)-oMFP(:,k-window))/2);
    number=number+1;
end
drMFP=zeros(3,n_r-2*window);
number=1;
for k=window+1:n_r-window
    drMFP(:,number)=lever*(rMFP(:,k)-rMFP(:,k-window))+(1-lever)*((rMFP(:,k+window)-rMFP(:,k-window))/2);
    number=number+1;
end

t_m_d=t_m(1:n_o-2*window);
t_ins_d=t_ins(1:n_r-2*window);
doMFP=[doMFP;vecnorm(doMFP,2)];
drMFP=[drMFP;vecnorm(drMFP,2)];

row_number=4;
figure
plot(t_m_d,doMFP(row_number,:))
hold on
plot(t_ins_d,drMFP(row_number,:))
legend('oMFP','rMFP')

end