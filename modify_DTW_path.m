%%
%DTW路径
function [dtwpath]=modify_DTW_path(s1,I,s2,J,w)%
    pathsig=1;
    dtwpath=zeros(I-2*w,J-2*w);    
    [~,DTW]=modify_DTW(s1,I,s2,J,w); % ,J1
  	%寻找路径,这里我采用了逆向搜索法
    i=I-2*w;
	j=J-2*w;%J1;%J-2*w
	while j>1||i>1
		if i>1 && j>1
			m=min(min(DTW(i-1,j),DTW(i-1,j-1)),DTW(i,j-1));
			if m==DTW(i-1,j)
				dtwpath(i-1,j)=pathsig;
				i=i-1;
            elseif m==DTW(i-1,j-1)
				dtwpath(i-1,j-1)=pathsig;
				i=i-1;
				j=j-1;
            elseif m==DTW(i,j-1)
				dtwpath(i,j-1)=pathsig;
				j=j-1;
            end
        elseif i==1
			dtwpath(1,j-1)=pathsig;
			j=1;
        elseif j==1
			dtwpath(i-1,1)=pathsig;
			i=1;
        end
      
    end            
	dtwpath(I-2*w,J-2*w)=pathsig;
    %dtwpath(I,J)=pathsig;
    %dtwpath(I-2*w,J1)=pathsig;
    
end