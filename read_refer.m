function refer=read_refer(path)

data=readmatrix(path);
refer=data(:,2:4)';
% t=data(end,1)-data(1,1);
end