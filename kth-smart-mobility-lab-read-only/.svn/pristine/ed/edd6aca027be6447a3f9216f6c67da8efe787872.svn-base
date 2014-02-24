function [xmitt ymitt]=find_center(A1,dir1,A2,dir2)

if dir1~=dir2
   A1=A1(:,end:-1:1); 
end
 
til=1:length(A1(1,:));
tol=1:length(A2(1,:));

ti=linspace(1,length(A1(1,:)),500);
to=linspace(1,length(A2(1,:)),500);

xil = interp1(til,A1(1,:),ti);
yil = interp1(til,A1(2,:),ti);
xol = interp1(tol,A2(1,:),to);
yol = interp1(tol,A2(2,:),to);

xmitt=zeros(1,length(ti));
ymitt=zeros(1,length(ti));

for i=1:length(ti)
    xmitt(i)=(xil(i)+xol(i))/2;
    ymitt(i)=(yil(i)+yol(i))/2;
end