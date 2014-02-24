function onelane_disp(A,scale,j)
d=0.15;

if scale<0.7
   k1=0.94;
   k2=0.85;
else
   k1=0.95;
   k2=0.88;
end

Bouter1=zeros(3,length(A(1,:)));
Binner1=zeros(3,length(A(1,:)));
Wouter=zeros(3,length(A(1,:)));
Winner=zeros(3,length(A(1,:)));
Bouter2=zeros(3,length(A(1,:)));
Binner2=zeros(3,length(A(1,:)));

 for i=1:length(A(1,:))-1
        point=[A(1,i+1)-A(1,i) A(2,i+1)-A(2,i) 0];
        orthogonal=cross(point,[0 0 1]);
        orthonormal=orthogonal/norm(orthogonal);

        Bouter1(:,i)=orthonormal*scale*1*d+[A(1,i) A(2,i) 0];
        Binner1(:,i)=-orthonormal*scale*1*d+[A(1,i) A(2,i) 0];
        Wouter(:,i)=orthonormal*scale*k1*d+[A(1,i) A(2,i) 0];
        Winner(:,i)=-orthonormal*scale*k1*d+[A(1,i) A(2,i) 0];
        Bouter2(:,i)=orthonormal*scale*1*k2*d+[A(1,i) A(2,i) 0];
        Binner2(:,i)=-orthonormal*scale*1*k2*d+[A(1,i) A(2,i) 0];
 end

 
if j==1
    fill([Bouter1(1,1:end-1) Bouter1(1,1) Binner1(1,1:end-1) Binner1(1,1)],[Bouter1(2,1:end-1) Bouter1(2,1) Binner1(2,1:end-1) Binner1(2,1)],'k')
    hold on
elseif j==2
    fill([Wouter(1,1:end-1) Wouter(1,1) Winner(1,1:end-1) Winner(1,1)],[Wouter(2,1:end-1) Wouter(2,1) Winner(2,1:end-1) Winner(2,1)],'w')
    hold on
elseif j==3
    fill([Bouter2(1,1:end-1) Bouter2(1,1) Binner2(1,1:end-1) Binner2(1,1)],[Bouter2(2,1:end-1) Bouter2(2,1) Binner2(2,1:end-1) Binner2(2,1)],'k')
    hold on
end
