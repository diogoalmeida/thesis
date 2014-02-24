function twolane_disp(A1,A2,scale,dir1,dir2,j)


A1=A1(1:2,:);
A2=A2(1:2,:);


if scale<0.7
    k1=0.94;
    k2=0.85;
else
    k1=0.95;
    k2=0.88;
end


[xmitt ymitt]=find_center(A1,dir1,A2,dir2);

d=0.15;

Bouter1=zeros(3,length(A1(1,:)));
Binner1=zeros(3,length(A2(1,:)));
Wouter=zeros(3,length(A1(1,:)));
Winner=zeros(3,length(A2(1,:)));
Bouter2=zeros(3,length(A1(1,:)));
Binner2=zeros(3,length(A2(1,:)));

for i=1:length(A1(1,:))-1
    point=[A1(1,i+1)-A1(1,i) A1(2,i+1)-A1(2,i) 0];
    orthogonal=cross(point,[0 0 1]);
    orthonormal=orthogonal/norm(orthogonal);
    
    Bouter1(:,i)=orthonormal*scale*1*d+[A1(1,i) A1(2,i) 0];
    Wouter(:,i)=orthonormal*scale*k1*d+[A1(1,i) A1(2,i) 0];
    Bouter2(:,i)=orthonormal*scale*1*k2*d+[A1(1,i) A1(2,i) 0];
end

for i=1:length(A2(1,:))-1
    point=[A2(1,i+1)-A2(1,i) A2(2,i+1)-A2(2,i) 0];
    orthogonal=cross(point,[0 0 1]);
    orthonormal=orthogonal/norm(orthogonal);
    if dir1==dir2
        Binner1(:,i)=-orthonormal*scale*1*d+[A2(1,i) A2(2,i) 0];
        Winner(:,i)=-orthonormal*scale*k1*d+[A2(1,i) A2(2,i) 0];
        Binner2(:,i)=-orthonormal*scale*1*k2*d+[A2(1,i) A2(2,i) 0];
    else
        Binner1(:,i)=orthonormal*scale*1*d+[A2(1,i) A2(2,i) 0];
        Winner(:,i)=orthonormal*scale*k1*d+[A2(1,i) A2(2,i) 0];
        Binner2(:,i)=orthonormal*scale*1*k2*d+[A2(1,i) A2(2,i) 0];
    end
end



c=6;
Binner1Sx=Binner1(1,1:c:end);
Binner1Sy=Binner1(2,1:c:end);
Winner1Sx=Winner(1,1:c:end);
Winner1Sy=Winner(2,1:c:end);
Binner2Sx=Binner2(1,1:c:end);
Binner2Sy=Binner2(2,1:c:end);


t1=1:length(Binner1Sx);
tt1=linspace(1,length(Binner1Sx),length(Binner1(1,:)));

Binner1Sx=spline(t1,Binner1Sx,tt1);
Binner1Sy=spline(t1,Binner1Sy,tt1);
Winner1Sx=spline(t1,Winner1Sx,tt1);
Winner1Sy=spline(t1,Winner1Sy,tt1);
Binner2Sx=spline(t1,Binner2Sx,tt1);
Binner2Sy=spline(t1,Binner2Sy,tt1);


Bouter1Sx=Bouter1(1,1:c:end);
Bouter1Sy=Bouter1(2,1:c:end);
Wouter1Sx=Wouter(1,1:c:end);
Wouter1Sy=Wouter(2,1:c:end);
Bouter2Sx=Bouter2(1,1:c:end);
Bouter2Sy=Bouter2(2,1:c:end);

t2=1:length(Bouter1Sx);
tt2=linspace(1,length(Bouter1Sx),length(Bouter1(1,:)));

Bouter1Sx=spline(t2,Bouter1Sx,tt2);
Bouter1Sy=spline(t2,Bouter1Sy,tt2);
Wouter1Sx=spline(t2,Wouter1Sx,tt2);
Wouter1Sy=spline(t2,Wouter1Sy,tt2);
Bouter2Sx=spline(t2,Bouter2Sx,tt2);
Bouter2Sy=spline(t2,Bouter2Sy,tt2);


Bouter1(1:2,2:end-2)=[Bouter1Sx(2:end-2); Bouter1Sy(2:end-2)];
Wouter(1:2,2:end-2)=[Wouter1Sx(2:end-2); Wouter1Sy(2:end-2)];
Bouter2(1:2,2:end-2)=[Bouter2Sx(2:end-2); Bouter2Sy(2:end-2)];
Binner1(1:2,2:end-2)=[Binner1Sx(2:end-2); Binner1Sy(2:end-2)];
Winner(1:2,2:end-2)=[Winner1Sx(2:end-2); Winner1Sy(2:end-2)];
Binner2(1:2,2:end-2)=[Binner2Sx(2:end-2); Binner2Sy(2:end-2)];


if j==1
    fill([Bouter1(1,1:end-1) Bouter1(1,1) Binner1(1,1:end-1) Binner1(1,1)],[Bouter1(2,1:end-1) Bouter1(2,1) Binner1(2,1:end-1) Binner1(2,1)],'k')
    hold on
elseif j==2
    fill([Wouter(1,1:end-1) Wouter(1,1) Winner(1,1:end-1) Winner(1,1)],[Wouter(2,1:end-1) Wouter(2,1) Winner(2,1:end-1) Winner(2,1)],'w')
    hold on
elseif j==3
    fill([Bouter2(1,1:end-1) Bouter2(1,1) Binner2(1,1:end-1) Binner2(1,1)],[Bouter2(2,1:end-1) Bouter2(2,1) Binner2(2,1:end-1) Binner2(2,1)],'k')
    hold on
elseif j==4
    
    p1=plot(xmitt,ymitt,'--w');
    set(p1,'LineWidth',3*scale)
    hold on
end
