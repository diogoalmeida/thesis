function TMplot(roadArray,TM,j,scale,bridge,roadInfo)


for ii=1:length(TM(1,:))
    if bridge==1 && roadInfo(abs(TM(1,ii)),6)==2 && roadInfo(TM(3,ii),6)==2
        
        A=[roadArray{abs(TM(1,ii))}(3:4,TM(2,ii)) roadArray{TM(3,ii)}(3:4,TM(4,ii))];

    elseif bridge==1 && roadInfo(abs(TM(1,ii)),6)==2 && roadInfo(TM(3,ii),6)~=2
        
        A=[roadArray{abs(TM(1,ii))}(3:4,TM(2,ii)) roadArray{TM(3,ii)}(1:2,TM(4,ii))];
   
    elseif bridge==1 && roadInfo(abs(TM(1,ii)),6)~=2 && roadInfo(TM(3,ii),6)==2
        
        A=[roadArray{abs(TM(1,ii))}(1:2,TM(2,ii)) roadArray{TM(3,ii)}(3:4,TM(4,ii))];
        
    else
        
        A=[roadArray{abs(TM(1,ii))}(1:2,TM(2,ii)) roadArray{TM(3,ii)}(1:2,TM(4,ii))];
        
    end
    
    d=0.15;
    a=0.35;

    if scale<0.7
        k1=0.94;
        k2=0.85;
    else
        k1=0.95;
        k2=0.88;
    end
    
    vec=[A(1,2)-A(1,1);A(2,2)-A(2,1)];
    normvec=vec/norm(vec);
    
    T1=[cos(-pi/2) -sin(-pi/2); sin(-pi/2) cos(-pi/2)];
    T2=[cos(+pi/2) -sin(+pi/2); sin(+pi/2) cos(+pi/2)];
    
    
    Bouter1=[A(:,1)+scale*T1*normvec*d+normvec*a*scale A(:,2)+scale*T1*normvec*d-normvec*a*scale];
    Binner1=[A(:,1)+scale*T2*normvec*d+normvec*a*scale A(:,2)+scale*T2*normvec*d-normvec*a*scale];
    Wouter=[A(:,1)+scale*T1*normvec*d*k1+normvec*a*scale A(:,2)+scale*T1*normvec*d*k1-normvec*a*scale];
    Winner=[A(:,1)+scale*T2*normvec*d*k1+normvec*a*scale A(:,2)+scale*T2*normvec*d*k1-normvec*a*scale];
    Bouter2=[A(:,1)+scale*T1*normvec*d*k2+normvec*a*scale A(:,2)+scale*T1*normvec*d*k2-normvec*a*scale];
    Binner2=[A(:,1)+scale*T2*normvec*d*k2+normvec*a*scale A(:,2)+scale*T2*normvec*d*k2-normvec*a*scale];
    
    
    if j==1
        fill([Bouter1(1,:)  Binner1(1,2:-1:1)],[Bouter1(2,:) Binner1(2,2:-1:1)],'k')
        hold on
    elseif j==2
        fill([Wouter(1,:) Winner(1,2:-1:1)],[Wouter(2,:) Winner(2,2:-1:1)],'w')
        hold on
    elseif j==3
        fill([Bouter2(1,:)  Binner2(1,2:-1:1)],[Bouter2(2,:) Binner2(2,2:-1:1)],'k')
        hold on
    end
    
end