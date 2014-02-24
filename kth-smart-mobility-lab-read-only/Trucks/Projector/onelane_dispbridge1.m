function onelane_dispbridge1(A1,A2,scale,SI1,EI1,SI2,EI2,j)

d=0.15;

if scale<0.7
   k1=0.94;
   k2=0.85;
else
   k1=0.95;
   k2=0.88;
end
    

[x0b,y0b,ioutb,joutb] = intersections(A1(end-1,:),A1(end,:),A2(end-1,:),A2(end,:),false);
ioutb=ioutb';
joutb=joutb';
ioutb=floor(ioutb);
joutb=ceil(joutb);

bridgevec=[0];
for jj=1:length(ioutb)-1
    diff=ioutb(jj)-ioutb(jj+1);
    if abs(diff)<2
        if bridgevec(end)~=ioutb(jj)
            bridgevec=[bridgevec ioutb(jj)];
        else
            bridgevec=bridgevec(1:end-1);
        end
        bridgevec=[bridgevec ioutb(jj+1)];
    end
end
aux83=logical(zeros(1,length(ioutb)));
for r=1:length(bridgevec)
    aux59=ioutb==bridgevec(r);
    aux83=aux83+aux59;
end
aux83=logical(aux83);

f1=ioutb(aux83);
f2=joutb(aux83);

SI1=f1(1:2:end)
EI1=f1(2:2:end)
SI2=f2(1:2:end)
EI2=f2(2:2:end)




















if SI1<EI1
    
else
    a=EI1;
    EI1=SI1;
    SI1=a;
end
Bouter1=zeros(3,length(A1(1,1:end-1)));
Binner1=zeros(3,length(A1(1,1:end-1)));
Wouter=zeros(3,length(A1(1,1:end-1)));
Winner=zeros(3,length(A1(1,1:end-1)));
Bouter2=zeros(3,length(A1(1,1:end-1)));
Binner2=zeros(3,length(A1(1,1:end-1)));

 for i=1:length(A1(1,:))-1
        point=[A1(1,i+1)-A1(1,i) A1(2,i+1)-A1(2,i) 0];
        orthogonal=cross(point,[0 0 1]);
        orthonormal=orthogonal/norm(orthogonal);

        Bouter1(:,i)=orthonormal*scale*1*d+[A1(1,i) A1(2,i) 0];
        Binner1(:,i)=-orthonormal*scale*1*d+[A1(1,i) A1(2,i) 0];
        Wouter(:,i)=orthonormal*scale*k1*d+[A1(1,i) A1(2,i) 0];
        Winner(:,i)=-orthonormal*scale*k1*d+[A1(1,i) A1(2,i) 0];
        Bouter2(:,i)=orthonormal*scale*1*k2*d+[A1(1,i) A1(2,i) 0];
        Binner2(:,i)=-orthonormal*scale*1*k2*d+[A1(1,i) A1(2,i) 0];
 end


 

if SI2<EI2
    
else
    a=EI2;
    EI2=SI2;
    SI2=a;
end
Bouter11=zeros(3,length(A2(1,:)));
Binner11=zeros(3,length(A2(1,:)));
Wouter1=zeros(3,length(A2(1,:)));
Winner1=zeros(3,length(A2(1,:)));
Bouter21=zeros(3,length(A2(1,:)));
Binner21=zeros(3,length(A2(1,:)));

 for i=1:length(A2(1,:))-1
        point=[A2(1,i+1)-A2(1,i) A2(2,i+1)-A2(2,i) 0];
        orthogonal=cross(point,[0 0 1]);
        orthonormal=orthogonal/norm(orthogonal);

        Bouter11(:,i)=orthonormal*scale*1*d+[A2(1,i) A2(2,i) 0];
        Binner11(:,i)=-orthonormal*scale*1*d+[A2(1,i) A2(2,i) 0];
        Wouter1(:,i)=orthonormal*scale*k1*d+[A2(1,i) A2(2,i) 0];
        Winner1(:,i)=-orthonormal*scale*k1*d+[A2(1,i) A2(2,i) 0];
        Bouter21(:,i)=orthonormal*scale*1*k2*d+[A2(1,i) A2(2,i) 0];
        Binner21(:,i)=-orthonormal*scale*1*k2*d+[A2(1,i) A2(2,i) 0];
 end

 



k=[SI1 SI1 EI1 EI1];
l=[SI2 EI2 SI2 EI2];
[a,minIndex]=min(sqrt((Wouter(1,k)-Wouter1(1,l)).^2+(Wouter(2,k)-Wouter1(2,l)).^2))

I1=k(minIndex);
I2=l(minIndex);

matrix1=[Wouter(:,I1) Wouter(:,I1) Winner(:,I1) Winner(:,I1)];
matrix2=[Wouter1(:,I2) Winner1(:,I2) Wouter1(:,I2) Winner1(:,I2)];


[a,maxIndex]=max(sqrt((matrix1(1,:)-matrix2(1,:)).^2+(matrix1(2,:)-matrix2(2,:)).^2))
%1 =>Wouter & Wouter1, 2=>Wouter & Winner1, 3=> Winner &Wouter1, 4=>Winner&Winner1

if maxIndex==1
    vec1=Wouter;
    vec2=Wouter1;
elseif maxIndex==2
    vec1=Wouter;
    vec2=Winner1;
elseif maxIndex==3
    vec1=Winner;
    vec2=Wouter1;
elseif maxIndex==4
    vec1=Winner;
    vec2=Winner1;
end

vec1=vec1(:,[SI1-1,SI1,SI1+1,round((SI1+EI1)/2),EI1-1,EI1,EI1+1]);
vec2=vec2(:,[SI2-1,SI2,SI2+1,round((SI2+EI2)/2),EI2-1,EI2,EI2+1]);

t1=1:length(vec1(1,:));
tt1=linspace(1,t1(end),20);

vec11(1,:)=spline(t1,vec1(1,:),tt1);
vec11(2,:)=spline(t1,vec1(2,:),tt1);

t2=1:length(vec2(1,:));
tt2=linspace(1,t2(end),15);

vec22(1,:)=spline(t2,vec2(1,:),tt2);
vec22(2,:)=spline(t2,vec2(2,:),tt2);




if j==1
    fill([Bouter1(1,:) Binner1(1,:)],[Bouter1(2,:) Binner1(2,:)],'k')
    hold on 


    fill([Bouter11(1,:) Binner11(1,:)],[Bouter11(2,:) Binner11(2,:)],'k')
    hold on
    
elseif j==2
    fill([Wouter(1,:) Winner(1,:)],[Wouter(2,:) Winner(2,:)],'w')
    hold on 


    fill([Wouter1(1,:) Winner1(1,:)],[Wouter1(2,:) Winner1(2,:)],'w')
    hold on


elseif j==4
    fill([Bouter2(1,:) Binner2(1,:)],[Bouter2(2,:) Binner2(2,:)],'k')
    hold on 


    fill([Bouter21(1,:) Binner21(1,:)],[Bouter21(2,:) Binner21(2,:)],'k')
    hold on

    p=plot(vec11(1,3:end-3),vec11(2,3:end-3),'w',vec22(1,3:end-3),vec22(2,3:end-3),'w');
    set(p,'linewidth',3*scale);
    hold on
end





% c=round(length(vec1(1,:))/25);
% 
% t1=1:length(vec1(1,1:c:end));
% tt1=linspace(1,t1(end),length(vec1));
% 
% vec1(1,:)=spline(t1,vec1(1,1:c:end),tt1);
% vec1(2,:)=spline(t1,vec1(2,1:c:end),tt1);
% 
% t2=1:length(vec2(1,1:c:end));
% tt2=linspace(1,t2(end),length(vec2));
% 
% vec2(1,:)=spline(t2,vec2(1,1:c:end),tt2);
% vec2(2,:)=spline(t2,vec2(2,1:c:end),tt2);
% 
% 



 
%     fill([Bouter1(1,:) Binner1(1,:)],[Bouter1(2,:) Binner1(2,:)],'k')
%     hold on
%     fill([Wouter(1,:) Winner(1,:)],[Wouter(2,:) Winner(2,:)],'w')
%     hold on   
    
    
%             fill([Wouter(1,SI1:EI1) Winner(1,SI1:EI1)],[Wouter(2,SI1:EI1) Winner(2,SI1:EI1)],'w')
%     hold on