function bridge_disp(A1,A2,scale,j,sparai,sparaj,dir1,dir2)

d=0.15;

if scale<0.7
   k1=0.94;
   k2=0.85;
else
   k1=0.95;
   k2=0.88;
end

SI1=[];
EI1=[];
SI2=[];
EI2=[];

[x0b,y0b,ioutb,joutb] = intersections(A1(1,:),A1(2,:),A2(1,:),A2(2,:),false);
ioutb=ioutb';
joutb=joutb';
ioutb=floor(ioutb)
joutb=floor(joutb)

i1diff=diff([-1 ioutb])==-1;
i3diff=diff([-1 ioutb])==0;
i5diff=diff([-1 ioutb])==1;
i7diff=diff([-1 ioutb])==2;
i9diff=diff([-1 ioutb])==-2;
i11diff=diff([-1 ioutb])==3;
i13diff=diff([-1 ioutb])==-3;



j2diff=diff([joutb -1])==-1;
j4diff=diff([joutb -1])==0;
j6diff=diff([joutb -1])==1;
j8diff=diff([joutb -1])==2;
j10diff=diff([joutb -1])==-2;
j12diff=diff([joutb -1])==3;
j14diff=diff([joutb -1])==-3;

vec=(i1diff+i3diff+i5diff+i7diff+i9diff+i11diff+i13diff).*(j2diff+j4diff+j6diff+j8diff+j10diff+j12diff+j14diff);
aux92=vec~=0;

if all(aux92)
    SI1=ioutb(1)-5;
    EI1=ioutb(end)+5;
    SI2=joutb(1)-5;
    EI2=joutb(end)+5;
    
else
    aux93=[0 aux92 0];
    SI=zeros(1,length(aux92));
    EI=zeros(1,length(aux92));
    for j=2:length(aux93)
        if aux93(j-1)==0 && aux93(j)~=0
            SI(j)=1;
        elseif aux93(j-1)~=0 && aux93(j)==0
            EI(j-1)=1;
        end
    end
    EI
    SI
    aux92
    SI1=ioutb(logical(SI))-5;
    EI1=ioutb(logical(EI))+5;
    SI2=joutb(logical(SI))-5;
    EI2=joutb(logical(EI))+5;
end

EI1
SI1
for g=1:length(SI1)

    if SI1(g)<EI1(g)

    else
        a=EI1(g);
        EI1(g)=SI1(g);
        SI1(g)=a;
    end
    Bouter1=zeros(3,length(A1(1,1:end-2)));
    Binner1=zeros(3,length(A1(1,1:end-2)));
    Wouter=zeros(3,length(A1(1,1:end-2)));
    Winner=zeros(3,length(A1(1,1:end-2)));
    Bouter2=zeros(3,length(A1(1,1:end-2)));
    Binner2=zeros(3,length(A1(1,1:end-2)));

     for i=1:length(A1(1,:))-2
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

    if SI2(g)<EI2(g)

    else
        a=EI2(g);
        EI2(g)=SI2(g);
        SI2(g)=a;
    end
    Bouter11=zeros(3,length(A2(1,:))-1);
    Binner11=zeros(3,length(A2(1,:))-1);
    Wouter1=zeros(3,length(A2(1,:))-1);
    Winner1=zeros(3,length(A2(1,:))-1);
    Bouter21=zeros(3,length(A2(1,:))-1);
    Binner21=zeros(3,length(A2(1,:))-1);

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





    k=[SI1(g) SI1(g) EI1(g) EI1(g)];
    l=[SI2(g) EI2(g) SI2(g) EI2(g)];
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

    vec1=vec1(:,[SI1(g)-1,SI1(g),SI1(g)+1,round((SI1(g)+EI1(g))/2),EI1(g)-1,EI1(g),EI1(g)+1]);
    vec2=vec2(:,[SI2(g)-1,SI2(g),SI2(g)+1,round((SI2(g)+EI2(g))/2),EI2(g)-1,EI2(g),EI2(g)+1]);

    t1=1:length(vec1(1,:));
    tt1=linspace(1,t1(end),30);

    vec11(1,:)=spline(t1,vec1(1,:),tt1);
    vec11(2,:)=spline(t1,vec1(2,:),tt1);

    t2=1:length(vec2(1,:));
    tt2=linspace(1,t2(end),30);

    vec22(1,:)=spline(t2,vec2(1,:),tt2);
    vec22(2,:)=spline(t2,vec2(2,:),tt2);




    if j==1
            fill([Bouter1(1,:) Bouter1(1,1) Binner1(1,:) Binner1(1,1)],[Bouter1(2,:) Bouter1(2,1) Binner1(2,:) Binner1(2,1)],'k')
            hold on 


        fill([Bouter11(1,:) Bouter11(1,1) Binner11(1,:) Binner11(1,1)],[Bouter11(2,:) Bouter11(2,1) Binner11(2,:) Binner11(2,1)],'k')
        hold on
    %     
    elseif j==2
        fill([Wouter(1,:) Wouter(1,1) Winner(1,:) Winner(1,1)],[Wouter(2,:) Wouter(2,1) Winner(2,:) Winner(2,1)],'w')
        hold on 


        fill([Wouter1(1,:) Wouter1(1,1) Winner1(1,:) Winner1(1,1)],[Wouter1(2,:) Wouter1(2,1) Winner1(2,:) Winner1(2,1)],'w')
        hold on


    elseif j==4
        fill([Bouter2(1,:) Bouter2(1,1) Binner2(1,:) Binner2(1,1)],[Bouter2(2,:) Bouter2(2,1) Binner2(2,:) Binner2(2,1)],'k')
        hold on 


        fill([Bouter21(1,:) Bouter21(1,1) Binner21(1,:) Binner21(1,1)],[Bouter21(2,:) Bouter21(2,1) Binner21(2,:) Binner21(2,1)],'k')
        hold on

    fill([vec11(1,3:end-3) vec22(1,end-3:-1:3)],[vec11(2,3:end-3) vec22(2,end-3:-1:3)],'k');


        p=plot(vec11(1,3:end-3),vec11(2,3:end-3),'w',vec22(1,3:end-3),vec22(2,3:end-3),'w');
        set(p,'linewidth',3*scale);
        hold on
    end
end
    
% 
% 
% 
% 









% 
% for j=2:length(bridgevec)
%     if bridgevec(j-1)==0 && bridgevec(j)~=0
%         SI1=[SI1 j];
%         SI2=SI1;
%     elseif bridgevec(j-1)~=0 && bridgevec(j)==0
%         EI1=[EI1 j-1];
%         EI2=EI1;
%     end
% end
% 
% 
% SI1
% EI1
% SI2
% EI2
% 
% 
% 


% 
% 
% 

% 
% bridgevec=[0];
% for jj=1:length(ioutb)-1
%     diff=ioutb(jj)-ioutb(jj+1);
%     if abs(diff)<2
%         if bridgevec(end)~=ioutb(jj)
%             bridgevec=[bridgevec ioutb(jj)];
%         else
%             bridgevec=bridgevec(1:end-1);
%         end
%         bridgevec=[bridgevec ioutb(jj+1)];
%     end
% end
% aux83=logical(zeros(1,length(ioutb)));
% for r=1:length(bridgevec)
%     aux59=ioutb==bridgevec(r);
%     aux83=aux83+aux59;
% end
% aux83=logical(aux83);
% 
% f1=ioutb(aux83);
% f2=joutb(aux83);
% 
% SI1=f1(1:2:end)
% EI1=f1(2:2:end)
% SI2=f2(1:2:end)
% EI2=f2(2:2:end)
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% % c=round(length(vec1(1,:))/25);
% % 
% % t1=1:length(vec1(1,1:c:end));
% % tt1=linspace(1,t1(end),length(vec1));
% % 
% % vec1(1,:)=spline(t1,vec1(1,1:c:end),tt1);
% % vec1(2,:)=spline(t1,vec1(2,1:c:end),tt1);
% % 
% % t2=1:length(vec2(1,1:c:end));
% % tt2=linspace(1,t2(end),length(vec2));
% % 
% % vec2(1,:)=spline(t2,vec2(1,1:c:end),tt2);
% % vec2(2,:)=spline(t2,vec2(2,1:c:end),tt2);
% % 
% % 
% 
% 
% 
%  
% %     fill([Bouter1(1,:) Binner1(1,:)],[Bouter1(2,:) Binner1(2,:)],'k')
% %     hold on
% %     fill([Wouter(1,:) Winner(1,:)],[Wouter(2,:) Winner(2,:)],'w')
% %     hold on   
%     
%     
% %             fill([Wouter(1,SI1:EI1) Winner(1,SI1:EI1)],[Wouter(2,SI1:EI1) Winner(2,SI1:EI1)],'w')
% %     hold on