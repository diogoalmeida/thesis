function [xOSF,yOSF,xISF,yISF]=dualLane(x,y,reverse,twoway,spacing)
% Bra fil som gör två filer


z=[0 0 1];
d=spacing/2;

xDL=[x x x(1)];                                                             %Double Lap
yDL=[y y y(1)];

t1=1:length(xDL);
tt=linspace(1,length(xDL),2000);

index=550:1550;

xCDS=spline(t1,xDL,tt);                                                     %Centerline Doublelap Spline
yCDS=spline(t1,yDL,tt); 
xCSS=xCDS(index);                                                           %Centerline Singlelap Spline
yCSS=yCDS(index);

% Tar bort första och sista för att de inte passar in
xDCS=[xCSS(2:1:end-1) xCSS(1)];                                             %Doublelap Centerline Spline
yDCS=[yCSS(2:1:end-1) yCSS(1)];
zDC=zeros(1,length(xDCS));                                                  %Doublelap Centerline

xDCSE=[xDCS(end-1:end) xDCS xDCS(1)];                                       %Doublelap Centerline Spline Expanded
yDCSE=[yDCS(end-1:end) yDCS yDCS(1)];

dInner=zeros(1,length(xDCS(1:end)));                                        %distance Inner lane
dOuter=zeros(1,length(xDCS(1:end)));                                        %distance Outer lane
angleLUT=[];

index1=find(xCDS<=x(1)+0.01 & xCDS>=x(1)-0.01);
index2=find(yCDS<=y(1)+0.01 & yCDS>=y(1)-0.01);
indexSP=[];                                                                 %Start Point
for ii=1:length(index1)
    for jj=1:length(index2)
        if index1(ii)==index2(jj)
            indexSP=[indexSP index2(jj)];
        end
    end
end
if numel(indexSP)==0
    msgbox('Total Failure, please try again.','Meltdown','custom',imread('jonas.jpg'))
    xOSF=[];
    yOSF=[];
    xISF=[];
    yISF=[];
else
    for j=1:length(xDCS(1:end))

        vec1=[xDCSE(j+1)-xDCSE(j) yDCSE(j+1)-yDCSE(j) 0];                       %vector between point 1 and 2
        vec2=[xDCSE(j+2*1)-xDCSE(j+1) yDCSE(j+2)-yDCSE(j+1) 0];                 %vector between point 2 and 3

        theta1=atan2(vec1(2),vec1(1));                                          %angle between vec1 and x-axis
        theta2=atan2(vec2(2),vec2(1));                                          %angle between vec2 and x-axis
        theta=theta1-theta2;                                                    %angle between vec1 and vec2. negative=>left turn, positive=>right turn

        if theta>6
            theta=theta-2*pi;
        elseif theta<-6
            theta=2*pi+theta;
        end

            angleLUT=[angleLUT theta];

        if abs(theta)<0.01
            dInner(j)=d;
            dOuter(j)=d;

        elseif theta>=0.01 && theta<0.025
            dInner(j)=d+0.05;
            dOuter(j)=d-0.05;
        elseif theta<=-0.01 && theta>-0.025
            dInner(j)=d-0.05;
            dOuter(j)=d+0.05;
        elseif theta>=0.025
            dInner(j)=d+0.1;
            dOuter(j)=d-0.1;
        elseif theta<=-0.025 
            dInner(j)=d-0.1;
            dOuter(j)=d+0.1;

        else
            dInner(j)=d;
            dOuter(j)=d;
        end
    end

    outer=zeros(3,length(xDCS));
    inner=zeros(3,length(xDCS));
    for i=1:length(xDCS)-1
        point=[xDCS(i+1)-xDCS(i) yDCS(i+1)-yDCS(i) zDC(i+1)-zDC(i)];
        orthogonal=cross(point,z);
        orthonormal=orthogonal/norm(orthogonal);

        outer(:,i)=orthonormal*dOuter(i)+[xDCS(i) yDCS(i) zDC(i)];
        inner(:,i)=-orthonormal*dInner(i)+[xDCS(i) yDCS(i) zDC(i)];
    end

    %Tar bort första och sista punkten eftersom den inte passar in
    xOD=[outer(1,2:50:end-1) outer(1,2:50:end-1)];                              %Outerline Doublelap
    yOD=[outer(2,2:50:end-1) outer(2,2:50:end-1)];
    xID=[inner(1,2:50:end-1) inner(1,2:50:end-1)];                              %Innerline Doublelap
    yID=[inner(2,2:50:end-1) inner(2,2:50:end-1)];
    xCD=[xDCS(2:50:end-1) xDCS(2:50:end-1)];                                %Centerline Doublelap
    yCD=[yDCS(2:50:end-1) yDCS(2:50:end-1)];


    t1=1:length(xOD);
    tt=linspace(1,length(xOD),2000);

    xODS=spline(t1,xOD,tt);                                                     %Outerline Doublelap Spline
    yODS=spline(t1,yOD,tt);
    xIDS=spline(t1,xID,tt);                                                     %Innerline Doublelap Spline
    yIDS=spline(t1,yID,tt);
    xCDS=spline(t1,xCD,tt);                                                     %Centerline Doublelap Spline
    yCDS=spline(t1,yCD,tt);

    index1=find(xCDS<=x(1)+0.005 & xCDS>=x(1)-0.005);
    index2=find(yCDS<=y(1)+0.005 & yCDS>=y(1)-0.005);
    indexSP=[];                                                                 %Start Point
    for ii=1:length(index1)
        for jj=1:length(index2)
            if index1(ii)==index2(jj)
                indexSP=[indexSP index2(jj)];
            end
        end
    end
    if numel(indexSP)==0
        msgbox('Total Failure, please try again.','Meltdown','custom',imread('jonas.jpg'))
        xOSF=[];
        yOSF=[];
        xISF=[];
        yISF=[];
    else
        xOSS=xODS(indexSP(1):indexSP(end));                                           %Outerline Singlelap Spline
        yOSS=yODS(indexSP(1):indexSP(end));
        xISS=xIDS(indexSP(1):indexSP(end));                                           %Innerline Singlelap Spline
        yISS=yIDS(indexSP(1):indexSP(end));
        xCSS=xCDS(indexSP(1):indexSP(end));                                           %Innerline Singlelap Spline
        yCSS=yCDS(indexSP(1):indexSP(end));

        if twoway
            xISS=xISS(end:-1:1);                                                       %Innerline Singlelap Spline Reverse
            yISS=yISS(end:-1:1);
        end

        lOuter=sqrt((xOSS(2)-xOSS(1))^2+(yOSS(2)-yOSS(1))^2);
        lInner=sqrt((xISS(2)-xISS(1))^2+(yISS(2)-yISS(1))^2);

        numWP=0.075/min([lInner lOuter]);

        xOSF=xOSS(1:ceil(numWP):end);                                           %Outerlane Singlelap Final
        yOSF=yOSS(1:ceil(numWP):end);
        xISF=xISS(1:ceil(numWP):end);                                          %Innerlane Singlelap Final
        yISF=yISS(1:ceil(numWP):end);
    end
    if reverse
            xOSF=xOSF(end:-1:1);
            yOSF=yOSF(end:-1:1);
            xISF=xISF(end:-1:1);
            yISF=yISF(end:-1:1);
    end

    if min([lInner lOuter])>min([sqrt((xOSS(end)-xOSS(1))^2+(yOSS(end)-yOSS(1))^2) sqrt((xISS(end)-xISS(1))^2+(yISS(end)-yISS(1))^2)])
        xOSF=xOSF(1:end-1);
        yOSF=yOSF(1:end-1);
        xISF=xISF(1:end-1);
        yISF=yISF(1:end-1);
    end
    
    
    
    
end




