function [xOSF1,yOSF1,xISF1,yISF1,xOSF2,yOSF2,xISF2,yISF2]=quadLane(x,y,reverse,fourway,spacing)
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

dInner1=zeros(1,length(xDCS(1:end)));                                        %distance Inner lane1
dInner2=zeros(1,length(xDCS(1:end)));                                        %distance Inner lane2
dOuter1=zeros(1,length(xDCS(1:end)));                                        %distance Outer lane1
dOuter2=zeros(1,length(xDCS(1:end)));                                        %distance Outer lane2
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
    xOSF1=[];
    yOSF1=[];
    xISF1=[];
    yISF1=[];
    xOSF2=[];
    yOSF2=[];
    xISF2=[];
    yISF2=[];
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
            dInner1(j)=d;
            dOuter1(j)=d;
            dInner2(j)=3*d;
            dOuter2(j)=3*d;

        elseif theta>=0.01 && theta<0.025
            dInner1(j)=d+0.05;
            dOuter1(j)=d-0.05;
            dInner2(j)=3*d+0.05;
            dOuter2(j)=3*d-0.05;
        elseif theta<=-0.01 && theta>-0.025
            dInner1(j)=d-0.05;
            dOuter1(j)=d+0.05;
            dInner2(j)=3*d-0.05;
            dOuter2(j)=3*d+0.05;
        elseif theta>=0.025
            dInner1(j)=d+0.1;
            dOuter1(j)=d-0.1;
            dInner2(j)=3*d+0.1;
            dOuter2(j)=3*d-0.1;
        elseif theta<=-0.025 
            dInner1(j)=d-0.1;
            dOuter1(j)=d+0.1;
            dInner2(j)=3*d-0.1;
            dOuter2(j)=3*d+0.1;

        else
            dInner1(j)=d;
            dOuter1(j)=d;
            dInner2(j)=3*d;
            dOuter2(j)=3*d;
        end
    end

    outer1=zeros(3,length(xDCS));
    inner1=zeros(3,length(xDCS));
    outer2=zeros(3,length(xDCS));
    inner2=zeros(3,length(xDCS));

    for i=1:length(xDCS)-1
        point=[xDCS(i+1)-xDCS(i) yDCS(i+1)-yDCS(i) zDC(i+1)-zDC(i)];
        orthogonal=cross(point,z);
        orthonormal=orthogonal/norm(orthogonal);

        outer1(:,i)=orthonormal*dOuter1(i)+[xDCS(i) yDCS(i) zDC(i)];
        inner1(:,i)=-orthonormal*dInner1(i)+[xDCS(i) yDCS(i) zDC(i)];
        outer2(:,i)=orthonormal*dOuter2(i)+[xDCS(i) yDCS(i) zDC(i)];
        inner2(:,i)=-orthonormal*dInner2(i)+[xDCS(i) yDCS(i) zDC(i)];
    end

    %Tar bort första och sista punkten eftersom den inte passar in
    xOD1=[outer1(1,2:50:end-1) outer1(1,2:50:end-1)];                              %Outerline Doublelap
    yOD1=[outer1(2,2:50:end-1) outer1(2,2:50:end-1)];
    xID1=[inner1(1,2:50:end-1) inner1(1,2:50:end-1)];                              %Innerline Doublelap
    yID1=[inner1(2,2:50:end-1) inner1(2,2:50:end-1)];
    xOD2=[outer2(1,2:50:end-1) outer2(1,2:50:end-1)];                              %Outerline Doublelap
    yOD2=[outer2(2,2:50:end-1) outer2(2,2:50:end-1)];
    xID2=[inner2(1,2:50:end-1) inner2(1,2:50:end-1)];                              %Innerline Doublelap
    yID2=[inner2(2,2:50:end-1) inner2(2,2:50:end-1)];
    xCD=[xDCS(2:50:end-1) xDCS(2:50:end-1)];                                %Centerline Doublelap
    yCD=[yDCS(2:50:end-1) yDCS(2:50:end-1)];


    t1=1:length(xOD1);
    tt=linspace(1,length(xOD1),2000);

    xODS1=spline(t1,xOD1,tt);                                                     %Outerline Doublelap Spline
    yODS1=spline(t1,yOD1,tt);
    xIDS1=spline(t1,xID1,tt);                                                     %Innerline Doublelap Spline
    yIDS1=spline(t1,yID1,tt);
    xODS2=spline(t1,xOD2,tt);                                                     %Outerline Doublelap Spline
    yODS2=spline(t1,yOD2,tt);
    xIDS2=spline(t1,xID2,tt);                                                     %Innerline Doublelap Spline
    yIDS2=spline(t1,yID2,tt);
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
        xOSF1=[];
        yOSF1=[];
        xISF1=[];
        yISF1=[];
        xOSF2=[];
        yOSF2=[];
        xISF2=[];
        yISF2=[];
    else
        xOSS1=xODS1(indexSP(1):indexSP(end));                                           %Outerline Singlelap Spline
        yOSS1=yODS1(indexSP(1):indexSP(end));
        xISS1=xIDS1(indexSP(1):indexSP(end));                                           %Innerline Singlelap Spline
        yISS1=yIDS1(indexSP(1):indexSP(end));
        xOSS2=xODS2(indexSP(1):indexSP(end));                                           %Outerline Singlelap Spline
        yOSS2=yODS2(indexSP(1):indexSP(end));
        xISS2=xIDS2(indexSP(1):indexSP(end));                                           %Innerline Singlelap Spline
        yISS2=yIDS2(indexSP(1):indexSP(end));
        xCSS=xCDS(indexSP(1):indexSP(end));                                           %Innerline Singlelap Spline
        yCSS=yCDS(indexSP(1):indexSP(end));

        if fourway==1
            xISS2=xISS2(end:-1:1);                                                       %Innerline Singlelap Spline Reverse
            yISS2=yISS2(end:-1:1);
        elseif fourway==2
            xISS1=xISS1(end:-1:1);                                                       %Innerline Singlelap Spline Reverse
            yISS1=yISS1(end:-1:1);
            xISS2=xISS2(end:-1:1);                                                       %Innerline Singlelap Spline Reverse
            yISS2=yISS2(end:-1:1);
        elseif fourway==3
            xOSS1=xOSS1(end:-1:1);                                                       %Outerline Singlelap Spline Reverse
            yOSS1=yOSS1(end:-1:1);
            xISS1=xISS1(end:-1:1);                                                       %Innerline Singlelap Spline Reverse
            yISS1=yISS1(end:-1:1);
            xISS2=xISS2(end:-1:1);                                                       %Innerline Singlelap Spline Reverse
            yISS2=yISS2(end:-1:1);
        end

        lOuter=sqrt((xOSS2(2)-xOSS2(1))^2+(yOSS2(2)-yOSS2(1))^2);
        lInner=sqrt((xISS2(2)-xISS2(1))^2+(yISS2(2)-yISS2(1))^2);

        numWP=0.075/min([lInner lOuter]);

        xOSF1=xOSS1(1:ceil(numWP):end);                                           %Outerlane Singlelap Final
        yOSF1=yOSS1(1:ceil(numWP):end);
        xISF1=xISS1(1:ceil(numWP):end);                                          %Innerlane Singlelap Final
        yISF1=yISS1(1:ceil(numWP):end);
        xOSF2=xOSS2(1:ceil(numWP):end);                                           %Outerlane Singlelap Final
        yOSF2=yOSS2(1:ceil(numWP):end);
        xISF2=xISS2(1:ceil(numWP):end);                                          %Innerlane Singlelap Final
        yISF2=yISS2(1:ceil(numWP):end);
    end

    if reverse
            xOSF1=xOSF1(end:-1:1);
            yOSF1=yOSF1(end:-1:1);
            xISF1=xISF1(end:-1:1);
            yISF1=yISF1(end:-1:1);
            xOSF2=xOSF2(end:-1:1);
            yOSF2=yOSF2(end:-1:1);
            xISF2=xISF2(end:-1:1);
            yISF2=yISF2(end:-1:1);
    end
    if min([lInner lOuter])>min([sqrt((xOSS2(end)-xOSS2(1))^2+(yOSS2(end)-yOSS2(1))^2) sqrt((xISS2(end)-xISS2(1))^2+(yISS2(end)-yISS2(1))^2)])
        xOSF1=xOSF1(1:end-1);
        yOSF1=yOSF1(1:end-1);
        xISF1=xISF1(1:end-1);
        yISF1=yISF1(1:end-1);
        xOSF2=xOSF2(1:end-1);
        yOSF2=yOSF2(1:end-1);
        xISF2=xISF2(1:end-1);
        yISF2=yISF2(1:end-1);
    end
end


