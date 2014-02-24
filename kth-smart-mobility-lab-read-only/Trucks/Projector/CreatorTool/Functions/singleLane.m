function [xCSF,yCSF]=singleLane(x,y,reverse)
% Bra fil som gör två filer


z=[0 0 1];
d=0.2;

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
 
    
    %Tar bort första och sista punkten eftersom den inte passar in
    xCD=[xDCS(2:50:end-1) xDCS(2:50:end-1)];                                %Centerline Doublelap
    yCD=[yDCS(2:50:end-1) yDCS(2:50:end-1)];


    t1=1:length(xCD);
    tt=linspace(1,length(xCD),2000);

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
        xCSF=[];
        yCSF=[];
    else
        
    xCSS=xCDS(indexSP(1):indexSP(end));                                           %Centerline Singlelap Spline
        yCSS=yCDS(indexSP(1):indexSP(end));


       
        lC=sqrt((xCSS(2)-xCSS(1))^2+(yCSS(2)-yCSS(1))^2);
        
        numC=0.075/lC;

xCSF=xCSS(1:ceil(numC):end);                                             %Centerlane Singlelap Final
yCSF=yCSS(1:ceil(numC):end);
    end

    if reverse
            xCSF=xCSF(end:-1:1);
            yCSF=yCSF(end:-1:1);
    end

    if lC>sqrt((xCSS(end)-xCSS(1))^2+(yCSS(end)-yCSS(1))^2)
        xCSS=xCSS(1:end-1);
        yCSS=yCSS(1:end-1);
    end
end


