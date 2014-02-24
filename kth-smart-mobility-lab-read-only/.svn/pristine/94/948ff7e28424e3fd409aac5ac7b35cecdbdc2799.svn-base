function [x1, y1, x2, y2, error, StartIndex1, EndIndex1, StartIndex2, EndIndex2]=twoway_bridge2(A1,A2,xb,yb);

% Bra fil som gör två filer med bro

x1=A1(1,:);
y1=A1(2,:);
x2=A2(1,:);
y2=A2(2,:);

Start1=(sqrt((xb(1)-x1).^2)+(yb(1)-y1).^2);
End1=(sqrt((xb(2)-x1).^2)+(yb(2)-y1).^2);
 

Start2=(sqrt((xb(1)-x2).^2)+(yb(1)-y2).^2);
End2=(sqrt((xb(2)-x2).^2)+(yb(2)-y2).^2);

[a1 StartIndex1]=min(Start1);
[b1 EndIndex1]=min(End1);
[a2 StartIndex2]=min(Start2);
[b2 EndIndex2]=min(End2);
    
k=7;

if abs(StartIndex1-EndIndex1)<k || abs(StartIndex2-EndIndex2)<k
    error=2;
elseif StartIndex1<5 || StartIndex2<5 || EndIndex1<5 || EndIndex2<5 ||...
        StartIndex1>length(x1)-5 || StartIndex2>length(x2)-5 ||...
        EndIndex1>length(x1)-5 || EndIndex2>length(x2)-5
    error=1;
else
    error=0;

    if (StartIndex1>EndIndex1 && StartIndex2<EndIndex2)
        xC1=x1(StartIndex1:-1:EndIndex1);
        yC1=y1(StartIndex1:-1:EndIndex1);
        xC2=x2(StartIndex2:EndIndex2);
        yC2=y2(StartIndex2:EndIndex2);
    elseif(StartIndex2>EndIndex2 && StartIndex1<EndIndex1)
        xC1=x1(StartIndex1:EndIndex1);
        yC1=y1(StartIndex1:EndIndex1);
        xC2=x2(StartIndex2:-1:EndIndex2);
        yC2=y2(StartIndex2:-1:EndIndex2);
    elseif (StartIndex2>EndIndex2 && StartIndex1>EndIndex1)
        xC1=x1(StartIndex1:-1:EndIndex1);
        yC1=y1(StartIndex1:-1:EndIndex1);
        xC2=x2(StartIndex2:-1:EndIndex2);
        yC2=y2(StartIndex2:-1:EndIndex2);
    else
        xC1=x1(StartIndex1:EndIndex1);
        yC1=y1(StartIndex1:EndIndex1);
        xC2=x2(StartIndex2:EndIndex2);
        yC2=y2(StartIndex2:EndIndex2);
    end


    t1=1:length(xC1);
    t2=1:length(xC2);

    tt1=linspace(1,length(xC1),50);
    tt2=linspace(1,length(xC2),50);


    xx1 = interp1(t1,xC1,tt1);
    yy1 = interp1(t1,yC1,tt1);
    xx2 = interp1(t2,xC2,tt2);
    yy2 = interp1(t2,yC2,tt2);

    xmitt=zeros(1,length(xx1));
    ymitt=zeros(1,length(xx1));


    for i=1:length(xx1)
        xmitt(i)=(xx1(i)+xx2(i))/2;
        ymitt(i)=(yy1(i)+yy2(i))/2;
    end

    if length(xC1)<10 || length(xC2)<10 || sqrt((xC1(1)-xC2(1))^2+(yC1(1)-yC2(1))^2)>0.4 || sqrt((xC1(end)-xC2(end))^2+(yC1(end)-yC2(end))^2)>0.8
        c=20;
    elseif length(xC1)<20 || length(xC2)<20 %|| sqrt((xC1(1)-xC2(1))^2+(yC1(1)-yC2(1))^2)>0.4 || sqrt((xC1(end)-xC2(end))^2+(yC1(end)-yC2(end))^2)>0.5
        c=15;
    elseif length(xC1)<30 || length(xC2)<30
        c=10;
    else
        c=8;
    end
    
    
    x1SL=linspace(xC1(1),xmitt(c),3);                                          % x1 Start Linspace osv...
    y1SL=linspace(yC1(1),ymitt(c),3);
    x1EL=linspace(xmitt(end-c-1),xC1(end),3);
    y1EL=linspace(ymitt(end-c-1),yC1(end),3);

    x2SL=linspace(xC2(1),xmitt(c),3);
    y2SL=linspace(yC2(1),ymitt(c),3);
    x2EL=linspace(xmitt(end-c-1),xC2(end),3);
    y2EL=linspace(ymitt(end-c-1),yC2(end),3);

    x1Ass=[x1SL xmitt(c+1:end-c-1) x1EL];                                       % Assemblering
    y1Ass=[y1SL ymitt(c+1:end-c-1) y1EL];
    x2Ass=[x2SL xmitt(c+1:end-c-1) x2EL];                                       
    y2Ass=[y2SL ymitt(c+1:end-c-1) y2EL];

    vec1=linspace(1,length(x1Ass),length(xC1));
    vec2=linspace(1,length(x2Ass),length(xC2));

    
    x1Ass=interp1(1:length(x1Ass),x1Ass,vec1);
    y1Ass=interp1(1:length(y1Ass),y1Ass,vec1);
    x2Ass=interp1(1:length(x2Ass),x2Ass,vec2);
    y2Ass=interp1(1:length(y2Ass),y2Ass,vec2);



    if (StartIndex1>EndIndex1 && StartIndex2<EndIndex2)
        x1(StartIndex1:-1:EndIndex1)=x1Ass;
        y1(StartIndex1:-1:EndIndex1)=y1Ass;  
        x2(StartIndex2:EndIndex2)=x2Ass;
        y2(StartIndex2:EndIndex2)=y2Ass;
    elseif(StartIndex2>EndIndex2 && StartIndex1<EndIndex1)
        x1(StartIndex1:EndIndex1)=x1Ass;
        y1(StartIndex1:EndIndex1)=y1Ass;  
        x2(StartIndex2:-1:EndIndex2)=x2Ass;
        y2(StartIndex2:-1:EndIndex2)=y2Ass;
    elseif (StartIndex2>EndIndex2 && StartIndex1>EndIndex1)
        x1(StartIndex1:-1:EndIndex1)=x1Ass;
        y1(StartIndex1:-1:EndIndex1)=y1Ass; 
        x2(StartIndex2:-1:EndIndex2)=x2Ass;
        y2(StartIndex2:-1:EndIndex2)=y2Ass;
    else
        x1(StartIndex1:EndIndex1)=x1Ass;
        y1(StartIndex1:EndIndex1)=y1Ass; 
        x2(StartIndex2:EndIndex2)=x2Ass;
        y2(StartIndex2:EndIndex2)=y2Ass;
    end
    
%     if abs(StartIndex1-EndIndex1)>2*k || abs(StartIndex2-EndIndex2)>2*k
% 
%         xS1=x1(1:2:end);
%         yS1=y1(1:2:end);
%         xS2=x2(1:2:end);
%         yS2=y2(1:2:end);
% 
% %         xS1=x1([1:2:StartIndex1-1 StartIndex1+1:2:EndIndex1-1 EndIndex1:2:end]);
% %         yS1=y1([1:2:StartIndex1-1 StartIndex1+1:2:EndIndex1-1 EndIndex1:2:end]);
% %         xS2=x2([1:2:StartIndex2-1 StartIndex2+1:2:EndIndex2-1 EndIndex2:2:end]);
% %         yS2=y2([1:2:StartIndex2-1 StartIndex2+1:2:EndIndex2-1 EndIndex2:2:end]);
%         
%         
%         t1=1:length(xS1);
%         tt1=linspace(1,length(xS1),length(x1));
% 
%         xS1=spline(t1,xS1,tt1);                                                     % Spline
%         yS1=spline(t1,yS1,tt1);
% 
%         t2=1:length(xS2);
%         tt2=linspace(1,length(xS2),length(x2));
% 
%         xS2=spline(t2,xS2,tt2);                                                     % Spline
%         yS2=spline(t2,yS2,tt2);  
% 
% 
% 
%         x1(10:end-10)=xS1(10:end-10);
%         y1(10:end-10)=yS1(10:end-10);
%         x2(10:end-10)=xS2(10:end-10);
%         y2(10:end-10)=yS2(10:end-10);
%     end
    
%     for i=1:length(x1)
%         plot(x1(i),y1(i),'*g')
%     end
    
end   

