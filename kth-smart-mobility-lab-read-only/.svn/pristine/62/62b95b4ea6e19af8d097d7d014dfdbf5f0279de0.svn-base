function roadSignPlot(scale,road,WP,typ,SL)

x=road(1,:);
y=road(2,:);


if WP==1
    preWP=length(x);
else
    preWP=WP-1;
end

theta=atan2(y(WP)-y(preWP),x(WP)-x(preWP));

x1=x(WP);
y1=y(WP);
    
if typ==1                                                                       % typ 1 är stopskylt, 2 är väjningsplikt, 3 är hastighetsskylt,   4 är Herr Gårman
                                                                                % vägnummer    WP på vägen     x-koordinat    y-koordinat     1=stopp  hastighetsbegränsning (0 om sådan saknas)
    t = (1/16:1/8:1)'*2*pi;
    x = sin(t);
    y = cos(t);
    vec1=[x'; y'];
    T=[cos(theta) -sin(theta); sin(theta) cos(theta)];

    vec2=zeros(2,length(vec1(1,:)));
    for i=1:length(vec1(1,:))
        vec2(:,i)=T*vec1(:,i);
    end

    fill(scale*0.1*1.06*vec2(1,:)+x1,scale*0.1*1.06*vec2(2,:)+y1,'w')
    hold on
    fill(scale*0.1*0.94*vec2(1,:)+x1,scale*0.1*0.94*vec2(2,:)+y1,'r')
    hold on

    % 0.7 hör ihop med linewidth 70
    ht = text(x1,y1,'STOP');
    set(ht,'Rotation',rad2deg(theta)-90,'HorizontalAlignment','center')
    set(ht,'FontSize',scale*0.05*110,'color','white')

elseif typ==2
    t = (1/6:1/3:3/2)'*2*pi;
    x = sin(t);
    y = cos(t);

    vec1=[x'; y'];
    T=[cos(theta-pi/2) -sin(theta-pi/2); sin(theta-pi/2) cos(theta-pi/2)];

    vec2=zeros(2,length(vec1(1,:)));
    for i=1:length(vec1(1,:))
        vec2(:,i)=T*vec1(:,i);
    end

    fill(scale*0.1*1.19*vec2(1,:)+x1,scale*0.1*1.19*vec2(2,:)+y1,'r')
    hold on
    fill(scale*0.1*vec2(1,:)+x1,scale*0.1*vec2(2,:)+y1,'y')
    hold on
elseif typ==3
    t = (0:0.01:1)'*2*pi;
    x = sin(t);
    y = cos(t);

    vec1=[x'; y'];
    T=[cos(theta) -sin(theta); sin(theta) cos(theta)];

    vec2=zeros(2,length(vec1(1,:)));
    for i=1:length(vec1(1,:))
        vec2(:,i)=T*vec1(:,i);
    end

    fill(scale*0.08*1.19*vec2(1,:)+x1,scale*0.08*1.19*vec2(2,:)+y1,'r')
    hold on
    fill(scale*0.08*vec2(1,:)+x1,scale*0.08*vec2(2,:)+y1,'y')
    hold on
 

    % % 0.7 hör ihop med linewidth 70
    ht = text(x1,y1,num2str(SL));
    set(ht,'Rotation',rad2deg(theta)-90,'HorizontalAlignment','center')
    set(ht,'FontSize',scale*0.1*170,'color','black')
    hold on
elseif typ==4
    b=0.3;
    im=imread('herrgarman2.png');
    im=imrotate(im,rad2deg(theta)-90);
    imagesc([x1-0.085*scale+b*scale*cos(theta-pi/2) x1+0.085*scale+b*scale*cos(theta-pi/2)], [y1+0.085*scale+b*scale*sin(theta-pi/2) y1-0.085*scale+b*scale*sin(theta-pi/2)], im);
    
    hold on
end












