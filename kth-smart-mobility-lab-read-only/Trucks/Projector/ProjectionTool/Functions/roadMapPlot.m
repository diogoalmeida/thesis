function cell_dots=roadMapPlot(TLState,bridgeState,roadArray,roadInfo,SignInfo,plotvec,borderPos,TM,scale,TLInfo,waypoints,borderVec)
box off
axis off
hold on

f1 = figure(1);
hold off
p=plot(0,0,'*');
set(p,'color',[0 1 0]);
hold on
set(f1,'Color',[1 1 1]);
set(gcf,'Toolbar','none','Menubar','none')
set(gcf,'Units','normal')
set(gca,'Position',[0 0 1 1])
set(gcf,'Position',borderPos)


fill([plotvec(1:2) plotvec(2) plotvec(1)],[plotvec(3) plotvec(3) plotvec(4) plotvec(4)],'g')
axis(plotvec)
hold on

antalRoads=length(roadInfo(:,1));

sticking=0;
spara=[];
for i=1:antalRoads
    if roadInfo(i,3)==1
        sticking=sticking+1;
    end
    spara=[spara sticking];
end


for j=1:4
    i=1;
    while i<=antalRoads
%         if ~isempty(TM)
%             TMplot(roadArray,TM,j,scale,bridgeState,roadInfo)
%         end
        if abs(roadInfo(i,2))==1
            onelane_disp(roadArray{i},scale,j)
            i=i+1;
        elseif abs(roadInfo(i,2))==2
            twolane_disp(roadArray{i},roadArray{i+1},scale,roadInfo(i,4),roadInfo(i+1,4),j)
            i=i+2;
        elseif abs(roadInfo(i,2))==3
            threelane_disp(roadArray{i},roadArray{i+1},roadArray{i+2},scale,roadInfo,j,i)
            i=i+3;
        elseif abs(roadInfo(i,2))==4
            fourlane_disp(roadArray{i},roadArray{i+1},roadArray{i+2},roadArray{i+3},scale,roadInfo,j,i)
            i=i+4;
        end
        axis(plotvec)
    end
    
    
    if bridgeState==1
        bridgeRoads=roadArray(roadInfo(:,6)==2);
        bridgeInfo=roadInfo(roadInfo(:,6)==2,:);
        plottedBridge=[];
        for i=1:length(bridgeRoads)
            if any(bridgeInfo(i,1)==plottedBridge)
            else
                bridge_disp(bridgeRoads{i}(3:4,:),roadArray{bridgeInfo(i,7)}(3:4,:),scale,j,spara(i),spara(bridgeInfo(i,7)),roadInfo(i,4),roadInfo(bridgeInfo(i,7),4))
                plottedBridge=[plottedBridge bridgeInfo(i,1), bridgeInfo(i,7)];
                
            end
        end
    end
end

% %TL
if ~isempty(TLInfo)
    cell_dots=TL_calc(roadArray,roadInfo,scale,TLInfo,bridgeState);
    TL_disp(cell_dots,TLState,scale,borderPos,plotvec)
else
    cell_dots={};
end


% vägnummer    WP på vägen     x-koordinat    y-koordinat     1=stopp  hastighetsbegränsning (0 om sådan saknas)
if ~isempty(SignInfo)
    for i=1:length(SignInfo(1,:))
        roadSignPlot(scale,roadArray{SignInfo(1,i)},SignInfo(2,i),SignInfo(5,i),SignInfo(6,i));
    end
end


if waypoints
    
    for ii=1:length(roadArray)
        if bridgeState==1 && roadInfo(ii,6)==2
            h=3;
            g=4;
        else
            h=1;
            g=2;
        end
        
        CR=[roadArray{ii}(h:g,:) roadArray{ii}(h:g,1)];
        plot(-1000,-1000,'.k')
        for kk=1:length(CR)-2
            
            theta=atan2(CR(2,kk+1)-CR(2,kk),CR(1,kk+1)-CR(1,kk));
            if kk==length(CR)-2
                ht = text(CR(1,kk),CR(2,kk),'>');
                set(ht,'Rotation',rad2deg(theta),'LineWidth',scale*3,'color','green')
                hold on
            else
                ht = text(CR(1,kk),CR(2,kk),'>');
                set(ht,'Rotation',rad2deg(theta),'LineWidth',scale*3,'color','yellow')
                hold on
            end
        end
    end
end










