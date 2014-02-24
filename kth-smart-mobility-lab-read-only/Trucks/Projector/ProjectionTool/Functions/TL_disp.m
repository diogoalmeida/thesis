function TL_disp(cell_dots,TLState,scale,borderPos,plotvec)
% CR current road, TLWP traffic light way-point

f1 = figure(1);
hold on
set(f1,'Color',[1 1 1]);
set(gcf,'Toolbar','none','Menubar','none')
set(gcf,'Units','normal')
set(gca,'Position',[0 0 1 1])
set(gcf,'Position',borderPos)



for i=1:length(cell_dots)
    if TLState(i)
        
        p2=plot(cell_dots{i}(1),cell_dots{i}(2),'*');
        set(p2,'LineWidth',12*scale,'color',[0 1 0])
        p2=plot(cell_dots{i}(1),cell_dots{i}(2),'o');
        set(p2,'LineWidth',12*scale,'color',[0 1 0])
        hold on
        
        p2=plot(cell_dots{i}(3),cell_dots{i}(4),'o');
        set(p2,'LineWidth',12*scale,'color',[0.4 0 0])
        p2=plot(cell_dots{i}(3),cell_dots{i}(4),'*');
        set(p2,'LineWidth',12*scale,'color',[0.4 0 0])
        
    else
        p2=plot(cell_dots{i}(1),cell_dots{i}(2),'o');
        set(p2,'LineWidth',12*scale,'color',[0 0.2 0])
        p2=plot(cell_dots{i}(1),cell_dots{i}(2),'*');
        set(p2,'LineWidth',12*scale,'color',[0 0.2 0])
        
        p2=plot(cell_dots{i}(3),cell_dots{i}(4),'*');
        set(p2,'LineWidth',12*scale,'color',[1 0 0])
        p2=plot(cell_dots{i}(3),cell_dots{i}(4),'o');
        set(p2,'LineWidth',12*scale,'color',[1 0 0])
    end

end
hold on
axis(plotvec)