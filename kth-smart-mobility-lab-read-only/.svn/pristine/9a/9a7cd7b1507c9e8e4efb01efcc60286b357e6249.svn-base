function cell_dots=TL_calc(roadArray,roadInfo,scale,TLInfo,bridge)
% CR current road, TLWP traffic light way-point

cell_dots=cell(length(TLInfo(1,:)));
TLWP=TLInfo(2,:);

for i=1:length(TLWP(1,:))
    if bridge==1 && roadInfo(TLInfo(1,i),6)==2
        h=3;
        g=4;
    else
        h=1;
        g=2;
    end
    
    if TLWP(i)==roadInfo(TLInfo(1,i),5)
        nextIndex=1;
    else
        nextIndex=TLWP(i)+1;
    end
    
    CR=roadArray{TLInfo(1,i)}(h:g,:);
    
    TLvec=(CR(:,nextIndex)-CR(:,TLWP(i)))/norm(CR(:,nextIndex)-CR(:,TLWP(i)));
    
    white=[[CR(1,TLWP(i))-TLvec(1)*0.12*scale CR(1,TLWP(i))+TLvec(1)*0.12*scale];[CR(2,TLWP(i))-TLvec(2)*0.12*scale CR(2,TLWP(i))+TLvec(2)*0.12*scale]];
    black=[[CR(1,TLWP(i))-TLvec(1)*0.11*scale CR(1,TLWP(i))+TLvec(1)*0.11*scale];[CR(2,TLWP(i))-TLvec(2)*0.11*scale CR(2,TLWP(i))+TLvec(2)*0.11*scale]];
    red=[CR(1,TLWP(i))+TLvec(1)*0.05*scale;CR(2,TLWP(i))+TLvec(2)*0.05*scale];
    green=[CR(1,TLWP(i))-TLvec(1)*0.05*scale;CR(2,TLWP(i))-TLvec(2)*0.05*scale];
    hold on
    
    p2=plot(white(1,:),white(2,:),'w');
    set(p2,'LineWidth',30*scale)
    hold on
    
    p2=plot(black(1,:),black(2,:),'k');
    set(p2,'LineWidth',27*scale)
    hold on
    
    cell_dots{i}=[green; red];
    
end
