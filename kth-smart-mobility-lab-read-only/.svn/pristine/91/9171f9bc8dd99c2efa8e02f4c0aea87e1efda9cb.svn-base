function disttraj=LUT_Maker(x,y)
    traj=[x',y'];
    disttraj=zeros(length(traj),length(traj));
    for i=1:size(traj,1)
        for j=1:size(traj,1)
            index=i;
            while 1
                next=rem(index,size(traj,1))+1;
                d_bw_wp=sqrt((traj(next,1)-traj(index,1))^2+...
                    (traj(next,2)-traj(index,2))^2);
                disttraj(i,j)=disttraj(i,j)+d_bw_wp;
                index=next;
                if index==j
                    break
                end
            end
        end
    end

