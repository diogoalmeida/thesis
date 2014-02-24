function TransitionMatrix=find_transition(savedRoads,roadInfo,TM)
%funktion som fyller en funktion

antalRoads=length(roadInfo(:,1));
sticking=0;
spara=[];
M=[];
for i=1:antalRoads
    if roadInfo(i,3)==1
        sticking=sticking+1;
    end
    spara=[spara sticking];
end


for i=1:antalRoads
    for j=1:antalRoads
        if i~=j
            if roadInfo(i,2)>=2 && roadInfo(j,2)>=2 && spara(i)==spara(j) && roadInfo(i,4)==roadInfo(j,4)
                if abs(roadInfo(i,3)-roadInfo(j,3))==1
                    w=7;
                    
                    fromvec=1:length(savedRoads{i}(1,:));
                    tovec=[fromvec(1+w:end) fromvec(1:w)];
                    
                    M=[M [i*ones(1,length(fromvec)); fromvec; j*ones(1,length(fromvec)); tovec]];
                    
                    if roadInfo(i,6)==2 && roadInfo(j,6)==2
                        fromvec=1:length(savedRoads{i}(1,:));
                        tovec=[fromvec(1+w:end) fromvec(1:w)];
                        
                        M=[M [-i*ones(1,length(fromvec)); fromvec; j*ones(1,length(fromvec)); tovec]];
                        
                    elseif (roadInfo(i,6)~=2 && roadInfo(j,6)==2)
                        dBL=sqrt((savedRoads{i}(1,1)-savedRoads{j}(1,1))^2+(savedRoads{i}(2,1)-savedRoads{j}(2,1))^2);
                        fromvec=[];
                        tovec=[];
                        for g=w+1:length(savedRoads{i}(1,:))
                            
                            if sqrt((savedRoads{i}(end-1,g)-savedRoads{j}(end-1,g))^2+(savedRoads{i}(end,g)-savedRoads{j}(end,g))^2)<=dBL*1.05 
                                fromvec=[fromvec g-w];
                                tovec=[tovec g];
                            end
                        end
                        
                        fromvec=[fromvec length(savedRoads{i}(1,:))-w:length(savedRoads{i}(1,:))];
                        tovec=[tovec 1:w+1];
                        
                        M=[M [-i*ones(1,length(fromvec)); fromvec; j*ones(1,length(tovec)); tovec]];
                        
                    elseif (roadInfo(i,6)==2 && roadInfo(j,6)~=2)
                        dBL=sqrt((savedRoads{i}(1,1)-savedRoads{j}(1,1))^2+(savedRoads{i}(2,1)-savedRoads{j}(2,1))^2);
                        fromvec=[];
                        tovec=[];
                        for g=w+1:length(savedRoads{i}(1,:))
                            
                            if sqrt((savedRoads{i}(end-1,g-w)-savedRoads{j}(end-1,(g-w)))^2+(savedRoads{i}(end,g-w)-savedRoads{j}(end,g-w))^2)<=dBL*1.05
                                fromvec=[fromvec g-w];
                                tovec=[tovec g];
                            end
                        end
                        
                        fromvec=[fromvec length(savedRoads{i}(1,:))-w:length(savedRoads{i}(1,:))];
                        tovec=[tovec 1:w+1];
                        
                        M=[M [-i*ones(1,length(fromvec)); fromvec; j*ones(1,length(tovec)); tovec]];
                    end
                end
                
            elseif roadInfo(i,2)>=2 && roadInfo(j,2)>=2 && spara(i)==spara(j) && roadInfo(i,4)~=roadInfo(j,4)
                % Do nothing
                %
                
            else 
                
                [x0,y0,iout,jout] = intersections(savedRoads{i}(1,:),savedRoads{i}(2,:),savedRoads{j}(1,:),savedRoads{j}(2,:),false);
                iout=iout';
                jout=jout';
                iout=floor(iout)
                jout=ceil(jout)

                if ~isempty(iout)
                    if any(iout-1==0)
                        aux=iout-1==0;
                        iout=iout-1;
                        iout(aux)=length(savedRoads{i}(1,:));
                    else
                        iout=iout-1;
                    end
                    if any(jout+1>length(savedRoads{j}(1,:)))
                        aux2=jout+1>savedRoads{j}(1,:);
                        jout=jout+1;
                        jout(aux2)=1;
                    else
                        jout=jout+1;
                    end
                    M=[M [i*ones(1,length(iout)); iout; j*ones(1,length(iout)); jout]];
                end
                
                
                if roadInfo(i,6)==2 
                    [x0b,y0b,ioutb,joutb] = intersections(savedRoads{i}(end-1,:),savedRoads{i}(end,:),savedRoads{j}(end-1,:),savedRoads{j}(end,:),false);
                    ioutb=ioutb';
                    joutb=joutb';
                    ioutb=floor(ioutb)
                    joutb=ceil(joutb)
                    
                    iroadWP=[];
                    dir1=1;
                    dir2=2;
                    if ~isempty(ioutb)
                        
                        if ioutb(1)<ioutb(end)
                            dir1=1;
                        else
                            dir1=2;
                        end
                        
                        if joutb(1)<joutb(end)
                            dir2=1;
                        else
                            dir2=2;
                        end
                        
                        if dir1==dir2 && dir1==1
                            i1diff=diff([-1 ioutb])==-1;
                            i2diff=diff([ioutb -1])==-1;
                            i3diff=diff([-1 ioutb])==0;
                            i4diff=diff([ioutb -1])==0;
                            i5diff=diff([-1 ioutb])==1;
                            i6diff=diff([ioutb -1])==1;
                            i7diff=diff([-1 ioutb])==2;
                            i8diff=diff([ioutb -1])==2;
                            i9diff=diff([-1 ioutb])==-2;
                            i10diff=diff([ioutb -1])==-2;
                            
                            j1diff=diff([-1 joutb])==-1;
                            j2diff=diff([joutb -1])==-1;
                            j3diff=diff([-1 joutb])==0;
                            j4diff=diff([joutb -1])==0;
                            j5diff=diff([-1 joutb])==1;
                            j6diff=diff([joutb -1])==1;
                            j7diff=diff([-1 joutb])==2;
                            j8diff=diff([joutb -1])==2;
                            j9diff=diff([-1 joutb])==-2;
                            j10diff=diff([joutb -1])==-2;
                            
                            vec=(i1diff+i2diff+i3diff+i4diff+i5diff+i6diff+i7diff+i8diff+i9diff+i10diff).*...
                                (j1diff+j2diff+j3diff+j4diff+j5diff+j6diff+j7diff+j8diff+j9diff+j10diff);
                            aux92=vec~=0;
                            
                            ibridgeWP=ioutb(aux92);
                            iroadWP=ioutb(logical(ones(1,length(ioutb))-aux92));
                            
                            jbridgeWP=joutb(aux92);
%                             jroadWP=joutb(logical(ones(1,length(joutb))-aux92));
                            
%                             if length(jbridgeWP)>1
%                                 M=[M [-i*ones(1,length(ibridgeWP)); ibridgeWP; j*ones(1,length(jbridgeWP)); jbridgeWP]];
%                             end
%                             if length(iroadWP)<=1
%                                 iroadWP=[];
%                                 jroadWP=[];
%                             end
%                             
                            M=[M [-i*ones(1,length(ioutb)); ioutb; j*ones(1,length(joutb)); joutb]];
%                             iroadWP=[];
%                             jroadWP=[];
                            
                            
                            
                        elseif dir1==dir2 && dir1==2
                            i1diff=diff([-1 ioutb])==-1;
                            i2diff=diff([ioutb -1])==-1;
                            i3diff=diff([-1 ioutb])==0;
                            i4diff=diff([ioutb -1])==0;
                            i5diff=diff([-1 ioutb])==1;
                            i6diff=diff([ioutb -1])==1;
                            i7diff=diff([-1 ioutb])==2;
                            i8diff=diff([ioutb -1])==2;
                            i9diff=diff([-1 ioutb])==-2;
                            i10diff=diff([ioutb -1])==-2;
                            
                            j1diff=diff([-1 joutb])==-1;
                            j2diff=diff([joutb -1])==-1;
                            j3diff=diff([-1 joutb])==0;
                            j4diff=diff([joutb -1])==0;
                            j5diff=diff([-1 joutb])==1;
                            j6diff=diff([joutb -1])==1;
                            j7diff=diff([-1 joutb])==2;
                            j8diff=diff([joutb -1])==2;
                            j9diff=diff([-1 joutb])==-2;
                            j10diff=diff([joutb -1])==-2;
                            
                            vec=(i1diff+i2diff+i3diff+i4diff+i5diff+i6diff+i7diff+i8diff+i9diff+i10diff).*...
                                (j1diff+j2diff+j3diff+j4diff+j5diff+j6diff+j7diff+j8diff+j9diff+j10diff);
                            aux92=vec~=0;
                            
                            ibridgeWP=ioutb(aux92);
%                             iroadWP=ioutb(logical(ones(1,length(ioutb))-aux92));
                            
                            jbridgeWP=joutb(aux92);
%                             jroadWP=joutb(logical(ones(1,length(joutb))-aux92));
                            if length(jbridgeWP)>1
                                M=[M [-i*ones(1,length(ibridgeWP)); ibridgeWP; j*ones(1,length(jbridgeWP)); jbridgeWP]];
                            end
%                             if length(iroadWP)<=1
%                                 iroadWP=[];
%                                 jroadWP=[];
%                             end
                            
%                         elseif dir1==1 && dir2==2
%                             i1diff=diff([-1 ioutb])==-1;
%                             i2diff=diff([ioutb -1])==-1;
%                             i3diff=diff([-1 ioutb])==0;
%                             i4diff=diff([ioutb -1])==0;
%                             i5diff=diff([-1 ioutb])==1;
%                             i6diff=diff([ioutb -1])==1;
%                             i7diff=diff([-1 ioutb])==2;
%                             i8diff=diff([ioutb -1])==2;
%                             i9diff=diff([-1 ioutb])==-2;
%                             i10diff=diff([ioutb -1])==-2;
%                             
%                             j1diff=diff([-1 joutb])==-1;
%                             j2diff=diff([joutb -1])==-1;
%                             j3diff=diff([-1 joutb])==0;
%                             j4diff=diff([joutb -1])==0;
%                             j5diff=diff([-1 joutb])==1;
%                             j6diff=diff([joutb -1])==1;
%                             j7diff=diff([-1 joutb])==2;
%                             j8diff=diff([joutb -1])==2;
%                             j9diff=diff([-1 joutb])==-2;
%                             j10diff=diff([joutb -1])==-2;
%                             
%                             vec=(i1diff+i2diff+i3diff+i4diff+i5diff+i6diff+i7diff+i8diff+i9diff+i10diff).*...
%                                 (j1diff+j2diff+j3diff+j4diff+j5diff+j6diff+j7diff+j8diff+j9diff+j10diff);
%                             aux92=vec~=0;
%                             iroadWP=ioutb(logical(ones(1,length(ioutb))-aux92));
%                             jroadWP=joutb(logical(ones(1,length(joutb))-aux92));
%                             
%                             if length(iroadWP)<=1
%                                 iroadWP=[];
%                                 jroadWP=[];
%                             end
%                             
%                         elseif dir1==2 && dir2==1
%                             i1diff=diff([-1 ioutb])==-1;
%                             i2diff=diff([ioutb -1])==-1;
%                             i3diff=diff([-1 ioutb])==0;
%                             i4diff=diff([ioutb -1])==0;
%                             i5diff=diff([-1 ioutb])==1;
%                             i6diff=diff([ioutb -1])==1;
%                             i7diff=diff([-1 ioutb])==2;
%                             i8diff=diff([ioutb -1])==2;
%                             i9diff=diff([-1 ioutb])==-2;
%                             i10diff=diff([ioutb -1])==-2;
%                             
%                             j1diff=diff([-1 joutb])==-1;
%                             j2diff=diff([joutb -1])==-1;
%                             j3diff=diff([-1 joutb])==0;
%                             j4diff=diff([joutb -1])==0;
%                             j5diff=diff([-1 joutb])==1;
%                             j6diff=diff([joutb -1])==1;
%                             j7diff=diff([-1 joutb])==2;
%                             j8diff=diff([joutb -1])==2;
%                             j9diff=diff([-1 joutb])==-2;
%                             j10diff=diff([joutb -1])==-2;
%                             
%                             vec=(i1diff+i2diff+i3diff+i4diff+i5diff+i6diff+i7diff+i8diff+i9diff+i10diff).*...
%                                 (j1diff+j2diff+j3diff+j4diff+j5diff+j6diff+j7diff+j8diff+j9diff+j10diff);
%                             aux92=vec~=0;
%                             iroadWP=ioutb(logical(ones(1,length(ioutb))-aux92));
%                             jroadWP=joutb(logical(ones(1,length(joutb))-aux92));
%                             
%                             if length(iroadWP)<=1
%                                 iroadWP=[];
%                                 jroadWP=[];
%                             end
                            
                        end
                    end
                end
            end
%             if ~isempty(iroadWP)
%                 if any(iroadWP-1==0)
%                     aux=iroadWP-1==0;
%                     iroadWP=iroadWP-1;
%                     iroadWP(aux)=length(savedRoads{i}(1,:));
%                 else
%                     iroadWP=iroadWP-1;
%                 end
%                 if any(jroadWP+1>length(savedRoads{j}(1,:)))
%                     aux2=jroadWP+1>savedRoads{j}(1,:);
%                     jroadWP=jroadWP+1;
%                     jroadWP(aux2)=1;
%                 else
%                     jroadWP=jroadWP+1;
%                 end
%                 M=[M [-i*ones(1,length(iroadWP)); iroadWP; j*ones(1,length(jroadWP)); jroadWP]];
%             end
        end
    end
    
    
    if ~isempty(TM)
        huj=TM(1,:)==i;
        hij=TM(3,:)==j;
        if any(huj+hij==2)
            hhoojj=huj+hij==2;
            m=TM(:,hhoojj);
        else
            m=[];
        end
        M=[m [i*ones(1,length(iout)); iout; j*ones(1,length(iout)); jout]];
    end
end

TransitionMatrix=M;

