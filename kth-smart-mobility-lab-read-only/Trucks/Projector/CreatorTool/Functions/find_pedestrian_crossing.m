function [SignInfo1 pCross1]=find_pedestrian_crossing(ii,jj,savedRoads,pedestrianRoads,SignInfo,pCross,typ)
%funktion som fyller en funktion



if typ==1
    
    for j=1:length(savedRoads)
        [x0,y0,iout,jout] = intersections(pedestrianRoads{ii}(1,:),pedestrianRoads{ii}(2,:),savedRoads{j}(1,:),savedRoads{j}(2,:),false);
        iout=iout';
        jout=jout';
        iout=floor(iout);
        jout=floor(jout);

        pCross=[pCross [-ii*ones(1,length(iout)); iout; j*ones(1,length(iout)); jout]];

        SignInfo=[SignInfo [j*ones(1,length(iout)); jout; savedRoads{j}(1,jout); savedRoads{j}(2,jout); 4*ones(1,length(iout)); zeros(1,length(iout))]];
    end
else
    if ~isempty(pedestrianRoads)
        for i=1:length(pedestrianRoads)
            for j=1:length(savedRoads)
                [x0,y0,iout,jout] = intersections(pedestrianRoads{i}(1,:),pedestrianRoads{i}(2,:),savedRoads{j}(1,:),savedRoads{j}(2,:),false);
                iout=iout';
                jout=jout';
                iout=floor(iout);
                jout=floor(jout);

                pCross=[pCross [-ii*ones(1,length(iout)); iout; j*ones(1,length(iout)); jout]];

                SignInfo=[SignInfo [j*ones(1,length(iout)); jout; savedRoads{j}(1,jout); savedRoads{j}(2,jout); 4*ones(1,length(iout)); zeros(1,length(iout))]];
            end
        end
    end
end



SignInfo1=SignInfo;
pCross1=pCross;

