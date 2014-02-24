function [savedRoads,roadInfo,TLInfo,TransitionMatrix,TLArray,TSInfo,savedTL]=load_project(path)

savedRoadsFile=importdata(strcat(path,'\','savedRoads.txt'));
roadInfo=importdata(strcat(path,'\','RoadInfo.txt'));
TLInfo=importdata(strcat(path,'\','TLInfo.txt'));
TransitionMatrix=importdata(strcat(path,'\','ManualTransitionMatrix.txt'));
TLMatrixFile=importdata(strcat(path,'\','TLMatrix.txt'));
TSInfo=importdata(strcat(path,'\','SignInfo.txt'));

antalRoads=length(roadInfo(:,1));
savedRoads={};
j=0;
for i=1:antalRoads
    if roadInfo(i,6)==2
        x1=savedRoadsFile(i*2-1+j,1:roadInfo(i,5));
        y1=savedRoadsFile(i*2+j,1:roadInfo(i,5));
        j=j+2;
        x2=savedRoadsFile(i*2-1+j,1:roadInfo(i,5));
        y2=savedRoadsFile(i*2+j,1:roadInfo(i,5));
        savedRoads{i}=[x1; y1; x2; y2];
    elseif roadInfo(i,6)==1
        x=savedRoadsFile(i*2-1+j,1:roadInfo(i,5));
        y=savedRoadsFile(i*2+j,1:roadInfo(i,5));
        savedRoads{i}=[x; y];
    end
end


TLArray={[] [] [] [] [] [] [] [] [] []};

savedTL=[];
for i=1:length(TLArray)
    if ~isempty(TLMatrixFile(i,:))
        finder=TLMatrixFile(i,:)~=0;
        TLArray{i}=TLMatrixFile(i,finder);
        savedTL=[savedTL TLMatrixFile(i,finder)];
    end
end

