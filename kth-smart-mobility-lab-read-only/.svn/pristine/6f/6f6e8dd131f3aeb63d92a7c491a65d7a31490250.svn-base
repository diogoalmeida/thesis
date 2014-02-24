function Matrix=cell2matrix(Array)
% Forms a matrix where the cells are converted to rows in a matrix
% Cells have to be vectors
%Array has to be on the form Array={[],[]...}

Arraysize=size(Array);
emptyCells = cellfun(@isempty,Array);
if min(emptyCells)~=1
    col=max(cellfun('length',Array));
    row=Arraysize(2);
    Matrix=zeros(row,col);
    for i=1:length(Matrix(:,1))
        h=length(Array{i});
        if h~=0
            Matrix(i,1:h)=Array{i};
        end
    end
else
    Matrix=[];
end