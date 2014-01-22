function cd_remote_callback(call_type)
hand = get_param(gcb, 'handle');
data = get_param(hand,'RTWdata')
return

if length(data) 
  if isfield(data,'Top')
    cr = find(data.Top == sprintf('\n'));
    data.Top(cr) = [];
    ansTop = reshape(data.Top,length(data.Top)/length(cr),length(cr))';
    ansTopEmpty = 0;
  end
  if isfield(data,'Middle')
    cr = find(data.Middle == sprintf('\n'));
    data.Middle(cr) = [];
    ansMiddle = reshape(data.Middle,length(data.Middle)/length(cr),length(cr))';
    ansMiddleEmpty = 0;
  end
  if isfield(data,'Bottom')
    cr = find(data.Bottom == sprintf('\n'));
    data.Bottom(cr) = [];
    ansBottom = reshape(data.Bottom,length(data.Bottom)/length(cr),length(cr))';
    ansBottomEmpty = 0;
  end
end

%display input dialog window
if length(findstr(codeSection, 'Function')) == 0
  prompt = {['Top of ' codeSection ], ['Bottom of ' codeSection ]};
  def = {ansTop, ansBottom};
  lineNo = [10;10];
else
  prompt={[codeSection ' Declaration Code'], [codeSection ' Execution Code'], ...
	[codeSection ' Exit Code']};
  def = {ansTop, ansMiddle, ansBottom};
  lineNo = [5;10;10];
end
title = [codeSection ' Custom Code'];
answer = inputdlg(prompt, title, lineNo, def, 'on');
if length(answer) == 0
  %if inputdlg was canceled, return without changing RTWdata
  return;
end

%reformat strings for RTWdata format of struct of strings
newdataempty = 1;
newdata.TLCFile = 'custcode';
newdata.Location = codeSection;
temp = answer{1};
if length(temp)
  temp = [temp' ;sprintf('\n')*ones(1,size(temp,1))];
  newdata.Top = temp(:)';
  newdataempty = 0;
end
temp = answer{2};
if length(findstr(codeSection, 'Function')) ~= 0
  if length(temp)
    temp = [temp' ;sprintf('\n')*ones(1,size(temp,1))];
    newdata.Middle = temp(:)';
    newdataempty = 0;
  end
  temp = answer{3};
  if length(temp)
    temp = [temp' ;sprintf('\n')*ones(1,size(temp,1))];
    newdata.Bottom = temp(:)';
    newdataempty = 0;
  end
else
  if length(temp)
    temp = [temp' ;sprintf('\n')*ones(1,size(temp,1))];
    newdata.Bottom = temp(:)';
    newdataempty = 0;
  end
end

%set the updated RTWdata parameter on the subsystem block
if newdataempty
  set_param(hand, 'RTWdata', []);
else
  set_param(hand, 'RTWdata', newdata);
end
