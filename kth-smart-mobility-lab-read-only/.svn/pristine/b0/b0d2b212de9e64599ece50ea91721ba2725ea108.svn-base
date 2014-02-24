function varargout = calibration_tool(varargin)
% CALIBRATION_TOOL MATLAB code for calibration_tool.fig
%      CALIBRATION_TOOL, by itself, creates a new CALIBRATION_TOOL or raises the existing
%      singleton*.
%
%      H = CALIBRATION_TOOL returns the handle to a new CALIBRATION_TOOL or the handle to
%      the existing singleton*.
%
%      CALIBRATION_TOOL('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in CALIBRATION_TOOL.M with the given input arguments.
%
%      CALIBRATION_TOOL('Property','Value',...) creates a new CALIBRATION_TOOL or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before calibration_tool_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to calibration_tool_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help calibration_tool

% Last Modified by GUIDE v2.5 25-Jun-2013 14:39:55

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @calibration_tool_OpeningFcn, ...
                   'gui_OutputFcn',  @calibration_tool_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before calibration_tool is made visible.
function calibration_tool_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to calibration_tool (see VARARGIN)

% Choose default command line output for calibration_tool
handles.output = hObject;
% [checked,Xmin,Xmax,Ymin,Ymax]=
close(figure(1))
global checked Xmin Xmax Ymin Ymax f1;
checked=0;
Xmin=[];
Xmax=[];
Ymin=[];
Ymax=[];

x1=-10:1:10;
y1=zeros(1,length(x1));
x2=y1;
y2=x1;

f1 = figure(1);
set(f1,'Color',[1 1 1]);
set(gcf,'Toolbar','none','Menubar','none')
set(gcf,'Units','normal')
set(gca,'Position',[0 0 1 1])
set(f1,'name','Calibration Image','numbertitle','off')

plot(x1,y1,x2,y2,'b')
text(0.5,9,'Y-axis','FontSize',30);
text(7.5,1,'X-axis','FontSize',30);
box off
axis off
% Update handles structure
guidata(hObject, handles);

% UIWAIT makes calibration_tool wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = calibration_tool_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global checked Xmin Xmax Ymin Ymax f1;
if checked==1 && isempty(Xmin)==0 && isempty(Xmax)==0 && isempty(Ymin)==0 && isempty(Ymax)==0
    monitorPos=get(f1, 'position');
    cal=[Xmin Xmax Ymin Ymax; monitorPos];
    dlmwrite('Calibration.txt',cal,'delimiter','\t','precision','%f')
    h = msgbox('The data was succesfully saved. You may now close the calibration tool.','SUCCESS!!!');
else
    h = msgbox('The data was not filled in correctly. Try again.','ERROR!!!');
end

% --- Executes on button press in checkbox1.
function checkbox1_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global checked;
checked = get(hObject,'Value');
% Hint: get(hObject,'Value') returns toggle state of checkbox1



function edit2_Callback(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global Xmin;
Xmin=str2double(get(hObject,'String'));
% Hints: get(hObject,'String') returns contents of edit2 as text
%        str2double(get(hObject,'String')) returns contents of edit2 as a double


% --- Executes during object creation, after setting all properties.
function edit2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit4_Callback(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global Xmax;
Xmax=str2double(get(hObject,'String'));
% Hints: get(hObject,'String') returns contents of edit4 as text
%        str2double(get(hObject,'String')) returns contents of edit4 as a double


% --- Executes during object creation, after setting all properties.
function edit4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit5_Callback(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global Ymin;
Ymin=str2double(get(hObject,'String'));
% Hints: get(hObject,'String') returns contents of edit5 as text
%        str2double(get(hObject,'String')) returns contents of edit5 as a double


% --- Executes during object creation, after setting all properties.
function edit5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit6_Callback(hObject, eventdata, handles)
% hObject    handle to edit6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global Ymax;
Ymax=str2double(get(hObject,'String'));
% Hints: get(hObject,'String') returns contents of edit6 as text
%        str2double(get(hObject,'String')) returns contents of edit6 as a double


% --- Executes during object creation, after setting all properties.
function edit6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
