function varargout = Road_Network_Projection_Tool(varargin)
% ROAD_NETWORK_PROJECTION_TOOL MATLAB code for Road_Network_Projection_Tool.fig
%      ROAD_NETWORK_PROJECTION_TOOL, by itself, creates a new ROAD_NETWORK_PROJECTION_TOOL or raises the existing
%      singleton*.
%
%      H = ROAD_NETWORK_PROJECTION_TOOL returns the handle to a new ROAD_NETWORK_PROJECTION_TOOL or the handle to
%      the existing singleton*.
%
%      ROAD_NETWORK_PROJECTION_TOOL('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in ROAD_NETWORK_PROJECTION_TOOL.M with the given input arguments.
%
%      ROAD_NETWORK_PROJECTION_TOOL('Property','Value',...) creates a new ROAD_NETWORK_PROJECTION_TOOL or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Road_Network_Projection_Tool_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Road_Network_Projection_Tool_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Road_Network_Projection_Tool

% Last Modified by GUIDE v2.5 26-Jul-2013 13:00:07

% Begin initialization code - DO NOT EDIT
clc
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Road_Network_Projection_Tool_OpeningFcn, ...
                   'gui_OutputFcn',  @Road_Network_Projection_Tool_OutputFcn, ...
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




% --- Executes just before Road_Network_Projection_Tool is made visible.
function Road_Network_Projection_Tool_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Road_Network_Projection_Tool (see VARARGIN)

% Choose default command line output for Road_Network_Projection_Tool
handles.output = hObject;

addpath([pwd '\Functions']);
addpath([pwd '\..\Images']);

% Update handles structure
guidata(hObject, handles);
global ip port on off NTL TLInfo roadArray scale bridgeState ...
    plotvec roadInfo SignInfo TransitionMatrix borderPos TLState ...
    connected waypoints running cell_dots
format long
if exist([pwd,'\..\Projects\'],'dir')
    path=[pwd,'\..\Projects\'];
else
    path=pwd;
end
path = uigetdir(path,'Select Project Folder');
if path==0
    path=pwd;
end

scale=get(handles.Scale_slider,'Value');

set(handles.scale_edit,'String',num2str(scale,'%.15f'));
running=false;
connected=false;
waypoints=false;
bridgeState=0;
TLState=[];
cell_dots={};
ip=get(handles.ipedit_TL,'String');
port=get(handles.porteditTL,'String');
off=imread('green_off.png');
on=imread('green_on.png');
set(handles.TL_LED,'Enable','inactive');
set(handles.TL_LED,'cdata',off);
set(handles.Network_LED,'Enable','inactive');
set(handles.Network_LED,'cdata',off);

if exist([path,'\','roadInfo.txt'],'file')
    roadInfo=importdata([path,'\','roadInfo.txt']);
else
    roadInfo=[];
end


if exist([path,'\','TLInfo.txt'],'file')
    TLInfo=importdata([path,'\','TLInfo.txt']);
    if ~isempty(TLInfo)
        NTL=length(TLInfo(1,:));
    end
else
    NTL=0;
    TLInfo=[];
end

if exist('../CalibrationTool/Calibration.txt','file')
    a=load('../CalibrationTool/Calibration.txt');
    plotvec=a(1,:);
    borderPos=a(2,:);
elseif exist('Calibration.txt','file')
    a=load('Calibration.txt');
    plotvec=a(1,:);
else
    msgbox('Calibration file not found.','Missing File','custom',imread('oops.png'))
end

plotvec=[-3 3 -3 3];
if exist([path,'\','savedRoads.txt'],'file') && ~isempty(roadInfo)
    savedRoads=importdata([path,'\','savedRoads.txt']);
    antalRoads=length(roadInfo(:,1));
    roadArray={};
    j=0;
    for i=1:antalRoads
        if roadInfo(i,6)==2
            x1=savedRoads(i*2-1+j,1:roadInfo(i,5));
            y1=savedRoads(i*2+j,1:roadInfo(i,5));
            j=j+2;
            x2=savedRoads(i*2-1+j,1:roadInfo(i,5));
            y2=savedRoads(i*2+j,1:roadInfo(i,5));
            roadArray{i}=[x1;y1;x2;y2];
        elseif roadInfo(i,6)==1
            x=savedRoads(i*2-1+j,1:roadInfo(i,5));
            y=savedRoads(i*2+j,1:roadInfo(i,5));
            roadArray{i}=[x;y];
        end
    end
else
    roadArray={};
end

if exist([path,'\','SignInfo.txt'],'file')
    SignInfo=importdata([path,'\','SignInfo.txt']);
else
    SignInfo=[];    
end

if exist([path,'\','TransitionMatrix.txt'],'file')
    TransitionMatrix=importdata([path,'\','TransitionMatrix.txt']);
else
    TransitionMatrix=[];    
end





% UIWAIT makes Road_Network_Projection_Tool wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = Road_Network_Projection_Tool_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in project_button.
function project_button_Callback(hObject, eventdata, handles)
% hObject    handle to project_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global bridgeState TLState connected NTL off tcpipClientTL ...
    roadArray roadInfo SignInfo plotvec TransitionMatrix ...
    borderPos scale TLInfo waypoints running cell_dots borderVec

running=true;
if isempty(roadArray) || isempty(roadInfo)
     msgbox('Transfer Road Network before projecting.','Missing File','custom',imread('oops.png'));
else  
   if connected
        box off
        axis off
        hold on
        if isempty(TLState) && NTL~=0
            TLState=ones(1,NTL);
        end
        cell_dots=roadMapPlot(TLState,bridgeState,roadArray,roadInfo,SignInfo,plotvec,borderPos,TransitionMatrix,scale,TLInfo,waypoints,borderVec);
        stopp=0;
        while stopp==0
            [stopp hej1 hopp1] = fread(tcpipClientTL,1,'double');
            [TLState hej hopp] = fread(tcpipClientTL,NTL,'double');
            if ~isempty(hopp1) || ~isempty(hopp)
                stopp=1;
            else
                TL_disp(cell_dots,TLState,scale,borderPos,plotvec);
                flushinput(tcpipClientTL);
            end
        end
        
        fclose(tcpipClientTL);
        set(handles.TL_LED,'cdata',off);
        connected=false;
    else
        box off
        axis off
        hold on
        if isempty(TLState) & NTL~=0
            TLState=ones(1,NTL);
        end
        scale=get(handles.Scale_slider,'value');
        cell_dots=roadMapPlot(TLState,bridgeState,roadArray,roadInfo,SignInfo,plotvec,borderPos,TransitionMatrix,scale,TLInfo,waypoints,borderVec);
    end
end








% --- Executes on button press in TL_inner_checkbox.
function TL_inner_checkbox_Callback(hObject, eventdata, handles)
% hObject    handle to TL_inner_checkbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of TL_inner_checkbox
global bridge TL_in TL_out
TL_in=get(hObject,'Value');
roadMapPlot(bridge,TL_out,TL_in)

% --- Executes on button press in TL_outer_checkbox.
function TL_outer_checkbox_Callback(hObject, eventdata, handles)
% hObject    handle to TL_outer_checkbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of TL_outer_checkbox
global bridge TL_in TL_out
TL_out=get(hObject,'Value');
roadMapPlot(bridge,TL_out,TL_in)


% --- Executes on button press in show_WP.
function show_WP_Callback(hObject, eventdata, handles)
% hObject    handle to show_WP (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global waypoints
on_off=get(hObject,'Value');
if on_off==1
    waypoints=true;
else
    waypoints=false;
end
project_button_Callback(hObject, eventdata, handles)


% Hint: get(hObject,'Value') returns toggle state of show_WP


% --- Executes on button press in connectTL.
function connectTL_Callback(hObject, eventdata, handles)
% hObject    handle to connectTL (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global on connected bridgeState tcpipClientTL;

ip=get(handles.ipedit_TL,'String');
TLport=get(handles.porteditTL,'String');

tcpipClientTL = tcpip(ip,str2double(TLport),'NetworkRole','Client');
set(tcpipClientTL,'InputBufferSize',1000000);
set(tcpipClientTL,'Timeout',10);
fopen(tcpipClientTL);

bridgeState=fread(tcpipClientTL,1,'int32');
fclose(tcpipClientTL);


TLport=num2str(str2double(TLport)+1);
tcpipClientTL = tcpip(ip,str2double(TLport),'NetworkRole','Client');
set(tcpipClientTL,'InputBufferSize',1000000);
set(tcpipClientTL,'Timeout',5);
% s.setsockopt(socket.IPPROTO_TCP, socket.TCP_QUICKACK, True)
fopen(tcpipClientTL);
connected=true;

set(handles.TL_LED,'cdata',on);








function ipedit_TL_Callback(hObject, eventdata, handles)
% hObject    handle to ipedit_TL (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ipedit_TL as text
%        str2double(get(hObject,'String')) returns contents of ipedit_TL as a double
global ip
ip=get(handles.ipedit_TL,'String');


% --- Executes during object creation, after setting all properties.
function ipedit_TL_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ipedit_TL (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function porteditTL_Callback(hObject, eventdata, handles)
% hObject    handle to porteditTL (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of porteditTL as text
%        str2double(get(hObject,'String')) returns contents of porteditTL as a double
global TLport
TLport=get(handles.porteditTL,'String');


% --- Executes during object creation, after setting all properties.
function porteditTL_CreateFcn(hObject, eventdata, handles)
% hObject    handle to porteditTL (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in Network_file_transfer.
function Network_file_transfer_Callback(hObject, eventdata, handles)
% hObject    handle to Network_file_transfer (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global on TLInfo NTL roadArray roadInfo SignInfo TransitionMatrix

ip=get(handles.IPeditNetwork,'String');
port=get(handles.PorteditNetwork,'String');

for i=0:4
    receivefile(port,ip)
end

set(handles.Network_LED,'cdata',on);

roadInfo=importdata('roadInfo.txt');

if exist('TLInfo.txt','file')
    TLInfo=dlmread('TLInfo.txt');
    NTL=length(TLInfo(1,:));
else
    TLInfo=[];
end

savedRoads=importdata('savedRoads.txt');
antalRoads=length(roadInfo(:,1));
roadArray={};
j=0;
for i=1:antalRoads
    if roadInfo(i,6)==2
        x1=savedRoads(i*2-1+j,1:roadInfo(i,5));
        y1=savedRoads(i*2+j,1:roadInfo(i,5));
        j=j+2;
        x2=savedRoads(i*2-1+j,1:roadInfo(i,5));
        y2=savedRoads(i*2+j,1:roadInfo(i,5));
        roadArray{i}=[x1 x1(1:2);y1 y1(1:2);x2 x2(1:2);y2 y2(1:2)];
    elseif roadInfo(i,6)==1
        x=savedRoads(i*2-1+j,1:roadInfo(i,5));
        y=savedRoads(i*2+j,1:roadInfo(i,5));
        roadArray{i}=[x x(1:2);y y(1:2)];
    end
end

if exist('SignInfo.txt','file')
    SignInfo=importdata('SignInfo.txt');
end
if exist('TransitionMatrix.txt','file')
    TransitionMatrix=importdata('TransitionMatrix.txt');
end







function PorteditNetwork_Callback(hObject, eventdata, handles)
% hObject    handle to PorteditNetwork (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of PorteditNetwork as text
%        str2double(get(hObject,'String')) returns contents of PorteditNetwork as a double
global port
port=get(hObject,'String');


% --- Executes during object creation, after setting all properties.
function PorteditNetwork_CreateFcn(hObject, eventdata, handles)
% hObject    handle to PorteditNetwork (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function IPeditNetwork_Callback(hObject, eventdata, handles)
% hObject    handle to IPeditNetwork (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of IPeditNetwork as text
%        str2double(get(hObject,'String')) returns contents of IPeditNetwork as a double
global ip
ip=get(hObject,'String');


% --- Executes during object creation, after setting all properties.
function IPeditNetwork_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IPeditNetwork (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



% --- Executes on button press in Network_LED.
function Network_LED_Callback(hObject, eventdata, handles)
% hObject    handle to Network_LED (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of Network_LED


% --- Executes on button press in TL_LED.
function TL_LED_Callback(hObject, eventdata, handles)
% hObject    handle to TL_LED (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of TL_LED


% --- Executes during object creation, after setting all properties.
function Network_LED_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Network_LED (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes during object creation, after setting all properties.
function TL_LED_CreateFcn(hObject, eventdata, handles)
% hObject    handle to TL_LED (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes on slider movement.
function Scale_slider_Callback(hObject, eventdata, handles)
% hObject    handle to Scale_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
global scale running
scale=get(hObject,'Value');
set(handles.scale_edit,'String',num2str(scale));
if running
    project_button_Callback(hObject, eventdata, handles)
end


% --- Executes during object creation, after setting all properties.
function Scale_slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Scale_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function scale_edit_Callback(hObject, eventdata, handles)
% hObject    handle to scale_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of scale_edit as text
%        str2double(get(hObject,'String')) returns contents of scale_edit as a double
global scale running
scale=str2double(get(hObject,'String'));
if isempty(scale) || isnan(scale)
    msgbox('Scale input must be a number.','Syntax Error','custom',imread('virus.png'))
else
    L = get(handles.Scale_slider,{'min','max','value'});  % Get the slider's info..
    if scale >= L{1} && scale <= L{2}
        set(handles.Scale_slider,'value',scale)  % E falls within range of slider.
    elseif scale>L{2}
        set(hObject,'string',L{2}) % User tried to set slider out of range.
        set(handles.Scale_slider,'value',L{2})
    else
        set(hObject,'string',L{1}) % User tried to set slider out of range. 
        set(handles.Scale_slider,'value',L{1})
    end
    if running
        project_button_Callback(hObject, eventdata, handles)
    end
end



% --- Executes during object creation, after setting all properties.
function scale_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to scale_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in TL_checkbox.
function TL_checkbox_Callback(hObject, eventdata, handles)
% hObject    handle to TL_checkbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of TL_checkbox
global connected TLState NTL running cell_dots scale borderPos plotvec

if connected
    
else
    toggle=get(hObject,'Value');
    if NTL~=0
        if toggle==0
            TLState=ones(1,NTL);
        else
            TLState=zeros(1,NTL);
        end
        if running
            TL_disp(cell_dots,TLState,scale,borderPos,plotvec);
        end
    end
end


% --- Executes on button press in bridge_checkbox.
function bridge_checkbox_Callback(hObject, eventdata, handles)
% hObject    handle to bridge_checkbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of bridge_checkbox
global connected bridgeState

if connected
    
else
    toggle=get(hObject,'Value');
    if toggle==1
        bridgeState=1;
        project_button_Callback(hObject, eventdata, handles)
    else
        bridgeState=0;
        project_button_Callback(hObject, eventdata, handles)
    end
end
