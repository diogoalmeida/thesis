function varargout = Road_Network_Creator(varargin)
% ROAD_NETWORK_CREATOR MATLAB code for Road_Network_Creator.fig
%      ROAD_NETWORK_CREATOR, by itself, creates a new ROAD_NETWORK_CREATOR Om raises the existing
%      singleton*.
%
%      H = ROAD_NETWORK_CREATOR returns the handle to a new ROAD_NETWORK_CREATOR Om the handle to
%      the existing singleton*.
%
%      ROAD_NETWORK_CREATOR('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in ROAD_NETWORK_CREATOR.M with the given input arguments.
%
%      ROAD_NETWORK_CREATOR('Property','Value',...) creates a new ROAD_NETWORK_CREATOR Om raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Road_Network_Creator_OpeningFcn gets called.  An
%      unrecognized property name Om invalid value makes property application
%      stop.  All inputs are passed to Road_Network_Creator_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Road_Network_Creator

% Last Modified by GUIDE v2.5 01-Nov-2013 16:29:41

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @Road_Network_Creator_OpeningFcn, ...
    'gui_OutputFcn',  @Road_Network_Creator_OutputFcn, ...
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


% --- Executes just before Road_Network_Creator is made visible.
function Road_Network_Creator_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Road_Network_Creator (see VARARGIN)

% Choose default command line output for Road_Network_Creator
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);
clc
global xvec yvec borderVec ax ay projBorder numLanes xO yO xI yI legenden ...
    xC yC savedRoads saved roadouterdef roadinnerdef ...
    roadinnroaddef defRoad xb yb TLInfo xbvec ybvec...
    roadInfo savedNames TM savedLUT BRNum1 BRNum2 xB1 yB1 xB2 ...
    yB2 TLNum TLArray foldername path savedTL delRoad TLSwitch TSInfo ...
    SI1 SI2 EI1 EI2 TSNum TPNum spacing picture...
    xO1 yO1 xI1 yI1 xO2 yO2 xI2 yI2 xNO yNO xNI yNI xNC yNC xP yP...
    DualDir tripleDir1 tripleDir2 quadDir1 quadDir2 quadDir3 pedestrianRoads...
    pCross WP_num DWP

addpath([pwd '\Functions']);
addpath([pwd '\..\Images']);

projBorder=1;
xvec=[];
yvec=[];
a=-1;
if exist([pwd,'/../CalibrationTool/Calibration.txt'],'file')
    a=load('../CalibrationTool/Calibration.txt');
elseif exist([pwd,'/Calibration.txt'],'file')
    a=load('Calibration.txt');
else
    msgbox('Calibration file not found.','Missing File','custom',imread('oops.png'));
end



ax=[a(1,1:2) a(1,2:-1:1) a(1,1)];
ay=[a(1,3) a(1,3) a(1,4) a(1,4) a(1,3)];
numLanes=1;
xO=[];
yO=[];
xI=[];
yI=[];
xC=[];
yC=[];
legenden=false;
savedRoads={};
pedestrianRoads={};
pCross=[];
saved=false;
roadouterdef=[];
roadinnerdef=[];
roadinnroaddef=[];
defRoad=1;
xb=[];
yb=[];
savedNames={};
TLInfo=[];
xbvec=[];
ybvec=[];
TM=[];
roadInfo=[];
savedLUT={};
BRNum1=[];
BRNum2=[];
xB1=[];
yB1=[];
xB2=[];
yB2=[];
TLNum=1;
savedTL=[];
TLArray={[] [] [] [] [] [] [] [] [] []};
delRoad=0;
TLSwitch=1;
TSInfo=[];
SI1=0; SI2=0; EI1=0; EI2=0;
TSNum=1;
TPNum=1;
WP_num=1;
DWP=[];
xO1=[]; yO1=[]; xI1=[]; yI1=[]; xO2=[]; yO2=[]; xI2=[]; yI2=[]; xNO=[]; yNO=[]; xNI=[]; yNI=[];xNC=[]; yNC=[]; xP=[]; yP=[];
DualDir=0; tripleDir1=0; tripleDir2=0; quadDir1=0; quadDir2=0; quadDir3=0;
set(handles.roadnum_selector_textbox,'String','0')
set(handles.TL_number_textbox,'String','')
foldername='default';
path=strcat(pwd,'\',foldername);
picture=0;
if exist('OptionsInfo.txt','file')
    vec=dlmread('OptionsInfo.txt');
    spacing=vec(1);
    xMin=vec(2);
    xMax=vec(3);
    yMin=vec(4);
    yMax=vec(5);
    borderVec=[xMin xMax yMin yMax];                                                      %borderVec=[Xmin Xmax Ymin Ymax]
else
    spacing=NaN;
    xMin=min(ax)-0.1;
    xMax=max(ax)+0.1;
    yMin=min(ay)-0.1;
    yMax=max(ay)+0.1;
    borderVec=[xMin xMax yMin yMax];
    options()
    while isnan(spacing) || any(isnan(borderVec))
        options()
    end
end
plot_window()


% UIWAIT makes Road_Network_Creator wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = Road_Network_Creator_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

% --- Executes when figure1 is resized.
function figure1_ResizeFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



% --- Executes during object creation, after setting all properties.
function wiring_table_CreateFcn(hObject, eventdata, handles)
% hObject    handle to wiring_table (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% --- Executes during object creation, after setting all properties.
function axes1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: place code in OpeningFcn to populate axes1


% --------------------------------------------------------------------
function Projector_border_toggle_OffCallback(hObject, eventdata, handles)
% hObject    handle to Projector_border_toggle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global projBorder
projBorder=1;                                            %On=2 Off=1
plot_window()


% --------------------------------------------------------------------
function Projector_border_toggle_OnCallback(hObject, eventdata, handles)
% hObject    handle to Projector_border_toggle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global projBorder
projBorder=2;                                            %On=2 Off=1
plot_window()

% --- Executes on mouse press over axes background.
function axes1_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to axes1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in undo_button.
function undo_button_Callback(hObject, eventdata, handles)
% hObject    handle to undo_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global xvec yvec
if ~isempty(xvec)
    xvec=xvec(1:end-1);
    yvec=yvec(1:end-1);
    plot_window()
end

% --------------------------------------------------------------------
function marker_toggle_OffCallback(hObject, eventdata, handles)
% hObject    handle to marker_toggle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global WP_num
WP_num=1;
plot_window

% --------------------------------------------------------------------
function marker_toggle_OnCallback(hObject, eventdata, handles)
% hObject    handle to marker_toggle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global WP_num
WP_num=2;
plot_window


% --- Executes on button press in way_point_delete.
function way_point_delete_Callback(hObject, eventdata, handles)
% hObject    handle to way_point_delete (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global DWP xvec yvec

if ~isempty(DWP)
    if DWP==1
        xvec=xvec(:,2:end);
        yvec=yvec(:,2:end);
    elseif DWP==length(xvec)
        xvec=xvec(:,1:end-1);
        yvec=yvec(:,1:end-1);
    else
        xvec=xvec(:,[1:DWP-1,DWP+1:end]);
        yvec=yvec(:,[1:DWP-1,DWP+1:end]);
    end
    set(handles.waypoint_edit_textbox,'String','')
    DWP=[];
    plot_window()
end


function waypoint_edit_textbox_Callback(hObject, eventdata, handles)
% hObject    handle to waypoint_edit_textbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of waypoint_edit_textbox as text
%        str2double(get(hObject,'String')) returns contents of waypoint_edit_textbox as a double
global DWP xvec
STL=str2double(get(hObject,'String'));
if ~isempty(xvec)
    if ~any(STL==1:length(xvec))
        msgbox(sprintf(['Way Point Number must be a whole number.','\n','Make sure to clean your chips fingers before proceeding!']),'Syntax Error','custom',imread('chips.png'))
    else
        DWP=STL;
    end
else
    msgbox('Place way points first.','Fatal Error!','custom',imread('fatal.png'));
end



% --- Executes during object creation, after setting all properties.
function waypoint_edit_textbox_CreateFcn(hObject, eventdata, handles)
% hObject    handle to waypoint_edit_textbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in num_lane.
function num_lane_Callback(hObject, eventdata, handles)
% hObject    handle to num_lane (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global numLanes DualDir tripleDir1 tripleDir2 quadDir1 quadDir2 quadDir3

numLanes=get(hObject,'Value');
if numLanes==2
    DualDir=1;
elseif numLanes==3
    DualDir=0;
elseif numLanes==4
    tripleDir1=0;
    tripleDir2=0;
elseif numLanes==5
    tripleDir1=0;
    tripleDir2=1;
elseif numLanes==6
    tripleDir1=1;
    tripleDir2=1;
elseif numLanes==7
    quadDir1=0;
    quadDir2=0;
    quadDir3=0;
elseif numLanes==8
    quadDir1=0;
    quadDir2=0;
    quadDir3=1;
elseif numLanes==9
    quadDir1=0;
    quadDir2=1;
    quadDir3=1;
elseif numLanes==10
    quadDir1=1;
    quadDir2=1;
    quadDir3=1;
end

% Hints: contents = cellstr(get(hObject,'String')) returns num_lane contents as cell array
%        contents{get(hObject,'Value')} returns selected item from num_lane





% --- Executes during object creation, after setting all properties.
function num_lane_CreateFcn(hObject, eventdata, handles)
% hObject    handle to num_lane (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in gen_road.
function gen_road_Callback(hObject, eventdata, handles)
% hObject    handle to gen_road (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global xvec yvec xO yO xI yI numLanes xC yC reverse spacing xO1 yO1 xI1 yI1...
    xO2 yO2 xI2 yI2 xNO yNO xNI yNI xNC yNC xP yP
if length(xvec)>2
    if numLanes==11
        [xP,yP]=singleLane(xvec,yvec,reverse);
        xI=[]; yI=[]; xO=[]; yO=[]; xO1=[]; yO1=[]; xI1=[]; yI1=[]; xO2=[]; yO2=[]; xI2=[]; yI2=[]; xNO=[]; yNO=[]; xNI=[]; yNI=[];xNC=[]; yNC=[]; xC=[]; yC=[];
        plot_window()
        
    elseif numLanes==10
        [xO1, yO1, xI1, yI1,xO2, yO2, xI2, yI2]=quadLane(xvec,yvec,reverse,3,spacing);
        xC=[]; yC=[]; xNO=[]; yNO=[]; xNI=[]; yNI=[];xNC=[]; yNC=[]; xO=[]; yO=[]; xI=[]; yI=[]; xP=[]; yP=[];
        plot_window()
    elseif numLanes==9
        [xO1, yO1, xI1, yI1,xO2, yO2, xI2, yI2]=quadLane(xvec,yvec,reverse,2,spacing);
        xC=[]; yC=[]; xNO=[]; yNO=[]; xNI=[]; yNI=[];xNC=[]; yNC=[]; xO=[]; yO=[]; xI=[]; yI=[]; xP=[]; yP=[];
        plot_window()
    elseif numLanes==8
        [xO1, yO1, xI1, yI1,xO2, yO2, xI2, yI2]=quadLane(xvec,yvec,reverse,1,spacing);
        xC=[]; yC=[]; xNO=[]; yNO=[]; xNI=[]; yNI=[];xNC=[]; yNC=[]; xO=[]; yO=[]; xI=[]; yI=[]; xP=[]; yP=[];
        plot_window()
    elseif numLanes==7
        [xO1, yO1, xI1, yI1,xO2, yO2, xI2, yI2]=quadLane(xvec,yvec,reverse,0,spacing);
        xC=[]; yC=[]; xNO=[]; yNO=[]; xNI=[]; yNI=[];xNC=[]; yNC=[]; xO=[]; yO=[]; xI=[]; yI=[]; xP=[]; yP=[];
        plot_window()
    elseif numLanes==6
        [xNO, yNO, xNI, yNI,xNC,yNC]=tripleLane(xvec,yvec,reverse,2,spacing);
        xC=[]; yC=[]; xO1=[]; yO1=[]; xI1=[]; yI1=[]; xO2=[]; yO2=[]; xI2=[]; yI2=[]; xO=[]; yO=[]; xI=[]; yI=[]; xP=[]; yP=[];
        plot_window()
    elseif numLanes==5
        [xNO, yNO, xNI, yNI,xNC,yNC]=tripleLane(xvec,yvec,reverse,1,spacing);
        xC=[]; yC=[]; xO1=[]; yO1=[]; xI1=[]; yI1=[]; xO2=[]; yO2=[]; xI2=[]; yI2=[]; xO=[]; yO=[]; xI=[]; yI=[]; xP=[]; yP=[];
        plot_window()
    elseif numLanes==4
        [xNO, yNO, xNI, yNI,xNC,yNC]=tripleLane(xvec,yvec,reverse,0,spacing);
        xC=[]; yC=[]; xO1=[]; yO1=[]; xI1=[]; yI1=[]; xO2=[]; yO2=[]; xI2=[]; yI2=[]; xO=[]; yO=[]; xI=[]; yI=[]; xP=[]; yP=[];
        plot_window()
    elseif numLanes==3
        [xO, yO, xI, yI]=dualLane(xvec,yvec,reverse,false,spacing);
        xC=[]; yC=[]; xO1=[]; yO1=[]; xI1=[]; yI1=[]; xO2=[]; yO2=[]; xI2=[]; yI2=[]; xNO=[]; yNO=[]; xNI=[]; yNI=[];xNC=[]; yNC=[]; xP=[]; yP=[];
        plot_window()
    elseif numLanes==2
        [xO, yO, xI, yI]=dualLane(xvec,yvec,reverse,true,spacing);
        xC=[]; yC=[]; xO1=[]; yO1=[]; xI1=[]; yI1=[]; xO2=[]; yO2=[]; xI2=[]; yI2=[]; xNO=[]; yNO=[]; xNI=[]; yNI=[];xNC=[]; yNC=[]; xP=[]; yP=[];
        plot_window()
    elseif numLanes==1
        [xC,yC]=singleLane(xvec,yvec,reverse);
        xI=[]; yI=[]; xO=[]; yO=[]; xO1=[]; yO1=[]; xI1=[]; yI1=[]; xO2=[]; yO2=[]; xI2=[]; yI2=[]; xNO=[]; yNO=[]; xNI=[]; yNI=[];xNC=[]; yNC=[]; xP=[]; yP=[];
        plot_window()
    end
else
    msgbox('Not enough input arguments. Minimum of three data points','Fatal Failure!','custom',imread('fatal.png'));
end

% --------------------------------------------------------------------
function legend_toggle_OnCallback(hObject, eventdata, handles)
% hObject    handle to legend_toggle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global legenden
legenden=true;
plot_window()

% --------------------------------------------------------------------
function legend_toggle_OffCallback(hObject, eventdata, handles)
% hObject    handle to legend_toggle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global legenden
legenden=false;
plot_window()

% --------------------------------------------------------------------
function TL_num_toggle_OffCallback(hObject, eventdata, handles)
% hObject    handle to TL_num_toggle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global TLNum
TLNum=1;
plot_window()


% --------------------------------------------------------------------
function TL_num_toggle_OnCallback(hObject, eventdata, handles)
% hObject    handle to TL_num_toggle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global TLNum
TLNum=2;
plot_window()



% --- Executes on button press in delete_button.
function delete_button_Callback(hObject, eventdata, handles)
% hObject    handle to delete_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global xO yO xI yI xC yC delRoad savedRoads roadInfo TLInfo TLArray ...
    TM TSInfo xO1 yO1 xI1 yI1 xO2 yO2 xI2 yI2 xNO yNO xNI yNI xNC yNC xP yP...
    pedestrianRoads pCross
if delRoad==0
    if ~isempty(xO) || ~isempty(xC) || ~isempty(xNC) || ~isempty(xI2) || ~isempty(xP)
        xO=[];
        yO=[];
        xI=[];
        yI=[];
        xC=[];
        yC=[]; xP=[]; yP=[];
        xO1=[]; yO1=[]; xI1=[]; yI1=[]; xO2=[]; yO2=[]; xI2=[]; yI2=[]; xNO=[]; yNO=[]; xNI=[]; yNI=[];xNC=[]; yNC=[];
    end
elseif delRoad>0
    if roadInfo(delRoad,7)~=0
        row=roadInfo(delRoad,7);
        roadInfo(row,7)=0;
        savedRoads{row}=savedRoads{row}(1:2,:);
    end
    if roadInfo(delRoad,2)==2 % Dual
        if roadInfo(delRoad,3)==1
            roadInfo(delRoad+1,2)=roadInfo(delRoad+1,2)-1;
            roadInfo(delRoad+1,3)=roadInfo(delRoad+1,3)-1;
        else
            roadInfo(delRoad-1,2)=roadInfo(delRoad-1,2)-1;
        end
        
    elseif roadInfo(delRoad,2)==3 % triple
        if roadInfo(delRoad,3)==1
            roadInfo(delRoad+1:delRoad+2,2)=roadInfo(delRoad+1:delRoad+2,2)-1;
            roadInfo(delRoad+1:delRoad+2,3)=roadInfo(delRoad+1:delRoad+2,3)-1;
        elseif roadInfo(delRoad,3)==2
            roadInfo(delRoad-1,2)=roadInfo(delRoad-1,2)-1;
            roadInfo(delRoad+1,2)=roadInfo(delRoad+1,2)-1;
            roadInfo(delRoad+1,3)=roadInfo(delRoad+1,3)-1;
        else
            roadInfo(delRoad-2:delRoad-1,2)=roadInfo(delRoad-2:delRoad-1,2)-1;
        end
        
    elseif roadInfo(delRoad,2)==4   % quad
        if roadInfo(delRoad,3)==1
            roadInfo(delRoad+1:delRoad+3,2)=roadInfo(delRoad+1:delRoad+3,2)-1;
            roadInfo(delRoad+1:delRoad+3,3)=roadInfo(delRoad+1:delRoad+3,3)-1;
        elseif roadInfo(delRoad,3)==2
            roadInfo(delRoad-1,2)=roadInfo(delRoad-1,2)-1;
            roadInfo(delRoad+1:delRoad+2,2)=roadInfo(delRoad+1:delRoad+2,2)-1;
            roadInfo(delRoad+1:delRoad+2,3)=roadInfo(delRoad+1:delRoad+2,3)-1;
        elseif roadInfo(delRoad,3)==3
            roadInfo(delRoad-2:delRoad-1,2)=roadInfo(delRoad-2:delRoad-1,2)-1;
            roadInfo(delRoad+1,2)=roadInfo(delRoad+1,2)-1;
            roadInfo(delRoad+1,3)=roadInfo(delRoad+1,3)-1;
        else
            roadInfo(delRoad-3:delRoad-1,2)=roadInfo(delRoad-3:delRoad-1,2)-1;
        end
    end
    if ~isempty(TLInfo)
        if max(TLInfo(1,:)==delRoad)>0
            aux=TLInfo(1,:)~=delRoad;
            b=1:length(TLInfo(1,:));
            savevec=b(aux);
            replace=1:length(savevec);
            for i=1:length(TLArray)
                nygrej=[];
                if ~isempty(TLArray{i}(1,:))
                    for j=1:length(TLArray{i}(1,:))
                        aux2=abs(TLArray{i}(1,j))==savevec;
                        if max(aux2)~=0
                            nygrej=[nygrej replace(aux2)*sign(TLArray{i}(1,j))];
                        end
                    end
                    TLArray{i}=nygrej;
                end
            end
            TLInfo=TLInfo(:,aux);
            aux=TLInfo(1,:)>delRoad;
            TLInfo(1,:)=TLInfo(1,:)-aux;
        end
    end
    
    if delRoad==1
        savedRoads=savedRoads(2:end);
        roadInfo=roadInfo(2:end,:);
    elseif delRoad==length(savedRoads)
        savedRoads=savedRoads(1:end-1);
        roadInfo=roadInfo(1:end-1,:);
    else
        savedRoads=savedRoads([1:delRoad-1 delRoad+1:end]);
        roadInfo=roadInfo([1:delRoad-1 delRoad+1:end],:);
    end
    roadInfo(:,1)=1:length(roadInfo(:,1));
    if ~isempty(TSInfo)
        if any(TSInfo(1,:)==delRoad)
            indexsame=TSInfo(1,:)==delRoad;
            indexnotsame=TSInfo(1,:)~=delRoad;
            TSInfo(1,:)=TSInfo(1,:)-indexsame;
            TSInfo=TSInfo(:,indexnotsame);
        end
    end
    if ~isempty(TM)
        hej=TM(1,:)~=delRoad;
        hoj=TM(3,:)~=delRoad;
        TM=TM(:,hej+hoj);
    end
elseif delRoad<0
    if abs(delRoad)==1
        pedestrianRoads=pedestrianRoads(2:end);
    elseif delRoad==length(pedestrianRoads)
        pedestrianRoads=pedestrianRoads(1:end-1);
    else
        pedestrianRoads=pedestrianRoads([1:abs(delRoad)-1 abs(delRoad)+1:end]);
    end
    
    if ~isempty(pCross)
        if any(pCross(1,:)==delRoad)
            aux45=pCross(1,:)==delRoad;
            aux46=pCross(1,:)~=delRoad;
            auxCross=pCross(:,aux45);
            for kk=1:length(auxCross(1,:))
                aux47=auxCross(3,kk)==TSInfo(1,:);
                aux48=auxCross(4,kk)==TSInfo(2,:);
                aux49=logical(ones(1,length(aux47))-aux47.*aux48);
                TSInfo=TSInfo(:,aux49);
            end
            pCross=pCross(:,aux46);
        end
    end
    
    
end
set(handles.roadnum_selector_textbox,'String','0')
set(handles.wiring_table,'Data',cell2matrix(TLArray),'ColumnFormat',{'numeric'})
plot_window()


% --- Executes on button press in undo_all_button.
function undo_all_button_Callback(hObject, eventdata, handles)
% hObject    handle to undo_all_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global xvec yvec
if ~isempty(xvec)
    xvec=[];
    yvec=[];
end
plot_window()


function roadnum_selector_textbox_Callback(hObject, eventdata, handles)
% hObject    handle to roadnum_selector_textbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of roadnum_selector_textbox as text
%        str2double(get(hObject,'String')) returns contents of roadnum_selector_textbox as a double
global delRoad savedRoads xvec pedestrianRoads
DR=str2double(get(hObject,'String'));
if ~isempty(savedRoads) || ~isempty(xvec) || ~isempty(pedestrianRoads)
    if ~isempty(pedestrianRoads)
        PD=1:length(pedestrianRoads);
    else
        PD=[];
    end
    if ~any(DR==[-PD(end:-1:1),0,1:length(savedRoads)])
        msgbox(sprintf(['Road Number must be a whole number.','\n','Make sure to clean your chips fingers before proceeding!']),'Syntax Error','custom',imread('chips.png'))
    else
        delRoad=DR;
    end
else
    msgbox('Create roads first.','Fatal Error!','custom',imread('fatal.png'));
end

% --- Executes during object creation, after setting all properties.
function roadnum_selector_textbox_CreateFcn(hObject, eventdata, handles)
% hObject    handle to roadnum_selector_textbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end





% --- Executes on selection change in default_popup.
function default_popup_Callback(hObject, eventdata, handles)
% hObject    handle to default_popup (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global defRoad roadouterdef roadinnerdef roadinnroaddef
defRoad=get(hObject,'Value');                                               %1=no road 2=inner+outer 3=innerroad 4=all 5=inner+outer w/ bridge 6=all w/ bridge
if defRoad==2
    roadouterdef=load('DefaultRoads/DefOuterLane.txt')';
    roadinnerdef=load('DefaultRoads/DefInnerLane.txt')';
    roadinnroaddef=[];
    plot_window()
elseif defRoad==3
    roadinnroaddef=load('DefaultRoads/DefInnerRoad.txt')';
    roadouterdef=[];
    roadinnerdef=[];
    plot_window()
elseif defRoad==4
    roadouterdef=load('DefaultRoads/DefOuterLane.txt')';
    roadinnerdef=load('DefaultRoads/DefInnerLane.txt')';
    roadinnroaddef=load('DefaultRoads/DefInnerRoad.txt')';
    plot_window()
elseif defRoad==5
    roadouterdef=load('DefaultRoads/DefOuterLaneBridge.txt')';
    roadinnerdef=load('DefaultRoads/DefInnerLaneBridge.txt')';
    roadinnroaddef=[];
    plot_window()
elseif defRoad==6
    roadouterdef=load('DefaultRoads/DefOuterLaneBridge.txt')';
    roadinnerdef=load('DefaultRoads/DefInnerLaneBridge.txt')';
    roadinnroaddef=load('DefaultRoads/DefInnerRoad.txt')';
    plot_window()
else
    roadouterdef=[];
    roadinnerdef=[];
    roadinnroaddef=[];
    plot_window()
end


% Hints: contents = cellstr(get(hObject,'String')) returns default_popup contents as cell array
%        contents{get(hObject,'Value')} returns selected item from default_popup


% --- Executes during object creation, after setting all properties.
function default_popup_CreateFcn(hObject, eventdata, handles)
% hObject    handle to default_popup (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



% --------------------------------------------------------------------
function waypoint_selector_OnCallback(hObject, eventdata, handles)
% hObject    handle to waypoint_selector (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global xvec yvec toggleon borderVec
toggleon=true;
while 1
    [x,y]=ginput(1);
    if toggleon==true;
        if x<borderVec(2)&&x>borderVec(1)&&y<borderVec(4)&&y>borderVec(3)
            xvec=[xvec x];
            yvec=[yvec y];
            plot_window()
        end
    else
        break
    end
    
end


% --------------------------------------------------------------------
function waypoint_selector_ClickedCallback(hObject, eventdata, handles)
% hObject    handle to waypoint_selector (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function waypoint_selector_OffCallback(hObject, eventdata, handles)
% hObject    handle to waypoint_selector (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global toggleon
toggleon=false;





% --- Executes on button press in bridge_button.
function bridge_button_Callback(hObject, eventdata, handles)
% hObject    handle to bridge_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global xb yb BRNum1 BRNum2 ...
    savedRoads xB1 yB1 xB2 yB2 SI1 SI2 EI1 EI2
if ~isempty(BRNum1) && ~isempty(BRNum2) && BRNum1~=BRNum2
    if length(xb)==2
        
        [xB1, yB1, xB2, yB2, error,SI1,EI1,SI2,EI2]=twoway_bridge2(savedRoads{BRNum1}(end-1:end,:),savedRoads{BRNum2}(end-1:end,:),xb,yb);
        xb=[]; yb=[];
        plot_window()
        if error==1
            msgbox(sprintf('Bridge too close to starting point. \n Please leave the building!'),'Major Error!','custom',imread('jonas.jpg'))
        elseif error==2
            msgbox(sprintf('Bridge points too close together. \n  Evacuate immediately!'),'General Alarm','custom',imread('jonas.jpg'))
        end
    else
        msgbox('Please place two bridge points.','Red Alert!','custom',imread('redalert.jpg'))
    end
else
    msgbox('Please select two different roads.','Red Alert!','custom',imread('redalert.jpg'))
end
set(handles.Bridge_road_num1,'String','')
set(handles.Bridge_road_num2,'String','')

% --- Executes on button press in remove_bridge_button.
function remove_bridge_button_Callback(hObject, eventdata, handles)
% hObject    handle to remove_bridge_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global xb yb xB1 yB1 xB2 yB2
xb=[]; yb=[]; xB1=[]; yB1=[]; xB2=[]; yB2=[];

plot_window()



% --------------------------------------------------------------------
function bridge_selector_ClickedCallback(hObject, eventdata, handles)
% hObject    handle to bridge_selector (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global xb yb borderVec

[x,y]=ginput(2);
if max(x)<borderVec(2) && min(x)>borderVec(1) && max(y)<borderVec(4) && min(y)>borderVec(3)
    xb=x;
    yb=y;
    plot_window()
end



% --- Executes on button press in trafficlight_button.
function trafficlight_button_Callback(hObject, eventdata, handles)
% hObject    handle to trafficlight_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function trafficlight_selector_ClickedCallback(hObject, eventdata, handles)
% hObject    handle to trafficlight_selector (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global savedRoads borderVec TLInfo

[x,y]=ginput(1);
if x<borderVec(2)&&x>borderVec(1)&&y<borderVec(4)&&y>borderVec(3)
    if ~isempty(savedRoads)
        dp=zeros(1,length(savedRoads));
        Ip=zeros(1,length(savedRoads));
        for road=1:length(savedRoads)
            [dp(road),Ip(road)] = min((savedRoads{road}(1,:)-x).^2+(savedRoads{road}(2,:)-y).^2);
        end
        [a,roadIndex]=min(dp);
        x=savedRoads{roadIndex}(1,Ip(roadIndex));
        y=savedRoads{roadIndex}(2,Ip(roadIndex));
        %         xTL=[xTL x];
        %         yTL=[yTL y];
        %         TLWP=[TLWP Ip(roadIndex)];
        %         roadTL=[roadTL roadIndex];
        %
        TLInfo=[TLInfo [roadIndex;Ip(roadIndex);x;y]];
        plot_window()
    else
        msgbox('Please save a road first.','Red Alert!','custom',imread('redalert.jpg'))
    end
end


% --- Executes on button press in undo_last_TL_button.
function undo_last_TL_button_Callback(hObject, eventdata, handles)
% hObject    handle to undo_last_TL_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global TLInfo
if ~isempty(TLInfo)
    TLInfo=TLInfo(:,1:end-1);
    plot_window()
end

% --- Executes on button press in undo_all_TL_button.
function undo_all_TL_button_Callback(hObject, eventdata, handles)
% hObject    handle to undo_all_TL_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global TLInfo
TLInfo=[];
plot_window()



function TL_remove_textbox_Callback(hObject, eventdata, handles)
% hObject    handle to TL_remove_textbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of TL_remove_textbox as text
%        str2double(get(hObject,'String')) returns contents of TL_remove_textbox as a double
global DSTL TLInfo
STL=str2double(get(hObject,'String'));
if ~isempty(TLInfo)
    if ~any(STL==1:length(TLInfo(1,:)))
        msgbox(sprintf(['Traffic Light Number must be a whole number.','\n','Make sure to clean your chips fingers before proceeding!']),'Syntax Error','custom',imread('chips.png'))
    else
        DSTL=STL;
    end
else
    msgbox('Place traffic lights first.','Fatal Error!','custom',imread('fatal.png'));
end


% --- Executes during object creation, after setting all properties.
function TL_remove_textbox_CreateFcn(hObject, eventdata, handles)
% hObject    handle to TL_remove_textbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in delete_STL.
function delete_STL_Callback(hObject, eventdata, handles)
% hObject    handle to delete_STL (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global DSTL TLInfo TLArray

for i=1:length(TLArray(1,:))
    aux2=abs(TLArray{i})~=DSTL;
    aux=abs(TLArray{i}(1,aux2))>DSTL;
    TLArray{i}=(abs(TLArray{i}(1,aux2))-aux).*sign(TLArray{i}(1,aux2));
end

if DSTL==1
    TLInfo=TLInfo(:,2:end);
elseif DSTL==length(TLInfo(1,:))
    TLInfo=TLInfo(:,1:end-1);
else
    TLInfo=TLInfo(:,[1:DSTL-1,DSTL+1:end]);
end
set(handles.wiring_table,'Data',cell2matrix(TLArray),'ColumnFormat',{'numeric'})
set(handles.TL_remove_textbox,'String','')
plot_window()






% --------------------------------------------------------------------
function WP_selector_ClickedCallback(hObject, eventdata, handles)
% hObject    handle to WP_selector (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global savedRoads borderVec TM

[x,y]=ginput(2);
hold on
if max(x)<borderVec(2) && min(x)>borderVec(1) && max(y)<borderVec(4) && min(y)>borderVec(3)
    if ~isempty(savedRoads)
        dp1=zeros(1,length(savedRoads));
        Ip1=zeros(1,length(savedRoads));
        dp2=zeros(1,length(savedRoads));
        Ip2=zeros(1,length(savedRoads));
        for road=1:length(savedRoads)
            [dp1(road),Ip1(road)] = min((savedRoads{road}(1,:)-x(1)).^2+(savedRoads{road}(2,:)-y(1)).^2);
            [dp2(road),Ip2(road)] = min((savedRoads{road}(1,:)-x(2)).^2+(savedRoads{road}(2,:)-y(2)).^2);
        end
        [a,road1]=min(dp1);
        [a,road2]=min(dp2);
        
        wp1=Ip1(road1);
        wp2=Ip2(road2);
        
        TM=[TM [road1; wp1; road2; wp2]];
        
        plot_window()
    else
    end
end



% --- Executes on button press in TP_undo_button.
function TP_undo_button_Callback(hObject, eventdata, handles)
% hObject    handle to TP_undo_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global TM
if ~isempty(TM)
    TM=TM(:,1:end-1);
    plot_window()
end


% --- Executes on button press in TP_undo_all_button.
function TP_undo_all_button_Callback(hObject, eventdata, handles)
% hObject    handle to TP_undo_all_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global TM
TM=[];
plot_window()


function TP_remove_edit_textbox_Callback(hObject, eventdata, handles)
% hObject    handle to TP_remove_edit_textbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of TP_remove_edit_textbox as text
%        str2double(get(hObject,'String')) returns contents of TP_remove_edit_textbox as a double
global TM DTP
STS=str2double(get(hObject,'String'));
if ~isempty(TM)
    if ~any(STS==1:length(TM(1,:)))
        msgbox(sprintf(['Transition Number must be a whole number.','\n','Make sure to clean your chips fingers before proceeding!']),'Syntax Error','custom',imread('chips.png'))
    else
        DTP=STS;
    end
else
    msgbox('Place transition way points first.','Fatal Error!','custom',imread('fatal.png'));
end


% --- Executes during object creation, after setting all properties.
function TP_remove_edit_textbox_CreateFcn(hObject, eventdata, handles)
% hObject    handle to TP_remove_edit_textbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in Delete_selected_TP_button.
function Delete_selected_TP_button_Callback(hObject, eventdata, handles)
% hObject    handle to Delete_selected_TP_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global DTP TM


if DTP==1
    TM=TM(:,2:end);
elseif DTP==length(TM(1,:))
    TM=TM(:,1:end-1);
else
    TM=TM(:,[1:DTP-1,DTP+1:end]);
end
set(handles.TP_remove_edit_textbox,'String','')
plot_window()

% --------------------------------------------------------------------
function TP_toggle_OffCallback(hObject, eventdata, handles)
% hObject    handle to TP_toggle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global TPNum
TPNum=1;
plot_window()


% --------------------------------------------------------------------
function TP_toggle_OnCallback(hObject, eventdata, handles)
% hObject    handle to TP_toggle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global TPNum
TPNum=2;
plot_window()



% --- Executes on button press in reverse_checkbox.
function reverse_checkbox_Callback(hObject, eventdata, handles)
% hObject    handle to reverse_checkbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of reverse_checkbox
global reverse

reverse=get(hObject,'Value');

% --- Executes on button press in options_button.
function options_button_Callback(hObject, eventdata, handles)
% hObject    handle to options_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global spacing borderVec
options();
while isnan(spacing) || any(isnan(borderVec))
    options()
end
plot_window()


function answer=options()
global spacing borderVec
xMin=borderVec(1);
xMax=borderVec(2);
yMin=borderVec(3);
yMax=borderVec(4);

prompt = {sprintf(['Enter Road Lane Spacing:','\n','\n','Note that confirmed roads can not be alterd later.','\n','Current Spacing is ',num2str(spacing),'.']),...
    sprintf(['\n','Enter Border Limits.','\n','\n','Current value for xMin is ',num2str(borderVec(1)),'.','\n','Enter new value for xMin:']),...
    sprintf(['Current value for xMax is ',num2str(borderVec(2)),'.','\n','Enter new value for xMax:']),...
    sprintf(['Current value for yMin is ',num2str(borderVec(3)),'.','\n','Enter new value for yMin:']),...
    sprintf(['Current value for yMax is ',num2str(borderVec(4)),'.','\n','Enter new value for yMax:'])};
dlg_title = 'Options';
num_lines = 1;
answer = inputdlg(prompt,dlg_title,num_lines);

if ~isempty(answer)
    if ~isempty(answer{1}) & ~isnan(answer{1})
        spacing=str2double(answer{1});
    end
    if ~isempty(answer{2}) & ~isnan(answer{2})
        xMin=str2double(answer{2});
    end
    if ~isempty(answer{3}) & ~isnan(answer{3})
        xMax=str2double(answer{3});
    end
    if ~isempty(answer{4}) & ~isnan(answer{4})
        yMin=str2double(answer{4});
    end
    if ~isempty(answer{5}) & ~isnan(answer{5})
        yMax=str2double(answer{5});
    end
end
borderVec=[xMin xMax yMin yMax];
vec=[spacing xMin xMax yMin yMax];
dlmwrite('OptionsInfo.txt',vec)





function Bridge_road_num1_Callback(hObject, eventdata, handles)
% hObject    handle to Bridge_road_num1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Bridge_road_num1 as text
%        str2double(get(hObject,'String')) returns contents of Bridge_road_num1 as a double
global BRNum1 savedRoads
B1=str2double(get(hObject,'String'));
if any(B1==1:length(savedRoads))
    BRNum1=B1;
else
    msgbox('Road number must be a numerical value.','Syntax Error','custom',imread('oops.png'))
end


function Bridge_road_num2_Callback(hObject, eventdata, handles)
% hObject    handle to Bridge_road_num2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Bridge_road_num2 as text
%        str2double(get(hObject,'String')) returns contents of Bridge_road_num2 as a double
global BRNum2 savedRoads
B2=str2double(get(hObject,'String'));
if any(B2==1:length(savedRoads))
    BRNum2=B2;
else
    msgbox('Road number must be a numerical value.','Syntax Error','custom',imread('oops.png'))
end

% --- Executes during object creation, after setting all properties.
function Bridge_road_num2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Bridge_road_num2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes during object creation, after setting all properties.
function Bridge_road_num1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Bridge_road_num1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



% --- Executes on selection change in traffic_light_button_number_popup.
function traffic_light_button_number_popup_Callback(hObject, eventdata, handles)
% hObject    handle to traffic_light_button_number_popup (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns traffic_light_button_number_popup contents as cell array
%        contents{get(hObject,'Value')} returns selected item from traffic_light_button_number_popup




% --- Executes during object creation, after setting all properties.
function traffic_light_button_number_popup_CreateFcn(hObject, eventdata, handles)
% hObject    handle to traffic_light_button_number_popup (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function traffic_light_number_textbox_Callback(hObject, eventdata, handles)
% hObject    handle to traffic_light_number_textbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of traffic_light_number_textbox as text
%        str2double(get(hObject,'String')) returns contents of traffic_light_number_textbox as a double


% --- Executes during object creation, after setting all properties.
function traffic_light_number_textbox_CreateFcn(hObject, eventdata, handles)
% hObject    handle to traffic_light_number_textbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in connect_traffic_light_button.
function connect_traffic_light_button_Callback(hObject, eventdata, handles)
% hObject    handle to connect_traffic_light_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



% --- Executes on selection change in traffic_light_number_popup.
function traffic_light_number_popup_Callback(hObject, eventdata, handles)
% hObject    handle to traffic_light_number_popup (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns traffic_light_number_popup contents as cell array
%        contents{get(hObject,'Value')} returns selected item from traffic_light_number_popup


% --- Executes during object creation, after setting all properties.
function traffic_light_number_popup_CreateFcn(hObject, eventdata, handles)
% hObject    handle to traffic_light_number_popup (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in TL_button_popupmenu.
function TL_button_popupmenu_Callback(hObject, eventdata, handles)
% hObject    handle to TL_button_popupmenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns TL_button_popupmenu contents as cell array
%        contents{get(hObject,'Value')} returns selected item from TL_button_popupmenu
global TLSwitch
TLSwitch=get(hObject,'Value');

% --- Executes during object creation, after setting all properties.
function TL_button_popupmenu_CreateFcn(hObject, eventdata, handles)
% hObject    handle to TL_button_popupmenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function TL_number_textbox_Callback(hObject, eventdata, handles)
% hObject    handle to TL_number_textbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of TL_number_textbox as text
%        str2double(get(hObject,'String')) returns contents of TL_number_textbox as a double
global TLIndex TLInfo
TL=str2double(get(hObject,'String'));
if ~any(abs(TL)==1:length(TLInfo(1,:)))
    msgbox(sprintf(['Traffic Light Number must be a whole number (not equal to zero).','\n','Make sure to clean your chips fingers before proceeding!']),'Syntax Error','custom',imread('chips.png'))
else
    TLIndex=TL;
end


% --- Executes during object creation, after setting all properties.
function TL_number_textbox_CreateFcn(hObject, eventdata, handles)
% hObject    handle to TL_number_textbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in wire_TL_button.
function wire_TL_button_Callback(hObject, eventdata, handles)
% hObject    handle to wire_TL_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global TLSwitch TLIndex TLArray savedTL TLInfo
if isnan(TLIndex) || TLIndex==0 || abs(TLIndex)>length(TLInfo(1,:)) || ~isnumeric(TLIndex)
    msgbox(sprintf(['Traffic Light Number must be a whole number (not equal to zero).','\n','Make sure to clean your chips fingers before proceeding!']),'Syntax Error','custom',imread('chips.png'))
elseif any(TLIndex==savedTL)
    msgbox('You have already wired this traffic light.','SBS Error!','custom',imread('sbs.png'))
else
    if isempty(TLArray{TLSwitch})
        TLArray{TLSwitch}=[TLIndex];
    else
        TLArray{TLSwitch}=[TLArray{TLSwitch}(1,:) TLIndex];
    end
    savedTL=[savedTL TLIndex];
    set(handles.wiring_table,'Data',cell2matrix(TLArray),'ColumnFormat',{'numeric'})
    set(handles.TL_number_textbox,'String','')
    
end

% --- Executes on button press in Unwired_TL_connections.
function Unwired_TL_connections_Callback(hObject, eventdata, handles)
% hObject    handle to Unwired_TL_connections (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global TLInfo savedTL
missing=[];
aux1=zeros(1,length(TLInfo(1,:)));
if isempty(savedTL)
    missing=TLInfo(1,:);
else
    for i=1:length(savedTL)
        aux=1:length(TLInfo(1,:))~=abs(savedTL(i));
        aux1=aux1+aux;
    end
    vec=1:length(TLInfo(1,:));
    aux=logical(i-1-aux1);
    missing=vec(aux);
end
missing=sort(missing);
if ~isempty(missing)
    msgbox(sprintf(['Following Traffic Lights are unwired:','\n',num2str(missing)]),'Lose Ends Detected!','custom',imread('virus.png'));
else
    msgbox('All Traffic Lights have been wired.','Connections Complete!','custom',imread('virus.png'));
end


% --- Executes on button press in clear_connections_button.
function clear_connections_button_Callback(hObject, eventdata, handles)
% hObject    handle to clear_connections_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global TLArray savedTL

TLArray={[],[],[],[],[],[],[],[],[],[]};
savedTL=[];
set(handles.TL_number_textbox,'String','')
set(handles.wiring_table,'Data',cell2matrix(TLArray),'ColumnFormat',{'numeric'})



% --- Executes on button press in Undo_last_traffic_sign_button_switch.
function Undo_last_traffic_sign_button_switch_Callback(hObject, eventdata, handles)
% hObject    handle to Undo_last_traffic_sign_button_switch (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global TSInfo
TSInfo=TSInfo(:,1:end-1);
plot_window()

% --- Executes on button press in Undo_all_TS_button.
function Undo_all_TS_button_Callback(hObject, eventdata, handles)
% hObject    handle to Undo_all_TS_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global TSInfo
TSInfo=[];
plot_window()

function TS_remove_edit_textbox_Callback(hObject, eventdata, handles)
% hObject    handle to TS_remove_edit_textbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of TS_remove_edit_textbox as text
%        str2double(get(hObject,'String')) returns contents of TS_remove_edit_textbox as a double
global DTS TSInfo
STS=str2double(get(hObject,'String'));
if ~isempty(TSInfo)
    if ~any(STS==1:length(TSInfo(1,:)))
        msgbox(sprintf(['Traffic Sign Number must be a whole number.','\n','Make sure to clean your chips fingers before proceeding!']),'Syntax Error','custom',imread('chips.png'))
    else
        DTS=STS;
    end
else
    msgbox('Place traffic sign first.','Fatal Error!','custom',imread('fatal.png'));
end


% --- Executes during object creation, after setting all properties.
function TS_remove_edit_textbox_CreateFcn(hObject, eventdata, handles)
% hObject    handle to TS_remove_edit_textbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in Delete_selected_traffic_sign_button.
function Delete_selected_traffic_sign_button_Callback(hObject, eventdata, handles)
% hObject    handle to Delete_selected_traffic_sign_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global DTS TSInfo


if DTS==1
    TSInfo=TSInfo(:,2:end);
elseif DTS==length(TSInfo(1,:))
    TSInfo=TSInfo(:,1:end-1);
else
    TSInfo=TSInfo(:,[1:DTS-1,DTS+1:end]);
end
set(handles.TS_remove_edit_textbox,'String','')
plot_window()


% --------------------------------------------------------------------
function stop_sign_selector_ClickedCallback(hObject, eventdata, handles)
% hObject    handle to stop_sign_selector (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global savedRoads borderVec TSInfo

[x,y]=ginput(1);
if x<borderVec(2)&&x>borderVec(1)&&y<borderVec(4)&&y>borderVec(3)
    if ~isempty(savedRoads)
        dp=zeros(1,length(savedRoads));
        Ip=zeros(1,length(savedRoads));
        for road=1:length(savedRoads)
            [dp(road),Ip(road)] = min((savedRoads{road}(1,:)-x).^2+(savedRoads{road}(2,:)-y).^2);
        end
        [a,roadIndex]=min(dp);
        x=savedRoads{roadIndex}(1,Ip(roadIndex));
        y=savedRoads{roadIndex}(2,Ip(roadIndex));
        TSInfo=[TSInfo [roadIndex;Ip(roadIndex); x; y; 1; 0]];                              % vägnummer    WP på vägen     x-koordinat    y-koordinat     1=stopp  hastighetsbegränsning (0 om sådan saknas)
        plot_window()
    else
        msgbox('Please save a road first.','Red Alert!','custom',imread('redalert.jpg'))
    end
end


% --------------------------------------------------------------------
function vajningsplikt_selector_ClickedCallback(hObject, eventdata, handles)
% hObject    handle to vajningsplikt_selector (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global savedRoads borderVec TSInfo

[x,y]=ginput(1);
if x<borderVec(2)&&x>borderVec(1)&&y<borderVec(4)&&y>borderVec(3)
    if ~isempty(savedRoads)
        dp=zeros(1,length(savedRoads));
        Ip=zeros(1,length(savedRoads));
        for road=1:length(savedRoads)
            [dp(road),Ip(road)] = min((savedRoads{road}(1,:)-x).^2+(savedRoads{road}(2,:)-y).^2);
        end
        [a,roadIndex]=min(dp);
        x=savedRoads{roadIndex}(1,Ip(roadIndex));
        y=savedRoads{roadIndex}(2,Ip(roadIndex));
        
        TSInfo=[TSInfo [roadIndex;Ip(roadIndex); x; y; 2; 0]];
        plot_window()
    else
        msgbox('Please save a road first.','Red Alert!','custom',imread('redalert.jpg'))
    end
end


% --------------------------------------------------------------------
function SpeedLimit_selector_ClickedCallback(hObject, eventdata, handles)
% hObject    handle to SpeedLimit_selector (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global savedRoads borderVec TSInfo

[x,y]=ginput(1);
if x<borderVec(2)&&x>borderVec(1)&&y<borderVec(4)&&y>borderVec(3)
    if ~isempty(savedRoads)
        dp=zeros(1,length(savedRoads));
        Ip=zeros(1,length(savedRoads));
        for road=1:length(savedRoads)
            [dp(road),Ip(road)] = min((savedRoads{road}(1,:)-x).^2+(savedRoads{road}(2,:)-y).^2);
        end
        [a,roadIndex]=min(dp);
        x = inputdlg('Enter Speed Limit:', 'Speed Limit', [1 20]);
        data = str2num(x{:});
        x=savedRoads{roadIndex}(1,Ip(roadIndex));
        y=savedRoads{roadIndex}(2,Ip(roadIndex));
        
        TSInfo=[TSInfo [roadIndex;Ip(roadIndex); x; y; 3; data]];
        plot_window()
    else
        msgbox('Please save a road first.','Red Alert!','custom',imread('redalert.jpg'))
    end
end


% --------------------------------------------------------------------
function sign_toggle_OffCallback(hObject, eventdata, handles)
% hObject    handle to sign_toggle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global TSNum
TSNum=1;
plot_window()

% --------------------------------------------------------------------
function sign_toggle_OnCallback(hObject, eventdata, handles)
% hObject    handle to sign_toggle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global TSNum
TSNum=2;
plot_window()


% --------------------------------------------------------------------
function crossing_selector_ClickedCallback(hObject, eventdata, handles)
% hObject    handle to crossing_selector (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global savedRoads borderVec TSInfo

[x,y]=ginput(1);
if x<borderVec(2)&&x>borderVec(1)&&y<borderVec(4)&&y>borderVec(3)
    if ~isempty(savedRoads)
        dp=zeros(1,length(savedRoads));
        Ip=zeros(1,length(savedRoads));
        for road=1:length(savedRoads)
            [dp(road),Ip(road)] = min((savedRoads{road}(1,:)-x).^2+(savedRoads{road}(2,:)-y).^2);
        end
        [a,roadIndex]=min(dp);
        x=savedRoads{roadIndex}(1,Ip(roadIndex));
        y=savedRoads{roadIndex}(2,Ip(roadIndex));
        
        TSInfo=[TSInfo [roadIndex;Ip(roadIndex); x; y; 4; 0]];
        plot_window()
    else
        msgbox('Please save a road first.','Red Alert!','custom',imread('redalert.jpg'))
    end
end


function plot_window()
global projBorder xvec yvec borderVec ax ay xO yO xI yI legenden xC yC ...
    savedRoads roadouterdef roadinnerdef roadinnroaddef defRoad TLInfo ...
    xb yb xB1 yB1 xB2 yB2 TLNum TSInfo TSNum TM TPNum picture path ...
    xO1 yO1 xI1 yI1 xO2 yO2 xI2 yI2 xNO yNO xNI yNI xNC yNC xP yP...
    pedestrianRoads WP_num

num1=0;
SIZE=1;

if picture==1
    fig=figure(1);
    TLsave=TLNum;
    TSsave=TSNum;
    TPsave=TPNum;
    Lsave=legenden;
    WPsave=WP_num;
    TLNum=2; TSNum=2; TPNum=2;
    legenden=true; WP_num=2;
    SIZE=0.5;
    
end

% Empty plot window
hold off
plot(1,1,'w')
axis(borderVec)
hold on

% Plot all saved roads
for i=1:length(savedRoads)
    for j=1:2:length(savedRoads{i}(:,1))
        plot([savedRoads{i}(j,:) savedRoads{i}(j,1)],[savedRoads{i}(j+1,:) savedRoads{i}(j+1,1)])
    end
    axis(borderVec)
    hold on
    normen=norm([savedRoads{i}(1,2)-savedRoads{i}(1,1),savedRoads{i}(2,2)-savedRoads{i}(2,1)]);
    q=quiver(savedRoads{i}(1,1),savedRoads{i}(2,1),(savedRoads{i}(1,2)-savedRoads{i}(1,1))/normen,(savedRoads{i}(2,2)-savedRoads{i}(2,1))/normen,0.1);
    set(q,'LineWidth',4,'Color','blue')
    adjust_quiver_arrowhead_size(q,30);
    num1=i;
    hold on
    if legenden
        text(savedRoads{i}(1,end),savedRoads{i}(2,end),num2str(num1),'FontSize',15*SIZE,'color','red','BackgroundColor',[0 0 1]);
        hold on
    end
end

% Plot all pedestrian roads
for i=1:length(pedestrianRoads)
    for j=1:2:length(pedestrianRoads{i}(:,1))
        p=plot([pedestrianRoads{i}(j,:) pedestrianRoads{i}(j,1)],[pedestrianRoads{i}(j+1,:) pedestrianRoads{i}(j+1,1)]);
        set(p,'color',[0.35 0.35 0.35]);
    end
    axis(borderVec)
    hold on
    normen=norm([pedestrianRoads{i}(1,2)-pedestrianRoads{i}(1,1),pedestrianRoads{i}(2,2)-pedestrianRoads{i}(2,1)]);
    q=quiver(pedestrianRoads{i}(1,1),pedestrianRoads{i}(2,1),(pedestrianRoads{i}(1,2)-pedestrianRoads{i}(1,1))/normen,(pedestrianRoads{i}(2,2)-pedestrianRoads{i}(2,1))/normen,0.1);
    set(q,'LineWidth',4,'Color',[0.35 0.35 0.35])
    adjust_quiver_arrowhead_size(q,30);
    num2=i;
    hold on
    if legenden
        text(pedestrianRoads{i}(1,end),pedestrianRoads{i}(2,end),['-',num2str(num2)],'FontSize',15*SIZE,'color','red','BackgroundColor',[0.35 0.35 0.35]);
        hold on
    end
end


% plot predefined roads
if defRoad==2 || defRoad==5
    p=plot([roadouterdef(1,:) roadouterdef(1,1)],[roadouterdef(2,:) roadouterdef(2,1)],[roadinnerdef(1,:) roadinnerdef(1,1)],[roadinnerdef(2,:) roadinnerdef(2,1)]);
    set(p,'Color',[1 0.7 0]);
    axis(borderVec)
    hold on
    normen1=norm([roadouterdef(1,2)-roadouterdef(1,1),roadouterdef(2,2)-roadouterdef(2,1)]);
    q1=quiver(roadouterdef(1,1),roadouterdef(2,1),(roadouterdef(1,2)-roadouterdef(1,1))/normen1,(roadouterdef(2,2)-roadouterdef(2,1))/normen1,0.1);
    set(q1,'LineWidth',4,'Color',[1 0.7 0])
    adjust_quiver_arrowhead_size(q1,30);
    hold on
    num1=num1+1;
    if legenden
        text(roadouterdef(1,end),roadouterdef(2,end),num2str(num1),'FontSize',15*SIZE,'color','black','BackgroundColor',[1 0.7 0]);
        hold on
    end
    normen2=norm([roadinnerdef(1,2)-roadinnerdef(1,1),roadinnerdef(2,2)-roadinnerdef(2,1)]);
    q2=quiver(roadinnerdef(1,1),roadinnerdef(2,1),(roadinnerdef(1,2)-roadinnerdef(1,1))/normen2,(roadinnerdef(2,2)-roadinnerdef(2,1))/normen2,0.1);
    set(q2,'LineWidth',4,'Color',[1 0.7 0])
    adjust_quiver_arrowhead_size(q2,30);
    hold on
    num1=num1+1;
    if legenden
        text(roadinnerdef(1,end),roadinnerdef(2,end),num2str(num1),'FontSize',15*SIZE,'color','black','BackgroundColor',[1 0.7 0]);
        hold on
    end
elseif defRoad==3
    p=plot([roadinnroaddef(1,:) roadinnroaddef(1,1)],[roadinnroaddef(2,:) roadinnroaddef(2,1)]);
    set(p,'Color',[1 0.7 0]);
    axis(borderVec)
    hold on
    normen=norm([roadinnroaddef(1,2)-roadinnroaddef(1,1),roadinnroaddef(2,2)-roadinnroaddef(2,1)]);
    q=quiver(roadinnroaddef(1,1),roadinnroaddef(2,1),(roadinnroaddef(1,2)-roadinnroaddef(1,1))/normen,(roadinnroaddef(2,2)-roadinnroaddef(2,1))/normen,0.1);
    set(q,'LineWidth',4,'Color',[1 0.7 0])
    adjust_quiver_arrowhead_size(q,30);
    hold on
    num1=num1+1;
    if legenden
        text(roadinnroaddef(1,end),roadinnroaddef(2,end),num2str(num1),'FontSize',15*SIZE,'color','black','BackgroundColor',[1 0.7 0]);
        hold on
    end
elseif defRoad==4 || defRoad==6
    p=plot([roadouterdef(1,:) roadouterdef(1,1)],[roadouterdef(2,:) roadouterdef(2,1)],[roadinnerdef(1,:) roadinnerdef(1,1)],[roadinnerdef(2,:) roadinnerdef(2,1)]...
        ,[roadinnroaddef(1,:) roadinnroaddef(1,1)],[roadinnroaddef(2,:) roadinnroaddef(2,1)]);
    set(p,'Color',[1 0.7 0]);
    axis(borderVec)
    hold on
    normen1=norm([roadouterdef(1,2)-roadouterdef(1,1),roadouterdef(2,2)-roadouterdef(2,1)]);
    q1=quiver(roadouterdef(1,1),roadouterdef(2,1),(roadouterdef(1,2)-roadouterdef(1,1))/normen1,(roadouterdef(2,2)-roadouterdef(2,1))/normen1,0.1);
    set(q1,'LineWidth',4,'Color',[1 0.7 0])
    adjust_quiver_arrowhead_size(q1,30);
    hold on
    num1=num1+1;
    if legenden
        text(roadouterdef(1,end),roadouterdef(2,end),num2str(num1),'FontSize',15*SIZE,'color','black','BackgroundColor',[1 0.7 0]);
        hold on
    end
    normen2=norm([roadinnerdef(1,2)-roadinnerdef(1,1),roadinnerdef(2,2)-roadinnerdef(2,1)]);
    q2=quiver(roadinnerdef(1,1),roadinnerdef(2,1),(roadinnerdef(1,2)-roadinnerdef(1,1))/normen2,(roadinnerdef(2,2)-roadinnerdef(2,1))/normen2,0.1);
    set(q2,'LineWidth',4,'Color',[1 0.7 0])
    adjust_quiver_arrowhead_size(q2,30);
    hold on
    num1=num1+1;
    if legenden
        text(roadinnerdef(1,end),roadinnerdef(2,end),num2str(num1),'FontSize',15*SIZE,'color','black','BackgroundColor',[1 0.7 0]);
        hold on
    end
    normen=norm([roadinnroaddef(1,2)-roadinnroaddef(1,1),roadinnroaddef(2,2)-roadinnroaddef(2,1)]);
    q=quiver(roadinnroaddef(1,1),roadinnroaddef(2,1),(roadinnroaddef(1,2)-roadinnroaddef(1,1))/normen,(roadinnroaddef(2,2)-roadinnroaddef(2,1))/normen,0.1);
    set(q,'LineWidth',4,'Color',[1 0.7 0])
    adjust_quiver_arrowhead_size(q,30);
    hold on
    num1=num1+1;
    if legenden
        text(roadinnroaddef(1,end),roadinnroaddef(2,end),num2str(num1),'FontSize',15*SIZE,'color','black','BackgroundColor',[1 0.7 0]);
        hold on
    end
end

%Plot projector border
if projBorder==2
    plot(ax,ay,'r')
    axis(borderVec)
    hold on
end

% plot generated one-lane road
if ~isempty(xC)
    p=plot([xC xC(1)],[yC yC(1)]);
    set(p,'Color',[0 .5 0]);
    hold on
    normenC=norm([xC(2)-xC(1),yC(2)-yC(1)]);
    q=quiver(xC(1),yC(1),(xC(2)-xC(1))/normenC,(yC(2)-yC(1))/normenC,0.1);
    set(q,'LineWidth',4,'Color',[0 .5 0])
    adjust_quiver_arrowhead_size(q,30);
    axis(borderVec)
    hold on
end

% plot generated two-lane road
if ~isempty(xO)
    p=plot([xO xO(1)],[yO yO(1)],[xI xI(1)],[yI yI(1)]);
    set(p,'Color',[0 .5 0]);
    hold on
    normenO=norm([xO(2)-xO(1),yO(2)-yO(1)]);
    q=quiver(xO(1),yO(1),(xO(2)-xO(1))/normenO,(yO(2)-yO(1))/normenO,0.1);
    set(q,'LineWidth',4,'Color',[0 .5 0])
    adjust_quiver_arrowhead_size(q,30);
    hold on
    normenI=norm([xI(2)-xI(1),yI(2)-yI(1)]);
    qI=quiver(xI(1),yI(1),(xI(2)-xI(1))/normenI,(yI(2)-yI(1))/normenI,0.1);
    set(qI,'LineWidth',4,'Color',[0 .5 0])
    adjust_quiver_arrowhead_size(qI,30);
    hold on
end

% plot generated three-lane road
if ~isempty(xNO)
    p=plot([xNO xNO(1)],[yNO yNO(1)],[xNI xNI(1)],[yNI yNI(1)],[xNC xNC(1)],[yNC yNC(1)]);
    set(p,'Color',[0 .5 0]);
    hold on
    normenO=norm([xNO(2)-xNO(1),yNO(2)-yNO(1)]);
    q=quiver(xNO(1),yNO(1),(xNO(2)-xNO(1))/normenO,(yNO(2)-yNO(1))/normenO,0.1);
    set(q,'LineWidth',4,'Color',[0 .5 0])
    adjust_quiver_arrowhead_size(q,30);
    hold on
    normenI=norm([xNI(2)-xNI(1),yNI(2)-yNI(1)]);
    qI=quiver(xNI(1),yNI(1),(xNI(2)-xNI(1))/normenI,(yNI(2)-yNI(1))/normenI,0.1);
    set(qI,'LineWidth',4,'Color',[0 .5 0])
    adjust_quiver_arrowhead_size(qI,30);
    hold on
    normenC=norm([xNC(2)-xNC(1),yNC(2)-yNC(1)]);
    qC=quiver(xNC(1),yNC(1),(xNC(2)-xNC(1))/normenC,(yNC(2)-yNC(1))/normenC,0.1);
    set(qC,'LineWidth',4,'Color',[0 .5 0])
    adjust_quiver_arrowhead_size(qC,30);
    hold on
end

% plot generated quad-lane road
if ~isempty(xO1)
    p=plot([xO1 xO1(1)],[yO1 yO1(1)],[xI1 xI1(1)],[yI1 yI1(1)],[xO2 xO2(1)],[yO2 yO2(1)],[xI2 xI2(1)],[yI2 yI2(1)]);
    set(p,'Color',[0 .5 0]);
    hold on
    normenO1=norm([xO1(2)-xO1(1),yO1(2)-yO1(1)]);
    q=quiver(xO1(1),yO1(1),(xO1(2)-xO1(1))/normenO1,(yO1(2)-yO1(1))/normenO1,0.1);
    set(q,'LineWidth',4,'Color',[0 .5 0])
    adjust_quiver_arrowhead_size(q,30);
    hold on
    normenI=norm([xI1(2)-xI1(1),yI1(2)-yI1(1)]);
    qI=quiver(xI1(1),yI1(1),(xI1(2)-xI1(1))/normenI,(yI1(2)-yI1(1))/normenI,0.1);
    set(qI,'LineWidth',4,'Color',[0 .5 0])
    adjust_quiver_arrowhead_size(qI,30);
    hold on
    normenO=norm([xO2(2)-xO2(1),yO2(2)-yO2(1)]);
    q=quiver(xO2(1),yO2(1),(xO2(2)-xO2(1))/normenO,(yO2(2)-yO2(1))/normenO,0.1);
    set(q,'LineWidth',4,'Color',[0 .5 0])
    adjust_quiver_arrowhead_size(q,30);
    hold on
    normenI=norm([xI2(2)-xI2(1),yI2(2)-yI2(1)]);
    qI=quiver(xI2(1),yI2(1),(xI2(2)-xI2(1))/normenI,(yI2(2)-yI2(1))/normenI,0.1);
    set(qI,'LineWidth',4,'Color',[0 .5 0])
    adjust_quiver_arrowhead_size(qI,30);
    hold on
end

% plot generated pedestrian road
if ~isempty(xP)
    p=plot([xP xP(1)],[yP yP(1)]);
    set(p,'Color',[0 .5 0]);
    hold on
    normenC=norm([xP(2)-xP(1),yP(2)-yP(1)]);
    q=quiver(xP(1),yP(1),(xP(2)-xP(1))/normenC,(yP(2)-yP(1))/normenC,0.1);
    set(q,'LineWidth',4,'Color',[0 .5 0])
    adjust_quiver_arrowhead_size(q,30);
    axis(borderVec)
    hold on
end

% plot generated bridge
if ~isempty(xB1)
    p=plot(xB1,yB1,xB2,yB2);
    set(p,'Color',[0 .5 0]);
    hold on
    normen1=norm([xB1(2)-xB1(1),yB1(2)-yB1(1)]);
    q=quiver(xB1(1),yB1(1),(xB1(2)-xB1(1))/normen1,(yB1(2)-yB1(1))/normen1,0.1);
    set(q,'LineWidth',4,'Color',[0 .5 0])
    adjust_quiver_arrowhead_size(q,30);
    hold on
    normen2=norm([xB2(2)-xB2(1),yB2(2)-yB2(1)]);
    qI=quiver(xB2(1),yB2(1),(xB2(2)-xB2(1))/normen2,(yB2(2)-yB2(1))/normen2,0.1);
    set(qI,'LineWidth',4,'Color',[0 .5 0])
    adjust_quiver_arrowhead_size(qI,30);
    hold on
end

% plot traffic lights
if ~isempty(TLInfo)
    for i=1:length(TLInfo(3,:))
        p=plot([TLInfo(3,i) TLInfo(3,i)],[TLInfo(4,i)-0.08 TLInfo(4,i)+0.08],'k');
        set(p,'LineWidth',7)
        axis(borderVec)
        hold on
        if TLNum==2
            text(TLInfo(3,i),TLInfo(4,i),num2str(i),'FontSize',15*SIZE,'color','yellow','BackgroundColor',[0 0 0]);
            hold on
        end
    end
    plot(TLInfo(3,:),TLInfo(4,:)+0.035,'.r',TLInfo(3,:),TLInfo(4,:)-0.035,'.g')
    axis(borderVec)
    hold on
end

% plot selected waypoints
if ~isempty(xvec)
    plot(xvec,yvec,'+m')
    axis(borderVec)
    hold on
    if WP_num==2;
        for i=1:length(xvec)
            text(xvec(i),yvec(i),num2str(i),'FontSize',15*SIZE,'color','magenta','BackgroundColor',[0 0 0]);
            hold on
        end
    end
end

% plot selected bridge waypoints
if length(xb)==2
    plot(xb,yb,'+c')
    axis(borderVec)
    hold on
end
% plot selected transition waypoints
if ~isempty(TM)
    for i=1:length(TM(1,:))
        xTP=[savedRoads{TM(1,i)}(1,TM(2,i)) savedRoads{TM(3,i)}(1,TM(4,i))];
        yTP=[savedRoads{TM(1,i)}(2,TM(2,i)) savedRoads{TM(3,i)}(2,TM(4,i))];
        
        plot(xTP,yTP,'+-k')
        normen2=norm([xTP(2)-xTP(1),yTP(2)-yTP(1)]);
        qTP=quiver(xTP(1),yTP(1),(xTP(2)-xTP(1))/normen2,(yTP(2)-yTP(1))/normen2,normen2);
        set(qTP,'LineWidth',2,'Color',[0 0 0])
        adjust_quiver_arrowhead_size(qTP,1/normen2*3);
        axis(borderVec)
        hold on
        if TPNum==2
            text((xTP(1)+xTP(2))/2,(yTP(1)+yTP(2))/2,num2str(i),'FontSize',15*SIZE,'color','white','BackgroundColor',[0 0 1]);
            hold on
        end
    end
end

% plot traffic signs
if ~isempty(TSInfo)
    scale=0.85;
    for i=1:length(TSInfo(1,:))
        x1=TSInfo(3,i);
        y1=TSInfo(4,i);
        if TSInfo(5,i)==1
            
            
            t = (1/16:1/8:1)'*2*pi;
            x = sin(t);
            y = cos(t);
            
            fill(scale*0.1*1.06*x+x1,scale*0.1*1.06*y+y1,'w')
            hold on
            fill(scale*0.1*0.85*x+x1,scale*0.1*0.85*y+y1,'r')
            hold on
            ht = text(x1,y1,'S');
            set(ht,'HorizontalAlignment','center')
            set(ht,'FontSize',scale*0.1*150*SIZE,'color','white')
            axis(borderVec)
        elseif TSInfo(5,i)==2
            t = (1/6:1/3:3/2)'*2*pi;
            x = sin(t);
            y = cos(t);
            
            fill(scale*0.08*1.49*x+x1,scale*0.08*1.49*y+y1,'r')
            hold on
            fill(scale*0.08*x+x1,scale*0.08*y+y1,'y')
            hold on
            axis(borderVec)
        elseif TSInfo(5,i)==3
            t = (0:0.01:1)'*2*pi;
            x = sin(t);
            y = cos(t);
            
            fill(scale*0.08*1.21*x+x1,scale*0.08*1.21*y+y1,'r')
            hold on
            fill(scale*0.08*x+x1,scale*0.08*y+y1,'y')
            hold on
            
            % % 0.7 hör ihop med linewidth 70
            ht = text(x1,y1,num2str(TSInfo(6,i)));
            set(ht,'HorizontalAlignment','center')
            set(ht,'FontSize',scale*10,'color','black')
            axis(borderVec)
        elseif TSInfo(5,i)==4
            imagesc([x1-0.075 x1+0.075], [y1+0.075 y1-0.075], imread('herrgarman.png'));
            hold on
            axis(borderVec)
        end
        if TSNum==2
            text(x1+0.07,y1,num2str(i),'FontSize',15*SIZE,'color','green','BackgroundColor',[0 0 0]);
            hold on
        end
    end
end



if picture==1
    %     for i=1:length(TransitionMatrix(1,:))
    %         plot(savedRoads{TransitionMatrix(1,i)}(1,TransitionMatrix(2,i)),savedRoads{TransitionMatrix(1,i)}(2,TransitionMatrix(2,i)),'*b')
    %         hold on
    %         plot(savedRoads{TransitionMatrix(3,i)}(1,TransitionMatrix(4,i)),savedRoads{TransitionMatrix(3,i)}(2,TransitionMatrix(4,i)),'*g')
    %         hold on
    %         axis(borderVec)
    %     end
    
    saveas(fig, [path,'\','RoadMap.png']);
    delete(fig);
    picture=0;
    TLNum=TLsave;
    TSNum=TSsave;
    TPNum=TPsave;
    legenden=Lsave;
    WP_num=WPsave;
end




% --- Executes on button press in add_button.
function add_button_Callback(hObject, eventdata, handles)
% hObject    handle to add_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global xO yO xI yI xC yC savedRoads xvec yvec xbvec ybvec roadInfo ...
    xB1 yB1 xB2 yB2 BRNum1 BRNum2 SI1 SI2 EI1 EI2 DualDir...
    xO1 yO1 xI1 yI1 xO2 yO2 xI2 yI2 xNO yNO xNI yNI xNC yNC xP yP tripleDir1...
    tripleDir2 quadDir1 quadDir2 quadDir3 pedestrianRoads TSInfo pCross

l=length(savedRoads);
l2=length(pedestrianRoads);
if ~isempty(xO) || ~isempty(xC) || ~isempty(xB1) || ~isempty(xI1) || ~isempty(xNC) || ~isempty(xP)
    if ~isempty(xI)
        savedRoads{l+1}=[[xO xO(1)];[yO yO(1)]];
        roadInfo=[roadInfo; [length(savedRoads) 2 1 0 length(xO)+1 1 0 0 0]];               % vägnummer    vägtyp  nummer inom vägtyp     motriktad inom vägtyp    antal WP    2 om bro, 1 annars    vägnr på systerväg inom bro
        savedRoads{l+2}=[[xI xI(1)];[yI yI(1)]];
        roadInfo=[roadInfo; [length(savedRoads) 2 2 DualDir length(xI)+1 1 0 0 0]];
        [TSInfo pCross]=find_pedestrian_crossing(1:l2,l+1,savedRoads,pedestrianRoads,TSInfo,pCross,2);
    elseif ~isempty(xC)
        savedRoads{l+1}=[[xC xC(1)];[yC yC(1)]];
        roadInfo=[roadInfo; [length(savedRoads) 1 1 0 length(xC)+1 1 0 0 0]];
        [TSInfo pCross]=find_pedestrian_crossing(1:l2,l+1:l+2,savedRoads,pedestrianRoads,TSInfo,pCross,2);
    elseif ~isempty(xB1)
        savedRoads{BRNum1}=[savedRoads{BRNum1}(1:2,:);[xB1 xB1(1)];[yB1 yB1(1)]];
        roadInfo(BRNum1,:)=[BRNum1 roadInfo(BRNum1,2) roadInfo(BRNum1,3) roadInfo(BRNum1,4) length(xB1)+1 length(savedRoads{BRNum1}(:,1))/2 BRNum2 SI1 EI1];
        savedRoads{BRNum2}=[savedRoads{BRNum2}(1:2,:);[xB2 xB2(1)];[yB2 yB2(1)]];
        roadInfo(BRNum2,:)=[BRNum2 roadInfo(BRNum2,2) roadInfo(BRNum2,3) roadInfo(BRNum2,4) length(xB2)+1 length(savedRoads{BRNum2}(:,1))/2 BRNum1 SI2 EI2];
        [TSInfo pCross]=find_pedestrian_crossing(1:l2,l+1:l+2,savedRoads,pedestrianRoads,TSInfo,pCross,2);
    elseif ~isempty(xNI) %triple
        savedRoads{l+1}=[[xNO xNO(1)];[yNO yNO(1)]];
        roadInfo=[roadInfo; [length(savedRoads) 3 1 0 length(xNO)+1 1 0 0 0]];
        savedRoads{l+2}=[[xNC xNC(1)];[yNC yNC(1)]];
        roadInfo=[roadInfo; [length(savedRoads) 3 2 tripleDir1 length(xNC)+1 1 0 0 0]];
        savedRoads{l+3}=[[xNI xNI(1)];[yNI yNI(1)]];
        roadInfo=[roadInfo; [length(savedRoads) 3 3 tripleDir2 length(xNI)+1 1 0 0 0]];
        [TSInfo pCross]=find_pedestrian_crossing(1:l2,l+1:l+3,savedRoads,pedestrianRoads,TSInfo,pCross,2);
    elseif ~isempty(xI1) %quad
        savedRoads{l+1}=[[xO2 xO2(1)];[yO2 yO2(1)]];
        roadInfo=[roadInfo; [length(savedRoads) 4 1 0 length(xO2)+1 1 0 0 0]];
        savedRoads{l+2}=[[xO1 xO1(1)];[yO1 yO1(1)]];
        roadInfo=[roadInfo; [length(savedRoads) 4 2 quadDir1 length(xO1)+1 1 0 0 0]];
        savedRoads{l+3}=[[xI1 xI1(1)];[yI1 yI1(1)]];
        roadInfo=[roadInfo; [length(savedRoads) 4 3 quadDir2 length(xI1)+1 1 0 0 0]];
        savedRoads{l+4}=[[xI2 xI2(1)];[yI2 yI2(1)]];
        roadInfo=[roadInfo; [length(savedRoads) 4 4 quadDir3 length(xI2)+1 1 0 0 0]];
        [TSInfo pCross]=find_pedestrian_crossing(1:l2,l+1:l+4,savedRoads,pedestrianRoads,TSInfo,pCross,2);
    elseif ~isempty(xP) % pedestrian
        pedestrianRoads{l2+1}=[[xP xP(1)];[yP yP(1)]];
        [TSInfo pCross]=find_pedestrian_crossing(l2+1,1:l,savedRoads,pedestrianRoads,TSInfo,pCross,1);
    end
    xbvec=xvec; ybvec=yvec;
    xvec=[]; yvec=[]; xO=[]; yO=[]; xI=[]; yI=[]; xC=[]; yC=[]; xB1=[]; yB1=[];
    xB2=[]; yB2=[]; BRNum1=[]; BRNum2=[]; SI1=0; SI2=0; EI1=0; EI2=0; xP=[]; yP=[];
    xO1=[]; yO1=[]; xI1=[]; yI1=[]; xO2=[]; yO2=[]; xI2=[]; yI2=[]; xNO=[]; yNO=[]; xNI=[]; yNI=[];xNC=[]; yNC=[];
    plot_window()
else
    msgbox('You have not generated a road system yet','System Meltdown!','custom',imread('bomb.jpg'));
end



% --- Executes on button press in add_default.
function add_default_Callback(hObject, eventdata, handles)
% hObject    handle to add_default (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global defRoad roadouterdef roadinnerdef roadinnroaddef savedRoads roadInfo savedLUT
l=length(savedRoads);
if defRoad==2
    savedRoads{l+1}=[roadouterdef(1,:);roadouterdef(2,:)];
    roadInfo=[roadInfo; [length(savedRoads) -2 1 0 length(roadouterdef(1,:)) 1 0 0 0]];
    savedRoads{l+2}=[roadinnerdef(1,:);roadinnerdef(2,:)];
    roadInfo=[roadInfo; [length(savedRoads) -2 2 1 length(roadinnerdef(1,:)) 1 0 0 0]];
    defRoad=1;
    plot_window()
    msgbox('Por um quando você pode ter os dois?','Pedros!','custom',imread('pedros.png'))
elseif defRoad==3
    savedRoads{l+1}=[roadinnroaddef(1,:);roadinnroaddef(2,:)];
    roadInfo=[roadInfo; [length(savedRoads) -1 1 0 length(roadinnroaddef(1,:)) 1 0 0 0]];
    defRoad=1;
    plot_window()
    msgbox('Regras internas estrada!','Pedros!','custom',imread('pedros2.png'))
elseif defRoad==4
    savedRoads{l+1}=[roadouterdef(1,:);roadouterdef(2,:)];
    roadInfo=[roadInfo; [length(savedRoads) -2 1 0 length(roadouterdef(1,:)) 1 0 0 0]];
    savedRoads{l+2}=[roadinnerdef(1,:);roadinnerdef(2,:)];
    roadInfo=[roadInfo; [length(savedRoads) -2 2 1 length(roadinnerdef(1,:)) 1 0 0 0]];
    savedRoads{l+3}=[roadinnroaddef(1,:);roadinnroaddef(2,:)];
    roadInfo=[roadInfo; [length(savedRoads) -1 1 0 length(roadinnroaddef(1,:)) 1 0 0 0]];
    defRoad=1;
    plot_window()
    msgbox('Estas são todas as nossas estradas!','Pedros!','custom',imread('pedros3.png'))
elseif defRoad==5
    savedRoads{l+1}=[roadouterdef(1,:);roadouterdef(2,:)];
    roadInfo=[roadInfo; [length(savedRoads) -3 1 0 length(roadouterdef(1,:)) 1 0 0 0]];
    savedRoads{l+2}=[roadinnerdef(1,:);roadinnerdef(2,:)];
    roadInfo=[roadInfo; [length(savedRoads) -3 2 1 length(roadinnerdef(1,:)) 1 0 0 0]];
    defRoad=1;
    plot_window()
    msgbox('Por um quando você pode ter os dois?','Pedros!','custom',imread('pedros.png'))
elseif defRoad==6
    savedRoads{l+1}=[roadouterdef(1,:);roadouterdef(2,:)];
    roadInfo=[roadInfo; [length(savedRoads) -3 1 0 length(roadouterdef(1,:)) 1 0 0 0]];
    savedRoads{l+2}=[roadinnerdef(1,:);roadinnerdef(2,:)];
    roadInfo=[roadInfo; [length(savedRoads) -3 2 1 length(roadinnerdef(1,:)) 1 0 0 0]];
    savedRoads{l+3}=[roadinnroaddef(1,:);roadinnroaddef(2,:)];
    roadInfo=[roadInfo; [length(savedRoads) -1 1 0 length(roadinnroaddef(1,:)) 1 0 0 0]];
    defRoad=1;
    plot_window()
    msgbox('Estas são todas as nossas estradas!','Pedros!','custom',imread('pedros3.png'))
else
    msgbox('You need to chose a default road to display before saving a default road.','Critical Mass Acquired','custom',imread('cmass.png'))
end

% --- Executes on button press in Save_Project_button.
function Save_Project_button_Callback(hObject, eventdata, handles)
% hObject    handle to Save_Project_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global path savedRoads savedLUT roadInfo TM TLInfo...
    TLArray TSInfo picture TransitionMatrix pedestrianRoads pCross

path_save=path;
path = uigetdir(pwd,'Select Project Folder');
if path==0
    path=path_save;
else
    
    
    if exist(strcat(path,'\','TLInfo.txt'),'file')~=0
        delete(strcat(path,'\','TLInfo.txt'))
    end
    if exist(strcat(path,'\','TransitionMatrix.txt'),'file')~=0
        delete(strcat(path,'\','TransitionMatrix.txt'))
    end
    if exist(strcat(path,'\','ManualTransitionMatrix.txt'),'file')~=0
        delete(strcat(path,'\','ManualTransitionMatrix.txt'))
    end
    if exist(strcat(path,'\','savedLUT.txt'),'file')~=0
        delete(strcat(path,'\','savedLUT.txt'))
    end
    if exist(strcat(path,'\','savedRoads.txt'),'file')~=0
        delete(strcat(path,'\','savedRoads.txt'))
    end
    if exist(strcat(path,'\','roadInfo.txt'),'file')~=0
        delete(strcat(path,'\','roadInfo.txt'))
    end
    if exist(strcat(path,'\','TLArray.txt'),'file')~=0
        delete(strcat(path,'\','TLArray.txt'))
    end
    if exist(strcat(path,'\','SignInfo.txt'),'file')~=0
        delete(strcat(path,'\','SignInfo.txt'))
    end
    
    col=max(cellfun('length',TLArray));
    if col==0
        col=1;
    end
    TLMatrix=zeros(10,col);
    for i=1:length(TLMatrix(:,1))
        h=length(TLArray{i});
        if h~=0
            TLMatrix(i,1:h)=TLArray{i};
        end
    end
    if ~isempty(roadInfo)
        rows=sum(roadInfo(:,6))*2;
        cols=max(roadInfo(:,5));
        saveMatrix=1000*ones(rows,cols);
        j=1;
        for i=1:length(savedRoads)
            if length(savedRoads{i}(:,1))==2
                savedLUT=LUT_Maker(savedRoads{i}(1,:),savedRoads{i}(2,:));
            else
                LUT1=LUT_Maker(savedRoads{i}(1,:),savedRoads{i}(2,:));
                LUT2=LUT_Maker(savedRoads{i}(3,:),savedRoads{i}(4,:));
                savedLUT=[LUT1;LUT2];
            end
            dlmwrite(strcat(path,'\','savedLUT.txt'), savedLUT,'-append','delimiter','\t','precision','%f')
            if roadInfo(i,6)==2
                saveMatrix(j:j+3,1:roadInfo(i,5))=savedRoads{i};
                j=j+4;
                length(savedRoads{i}(1,:))
            elseif roadInfo(i,6)==1
                roadInfo(:,5)
                length(savedRoads{i}(1,:))
                saveMatrix(j:j+1,1:roadInfo(i,5))=savedRoads{i};
                j=j+2;
            end
        end
    else
        saveMatrix=[];
    end
    
    PRmatrix=[];
    if ~isempty(pedestrianRoads)
        for hh=1:length(pedestrianRoads)
            if hh==1
                PRmatrix=pedestrianRoads{hh};
            else
                l1=length(PRmatrix(1,:));
                l2=length(pedestrianRoads{hh});
                if l1==l2
                    PRmatrix=[PRmatrix; pedestrianRoads{hh}];
                elseif l1<l2
                    PRmatrix=[[PRmatrix 1000*ones(2,l2-l1)]; pedestrianRoads{hh}];
                else
                    PRmatrix=[PRmatrix; [pedestrianRoads{hh}  1000*ones(2,l1-l2)]];
                end
            end
        end
    end
    
    if ~isempty(roadInfo)
        TransitionMatrix=find_transition(savedRoads,roadInfo,TM);
        %         TransitionMatrix                          % For debugging
        %         if ~isempty(TransitionMatrix)
        %             for i=1:length(TransitionMatrix(2,:))
        %
        %
        %                 if TransitionMatrix(1,i)<0
        %                 plot(savedRoads{abs(TransitionMatrix(1,i))}(end-1,abs(TransitionMatrix(2,i))),savedRoads{abs(TransitionMatrix(1,i))}(end,TransitionMatrix(2,i)),'*g')
        %                 hold on
        %
        %                 plot(savedRoads{TransitionMatrix(3,i)}(end-1,TransitionMatrix(4,i)),savedRoads{TransitionMatrix(3,i)}(end,TransitionMatrix(4,i)),'*r')
        %                 hold on
        %                 else
        %                 plot(savedRoads{abs(TransitionMatrix(1,i))}(1,abs(TransitionMatrix(2,i))),savedRoads{TransitionMatrix(1,i)}(2,TransitionMatrix(2,i)),'*g')
        %                 hold on
        %
        %                 plot(savedRoads{TransitionMatrix(3,i)}(1,TransitionMatrix(4,i)),savedRoads{TransitionMatrix(3,i)}(2,TransitionMatrix(4,i)),'*r')
        %                 hold on
        %                 end
        %             end
        %         end
        
    else
        TransitionMatrix=[];
    end
    
    picture=1;
    plot_window()
    dlmwrite(strcat(path,'\','savedRoads.txt'), saveMatrix,'delimiter','\t','precision','%f')
    dlmwrite(strcat(path,'\','roadInfo.txt'), roadInfo,'delimiter','\t','precision','%.f')
    dlmwrite(strcat(path,'\','TransitionMatrix.txt'), TransitionMatrix,'delimiter','\t','precision','%f')
    dlmwrite(strcat(path,'\','ManualTransitionMatrix.txt'), TM,'delimiter','\t','precision','%f')
    dlmwrite(strcat(path,'\','TLInfo.txt'), TLInfo,'delimiter','\t','precision','%f')
    dlmwrite(strcat(path,'\','TLMatrix.txt'), TLMatrix,'delimiter','\t','precision','%f')
    dlmwrite(strcat(path,'\','SignInfo.txt'), TSInfo,'delimiter','\t','precision','%f')
    dlmwrite(strcat(path,'\','PedestrianCrossingInfo.txt'), pCross,'delimiter','\t','precision','%f')
    dlmwrite(strcat(path,'\','PedestrianRoads.txt'), PRmatrix,'delimiter','\t','precision','%f')
    
    msgbox('Save successful.','Meltdown Avoided!','custom',imread('jonas.jpg'))
end

% --- Executes on button press in Load_project_button.
function Load_project_button_Callback(hObject, eventdata, handles)
% hObject    handle to Load_project_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global savedRoads roadInfo TM TLInfo TLArray TSInfo path savedTL
path_save=path;
path = uigetdir(pwd,'Select Project Folder');
if path==0
    path=path_save;
else
    [savedRoads,roadInfo,TLInfo,TM,TLArray,TSInfo,savedTL]=load_project(path);
    set(handles.wiring_table,'Data',cell2matrix(TLArray),'ColumnFormat',{'numeric'})
    plot_window()
end

% --- Executes on button press in delete_all_button.
function delete_all_button_Callback(hObject, eventdata, handles)
% hObject    handle to delete_all_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global xvec yvec xO yO xI yI xC yC savedRoads saved xb yb ...
    TLInfo xbvec ybvec roadInfo savedNames TM savedLUT BRNum1 BRNum2...
    xB1 yB1 xB2 yB2 TLArray savedTL TSInfo SI1 SI2 EI1 EI2...
    xO1 yO1 xI1 yI1 xO2 yO2 xI2 yI2 xNO yNO xNI yNI xNC yNC xP yP...
    pedestrianRoads pCross


xvec=[];
yvec=[];
xO=[];
yO=[];
xI=[];
yI=[];
xC=[];
yC=[];
xP=[];
yP=[];
savedRoads={};
pedestrianRoads={};
pCross=[];
saved=false;
xb=[];
yb=[];
savedNames={};
TLInfo=[];
xbvec=[];
ybvec=[];
roadInfo=[];
TM=[];
savedLUT={};
BRNum1=[];
BRNum2=[];
xB1=[];
yB1=[];
xB2=[];
yB2=[];
xO1=[]; yO1=[]; xI1=[]; yI1=[]; xO2=[]; yO2=[]; xI2=[]; yI2=[]; xNO=[]; yNO=[]; xNI=[]; yNI=[];xNC=[]; yNC=[];
SI1=0; SI2=0; EI1=0; EI2=0;
set(handles.Bridge_road_num1,'String','')
set(handles.Bridge_road_num2,'String','')
set(handles.TL_number_textbox,'String','')
set(handles.roadnum_selector_textbox,'String','0')
TLArray={[] [] [] [] [] [] [] [] [] []};
savedTL=[];
TSInfo=[];
set(handles.wiring_table,'Data',cell2matrix(TLArray),'ColumnFormat',{'numeric'})
plot_window()

% --------------------------------------------------------------------
function calib_ClickedCallback(hObject, eventdata, handles)
% hObject    handle to calib (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if exist([pwd,'/../CalibrationTool/calibration_tool.m'],'file')
    run([pwd,'/../CalibrationTool/calibration_tool.m']);
else
    msgbox('Calibration script not found.','Missing File','custom',imread('oops.png'));
end
