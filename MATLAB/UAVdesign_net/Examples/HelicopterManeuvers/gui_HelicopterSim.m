function varargout = gui_HelicopterSim(varargin)
% GUI_HELICOPTERSIM M-file for gui_HelicopterSim.fig
%      GUI_HELICOPTERSIM, by itself, creates a new GUI_HELICOPTERSIM or raises the existing
%      singleton*.
%
%      H = GUI_HELICOPTERSIM returns the handle to a new GUI_HELICOPTERSIM or the handle to
%      the existing singleton*.
%
%      GUI_HELICOPTERSIM('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in GUI_HELICOPTERSIM.M with the given input arguments.
%
%      GUI_HELICOPTERSIM('Property','Value',...) creates a new GUI_HELICOPTERSIM or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before gui_HelicopterSim_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to gui_HelicopterSim_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help gui_HelicopterSim

% Last Modified by GUIDE v2.5 27-Sep-2011 09:52:48

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @gui_HelicopterSim_OpeningFcn, ...
                   'gui_OutputFcn',  @gui_HelicopterSim_OutputFcn, ...
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


% --- Executes just before gui_HelicopterSim is made visible.
function gui_HelicopterSim_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to gui_HelicopterSim (see VARARGIN)

% Choose default command line output for gui_HelicopterSim
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes gui_HelicopterSim wait for user response (see UIRESUME)
% uiwait(handles.figure1);
a = evalin('base', 'exist(''helicopter_sim.mat'', ''file'');'); 
if a
    evalin('base', 'load helicopter_sim.mat');
    
    a = evalin('base', 'in_omega_MR;');             set(handles.rpm_mr,             'String', mat2str(a));
    a = evalin('base', 'in_omega_TR;');             set(handles.rpm_tr,             'String', mat2str(a));
    a = evalin('base', 'in_pressure_altitude;');    set(handles.pressure_altitude,  'String', mat2str(a));
    a = evalin('base', 'in_outside_air_temp;');     set(handles.outside_air_temp,   'String', mat2str(a));
    a = evalin('base', 'in_CMD_ini;');
    set(handles.collective,   'String', mat2str(a(1)));
    set(handles.stick_a1,     'String', mat2str(a(2)));
    set(handles.stick_b1,     'String', mat2str(a(3)));
    set(handles.pedals, 'String', mat2str(a(4)));
    
    a = evalin('base', 'in_xc_EOM_ini;');
    b = evalin('base', 'in_d_ini;');
    a = reshape([a; b], 3, 9)';
    data = get(handles.table_ini_states, 'Data');
    for i=1:9
        for j=1:3
            data{i, j} = a(i, j);
        end
    end
    set(handles.table_ini_states, 'Data', data);
    
    a = evalin('base', 'in_cmd_step_time;');
    b = evalin('base', 'in_cmd_final_value;');
    a = reshape([b; a], 4, 2);
    set(handles.table_maneuver, 'Data', a);
end



% --- Outputs from this function are returned to the command line.
function varargout = gui_HelicopterSim_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function pressure_altitude_Callback(hObject, eventdata, handles)
% hObject    handle to pressure_altitude (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of pressure_altitude as text
%        str2double(get(hObject,'String')) returns contents of pressure_altitude as a double


% --- Executes during object creation, after setting all properties.
function pressure_altitude_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pressure_altitude (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function outside_air_temp_Callback(hObject, eventdata, handles)
% hObject    handle to outside_air_temp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of outside_air_temp as text
%        str2double(get(hObject,'String')) returns contents of outside_air_temp as a double


% --- Executes during object creation, after setting all properties.
function outside_air_temp_CreateFcn(hObject, eventdata, handles)
% hObject    handle to outside_air_temp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function collective_Callback(hObject, eventdata, handles)
% hObject    handle to collective (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of collective as text
%        str2double(get(hObject,'String')) returns contents of collective as a double


% --- Executes during object creation, after setting all properties.
function collective_CreateFcn(hObject, eventdata, handles)
% hObject    handle to collective (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function stick_a1_Callback(hObject, eventdata, handles)
% hObject    handle to stick_a1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of stick_a1 as text
%        str2double(get(hObject,'String')) returns contents of stick_a1 as a double


% --- Executes during object creation, after setting all properties.
function stick_a1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to stick_a1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function stick_b1_Callback(hObject, eventdata, handles)
% hObject    handle to stick_b1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of stick_b1 as text
%        str2double(get(hObject,'String')) returns contents of stick_b1 as a double


% --- Executes during object creation, after setting all properties.
function stick_b1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to stick_b1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function pedals_Callback(hObject, eventdata, handles)
% hObject    handle to pedals (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of pedals as text
%        str2double(get(hObject,'String')) returns contents of pedals as a double


% --- Executes during object creation, after setting all properties.
function pedals_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pedals (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function rpm_mr_Callback(hObject, eventdata, handles)
% hObject    handle to rpm_mr (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of rpm_mr as text
%        str2double(get(hObject,'String')) returns contents of rpm_mr as a double


% --- Executes during object creation, after setting all properties.
function rpm_mr_CreateFcn(hObject, eventdata, handles)
% hObject    handle to rpm_mr (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function rpm_tr_Callback(hObject, eventdata, handles)
% hObject    handle to rpm_tr (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of rpm_tr as text
%        str2double(get(hObject,'String')) returns contents of rpm_tr as a double


% --- Executes during object creation, after setting all properties.
function rpm_tr_CreateFcn(hObject, eventdata, handles)
% hObject    handle to rpm_tr (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in button_configure_parameters.
function button_configure_parameters_Callback(hObject, eventdata, handles)
% hObject    handle to button_configure_parameters (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in button_ok.
function button_ok_Callback(hObject, eventdata, handles)
% hObject    handle to button_ok (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

evalin('base', ['in_omega_MR='          get(handles.rpm_mr,             'String') ';']);
evalin('base', ['in_omega_TR='          get(handles.rpm_tr,             'String') ';']);
evalin('base', ['in_pressure_altitude=' get(handles.pressure_altitude,  'String') ';']);
evalin('base', ['in_outside_air_temp='  get(handles.outside_air_temp,   'String') ';']);
evalin('base', ['in_CMD_ini=['          get(handles.collective,         'String') ';' ...
                                        get(handles.stick_a1,           'String') ';' ...
                                        get(handles.stick_b1,           'String') ';' ...
                                        get(handles.pedals,             'String') '];']);

data = get(handles.table_ini_states, 'Data');
data = data';
xc_EOM = [data{1:3, 1:4}]';
d =      [data{1:3, 5:9}]';
evalin('base', ['in_xc_EOM_ini='        mat2str(xc_EOM) ';']);
evalin('base', ['in_d_ini='             mat2str(d)      ';']);

data = get(handles.table_maneuver, 'Data');
cmd_step_time =   data(1:4, 2);
cmd_final_value = data(1:4, 1);
evalin('base', ['in_cmd_step_time='     mat2str(cmd_step_time)   ';']);
evalin('base', ['in_cmd_final_value='   mat2str(cmd_final_value) ';']);
evalin('base', 'save helicopter_sim.mat in_omega_MR in_omega_TR in_pressure_altitude in_outside_air_temp in_CMD_ini in_xc_EOM_ini in_d_ini in_cmd_step_time in_cmd_final_value');
delete(handles.figure1);


% --- Executes on button press in button_cancel.
function button_cancel_Callback(hObject, eventdata, handles)
% hObject    handle to button_cancel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
delete(handles.figure1);
