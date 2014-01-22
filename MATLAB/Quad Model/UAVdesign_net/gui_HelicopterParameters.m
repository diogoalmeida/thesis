function varargout = gui_HelicopterParameters(varargin)
% GUI_HELICOPTERPARAMETERS M-file for gui_HelicopterParameters.fig
%      GUI_HELICOPTERPARAMETERS, by itself, creates a new GUI_HELICOPTERPARAMETERS or raises the existing
%      singleton*.
%
%      H = GUI_HELICOPTERPARAMETERS returns the handle to a new GUI_HELICOPTERPARAMETERS or the handle to
%      the existing singleton*.
%
%      GUI_HELICOPTERPARAMETERS('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in GUI_HELICOPTERPARAMETERS.M with the given input arguments.
%
%      GUI_HELICOPTERPARAMETERS('Property','Value',...) creates a new GUI_HELICOPTERPARAMETERS or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before gui_HelicopterParameters_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to gui_HelicopterParameters_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help gui_HelicopterParameters

% Last Modified by GUIDE v2.5 29-Nov-2008 12:04:20

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @gui_HelicopterParameters_OpeningFcn, ...
                   'gui_OutputFcn',  @gui_HelicopterParameters_OutputFcn, ...
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


% --- Executes just before gui_HelicopterParameters is made visible.
function gui_HelicopterParameters_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to gui_HelicopterParameters (see VARARGIN)

% Choose default command line output for gui_HelicopterParameters
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes gui_HelicopterParameters wait for user response (see UIRESUME)
% uiwait(handles.helicopter);
if nargin >= 5 && ischar(varargin{1}) && strcmp(varargin{1}, 'Block')
    if ischar(varargin{2})
        handles.calling_block = varargin{2};
    end
end

heli.units = 1;
heli.gw = 20000;
heli.inertia = [5000 0 0; 0 35000 0; 0 0 35000];
heli.geometry = [336   0   72   0  0  0; ...
                 340   0  162   0  0  0; ...
                 780  12  150  90  0  0; ...
                 340   0  114   0  0  0; ...
                 680   0   80   0 -4  0; ...
                 780   0  114 -90  0  4 ];

heli.MR_param = {'Lift-curve slope',        'a0',       0,      'CL/rad',   'CL/rad';
                 'Drag polar 0',            'del0',     0,      'CD',       'CD';
                 'Drag polar 1',            'del1'      0,      'CD/CL',    'CD/CL';
                 'Drag polar 2',            'del2',     0,      'CD/CL^2',  'CD/CL^2';
                 'Precone angle',           'precone',  0,      'deg',      'deg';
                 'Delta 3',                 'delta3',   0,      'deg',      'deg';
                 'Flapping hinge offset',   'e',        0,      'm',        'ft';
                 'K beta',                  'khub',     0,      'N-m/deg',  'ft-lb/deg';
                 'Number of blades',        'nb',       4,      '--',       '--';
                 'Radius',                  'radius',   0,      'm',        'ft';
                 'Chord',                   'chord',    0,      'm',        'ft';
                 'Tip loss factor (min)',   'btmin',    0,      '--',       '--';
                 'Tip loss factor (max)',   'btmax',    0,      '--',       '--';
                 'Torque BT (0=prev. settings)', 'btq', 1,      '--',       '--';
                 'Twist',                   'twist',    0,      'deg',      'deg';
                 'Blade flapping inertia',  'iflap',    0,      'kg-m^2',   'slug-ft^2';
                 '    or  Distributed mass',        'mbar',     0,      'kg/m',     'slugs/ft';
                 'Mast height',             'lmast',    0,      'm',        'ft'};
heli.MR_hub_type = 1;
heli.MR_dir_rotation = -1;

heli.TR_param = heli.MR_param;
heli.TR_hub_type = 1;
heli.TR_dir_rotation = -1;

heli.FUSE_param = {'Carea',                 'carea',    0,      'm^2',      'ft^2';
                 'Clen',                    'clen',     0,      'm',        'ft';
                 'CXuu',                    'cxu',      0,      'm^2',      'ft^2';
                 'CXvu',                    'cxv',      0,      'm^2',      'ft^2';
                 'CXwu',                    'cxw',      0,      'm^2',      'ft^2';
                 'CYuv',                    'cyu',      0,      'm^2',      'ft^2';
                 'CYvv',                    'cyv',      0,      'm^2',      'ft^2';
                 'CYwv',                    'cyw',      0,      'm^2',      'ft^2';
                 'CZuw',                    'czu',      0,      'm^2',      'ft^2';
                 'CZvw',                    'czv',      0,      'm^2',      'ft^2';
                 'CZww',                    'czw',      0,      'm^2',      'ft^2';
                 'CMuu',                    'cmu',      0,      'm^3',      'ft^3';
                 'CMuv',                    'cmv',      0,      'm^3',      'ft^3';
                 'CMuw',                    'cmw',      0,      'm^3',      'ft^3';
                 'CNuu',                    'cnu',      0,      'm^3',      'ft^3';
                 'CNuv',                    'cnv',      0,      'm^3',      'ft^3';
                 'CNuw',                    'cnw',      0,      'm^3',      'ft^3'};

heli.HW_param = {'Wing span',               'span',     0,      'm',        'ft';
                 'Chord',                   'chord',    0,      'm',        'ft';
                 'Dihedral',                'dihedral', 0,      'deg',      'deg';
                 'Incidence angle',         'incid',    0,      'deg',      'deg';
                 'Sweep',                   'sweep',    0,      'deg',      'deg';
                 'Taper',                   'taper',    0,      'm^2',      'ft^2';
                 'Stall angle',             'stall',    0,      'deg',      'deg';
                 'Lift-curve slope before stall', 'a0', 0,      'CL/rad',   'CL/rad';
                 '  Coef. for sin(2*alpha)','a1s2',     0,      'CL',       'CL';
                 'Drag coef. (free term)',  'cd0',      0,      'CD',       'CD';
                 '  Coef. for sin(alpha)',  'cd1s1',    0,      'CD',       'CD';
                 '  Coef. for cos(alpha)^2','cd2c1',    0,      'CD',       'CD';
                 'Moment coef. (free term)','cm0',      0,      'm^-1',     'ft^-1';
                 '  Coef. for sin(alpha)',  'cm1s1',    0,      'm^-1',     'ft^-1'};

heli.VF_param = heli.HW_param;

if isfield(handles,'calling_block')
    A = eval( get_param([handles.calling_block '/param_MR'], 'Value') );
    if size(A,1) == (size(heli.MR_param, 1) + 2) && size(A,2) == 1
        for i=1:size(heli.MR_param, 1)
            heli.MR_param{i,3} = A(i);
        end
        heli.MR_hub_type = A(length(A)-1);
        heli.MR_dir_rotation = A(length(A));
    end
    
    A = eval( get_param([handles.calling_block '/param_TR'], 'Value') );
    if size(A,1) == (size(heli.TR_param, 1) + 2) && size(A,2) == 1
        for i=1:size(heli.TR_param, 1)
            heli.TR_param{i,3} = A(i);
        end
        heli.TR_hub_type = A(length(A)-1);
        heli.TR_dir_rotation = A(length(A));
    end
    
    A = eval( get_param( [handles.calling_block '/param_FUSE'], 'Value') );
    if size(A,1) == size(heli.FUSE_param, 1) && size(A,2) == 1
        for i=1:size(heli.FUSE_param, 1)
            heli.FUSE_param{i,3} = A(i);
        end
    end
    
    A = eval( get_param( [handles.calling_block '/param_HW'], 'Value') );
    if size(A,1) == size(heli.HW_param, 1) && size(A,2) == 1
        for i=1:size(heli.HW_param, 1)
            heli.HW_param{i,3} = A(i);
        end
    end
    
    A = eval( get_param( [handles.calling_block '/param_VF'], 'Value') );
    if size(A,1) == size(heli.VF_param, 1) && size(A,2) == 1
        for i=1:size(heli.VF_param, 1)
            heli.VF_param{i,3} = A(i);
        end
    end
    
    A = eval( get_param( [handles.calling_block '/param_gw'], 'Value') );
    if length(A) == 1
        heli.gw = A;
    end
    
    A = eval( get_param( [handles.calling_block '/param_inertia'], 'Value') );
    if size(A) == size(heli.inertia)
        heli.inertia = A;
    end
    
    A = eval( get_param( [handles.calling_block '/geom_CG'], 'Value') )';
    if size(A) == [1 6]
        heli.geometry(1, :) = A;
    end
    
    A = eval( get_param( [handles.calling_block '/geom_MR'], 'Value') )';
    if size(A) == [1 6]
        heli.geometry(2, :) = A;
    end
    
    A = eval( get_param( [handles.calling_block '/geom_TR'], 'Value') )';
    if size(A) == [1 6]
        heli.geometry(3, :) = A;
    end
    
    A = eval( get_param( [handles.calling_block '/geom_FUSE'], 'Value') )';
    if size(A) == [1 6]
        heli.geometry(4, :) = A;
    end
    
    A = eval( get_param( [handles.calling_block '/geom_HW'], 'Value') )';
    if size(A) == [1 6]
        heli.geometry(5, :) = A;
    end
    
    A = eval( get_param( [handles.calling_block '/geom_VF'], 'Value') )';
    if size(A) == [1 6]
        heli.geometry(6, :) = A;
    end
    
    A = eval( get_param( [handles.calling_block '/param_units'], 'Value') );
    if length(A) == 1
        heli.units = A;
    end
end
    
handles.old_page = 1;
handles.heli = heli;
guidata(hObject, handles);

set(handles.table_parameters, 'Data', heli.MR_param(:, [1 2 3 heli.units+4]));
set(handles.radio_teetering, 'Value', 1-heli.MR_hub_type);
set(handles.radio_articulated, 'Value', heli.MR_hub_type);
set(handles.radio_CW, 'Value', (handles.heli.MR_dir_rotation + 1)/2);
set(handles.radio_CCW, 'Value', 1-(handles.heli.MR_dir_rotation + 1)/2);
if isfield(handles,'calling_block')
    set(handles.popup_units, 'Value', heli.units + 1);
    set(handles.edit_weight, 'String', mat2str(heli.gw));
    set(handles.inertia, 'Data', heli.inertia);
    set(handles.geometry, 'Data', heli.geometry);
    popup_units_Callback(hObject, eventdata, handles);
end

% --- Outputs from this function are returned to the command line.
function varargout = gui_HelicopterParameters_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on selection change in popup_units.
function popup_units_Callback(hObject, eventdata, handles)
% hObject    handle to popup_units (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = get(hObject,'String') returns popup_units contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popup_units

switch get(handles.popup_units, 'Value')
    case 1
        handles.heli.units = 0;
        units = 0;
        set(handles.units1, 'String', 'Kg');
        set(handles.units2, 'String', 'N-m^2');
        set(handles.units3, 'String', '-----  m  -----');
    case 2
        handles.heli.units = 1;
        units = 1;
        set(handles.units1, 'String', 'lbs');
        set(handles.units2, 'String', 'slug-ft^2');
        set(handles.units3, 'String', '-----  in  -----');
end
switch handles.old_page
    case 1
        set(handles.table_parameters, 'Data', handles.heli.MR_param(:, [1 2 3 units+4]));
    case 2
        set(handles.table_parameters, 'Data', handles.heli.TR_param(:, [1 2 3 units+4]));
    case 3
        set(handles.table_parameters, 'Data', handles.heli.FUSE_param(:, [1 2 3 units+4]));
    case 4
        set(handles.table_parameters, 'Data', handles.heli.HW_param(:, [1 2 3 units+4]));
    case 5
        set(handles.table_parameters, 'Data', handles.heli.VF_param(:, [1 2 3 units+4]));
end
guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function popup_units_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popup_units (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_weight_Callback(hObject, eventdata, handles)
% hObject    handle to edit_weight (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_weight as text
%        str2double(get(hObject,'String')) returns contents of edit_weight as a double


% --- Executes during object creation, after setting all properties.
function edit_weight_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_weight (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popup_parameters.
function popup_parameters_Callback(hObject, eventdata, handles)
% hObject    handle to popup_parameters (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = get(hObject,'String') returns popup_parameters contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popup_parameters

units = handles.heli.units;

switch handles.old_page
    case 1
        data = get(handles.table_parameters, 'Data');
        handles.heli.MR_param(:, 3) = data(:, 3);
        handles.heli.MR_hub_type = get(handles.radio_articulated, 'Value');
        handles.heli.MR_dir_rotation = get(handles.radio_CW, 'Value') * 2 - 1;
    case 2
        data = get(handles.table_parameters, 'Data');
        handles.heli.TR_param(:, 3) = data(:, 3);
        handles.heli.TR_hub_type = get(handles.radio_articulated, 'Value');
        handles.heli.TR_dir_rotation = get(handles.radio_CW, 'Value') * 2 - 1;
    case 3
        data = get(handles.table_parameters, 'Data');
        handles.heli.FUSE_param(:, 3) = data(:, 3);
    case 4
        data = get(handles.table_parameters, 'Data');
        handles.heli.HW_param(:, 3) = data(:, 3);
    case 5
        data = get(handles.table_parameters, 'Data');
        handles.heli.VF_param(:, 3) = data(:, 3);
end

handles.old_page = get(hObject, 'Value');
switch handles.old_page;
    case 1
        set(handles.table_parameters, 'Data', handles.heli.MR_param(:, [1 2 3 units+4]));
        set(handles.radio_articulated, 'Value', handles.heli.MR_hub_type);
        set(handles.radio_teetering, 'Value', 1-handles.heli.MR_hub_type);
        set(handles.radio_CCW, 'Value', (handles.heli.MR_dir_rotation <= 0));
        set(handles.radio_CW, 'Value', (handles.heli.MR_dir_rotation > 0));
        set(handles.hub_type, 'Visible', 'on');
        set(handles.dir_rotation, 'Visible', 'on');
    case 2
        set(handles.table_parameters, 'Data', handles.heli.TR_param(:, [1 2 3 units+4]));
        set(handles.radio_articulated, 'Value', handles.heli.TR_hub_type);
        set(handles.radio_teetering, 'Value', 1-handles.heli.TR_hub_type);
        set(handles.radio_CCW, 'Value', (handles.heli.TR_dir_rotation <= 0));
        set(handles.radio_CW, 'Value', (handles.heli.TR_dir_rotation > 0));
        set(handles.hub_type, 'Visible', 'on');
        set(handles.dir_rotation, 'Visible', 'on');
    case 3
        set(handles.table_parameters, 'Data', handles.heli.FUSE_param(:, [1 2 3 units+4]));
        set(handles.hub_type, 'Visible', 'off');
        set(handles.dir_rotation, 'Visible', 'off');
    case 4
        set(handles.table_parameters, 'Data', handles.heli.HW_param(:, [1 2 3 units+4]));
        set(handles.hub_type, 'Visible', 'off');
        set(handles.dir_rotation, 'Visible', 'off');
    case 5
        set(handles.table_parameters, 'Data', handles.heli.VF_param(:, [1 2 3 units+4]));
        set(handles.hub_type, 'Visible', 'off');
        set(handles.dir_rotation, 'Visible', 'off');
end

guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function popup_parameters_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popup_parameters (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in button_OK.
function button_OK_Callback(hObject, eventdata, handles)
% hObject    handle to button_OK (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

switch handles.old_page
    case 1
        data = get(handles.table_parameters, 'Data');
        handles.heli.MR_param(:, 3) = data(:, 3);
        handles.heli.MR_hub_type = get(handles.radio_articulated, 'Value');
        handles.heli.MR_dir_rotation = get(handles.radio_CW, 'Value') * 2 - 1;
    case 2
        data = get(handles.table_parameters, 'Data');
        handles.heli.TR_param(:, 3) = data(:, 3);
        handles.heli.TR_hub_type = get(handles.radio_articulated, 'Value');
        handles.heli.TR_dir_rotation = get(handles.radio_CW, 'Value') * 2 - 1;
    case 3
        data = get(handles.table_parameters, 'Data');
        handles.heli.FUSE_param(:, 3) = data(:, 3);
    case 4
        data = get(handles.table_parameters, 'Data');
        handles.heli.HW_param(:, 3) = data(:, 3);
    case 5
        data = get(handles.table_parameters, 'Data');
        handles.heli.VF_param(:, 3) = data(:, 3);
end

if isfield(handles,'calling_block')
    set_param( [handles.calling_block '/param_MR'], 'Value', ...
        mat2str([[handles.heli.MR_param{:,3}]'; handles.heli.MR_hub_type; handles.heli.MR_dir_rotation]) );
    set_param( [handles.calling_block '/param_TR'], 'Value', ...
        mat2str([[handles.heli.TR_param{:,3}]'; handles.heli.TR_hub_type; handles.heli.TR_dir_rotation]) );
    set_param( [handles.calling_block '/param_FUSE'], 'Value', ...
        mat2str([handles.heli.FUSE_param{:,3}]') );
    set_param( [handles.calling_block '/param_HW'], 'Value', ...
        mat2str([handles.heli.HW_param{:,3}]') );
    set_param( [handles.calling_block '/param_VF'], 'Value', ...
        mat2str([handles.heli.VF_param{:,3}]') );
    set_param( [handles.calling_block '/param_gw'], 'Value', ...
        get(handles.edit_weight, 'String') );
    set_param( [handles.calling_block '/param_inertia'], 'Value', ...
        mat2str(get(handles.inertia, 'Data')) );
    geometry = get(handles.geometry, 'Data')';
    set_param( [handles.calling_block '/geom_CG'],   'Value', mat2str(geometry(:,1)) );
    set_param( [handles.calling_block '/geom_MR'],   'Value', mat2str(geometry(:,2)) );
    set_param( [handles.calling_block '/geom_TR'],   'Value', mat2str(geometry(:,3)) );
    set_param( [handles.calling_block '/geom_FUSE'], 'Value', mat2str(geometry(:,4)) );
    set_param( [handles.calling_block '/geom_HW'],   'Value', mat2str(geometry(:,5)) );
    set_param( [handles.calling_block '/geom_VF'],   'Value', mat2str(geometry(:,6)) );
    set_param( [handles.calling_block '/param_units'],'Value', mat2str(handles.heli.units) );

%    handles.output = [];
%    handles.output.units = handles.heli.units;
%    handles.output.gw = str2double(get(handles.edit_weight, 'String'));
%    handles.output.inertia = get(handles.inertia, 'Data');
%    handles.output.geometry = get(handles.geometry, 'Data');
%    handles.output.MR = [[handles.heli.MR_param{:,3}]'; handles.heli.MR_hub_type; handles.heli.MR_dir_rotation];
%    handles.output.TR = [[handles.heli.TR_param{:,3}]'; handles.heli.TR_hub_type; handles.heli.TR_dir_rotation];
%    handles.output.FUSE = [handles.heli.FUSE_param{:,3}]';
%    handles.output.HW = [handles.heli.HW_param{:,3}]';
%    handles.output.VF = [handles.heli.VF_param{:,3}]';
%    guidata(hObject, handles);
end
delete(handles.helicopter);


% --- Executes on button press in button_cancel.
function button_cancel_Callback(hObject, eventdata, handles)
% hObject    handle to button_cancel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

delete(handles.helicopter);


% --- Executes during object creation, after setting all properties.
function helicopter_CreateFcn(hObject, eventdata, handles)
% hObject    handle to helicopter (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
