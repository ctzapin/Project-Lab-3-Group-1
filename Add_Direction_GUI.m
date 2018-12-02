function varargout = Add_Direction_GUI(varargin)
% ADD_DIRECTION_GUI MATLAB code for Add_Direction_GUI.fig
%      ADD_DIRECTION_GUI, by itself, creates a new ADD_DIRECTION_GUI or raises the existing
%      singleton*.
%
%      H = ADD_DIRECTION_GUI returns the handle to a new ADD_DIRECTION_GUI or the handle to
%      the existing singleton*.
%
%      ADD_DIRECTION_GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in ADD_DIRECTION_GUI.M with the given input arguments.
%
%      ADD_DIRECTION_GUI('Property','Value',...) creates a new ADD_DIRECTION_GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Add_Direction_GUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Add_Direction_GUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Add_Direction_GUI

% Last Modified by GUIDE v2.5 02-Nov-2018 18:06:41

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Add_Direction_GUI_OpeningFcn, ...
                   'gui_OutputFcn',  @Add_Direction_GUI_OutputFcn, ...
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


% --- Executes just before Add_Direction_GUI is made visible.
function Add_Direction_GUI_OpeningFcn(hObject, eventdata, handles, varargin)

handles.output = hObject;

set(handles.Pop_up_Menu,'string',{'Forward','Left','Right'});
set(handles.Pop_up_Menu,'value',1);
set(handles.Free_Text,'String',0);
setappdata(0,'Direction_Value',1); % 1 = forward 2 = left 3 = right
setappdata(0,'Duration_Value',0);
set(handles.unit,'string','Centimeters')

guidata(hObject, handles);

function varargout = Add_Direction_GUI_OutputFcn(hObject, eventdata, handles) 
varargout{1} = handles.output;


% --- Executes on selection change in Pop_up_Menu.
function Pop_up_Menu_Callback(hObject, eventdata, handles)
setappdata(0,'Direction_Value',get(handles.Pop_up_Menu,'value'));
if (getappdata(0,'Direction_Value') == 1)
set(handles.unit,'string','Centimeters')
elseif(getappdata(0,'Direction_Value') == 2)
set(handles.unit,'string','Degrees')
else
set(handles.unit,'string','Degrees')
end

function Pop_up_Menu_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Free_Text_Callback(hObject, eventdata, handles)

if (getappdata(0,'Direction_Value') == 1)
    Distance = str2num(get(handles.Free_Text,'string'));
    setappdata(0,'Duration_Value',round(Centimeters_To_Milliseconds(Distance)))
    
end
if (getappdata(0,'Direction_Value') == 2)
    Angle = str2num(get(handles.Free_Text,'string'));
    setappdata(0,'Duration_Value',round(Degrees_To_Milliseconds(Angle)))
end
if (getappdata(0,'Direction_Value') == 3)
    Angle = str2num(get(handles.Free_Text,'string'));
    setappdata(0,'Duration_Value',round(Degrees_To_Milliseconds(Angle)))
end
    

function Free_Text_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in Confirm.
function Confirm_Callback(hObject, eventdata, handles)
%disp(getappdata(0,'Duration_Value'))
%disp(getappdata(0,'Direction_Value'))
closereq



function unit_Callback(hObject, eventdata, handles)



% --- Executes during object creation, after setting all properties.
function unit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to unit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
