function varargout = Main_GUI_Page(varargin)
% MAIN_GUI_PAGE MATLAB code for Main_GUI_Page.fig
%      MAIN_GUI_PAGE, by itself, creates a new MAIN_GUI_PAGE or raises the existing
%      singleton*.
%
%      H = MAIN_GUI_PAGE returns the handle to a new MAIN_GUI_PAGE or the handle to
%      the existing singleton*.
%
%      MAIN_GUI_PAGE('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in MAIN_GUI_PAGE.M with the given input arguments.
%
%      MAIN_GUI_PAGE('Property','Value',...) creates a new MAIN_GUI_PAGE or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Main_GUI_Page_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Main_GUI_Page_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Main_GUI_Page

% Last Modified by GUIDE v2.5 31-Oct-2018 18:59:59

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Main_GUI_Page_OpeningFcn, ...
                   'gui_OutputFcn',  @Main_GUI_Page_OutputFcn, ...
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


% --- Executes just before Main_GUI_Page is made visible.
function Main_GUI_Page_OpeningFcn(hObject, eventdata, handles, varargin)

handles.output = hObject;

%SerialPort = serial('COM5','BaudRate',115200); %defining serial port COM5 at 9600 baud
SerialPort = serial('COM9','BaudRate',115200); %defining serial port COM5 at 9600 baud
setappdata(0,'SerialPort',SerialPort);         %setting this serial port to app data.
pause on                                       % to enable pause function

SerialPort.BytesAvailableFcnCount = 1;
SerialPort.BytesAvailableFcnMode = 'byte';
SerialPort.BytesAvailableFcn = @Serial_Receive;

%setting all of the initial app data values//////////////////////////////
setappdata(0,'CurrentAngleNumber','');
setappdata(0,'CurrentDistanceNumber','');
setappdata(0,'CurrentForwardDistanceNumber','');
setappdata(0,'CurrentLeftDistanceNumber','');
setappdata(0,'CurrentRightDistanceNumber','');
setappdata(0,'AngleReceivingMode',0);
setappdata(0,'DistanceReceivingMode',0);
setappdata(0,'ForwardReceivingMode',0);
setappdata(0,'RightReceivingMode',0);
setappdata(0,'LeftReceivingMode',0);
setappdata(0,'RoverXCoordinate',0);
setappdata(0,'RoverYCoordinate',0);
setappdata(0,'RoverDegrees',90);
setappdata(0,'RealDegrees',0);
setappdata(0,'Xset',[0]);
setappdata(0,'Yset',[0]);
axes(handles.Map_Graph)
setappdata(0,'Map_Graph',handles.Map_Graph)
setappdata(0,'Rover_Direction',handles.Rover_Direction)
setappdata(0,'Y_Coordinate',handles.Y_Coordinate)
setappdata(0,'X_Coordinate',handles.X_Coordinate)

% creating the my colors matrix/////////////////////////////////////////
matrix2 = zeros(size(getappdata(0,'Xset'), 1), 1);
matrix2(1)= 1;
myColors = zeros(size(getappdata(0,'Xset'), 1), 3);
rowsToSetBlue = matrix2 == 0;
rowsToSetRed = matrix2 == 1;

myColors(rowsToSetBlue, 1) = 0;
myColors(rowsToSetBlue, 2) = 0;
myColors(rowsToSetBlue, 3) = 1;

myColors(rowsToSetRed, 1) = 1;
myColors(rowsToSetRed, 2) = 0;
myColors(rowsToSetRed, 3) = 0;

%plotting the initial position of the rover colored red/////////////////
scatter(getappdata(0,'Xset'),getappdata(0,'Yset'), 60, myColors, 'filled');
grid on;

%setting up the Directions array and directions Display/////////////////
Directions_Array = [];
setappdata(0,'Directions_Array',Directions_Array)
Directions_Display_Array = {};
setappdata(0,'Directions_Display_Array',Directions_Display_Array)
set(handles.List_Of_Directions,'string',{});

guidata(hObject, handles);


%might need to remove parameters

function Serial_Receive(sObject, eventdata)
SerialPort = getappdata(0, 'SerialPort');
ByteAvailable = SerialPort.BytesAvailable;
ReadByte = char(fread(SerialPort,1,'uint8'));
% put all that stuff in
if (ReadByte == '0'||ReadByte == '1'||ReadByte == '2'||ReadByte == '3'||ReadByte == '4'||ReadByte == '5'||ReadByte == '6'||ReadByte == '7'||ReadByte == '8'||ReadByte == '9'||ReadByte == 'C'||ReadByte == 'D'||ReadByte == ','||ReadByte == 'F'||ReadByte == 'L'||ReadByte == 'R')

    if (ReadByte == 'C')
        setappdata(0,'AngleReceivingMode',1);
        setappdata(0,'DistanceReceivingMode',0);
        setappdata(0,'ForwardReceivingMode',0);
        setappdata(0,'RightReceivingMode',0);
        setappdata(0,'LeftReceivingMode',0);
        setappdata(0,'CurrentAngleNumber','');
        
    elseif (ReadByte == ',')
        
        if (getappdata(0,'LeftReceivingMode')== 1)
            setappdata(0,'LeftReceivingMode',0);
            LeftAngle = str2num(getappdata(0,'CurrentLeftDistanceNumber'));
            Degrees = Milliseconds_To_Degrees( LeftAngle );
            disp('TurnLeftFor:')
            disp(Degrees)
            New_Rover_Degrees = getappdata(0,'RoverDegrees')+ Degrees;
            if (New_Rover_Degrees >= 360)
                New_Rover_Degrees = New_Rover_Degrees - 360;
            end
            if (New_Rover_Degrees < 0)
                New_Rover_Degrees = 360 + New_Rover_Degrees;
            end
            setappdata(0,'RoverDegrees',New_Rover_Degrees)
            set(getappdata(0,'Rover_Direction'),'string',New_Rover_Degrees)
            return
        end
        
        if (getappdata(0,'RightReceivingMode')== 1)
            setappdata(0,'RightReceivingMode',0);
            RightAngle = str2num(getappdata(0,'CurrentRightDistanceNumber'));
            Degrees = Milliseconds_To_Degrees( RightAngle );
            disp('TurnRightFor:')
            disp(Degrees)
            New_Rover_Degrees = getappdata(0,'RoverDegrees')- Degrees;
            if (New_Rover_Degrees >= 360)
                New_Rover_Degrees = New_Rover_Degrees - 360;
            end
            if (New_Rover_Degrees < 0)
                New_Rover_Degrees = 360 + New_Rover_Degrees;
            end
            setappdata(0,'RoverDegrees',New_Rover_Degrees)
            
            set(getappdata(0,'Rover_Direction'),'string',New_Rover_Degrees)
            return
        end
        
        if (getappdata(0,'ForwardReceivingMode')== 1)
            setappdata(0,'ForwardReceivingMode',0);
            ForwardDistance = str2num(getappdata(0,'CurrentForwardDistanceNumber'));
            Distance = Milliseconds_To_Centimeters( ForwardDistance );
            disp('Move Forward:');
            disp(Distance);
            RoverX = getappdata(0,'RoverXCoordinate');
            RoverY = getappdata(0,'RoverYCoordinate');
            Degrees = getappdata(0,'RoverDegrees');
            if (Degrees == 0)
                
                RoverY = RoverY;
                RoverX = RoverX + Distance;
                
            elseif ((Degrees>0) && (Degrees<90))
                RoverX = RoverX+ (Distance*cosd(Degrees));
                RoverY = RoverY+ (Distance*sind(Degrees));
                
            elseif (Degrees == 90)
                
                RoverX = RoverX; 
                RoverY = RoverY + Distance;
                
            elseif ((Degrees>90) && (Degrees<180))
                
                RoverX = RoverX- (Distance*cosd(180 - Degrees));
                RoverY = RoverY+ (Distance*sind(180 - Degrees));
                
            elseif (Degrees == 180)
                
                RoverX = RoverX - Distance
                
            elseif ((Degrees>180) && (Degrees<270))
                
                RoverX = RoverX - (Distance*cosd(Degrees-180));
                RoverY = RoverY - (Distance*sind(Degrees-180));
                
            elseif (Degrees == 270)
                
                RoverY = RoverY - Distance
                
            elseif ((Degrees>270) && (Degrees<360))
                
                RoverX = RoverX+ (Distance*cosd(360-Degrees));
                RoverY = RoverY- (Distance*sind(360-Degrees));
            end
                
            setappdata(0,'RoverXCoordinate',RoverX);
            setappdata(0,'RoverYCoordinate',RoverY);
            
            set(getappdata(0,'Y_Coordinate'),'string',RoverY)
            set(getappdata(0,'X_Coordinate'),'string',RoverX)
            return
        end
        
        
        
        setappdata(0,'AngleReceivingMode',0);
        setappdata(0,'DistanceReceivingMode',0);
        disp (str2num(getappdata(0,'CurrentAngleNumber')));
        disp (str2num(getappdata(0,'CurrentDistanceNumber')));
        
        % Updating Rover point displayed on graph/////////////////////
        Xset = getappdata(0,'Xset');
        Yset = getappdata(0,'Yset');
        Xset(1)= getappdata(0,'RoverXCoordinate');
        Yset(1)= getappdata(0,'RoverYCoordinate');
        setappdata(0,'Yset', Yset);
        setappdata(0,'Xset', Xset);
        % Creating a new color map for the red dot///////////////////
        matrix2 = zeros(size(getappdata(0,'Xset'), 1), 1);
        matrix2(1)= 1;
        myColors = zeros(size(getappdata(0,'Xset'), 1), 3);
        rowsToSetBlue = matrix2 == 0;
        rowsToSetRed = matrix2 == 1;

        myColors(rowsToSetBlue, 1) = 0;
        myColors(rowsToSetBlue, 2) = 0;
        myColors(rowsToSetBlue, 3) = 1;

        myColors(rowsToSetRed, 1) = 1;
        myColors(rowsToSetRed, 2) = 0;
        myColors(rowsToSetRed, 3) = 0;
        
        % Updating the Scatter plot//////////////////////////////////
        axes(getappdata(0,'Map_Graph'))
        scatter(getappdata(0,'Xset'), getappdata(0,'Yset'), 60, myColors, 'Parent', getappdata(0,'Map_Graph'), 'filled');
        grid on;

        
        
        
        
        
        
        
        Subtract_Number = (str2num(getappdata(0,'CurrentAngleNumber')))-90;
        RealDegrees = (getappdata(0,'RoverDegrees')) + Subtract_Number;
        setappdata(0,'RealDegrees',RealDegrees);
        
        if (RealDegrees >= 360)
            RealDegrees = RealDegrees - 360;
        end
        if (RealDegrees < 0)
            RealDegrees = RealDegrees + 360;
        end

        if (RealDegrees == 0)
            disp('0 degrees real')
            Xset = getappdata(0,'Xset')
            Xset = [Xset,getappdata(0,'RoverXCoordinate')+str2num(getappdata(0,'CurrentDistanceNumber'))];
            setappdata(0,'Xset',Xset)
            Yset = getappdata(0,'Yset')
            Yset = [Yset,getappdata(0,'RoverYCoordinate')];
            setappdata(0,'Yset',Yset)
            
        elseif ((RealDegrees > 0) && (RealDegrees < 90))    
            disp('between 0 and 90 real')
            Xset = getappdata(0,'Xset');
            Yset = getappdata(0,'Yset');
            New_X_Coord = getappdata(0,'RoverXCoordinate')+ (str2num(getappdata(0,'CurrentDistanceNumber'))*cosd(RealDegrees));
            New_Y_Coord = getappdata(0,'RoverYCoordinate')+ (str2num(getappdata(0,'CurrentDistanceNumber'))*sind(RealDegrees));
            Xset = [Xset,New_X_Coord];
            Yset = [Yset,New_Y_Coord];
            setappdata(0,'Xset',Xset);
            setappdata(0,'Yset',Yset);
        elseif (RealDegrees == 90)
            disp('90 degrees real')
            Yset = getappdata(0,'Yset');
            Xset = getappdata(0,'Xset');
            Yset = [Yset,getappdata(0,'RoverYCoordinate')+str2num(getappdata(0,'CurrentDistanceNumber'))];
            Xset = [Xset,getappdata(0,'RoverXCoordinate')];
            setappdata(0,'Yset',Yset);
            setappdata(0,'Xset',Xset);
        
        elseif ((RealDegrees > 90) && (RealDegrees < 180))    
            disp('between 90 and 180 real')
            Xset = getappdata(0,'Xset');
            Yset = getappdata(0,'Yset');
            New_X_Coord = getappdata(0,'RoverXCoordinate')- (str2num(getappdata(0,'CurrentDistanceNumber'))*cosd(180 - RealDegrees));
            New_Y_Coord = getappdata(0,'RoverYCoordinate')+ (str2num(getappdata(0,'CurrentDistanceNumber'))*sind(180 - RealDegrees));
            Xset = [Xset,New_X_Coord];
            Yset = [Yset,New_Y_Coord];
            setappdata(0,'Xset',Xset)
            setappdata(0,'Yset',Yset)
        elseif (RealDegrees == 180)
            disp('180 degrees real')
            Xset = getappdata(0,'Xset')
            Yset = getappdata(0,'Yset')
            Xset = [Xset,getappdata(0,'RoverXCoordinate')-str2num(getappdata(0,'CurrentDistanceNumber'))]
            Yset = [Yset,getappdata(0,'RoverYCoordinate')]
            setappdata(0,'Xset',Xset)
            setappdata(0,'Yset',Yset)
        elseif ((RealDegrees > 180) && (RealDegrees < 270))    
            disp('between 180 and 270 real')
            Xset = getappdata(0,'Xset');
            Yset = getappdata(0,'Yset');
            New_X_Coord = getappdata(0,'RoverXCoordinate')- (str2num(getappdata(0,'CurrentDistanceNumber'))*cosd(RealDegrees-180));
            New_Y_Coord = getappdata(0,'RoverYCoordinate')- (str2num(getappdata(0,'CurrentDistanceNumber'))*sind(RealDegrees-180));
            Xset = [Xset,New_X_Coord];
            Yset = [Yset,New_Y_Coord];
            setappdata(0,'Xset',Xset)
            setappdata(0,'Yset',Yset)
            
        elseif (RealDegrees == 270)
            disp('270 degrees real')
            Yset = getappdata(0,'Yset')
            Yset = [Yset,getappdata(0,'RoverYCoordinate')-str2num(getappdata(0,'CurrentDistanceNumber'))];
            setappdata(0,'Yset',Yset)
            Xset = getappdata(0,'Xset')
            Xset = [Xset,getappdata(0,'RoverXCoordinate')];
            setappdata(0,'Xset',Xset)
            
        elseif ((RealDegrees > 270) && (RealDegrees < 360)) 
            disp('between 270 and 360 real')
            Xset = getappdata(0,'Xset');
            Yset = getappdata(0,'Yset');
            New_X_Coord = getappdata(0,'RoverXCoordinate')+ (str2num(getappdata(0,'CurrentDistanceNumber'))*cosd(360-RealDegrees));
            New_Y_Coord = getappdata(0,'RoverYCoordinate')- (str2num(getappdata(0,'CurrentDistanceNumber'))*sind(360-RealDegrees));
            Xset = [Xset,New_X_Coord];
            Yset = [Yset,New_Y_Coord];
            setappdata(0,'Xset',Xset)
            setappdata(0,'Yset',Yset)
            
        end
    elseif (ReadByte == 'D')
        setappdata(0,'AngleReceivingMode',0);
        setappdata(0,'DistanceReceivingMode',1);
        setappdata(0,'ForwardReceivingMode',0);
        setappdata(0,'RightReceivingMode',0);
        setappdata(0,'LeftReceivingMode',0);
        setappdata(0,'CurrentDistanceNumber','');
        
    elseif (getappdata(0,'AngleReceivingMode')== 1)
        Old_Angle = getappdata(0,'CurrentAngleNumber');
        Angle = strcat(Old_Angle,ReadByte);
        setappdata(0,'CurrentAngleNumber',Angle);
        
    elseif (getappdata(0,'DistanceReceivingMode')== 1)
        Old_Distance = getappdata(0,'CurrentDistanceNumber');
        Distance = strcat(Old_Distance,ReadByte);
        setappdata(0,'CurrentDistanceNumber',Distance);
        
    elseif (ReadByte == 'F')
        setappdata(0,'ForwardReceivingMode',1);
        setappdata(0,'RightReceivingMode',0);
        setappdata(0,'LeftReceivingMode',0);
        setappdata(0,'AngleReceivingMode',0);
        setappdata(0,'DistanceReceivingMode',0);
        setappdata(0,'CurrentForwardDistanceNumber','');
        
    elseif (getappdata(0,'ForwardReceivingMode')== 1)
        Old_Distance = getappdata(0,'CurrentForwardDistanceNumber');
        Distance = strcat(Old_Distance,ReadByte);
        setappdata(0,'CurrentForwardDistanceNumber',Distance);
        
    elseif (ReadByte == 'L')
        setappdata(0,'ForwardReceivingMode',0);
        setappdata(0,'RightReceivingMode',0);
        setappdata(0,'LeftReceivingMode',1);
        setappdata(0,'AngleReceivingMode',0);
        setappdata(0,'DistanceReceivingMode',0);
        setappdata(0,'CurrentLeftDistanceNumber','');
        
    elseif (getappdata(0,'LeftReceivingMode')== 1)
        Old_Distance = getappdata(0,'CurrentLeftDistanceNumber');
        Distance = strcat(Old_Distance,ReadByte);
        setappdata(0,'CurrentLeftDistanceNumber',Distance);
                
    elseif (ReadByte == 'R')
        
        setappdata(0,'ForwardReceivingMode',0);
        setappdata(0,'RightReceivingMode',1);
        setappdata(0,'LeftReceivingMode',0);
        setappdata(0,'AngleReceivingMode',0);
        setappdata(0,'DistanceReceivingMode',0);
        setappdata(0,'CurrentRightDistanceNumber','');
        
    elseif (getappdata(0,'RightReceivingMode')== 1)
        Old_Distance = getappdata(0,'CurrentRightDistanceNumber');
        Distance = strcat(Old_Distance,ReadByte);
        setappdata(0,'CurrentRightDistanceNumber',Distance);

        
    end
       
 
else
    return
    %garbage
end







% --- Outputs from this function are returned to the command line.
function varargout = Main_GUI_Page_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in Connect_To_Rover.
function Connect_To_Rover_Callback(hObject, eventdata, handles)
display('Connecting...')
fopen(getappdata(0,'SerialPort'));             %open the serial port
display('Connected.')


% --- Executes on button press in Disconnect_From_Rover.
function Disconnect_From_Rover_Callback(hObject, eventdata, handles)
display ('Disconnecting...')
fclose(getappdata(0,'SerialPort'));            %close serial port
display ('Disconnected.')



function X_Coordinate_Callback(hObject, eventdata, handles)
% hObject    handle to X_Coordinate (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of X_Coordinate as text
%        str2double(get(hObject,'String')) returns contents of X_Coordinate as a double


% --- Executes during object creation, after setting all properties.
function X_Coordinate_CreateFcn(hObject, eventdata, handles)
% hObject    handle to X_Coordinate (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Y_Coordinate_Callback(hObject, eventdata, handles)
% hObject    handle to Y_Coordinate (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Y_Coordinate as text
%        str2double(get(hObject,'String')) returns contents of Y_Coordinate as a double


% --- Executes during object creation, after setting all properties.
function Y_Coordinate_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Y_Coordinate (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Rover_Direction_Callback(hObject, eventdata, handles)
% hObject    handle to Rover_Direction (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Rover_Direction as text
%        str2double(get(hObject,'String')) returns contents of Rover_Direction as a double


% --- Executes during object creation, after setting all properties.
function Rover_Direction_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Rover_Direction (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in Exit_Scanning_Mode.
function Exit_Scanning_Mode_Callback(hObject, eventdata, handles)
disp('B')
fprintf(getappdata(0,'SerialPort'),'B');       %send 'B' character to serial port


% --- Executes on button press in Enter_Scanning_Mode.
function Enter_Scanning_Mode_Callback(hObject, eventdata, handles)
disp('A')
fprintf(getappdata(0,'SerialPort'),'A');       %send 'A' character to serial port



% --- Executes on button press in Enter_Path_Follower.
function Enter_Path_Follower_Callback(hObject, eventdata, handles)
disp('a')
fprintf(getappdata(0,'SerialPort'),'a');       %send 'a' character to serial port



% --- Executes on button press in Exit_Path_Follower.
function Exit_Path_Follower_Callback(hObject, eventdata, handles)
disp('b')
fprintf(getappdata(0,'SerialPort'),'b');       %send 'a' character to serial port


% --- Executes on selection change in List_Of_Directions.
function List_Of_Directions_Callback(hObject, eventdata, handles)
% hObject    handle to List_Of_Directions (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns List_Of_Directions contents as cell array
%        contents{get(hObject,'Value')} returns selected item from List_Of_Directions


% --- Executes during object creation, after setting all properties.
function List_Of_Directions_CreateFcn(hObject, eventdata, handles)
% hObject    handle to List_Of_Directions (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in Add_Direction.
function Add_Direction_Callback(hObject, eventdata, handles)
set(handles.List_Of_Directions,'value',1);
Add_Direction_GUI
uiwait(Add_Direction_GUI)
Duration = getappdata(0,'Duration_Value');
Direction = getappdata(0,'Direction_Value');
%[rows,columns] = size(getappdata(0,'Direction_Array'));
Array = getappdata(0,'Directions_Array');
Display_Array = getappdata(0,'Directions_Display_Array');
setappdata(0,'Directions_Array',[Array;Direction,Duration])
%disp(getappdata(0,'Directions_Array'));
if (Direction == 1)
    str1 = 'Go forward for: ';
    str2 = num2str(Duration);
    str3 = ' milliseconds';
    Final_String = strcat(str1,str2,str3);
elseif (Direction == 2)
    str1 = 'Go left for: ';
    str2 = num2str(Duration);
    str3 = ' milliseconds';
    Final_String = strcat(str1,str2,str3);
elseif (Direction == 3)
    str1 = 'Go right for: ';
    str2 = num2str(Duration);
    str3 = ' milliseconds';
    Final_String = strcat(str1,str2,str3);
end

if (isempty(Display_Array))
setappdata(0,'Directions_Display_Array',{Final_String})
else
setappdata(0,'Directions_Display_Array',[Display_Array; Final_String])  
end
set(handles.List_Of_Directions,'string',getappdata(0,'Directions_Display_Array'))


% --- Executes on button press in Remove_Direction.
function Remove_Direction_Callback(hObject, eventdata, handles)
if (~isempty(getappdata(0,'Directions_Array')))
rowToDelete = get(handles.List_Of_Directions,'value');
set(handles.List_Of_Directions,'value',1);
Array = getappdata(0,'Directions_Array');
Display_Array = getappdata(0,'Directions_Display_Array');
Array(rowToDelete, :) = [];
Display_Array(rowToDelete, :) = [];
setappdata(0,'Directions_Array', Array);
setappdata(0,'Directions_Display_Array',Display_Array);
set(handles.List_Of_Directions,'string',Display_Array);
end


% --- Executes on button press in Finish_Directions.
function Finish_Directions_Callback(hObject, eventdata, handles)
Transmit_All_Directions(getappdata(0,'Directions_Array'));
pause(.1)
disp('d')
fprintf(getappdata(0,'SerialPort'),'d');       %send 'd' character to serial port
set(handles.List_Of_Directions,'value',1);
set(handles.List_Of_Directions,'string',{});
Directions_Array = [];
setappdata(0,'Directions_Array',Directions_Array)
Directions_Display_Array = {};
setappdata(0,'Directions_Display_Array',Directions_Display_Array)


% --- Executes on button press in Begin_Directions.
function Begin_Directions_Callback(hObject, eventdata, handles)
disp('c')
fprintf(getappdata(0,'SerialPort'),'c');       %send 'a' character to serial port
