function varargout = Arm3DoFGui(varargin)
% ARM3DOFGUI MATLAB code for Arm3DoFGui.fig
%      ARM3DOFGUI, by itself, creates a new ARM3DOFGUI or raises the existing
%      singleton*.
%
%      H = ARM3DOFGUI returns the handle to a new ARM3DOFGUI or the handle to
%      the existing singleton*.
%
%      ARM3DOFGUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in ARM3DOFGUI.M with the given input arguments.
%
%      ARM3DOFGUI('Property','Value',...) creates a new ARM3DOFGUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Arm3DoFGui_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Arm3DoFGui_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Arm3DoFGui

% Last Modified by GUIDE v2.5 20-Dec-2012 13:35:53

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Arm3DoFGui_OpeningFcn, ...
                   'gui_OutputFcn',  @Arm3DoFGui_OutputFcn, ...
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


% --- Executes just before Arm3DoFGui is made visible.
function Arm3DoFGui_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Arm3DoFGui (see VARARGIN)

% Choose default command line output for Arm3DoFGui
handles.output = hObject;

% Load parameters
handles.params = ParametersFunction();

%Initialize as not connected
handles.fid = -1;
handles.IPin = '';
handles.IPout = '';
handles.connected = 0;
handles.port_in = 3100;
handles.port_out = 3490;
% handles.COM = '';

%initialize trajectories and trajectory params
handles.num_pts = 1;
handles.t = 0; handles.T = 0; handles.dt = 0.001;
handles.xTraj = 0; handles.yTraj = 0; 
handles.thTraj = 0;
handles.armConfig = 'ELBOW_UP';
handles.th1Des = 0; handles.th2Des = 0;
handles.th3Des = 0;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes Arm3DoFGui wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = Arm3DoFGui_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function TCPIPeditBox_Callback(hObject, eventdata, handles)
% hObject    handle to TCPIPeditBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of TCPIPeditBox as text
%        str2double(get(hObject,'String')) returns contents of TCPIPeditBox as a double


% --- Executes during object creation, after setting all properties.
function TCPIPeditBox_CreateFcn(hObject, eventdata, handles)
% hObject    handle to TCPIPeditBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in COMbutton.
function COMbutton_Callback(hObject, eventdata, handles)
% hObject    handle to COMbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
while ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end

set(handles.outputText,'string','Attempting to connect');
drawnow();

comString = get(handles.TCPIPeditBox,'String');
handles.IPin = tcpip(comString, handles.port_in);
handles.IPout = tcpip(comString, handles.port_out);
% handles.COM = serial(comString, 'baudrate',9600);
% set(handles.COM,'ByteOrder','littleEndian');
% set(handles.COM,'FlowControl','hardware');
% set(handles.COM,'InputBufferSize',4096);
% set(handles.COM,'OutputBufferSize',4096);

set(handles.IPin,'ByteOrder','littleEndian');
set(handles.IPin,'InputBufferSize',4095);
set(handles.IPout,'ByteOrder','littleEndian');
set(handles.IPout,'OutputBufferSize',4095);
try 
    fopen(handles.IPin);
    fopen(handles.IPout);
%     fopen(handles.COM);
    
    drawnow();
    handles.connected = 1;
    %Send control gains
    cmd = 6;
%     fprintf(handles.IPout,'%d\r',cmd);
    fwrite(handles.IPout,cmd,'int32');
%     fwrite(handles.COM,cmd,'int32');
    text = fscanf(handles.IPin,'%s');
%     text = fscanf(handles.COM,'%s');
%     disp('Is PC104 ready? for control?');
    if(isempty(strfind(text,'CONTROL')))
        set(handles.outputText,'string','Problem updating gains 1, reconnect');
        disp(text);
        handles.connected = 0;
    else
        disp('Sending control gains');
        for i = 1:3
            fwrite(handles.IPout,handles.params.kp(i),'double');
            %         fprintf(handles.IPout,'%f\r',handles.params.kp(i));
%             fwrite(handles.COM,handles.params.kp(i),'double');
        end
        for i = 1:3
            fwrite(handles.IPout, handles.params.kd(i),'double');
            %         fprintf(handles.IPout,'%f\r',handles.params.kd(i));
%             fwrite(handles.COM,handles.params.kd(i),'double');
        end
        for i = 1:3
            fwrite(handles.IPout, handles.params.ki(i),'double');
            %         fprintf(handles.IPout,'%f\r',handles.params.ki(i));
%             fwrite(handles.COM,handles.params.ki(i),'double');
        end
        text = fscanf(handles.IPin,'%s');
%         text = fscanf(handles.COM,'%s');
        if(isempty(strfind(text,'GAINSUPDATED')))
            set(handles.outputText,'string','Problem updating gains 2, reconnect');
            handles.connected = 0;
        end
    end
catch err
    set(handles.outputText,'String',['Could not connect to ', comString]);
    handles.connected = 0;
end
if(handles.connected)
    set(handles.outputText,'String',['Connected to ', comString]);
end
disp(['connected: ' num2str(handles.connected)])
guidata(hObject, handles);


% --- Executes on selection change in trajPopMenu.
function trajPopMenu_Callback(hObject, eventdata, handles)
% hObject    handle to trajPopMenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns trajPopMenu contents as cell array
%        contents{get(hObject,'Value')} returns selected item from trajPopMenu


% --- Executes during object creation, after setting all properties.
function trajPopMenu_CreateFcn(hObject, eventdata, handles)
% hObject    handle to trajPopMenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in trajButton.
function trajButton_Callback(hObject, eventdata, handles)
% hObject    handle to trajButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
flag1 = 1;
%Get popmenu value
popVal = get(handles.trajPopMenu,'val');
if(popVal == 1)
    %Load trajectory
    try
        t = evalin('base','t');
        xDes = evalin('base','xDes');
        yDes = evalin('base','yDes');
        thDes = evalin('base','thDes');
        num_pts = length(t);
        if(length(xDes) ~= num_pts || ...
                length(yDes) ~= num_pts || ...
                length(thDes) ~= num_pts)
            set(handles.outputText,'string','Length of trajectories are not the same');
            guidata(hObject,handles);
            return
        end
    catch err
        set(handles.outputText,'string', ...
            'Could not load variables from workspace, make sure variables exist as xDes, yDes, thDes, and t');
        guidata(hObject,handles);
        return;
    end
elseif(popVal == 2)
    T = str2double(get(handles.trajTimeEdit,'string'));
    t = 0:handles.dt:T;
    num_pts = length(t);
    xPoly = str2num(get(handles.xTrajEdit,'string')); %#ok<*ST2NM>
    xDes = polyval(xPoly,t);
    yPoly = str2num(get(handles.yTrajEdit,'string'));
    yDes = polyval(yPoly,t);
    thPoly = str2num(get(handles.thTrajEdit,'string'));
    thDes = polyval(thPoly,t);
    set(handles.outputText,'string','Polynomials loaded');
    drawnow();
else
    try
        t = evalin('base','t');
        th1Des = evalin('base','th1Des');
        th2Des = evalin('base','th2Des');
        th3Des = evalin('base','th3Des');
        num_pts = length(t);
        if(length(th1Des) ~= num_pts || ...
                length(th2Des) ~= num_pts || ...
                length(th3Des) ~= num_pts)
            set(handles.outputText,'string','Length of trajectories are not the same');
            guidata(hObject,handles);
            return
        end
    catch err2
        set(handles.outputText,'string', ...
            'Could not load variables from workspace, make sure variables exist as xDes, yDes, thDes, and t');
        guidata(hObject,handles);
        return;
    end
end
%figure out if arm should be elbow up or down
if(get(handles.elbowUpRadio,'value'))
    handles.armConfig = 'ELBOW_UP';
else
    handles.armConfig = 'ELBOW_DOWN';
end

if(popVal ~= 3)
    [th1Des th2Des th3Des flag1] = generateTrajectory(xDes, yDes, thDes, ...
        handles.armConfig, handles.params);
else
    [xDes, yDes, thDes] = forwardKin3DoF(th1Des,th2Des,th3Des,handles.params);
    if(any(abs(th1Des >= handles.params.theta1_max)) || ...
            any(abs(th2Des >= handles.params.theta2_max)))
        flag1 = -2;
    end
end

%Make sure flag is good
if(flag1 ~= 1)
    if(flag1 == -1)
        %arm reach exceeded
        set(handles.outputText,'string','Trajectory bad, arm reach exceeded');
    elseif (flag1 == -2)
        %joint angle exceeded
        set(handles.outputText,'string','Trajectory bad, joint angles exceeded');    
    end
    guidata(hObject,handles);
    return
end

set(handles.outputText,'string','Trajectory is good');
drawnow();

handles.xDes = xDes; handles.yDes = yDes; handles.thDes = thDes;
handles.th1Des = th1Des; handles.th2Des = th2Des; handles.th3Des = th3Des;
handles.t = t; handles.num_pts = num_pts;

%Make sure we are connected
if(~handles.connected)
    set(handles.outputText,'string','Not connected, cannot send trajectory');
    guidata(hObject,handles);
    return
end

%Now, send trajectory

%Tell PC104 we want to send it the number of data points in the trajectory
cmd = 1;
% fprintf(handles.COM,'%d\r',cmd);
fwrite(handles.IPout,cmd,'int32');
% fwrite(handles.COM,cmd,'int32');

reading = fscanf(handles.IPin,'%s');
if(isempty(strfind(reading,'GETTRAJ')))
    disp('PC104 didnt receive command')
    disp(reading)
    fwrite(handles.IPout,cmd,'int32');
end

%send it the # of data points
% fprintf(handles.COM,'%d\r',num_pts);
fwrite(handles.IPout,num_pts,'int32');
% fwrite(handles.COM,num_pts,'int32');

%Make sure we get the right number of data points
% reading = fscanf(handles.IPin,'%d',[1 1]);
% reading = fread(handles.COM,1,'int32');
reading = fread(handles.IPin,1,'int32');
if(isempty(reading) || reading ~= num_pts)
    disp(reading)
    set(handles.outputText,'string','Something weird happened with tcpip comm, please restart');
    guidata(hObject,handles);
    return
end

%Check to make sure PC104 had enough memory to allocate for the trajectories
text = fscanf(handles.IPin,'%s');
% text = fscanf(handles.COM,'%s');
if(~isempty(strfind(text,'MEMFAIL')))
    set(handles.outputText,'string','PC104 did not have enough memory to allocate trajectory');
    guidata(hObject,handles);
    return
end

%Send joint trajectories to PC104
cmd = 2;
fwrite(handles.IPout,cmd, 'int32');
% fwrite(handles.COM,cmd,'int32');
% fprintf(handles.COM,'%d\r',cmd);
text = fscanf(handles.IPin,'%s');
% fscanf(handles.COM,'%s');
while(isempty(strfind(text,'SENDDATA')))
%     fprintf(handles.COM,'%d\r',cmd);
    fwrite(handles.IPout,cmd,'int32');
%     fwrite(handles.COM,cmd,'int32');
    text = fscanf(handles.IPin,'%s');
%     text = fscanf(handles.COM,'%s');
end
set(handles.outputText,'string','Sending data to PC104');
drawnow();

%Joint 1
for i = 1:handles.num_pts
%     fprintf(handles.COM,'%f\r',th1Des(i));
    fwrite(handles.IPout,th1Des(i),'double');
%     fwrite(handles.COM,th1Des(i),'double');
end
text = fscanf(handles.IPin,'%s');
% text = fscanf(handles.COM,'%s');
while(isempty(strfind(text,'DONETRAJ1')))
    text = fscanf(handles.IPin,'%s');
%     text = fscanf(handles.COM,'%s');
end
set(handles.outputText,'string','Sent angle 1');
drawnow();

%Joint 2
for i = 1:handles.num_pts
%     fprintf(handles.COM,'%f\r',th2Des(i));
    fwrite(handles.IPout,th2Des(i),'double');
%     fwrite(handles.COM,th2Des(i),'double');
end
text = fscanf(handles.IPin,'%s');
% text = fscanf(handles.COM,'%s');
while(isempty(strfind(text,'DONETRAJ2')))
    text = fscanf(handles.IPin,'%s');
%     text = fscanf(handles.COM,'%s');
end
set(handles.outputText,'string','Sent angle 2');
drawnow();

%Joint 3
for i = 1:handles.num_pts
%     fprintf(handles.COM,'%f\r',th3Des(i));
    fwrite(handles.IPout,th3Des(i),'double');
%     fwrite(handles.COM,th3Des(i),'double');
end
text = fscanf(handles.IPin,'%s');
% text = fscanf(handles.COM,'%s');
while(isempty(strfind(text,'DONETRAJ3')))
    text = fscanf(handles.IPin,'%s');
%     text = fscanf(handles.COM,'%s');
end
set(handles.outputText,'string','Sent angle 3');
drawnow();

set(handles.outputText,'string','Trajectory sent succesfully');

guidata(hObject,handles);



function xTrajEdit_Callback(hObject, eventdata, handles)
% hObject    handle to xTrajEdit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of xTrajEdit as text
%        str2double(get(hObject,'String')) returns contents of xTrajEdit as a double


% --- Executes during object creation, after setting all properties.
function xTrajEdit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to xTrajEdit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function yTrajEdit_Callback(hObject, eventdata, handles)
% hObject    handle to yTrajEdit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of yTrajEdit as text
%        str2double(get(hObject,'String')) returns contents of yTrajEdit as a double


% --- Executes during object creation, after setting all properties.
function yTrajEdit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to yTrajEdit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function thetaTrajEdit_Callback(hObject, eventdata, handles)
% hObject    handle to thetaTrajEdit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of thetaTrajEdit as text
%        str2double(get(hObject,'String')) returns contents of thetaTrajEdit as a double


% --- Executes during object creation, after setting all properties.
function thetaTrajEdit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to thetaTrajEdit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function trajTimeEdit_Callback(hObject, eventdata, handles)
% hObject    handle to trajTimeEdit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of trajTimeEdit as text
%        str2double(get(hObject,'String')) returns contents of trajTimeEdit as a double


% --- Executes during object creation, after setting all properties.
function trajTimeEdit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to trajTimeEdit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in sendHomeButton.
function sendHomeButton_Callback(hObject, eventdata, handles)
% hObject    handle to sendHomeButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%check if connected
if(~handles.connected)
    set(handles.outputText,'string','Not connected to PC104');
    guidata(hObject,handles);
    return
end

%see if trajectories have been defined
if(length(handles.th1Des) > 1)
    handles.home1 = handles.th1Des(1);
    handles.home2 = handles.th2Des(1);
    handles.home3 = handles.th3Des(1);
else
    handles.home1 = 0;
    handles.home2 = 0;
    handles.home3 = 0;
end

%Tell PC104 we are going to send it home positions
cmd = 7;
% fprintf(handles.COM,'%d\r',cmd);
% fprintf(handles.COM,'%f\r',handles.home1);
% fprintf(handles.COM,'%f\r',handles.home2);
% fprintf(handles.COM,'%f\r',handles.home3);
fwrite(handles.IPout,cmd,'int32');
fwrite(handles.IPout,handles.home1,'double');
fwrite(handles.IPout,handles.home2,'double');
fwrite(handles.IPout,handles.home3,'double');
text = fscanf(handles.IPin,'%s');
% fwrite(handles.COM,cmd,'int32');
% fwrite(handles.COM,handles.home1,'double');
% fwrite(handles.COM,handles.home2,'double');
% fwrite(handles.COM,handles.home3,'double');
% text = fscanf(handles.COM,'%s');
while(isempty(strfind(text,'HOMEUPDATED')))
    text = fscanf(handles.IPin,'%s');
%     text = fscanf(handles.COM,'%s');
end

set(handles.outputText,'string',['Home positions set to ', ...
    num2str(handles.home1), ', ', num2str(handles.home2), ', ', ...
    num2str(handles.home3), ' radians']);
guidata(hObject,handles);



% --- Executes on button press in goHomeButton.
function goHomeButton_Callback(hObject, eventdata, handles)
% hObject    handle to goHomeButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%check for connection
if(~handles.connected)
    set(handles.outputText,'string','Not connected');
    guidata(hObject,handles);
    return
end

%Tell PC104 to go home
cmd = 4;
% fprintf(handles.COM,'%d\r',cmd);
fwrite(handles.IPout,cmd,'int32');
% fwrite(handles.COM,cmd,'int32');
%Check for PC104 writing back
text = fscanf(handles.IPin,'%s');
% text = fscanf(handles.COM,'%s');
while(isempty(strfind(text,'HOME')))
%     fprintf(handles.COM,'%d\r',cmd);
    fwrite(handles.IPout,cmd,'int32');
    text = fscanf(handles.IPin,'%s');
%     fwrite(handles.COM,cmd,'int32');
%     text = fscanf(handles.COM,'%s');
end
set(handles.outputText,'string','Going home');
guidata(hObject,handles);

% --- Executes on button press in recTrajButton.
function recTrajButton_Callback(hObject, eventdata, handles)
% hObject    handle to recTrajButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%check for connection
if(~handles.connected)
    set(handles.outputText,'string','Not connected');
    guidata(hObject,handles);
    return
end

%Send a 3 to tell the PC104 to give us everything
cmd = 3;
% fprintf(handles.COM,'%d\r',cmd);
fwrite(handles.IPout,cmd,'int32');
% fwrite(handles.COM,cmd,'int32');

%wait for start command
text = fscanf(handles.IPin,'%s');
% text = fscanf(handles.COM,'%s');
while(isempty(strfind(text,'START')))
    text = fscanf(handles.IPin,'%s');
%     text = fscanf(handles.COM,'%s');
end

set(handles.outputText,'string','Ready to receive trajectory');
drawnow();

handles.th1Act = zeros(1,handles.num_pts);
handles.th2Act = zeros(1,handles.num_pts);
handles.th3Act = zeros(1,handles.num_pts);
handles.control1 = zeros(1,handles.num_pts);
handles.control2 = zeros(1,handles.num_pts);
handles.control3 = zeros(1,handles.num_pts);
handles.loopTimes = zeros(1,handles.num_pts);
%Read positions
for i = 1:handles.num_pts
%     reading = fscanf(handles.IPin,'%f',[1 1]);
%     handles.th1Act(i) = reading;
%     handles.th1Act(i) = fread(handles.COM,1,'double');
    handles.th1Act(i) = fread(handles.IPin,1,'double');
end
set(handles.outputText,'string','Received joint 1');
drawnow();
for i = 1:handles.num_pts
%     reading = fscanf(handles.IPin,'%f',[1 1]);
%     handles.th2Act(i) = reading;
%     handles.th2Act(i) = fread(handles.COM,1,'double');
    handles.th2Act(i) = fread(handles.IPin,1,'double');
end
set(handles.outputText,'string','Received joint 2');
drawnow();
for i = 1:handles.num_pts
%     reading = fscanf(handles.IPin,'%f',[1 1]);
%     handles.th3Act(i) = reading;
%     handles.th3Act(i) = fread(handles.COM,1,'double');
    handles.th3Act(i) = fread(handles.IPin,1,'double');
end
text = fscanf(handles.IPin,'%s');
% text = fscanf(handles.COM,'%s');
if isempty(strfind(text,'POSEND'))
    set(handles.outputText,'string','Error receiving trajectory data');
    guidata(hObject,handles);
    return
end
set(handles.outputText,'string','Received all joints, receiving control signals');
drawnow();
%Read controls
for i = 1:handles.num_pts
%     reading = fscanf(handles.IPin,'%f',[1 1]);
%     handles.control1(i) = reading;
%     handles.control1(i) = fread(handles.COM,1,'double');
    handles.control1(i) = fread(handles.IPin,1,'double');
end
set(handles.outputText,'string','Received control 1');
drawnow();
for i = 1:handles.num_pts
%     reading = fscanf(handles.IPin,'%f',[1 1]);
%     handles.control2(i) = reading;
%     handles.control2(i) = fread(handles.COM,1,'double');
    handles.control2(i) = fread(handles.IPin,1,'double');
end
set(handles.outputText,'string','Received control 2');
drawnow();
for i = 1:handles.num_pts
%     reading = fscanf(handles.IPin,'%f',[1 1]);
%     handles.control3(i) = reading;
%     handles.control3(i) = fread(handles.COM,1,'double');
    handles.control3(i) = fread(handles.IPin,1,'double');
end
set(handles.outputText,'string','Received control 3');
drawnow();
disp('got here');
for i = 1:handles.num_pts
%     reading = fscanf(handles.IPin,'%lu',[1 1]);
%     reading = fscanf(handles.COM,'%lu',[1 1]);
    handles.loopTimes(i) = fread(handles.IPin,1,'double');
end
disp(i);
text = fscanf(handles.IPin,'%s');
% text = fread(handles.COM,'%s');
if(isempty(strfind(text,'END')))
    disp(text);
    set(handles.outputText,'string','Error receiving control data');
    return
end
disp(text);
%Time to get camera information
for i = 1:handles.num_pts
    try
        temp = fread(handles.IPin,1,'double');
        handles.camX(i) = temp;
    catch err
        disp(temp);
        disp(i);
    end
end
for i = 1:handles.num_pts
    handles.camY(i) = fread(handles.IPin,1,'double');
end
for i = 1:handles.num_pts
    handles.cam1(i) = fread(handles.IPin,1,'double');
end
for i = 1:handles.num_pts
    try
        temp = fread(handles.IPin,1,'double');
        handles.cam2(i) = temp;
    catch
        disp(temp);
        disp(i);
    end
end
for i = 1:handles.num_pts
    handles.objX(i) = fread(handles.IPin,1,'double');
end
for i = 1:handles.num_pts
    handles.objY(i) = fread(handles.IPin,1,'double');
end
for i = 1:handles.num_pts
    handles.objTh(i) = fread(handles.IPin,1,'double');
end

time = fscanf(handles.IPin,'%lu',[1 1]);
% time = fscanf(handles.COM,'%lu',[1 1]);
text = fscanf(handles.IPin,'%s');
% text = fscanf(handles.COM,'%s');
if(isempty(strfind(text,'RESET')))
    set(handles.outputText,'string','Error receiving control data');
    return
end

set(handles.outputText,'string',['Position and control data received, soft reset. Max loop time: ', num2str(time), 'nsec.']);
guidata(hObject,handles);

% --- Executes on button press in executeButton.
function executeButton_Callback(hObject, eventdata, handles)
% hObject    handle to executeButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%check for connection
if(~handles.connected)
    set(handles.outputText,'string','Not connected');
    guidata(hObject,handles);
    return
end

cmd = 5;
% fprintf(handles.COM,'%d\r',cmd);
fwrite(handles.IPout,cmd,'int32');
text = fscanf(handles.IPin,'%s');
% fwrite(handles.COM,cmd,'int32');
% text = fscanf(handles.COM,'%s');
while(isempty(strfind(text,'EXECUTE')))
%     fprintf(handles.COM,'%d\r',cmd);
    fwrite(handles.IPout,cmd,'int32');
    text = fscanf(handles.IPin,'%s');
%     fwrite(handles.COM,cmd,'int32');
%     text = fscanf(handles.COM,'%s');
end

set(handles.outputText,'string', ...
    ['Executing, please wait ', num2str(handles.t(end)), 'seconds']);
pause(handles.t(end));
set(handles.outputText,'string','Trajectory executed');
drawnow();
guidata(hObject,handles);


% --- Executes on button press in encoderPosButton.
function encoderPosButton_Callback(hObject, eventdata, handles)
% hObject    handle to encoderPosButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%check for connection
if(~handles.connected)
    set(handles.outputText,'string','Not connected');
    guidata(hObject,handles);
    return
end

cmd = 8;
% fprintf(handles.COM,'%d\r',cmd);
fwrite(handles.IPout,cmd,'int32');
% reading = fscanf(handles.IPin, '%f',[1 1]);
% pos1 = reading;
% reading = fscanf(handles.IPin, '%f',[1 1]);
% pos2 = reading;
% reading = fscanf(handles.IPin, '%f',[1 1]);
% pos3 = reading;
% fwrite(handles.COM,cmd,'int32');
% pos1 = fread(handles.COM,1,'double');
% pos2 = fread(handles.COM,1,'double');
% pos3 = fread(handles.COM,1,'double');
pos1 = fread(handles.IPin,1,'double');
pos2 = fread(handles.IPin,1,'double');
pos3 = fread(handles.IPin,1,'double');

set(handles.outputText,'string', ...
    ['Pos1: ', num2str(pos1), ', Pos2: ', num2str(pos2), ...
    ', Pos3: ', num2str(pos3)]);
guidata(hObject,handles);

% --- Executes on button press in STOP.
function STOP_Callback(hObject, eventdata, handles)
% hObject    handle to STOP (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%check for connection
if(~handles.connected)
    set(handles.outputText,'string','Not connected');
    guidata(hObject,handles);
    return
end

cmd = 9;
% fprintf(handles.COM,'%d\r',cmd);
fwrite(handles.IPout,cmd,'int32');
text = fscanf(handles.IPin,'%s');
% fwrite(handles.COM,cmd,'int32');
% text = fscanf(handles.COM,'%s');
while(isempty(strfind(text,'STOP')))
%     fprintf(handles.COM,'%d\r',cmd);
    fwrite(handles.IPout,cmd,'int32');
    text = fscanf(handles.IPin,'%s');
%     fwrite(handles.COM,cmd,'int32');
%     text = fscanf(handles.COM,'%s');
end
fscanf(handles.IPin,'%lu',[1 1]);
fscanf(handles.IPin,'%s');
% fscanf(handles.COM,'%lu',[1 1]);
% fscanf(handles.COM,'%s');

set(handles.outputText,'string','Stopped and reset!');
guidata(hObject,handles);

% --- Executes on button press in plotButton.
function plotButton_Callback(hObject, eventdata, handles)
% hObject    handle to plotButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%Calculate joint vels
th1DesVel = derivative(handles.th1Des)/handles.dt;
th2DesVel = derivative(handles.th2Des)/handles.dt;
th3DesVel = derivative(handles.th3Des)/handles.dt;
th1ActVel = derivative(handles.th1Act)/handles.dt;
th2ActVel = derivative(handles.th2Act)/handles.dt;
th3ActVel = derivative(handles.th3Act)/handles.dt;
%Calculate joint accels
th1DesAcc = derivative(th1DesVel)/handles.dt;
th2DesAcc = derivative(th2DesVel)/handles.dt;
th3DesAcc = derivative(th3DesVel)/handles.dt;
th1ActAcc = derivative(th1ActVel)/handles.dt;
th2ActAcc = derivative(th2ActVel)/handles.dt;
th3ActAcc = derivative(th3ActVel)/handles.dt;

%Calculate x-y-th positions
[xDes yDes thDes] = ...
    forwardKin3DoF(handles.th1Des, handles.th2Des, handles.th3Des, handles.params);
[xAct yAct thAct] = ...
    forwardKin3DoF(handles.th1Act, handles.th2Act, handles.th3Act, handles.params);
%Calculate x-y-th vels
xDesVel = derivative(xDes)/handles.dt;
yDesVel = derivative(yDes)/handles.dt;
thDesVel = derivative(thDes)/handles.dt;
xActVel = derivative(xAct)/handles.dt;
yActVel = derivative(yAct)/handles.dt;
thActVel = derivative(thAct)/handles.dt;
%Calculate x-y-th accel
xDesAcc = derivative(xDesVel)/handles.dt;
yDesAcc = derivative(yDesVel)/handles.dt;
thDesAcc = derivative(thDesVel)/handles.dt;
xActAcc = derivative(xActVel)/handles.dt;
yActAcc = derivative(yActVel)/handles.dt;
thActAcc = derivative(thActVel)/handles.dt;

popMenuVal = get(handles.plotPopMenu,'Value');
switch popMenuVal
    case 1
        %Plot x-y-th position
        %Activate axis 1
        axes(handles.axes1);
        plot(handles.t,xDes,'-b');
        hold on
        plot(handles.t,xAct,'-r');
        title('x position'); 
        xlabel('Time (s)'); ylabel('x (m)');
        ylim([-0.6 0.6]);
        hold off
        %Joint 2
        %activate axis 2
        axes(handles.axes2);
        plot(handles.t,yDes,'-b');
        hold on
        plot(handles.t,yAct,'-r');
        title('y position'); 
        xlabel('Time (s)'); ylabel('y (m)');
        ylim([-0.6 0.6]);
        hold off
        %Joint 3
        %Activate axis 3
        axes(handles.axes3);
        plot(handles.t,thDes,'-b');
        hold on
        plot(handles.t,thAct,'-r');
        title('Theta position in world frame'); 
        xlabel('Time (s)'); ylabel('Theta (rad)');
        hold off
    case 2
        %Plot x-y-th velocity
        %Activate axis 1
        axes(handles.axes1);
        plot(handles.t,xDesVel,'-b');
        hold on
        plot(handles.t,xActVel,'-r');
        title('x velocity'); 
        xlabel('Time (s)'); ylabel('Velocity (m/s)');
        hold off
        %Joint 2
        axes(handles.axes2);
        plot(handles.t,yDesVel,'-b');
        hold on
        plot(handles.t,yActVel,'-r');
        title('y velocity'); 
        xlabel('Time (s)'); ylabel('Velocity (m/s)');
        hold off
        %Joint 3
        axes(handles.axes3);
        plot(handles.t,thDesVel,'-b');
        hold on
        plot(handles.t,thActVel,'-r');
        title('Theta velocity in world frame'); 
        xlabel('Time (s)'); ylabel('Angular Velocity (rad/s)');
        hold off
    case 3
        %Plot x-y-th acceleration
        axes(handles.axes1);
        plot(handles.t,xDesAcc,'-b');
        hold on
        plot(handles.t,xActAcc,'-r');
        title('x acceleration'); 
        xlabel('Time (s)'); ylabel('Acceleration (m/s^2)');
        hold off
        %Joint 2
        axes(handles.axes2);
        plot(handles.t,yDesAcc,'-b');
        hold on
        plot(handles.t,yActAcc,'-r');
        title('y acceleration'); 
        xlabel('Time (s)'); ylabel('Acceleration (m/s^2)');
        hold off
        %Joint 3
        axes(handles.axes3);
        plot(handles.t,thDesAcc,'-b');
        hold on
        plot(handles.t,thActAcc,'-r');
        title('Theta acceleration in world frame'); 
        xlabel('Time (s)'); ylabel('Angular Acceleration (rad/s^2)');
        hold off
    case 4
        %Plot control values as currents
        axes(handles.axes1);
        plot(handles.t,handles.control1,'-r');
        hold on
        title('Control 1'); 
        xlabel('Time (s)'); ylabel('Current (A)');
        ylim([-5.4 5.4]);
        hold off
        %Joint 2
        axes(handles.axes2);
        plot(handles.t,handles.control2,'-r');
        hold on
        title('Control 2'); 
        xlabel('Time (s)'); ylabel('Current (A)');
        ylim([-2.1 2.1]);
        hold off
        %Joint 3
        axes(handles.axes3);
        plot(handles.t,handles.control3,'-r');
        hold on
        title('Control 3'); 
        xlabel('Time (s)'); ylabel('Current (A)');
        ylim([-1.6 1.6]);
        hold off
    case 5
        %Plot joint positions
        axes(handles.axes1);
        plot(handles.t,handles.th1Des,'-b');
        hold on
        plot(handles.t,handles.th1Act,'-r');
        title('Joint 1 position'); 
        xlabel('Time (s)'); ylabel('Joint 1 angle (rad)');
        ylim([-handles.params.theta1_max handles.params.theta1_max]);
        hold off
        %Joint 2
        axes(handles.axes2);
        plot(handles.t,handles.th2Des,'-b');
        hold on
        plot(handles.t,handles.th2Act,'-r');
        title('Joint 2 position'); 
        xlabel('Time (s)'); ylabel('Joint 2 angle (rad)');
        ylim([-handles.params.theta2_max handles.params.theta2_max]);
        hold off
        %Joint 3
        axes(handles.axes3);
        plot(handles.t,handles.th3Des,'-b');
        hold on
        plot(handles.t,handles.th3Act,'-r');
        title('Joint 3 position'); 
        xlabel('Time (s)'); ylabel('Joint 3 angle (rad)');
        hold off
    case 6
        %Plot joint velocities
        axes(handles.axes1);
        plot(handles.t,th1DesVel,'-b');
        hold on
        plot(handles.t,th1ActVel,'-r');
        title('Joint 1 velocity'); 
        xlabel('Time (s)'); ylabel('Joint 1 velocity (rad/s)');
        hold off
        %Joint 2
        axes(handles.axes2);
        plot(handles.t,th2DesVel,'-b');
        hold on
        plot(handles.t,th2ActVel,'-r');
        title('Joint 2 velocity'); 
        xlabel('Time (s)'); ylabel('Joint 2 velocity (rad/s)');
        hold off
        %Joint 3
        axes(handles.axes3);
        plot(handles.t,th3DesVel,'-b');
        hold on
        plot(handles.t,th3ActVel,'-r');
        title('Joint 3 velocity'); 
        xlabel('Time (s)'); ylabel('Joint 3 velocity (rad/s)');
        hold off
    case 7
        %Plot joint accelerations
        axes(handles.axes1);
        plot(handles.t,th1DesAcc,'-b');
        hold on
        plot(handles.t,th1ActAcc,'-r');
        title('Joint 1 acceleration'); 
        xlabel('Time (s)'); ylabel('Joint 1 acceleration (rad/s^2)');
        hold off
        %Joint 2
        axes(handles.axes2);
        plot(handles.t,th2DesAcc,'-b');
        hold on
        plot(handles.t,th2ActAcc,'-r');
        title('Joint 2 acceleration'); 
        xlabel('Time (s)'); ylabel('Joint 2 acceleration (rad/s^2)');
        hold off
        %Joint 3
        axes(handles.axes3);
        plot(handles.t,th3DesAcc,'-b');
        hold on
        plot(handles.t,th3ActAcc,'-r');
        title('Joint 3 acceleration'); 
        xlabel('Time (s)'); ylabel('Joint 3 acceleration (rad/s^2)');
        hold off
    case 8
        %Plot x-y end point position in new window
        figure;
        plot(xDes,yDes,'-b');
        hold on
        plot(xAct,yAct,'-r');
        axis('equal');
        title('End point position')
        xlabel('x (m)'); ylabel('y (m)');
        hold off
    case 9
        %Plot loop times in a new window
        figure;
        plot(handles.loopTimes,'-b');
        title('Time between control loop function calls')
        xlabel('Iteration'); ylabel('Time (ms)');
        set(handles.outputText,'string', ...
            ['Average loop time: ', num2str(mean(handles.loopTimes)), ...
            'ms std dev: ', num2str(std(handles.loopTimes)), 'ms.']);
end

% --- Executes on button press in resetEncoderButton.
function resetEncoderButton_Callback(hObject, eventdata, handles)
% hObject    handle to resetEncoderButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%check for connection
if(~handles.connected)
    set(handles.outputText,'string','Not connected');
    guidata(hObject,handles);
    return
end

cmd = 10;
% fprintf(handles.COM,'%d\r',cmd);
fwrite(handles.IPout,cmd,'int32');
text = fscanf(handles.IPin,'%s');
% fwrite(handles.COM,cmd,'int32');
% text = fscanf(handles.COM,'%s');
while(isempty(strfind(text,'ENCRESET')))
%     fprintf(handles.COM,'%d\r',cmd);
    fwrite(handles.IPout,cmd,'int32');
    text = fscanf(handles.IPin,'%s');
%     fwrite(handles.COM,cmd,'int32');
%     text = fscanf(handles.COM,'%s');
end

set(handles.outputText,'string','Encoder counts reset');
guidata(hObject,handles);

% --- Executes on selection change in plotPopMenu.
function plotPopMenu_Callback(hObject, eventdata, handles)
% hObject    handle to plotPopMenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns plotPopMenu contents as cell array
%        contents{get(hObject,'Value')} returns selected item from plotPopMenu


% --- Executes during object creation, after setting all properties.
function plotPopMenu_CreateFcn(hObject, eventdata, handles)
% hObject    handle to plotPopMenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in saveButton.
function saveButton_Callback(hObject, eventdata, handles)
% hObject    handle to saveButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
c = clock;
saveString = ['handles_', num2str(c(1)), '_', num2str(c(2)), '-', ...
    num2str(c(3)), '_', num2str(c(4)), '_', num2str(c(5)), '.mat'];
save(saveString, 'handles');

set(handles.outputText,'string', ...
    ['handles variable structure saved as ', saveString]);
guidata(hObject,handles);


% --- Executes on button press in pushbutton13.
function pushbutton13_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%check for connection
if(~handles.connected)
    set(handles.outputText,'string','Not connected');
    guidata(hObject,handles);
    return
end

%Miscellaneous button press
cmd = 11;
fwrite(handles.IPout,cmd,'int32');

text = fscanf(handles.IPin,'%s');
if(~isempty(strfind(text,'JOINTANGLESCAM')))
    jointAngle1 = fread(handles.IPin,1,'double');
    jointAngle2 = fread(handles.IPin,1,'double');

end
for i = 1:3
    tempValsX(i) = fread(handles.IPin,1,'double');
    tempValsY(i) = fread(handles.IPin,1,'double');
    tempValsArea(i) = fread(handles.IPin,1,'int32');
end
    set(handles.outputText,'string', ['Angle1: ' num2str(jointAngle1) ...
        ' Angle2: ' num2str(jointAngle2) ' X1: ' num2str(tempValsX(1)) ...
        ' Y1: ' num2str(tempValsY(1)) ' Area1: ' num2str(tempValsArea(1)) ...
        ' X2: ' num2str(tempValsX(2)) ' Y2: ' num2str(tempValsY(2)) ...
        ' Area2: ' num2str(tempValsArea(2)) ' X3: ' num2str(tempValsX(3)) ...
        ' Y3: ' num2str(tempValsY(3)) ' Area3: ' num2str(tempValsArea(3))]);
% text = fscanf(handles.IPin,'%s');
% if(~isempty(strfind(text,'OBJORDER')))
%     xVal = zeros(1,3);
%     yVal = zeros(1,3);
%     areaVal = zeros(1,3);
%     for i = 1:3
%        xVal(i) = fread(handles.IPin,1,'double'); 
%        yVal(i) = fread(handles.IPin,1,'double');
%        areaVal(i) = fread(handles.IPin,1,'int32');
%     end
%     set(handles.outputText,'string', ['X1: ' num2str(xVal(1)) ' Y1: ' ...
%        num2str(yVal(1)) ' Area1: ' num2str(areaVal(1)) '\n ' ...
%        'X2: ' num2str(xVal(2)) ' Y2: ' num2str(yVal(2)) ' ' ...
%        'Area2: ' num2str(areaVal(2)) ' \n X3: ' ...
%        num2str(xVal(3)) ' Y3: ' num2str(yVal(3)) ...
%        'Area3: ' num2str(areaVal(3))]);
% end

% text = fscanf(handles.IPin,'%s');
% if(~isempty(strfind(text,'NOTIMING')))
%     xVal = fread(handles.IPin,1,'double');
%     yVal = fread(handles.IPin,1,'double');
%     set(handles.outputText,'string',['x: ' num2str(xVal) ...
%         ' y: ' num2str(yVal)]);
% elseif(~isempty(strfind(text,'TIMING')))
%     set(handles.outputText,'string','Getting timing data');
%     handles.timing = zeros(1,10001);
%     for i = 1:10001
%         handles.timing(i) = fread(handles.IPin,1,'double');
%     end
%     timingAvg = mean(handles.timing(2:end));
%     timingStd = std(handles.timing(2:end));
%     timingMax = max(handles.timing(2:end));
%     timingMin = min(handles.timing(2:end));
%     set(handles.outputText,'string', ...
%         ['Average: ' num2str(timingAvg) 'ms, Standard dev: ' ...
%         num2str(timingStd) 'ms, Max: ', num2str(timingMax), ...
%         'ms, Min: ', num2str(timingMin)]);
%     figure;
%     plot(handles.timing(2:end));
%     save('timingFromCameraInLoop.mat','handles');
% else
%     disp(text);
%     set(handles.outputText,'string','misc weird');
%     text = fscanf(handles.IPin,'%s');
%     disp(text);
% end
% 
% fprintf(handles.COM,'%d\r',cmd);
% %Open the com port and read in
% COM = serial('COM23','baudrate',9600);
% set(COM,'ByteOrder','littleEndian');
% set(COM,'FlowControl','hardware');
% set(COM,'InputBufferSize',4096);
% set(COM,'OutputBufferSize',4096);
% 
% fopen(COM);
% fwrite(handles.IPout,cmd,'int32');
% disp(['Received: ', fscanf(COM,'%s')]);
% % cmd = 'Whatup\r\n';
% % fprintf(COM,'%s\r',cmd);
% fclose(COM);
