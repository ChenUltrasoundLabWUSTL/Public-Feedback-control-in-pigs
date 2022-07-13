function varargout = MatlabScript_FeedbackControlPig(varargin)
% MATLABSCRIPT_FEEDBACKCONTROLPIG MATLAB code for MatlabScript_FeedbackControlPig.fig
%      MATLABSCRIPT_FEEDBACKCONTROLPIG, by itself, creates a new MATLABSCRIPT_FEEDBACKCONTROLPIG or raises the existing
%      singleton*.
%
%      H = MATLABSCRIPT_FEEDBACKCONTROLPIG returns the handle to a new MATLABSCRIPT_FEEDBACKCONTROLPIG or the handle to
%      the existing singleton*.
%
%      MATLABSCRIPT_FEEDBACKCONTROLPIG('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in MATLABSCRIPT_FEEDBACKCONTROLPIG.M with the given input arguments.
%
%      MATLABSCRIPT_FEEDBACKCONTROLPIG('Property','Value',...) creates a new MATLABSCRIPT_FEEDBACKCONTROLPIG or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before MatlabScript_FeedbackControlPig_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to MatlabScript_FeedbackControlPig_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help MatlabScript_FeedbackControlPig

% Last Modified by GUIDE v2.5 13-Jul-2022 08:54:35

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @MatlabScript_FeedbackControlPig_OpeningFcn, ...
    'gui_OutputFcn',  @MatlabScript_FeedbackControlPig_OutputFcn, ...
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

% --- Executes just before MatlabScript_FeedbackControlPig is made visible.
function MatlabScript_FeedbackControlPig_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to MatlabScript_FeedbackControlPig (see VARARGIN)

% Choose default command line output for MatlabScript_FeedbackControlPig
handles.output = hObject;
% Delete all connected instruments
delete(instrfind);
% Update handles structure
guidata(hObject, handles);
addpath(pwd)
addpath(genpath('C:\Users\HongChen Lab\Documents\MATLAB\Add-Ons\Collections'))
addpath(genpath('C:\Users\Chen lab\Documents\MATLAB\Add-Ons\Collections'));
addpath('Code from Acer\picosdk-ps5000a-matlab-instrument-driver-master\picosdk-ps5000a-matlab-instrument-driver-master');
addpath('Code from Acer\picotech-picosdk-matlab-picoscope-support-toolbox-v1.0-12-g7e73524\picotech-picosdk-matlab-picoscope-support-toolbox-7e73524');
addpath('Code from Acer\picotech-picosdk-ps5000a-matlab-instrument-driver-4d3f059');

% --- Outputs from this function are returned to the command line.
function varargout = MatlabScript_FeedbackControlPig_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


function motor_speed_Callback(hObject, eventdata, handles)
% hObject    handle to motor_speed (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.experimental_motor_speed = round(str2double(get(handles.motor_speed,'String')));
guidata(hObject, handles);
check_if_ready(hObject, handles);


% --- Executes during object creation, after setting all properties.
function motor_speed_CreateFcn(hObject, eventdata, handles)
% hObject    handle to motor_speed (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function motor_step_size_Callback(hObject, eventdata, handles)
% hObject    handle to motor_step_size (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.experimental_motor_step_size = str2double(get(handles.motor_step_size,'String'));
guidata(hObject, handles);
check_if_ready(hObject, handles);

% --- Executes during object creation, after setting all properties.
function motor_step_size_CreateFcn(hObject, eventdata, handles)
% hObject    handle to motor_step_size (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes when motor_controls is resized.
function motor_controls_SizeChangedFcn(hObject, eventdata, handles)
% hObject    handle to motor_controls (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% --- Executes on button press in reconnect_motor_pushbutton.
function reconnect_motor_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to reconnect_motor_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% ret = CsMl_FreeSystem(handle);
% clear all; close all; clc
% delete(instrfindall)
% fgen_inital_Verasonics
% global fgen
% freq    = str2num(get(handles.frequency,'String'));      %frequency [MHz]
% cycle   = str2num(get(handles.BurstCount,'String'));        %burst count
% PRF     = str2num(get(handles.PRF,'String'));        %burst period [s]
% volt    = str2num(get(handles.voltage,'String'));    %voltage [mVppk]
% fgen_initialize_UTSW(freq,volt,0,cycle,1/PRF);
motor_initialize_Homing
handles.MotordisX = 0;
handles.MotordisY = 0;
handles.MotordisZ = 0;
fprintf('Motor is homed\n');
guidata(hObject, handles);

function displacement_Callback(hObject, eventdata, handles)
% hObject    handle to displacement (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of displacement as text
%        str2double(get(hObject,'String')) returns contents of displacement as a double



% --- Executes during object creation, after setting all properties.
function displacement_CreateFcn(hObject, eventdata, handles)
% hObject    handle to displacement (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% Manual movement of motor defined function
function manual_motor_move(hObject, handles, motor_dim, is_neg)
motor_movable = true;

% Check if valid motor step size
motor_movement_size = str2double(get(handles.motor_step_size,'String'));
if isnan(motor_movement_size) == true
    motor_movable = false;
end
% Check if valid movement speed
motor_movement_speed = round(str2double(get(handles.motor_speed,'String')));
if isnan(motor_movement_speed) == true
    motor_movable = false;
end
% Check if valid displacement
motor_movement_displacement = str2double(get(handles.displacement,'String'));
if isnan(motor_movement_displacement) == true
    motor_movable = false;
elseif is_neg
    motor_movement_displacement = -motor_movement_displacement;
end

% Move the motor
if motor_movable
    if abs(motor_movement_displacement) > 0          % CP
        motor_move(motor_dim, motor_movement_displacement);
    end
end


% --- Executes on button press in y_neg.
function y_neg_Callback(hObject, eventdata, handles)
% hObject    handle to y_neg (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

fprintf(['Y: -',get(handles.displacement,'String'),'mm \n'])
manual_motor_move(hObject, handles, 'Y', true);
handles.MotordisY = handles.MotordisY-str2double(get(handles.displacement,'String'));
guidata(hObject, handles);


% --- Executes on button press in y_pos.
function y_pos_Callback(hObject, eventdata, handles)
% hObject    handle to y_pos (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
fprintf(['Y: +',get(handles.displacement,'String'),'mm \n'])
manual_motor_move(hObject, handles, 'Y', false);
handles.MotordisY = handles.MotordisY+str2double(get(handles.displacement,'String'));
guidata(hObject, handles);


% --- Executes on button press in x_pos.
function x_pos_Callback(hObject, eventdata, handles)
% hObject    handle to x_pos (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
fprintf(['X: +',get(handles.displacement,'String'),'mm \n'])
manual_motor_move(hObject, handles, 'X', false);
handles.MotordisX = handles.MotordisX+str2double(get(handles.displacement,'String'));
guidata(hObject, handles);


% --- Executes on button press in x_neg.
function x_neg_Callback(hObject, eventdata, handles)
% hObject    handle to x_neg (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
fprintf(['X: -',get(handles.displacement,'String'),'mm \n'])
manual_motor_move(hObject, handles,'X', true);
handles.MotordisX = handles.MotordisX-str2double(get(handles.displacement,'String'));
guidata(hObject, handles);


% --- Executes on button press in z_neg.
function z_neg_Callback(hObject, eventdata, handles)
% hObject    handle to z_neg (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
fprintf(['Z: -',get(handles.displacement,'String'),'mm \n'])
manual_motor_move(hObject, handles,'Z', true);
handles.MotordisZ = handles.MotordisZ-str2double(get(handles.displacement,'String'));
guidata(hObject, handles);


% --- Executes on button press in z_pos.
function z_pos_Callback(hObject, eventdata, handles)
% hObject    handle to z_pos (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
fprintf(['Z: +',get(handles.displacement,'String'),'mm \n'])
manual_motor_move(hObject, handles,'Z', false);
handles.MotordisZ = handles.MotordisZ+str2double(get(handles.displacement,'String'));
guidata(hObject, handles);



% --- Executes on button press in Choose_file.
function Choose_file_Callback(hObject, eventdata, handles)
% hObject    handle to Choose_file (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
folder_name = uigetdir;
set(handles.directory,'String',folder_name);
%check_save_ready(hObject, handles);
guidata(hObject,handles);


function directory_Callback(hObject, eventdata, handles)
% hObject    handle to directory (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of directory as text
%        str2double(get(hObject,'String')) returns contents of directory as a double


% --- Executes during object creation, after setting all properties.
function directory_CreateFcn(hObject, eventdata, handles)
% hObject    handle to directory (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function studyID_Callback(hObject, eventdata, handles)
% hObject    handle to studyID (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of studyID as text
%        str2double(get(hObject,'String')) returns contents of studyID as a double


% --- Executes during object creation, after setting all properties.
function studyID_CreateFcn(hObject, eventdata, handles)
% hObject    handle to studyID (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function frequency_Callback(hObject, eventdata, handles)
% hObject    handle to frequency (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of frequency as text
%        str2double(get(hObject,'String')) returns contents of frequency as a double


% --- Executes during object creation, after setting all properties.
function frequency_CreateFcn(hObject, eventdata, handles)
% hObject    handle to frequency (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function voltage_Callback(hObject, eventdata, handles)
% hObject    handle to voltage (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of voltage as text
%        str2double(get(hObject,'String')) returns contents of voltage as a double


% --- Executes during object creation, after setting all properties.
function voltage_CreateFcn(hObject, eventdata, handles)
% hObject    handle to voltage (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function PRF_Callback(hObject, eventdata, handles)
% hObject    handle to PRF (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of PRF as text
%        str2double(get(hObject,'String')) returns contents of PRF as a double


% --- Executes during object creation, after setting all properties.
function PRF_CreateFcn(hObject, eventdata, handles)
% hObject    handle to PRF (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function BurstCount_Callback(hObject, eventdata, handles)
% hObject    handle to BurstCount (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of BurstCount as text
%        str2double(get(hObject,'String')) returns contents of BurstCount as a double


% --- Executes during object creation, after setting all properties.
function BurstCount_CreateFcn(hObject, eventdata, handles)
% hObject    handle to BurstCount (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function duration_Callback(hObject, eventdata, handles)
% hObject    handle to duration (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of duration as text
%        str2double(get(hObject,'String')) returns contents of duration as a double


% --- Executes during object creation, after setting all properties.
function duration_CreateFcn(hObject, eventdata, handles)
% hObject    handle to duration (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function sampleNum_Callback(hObject, eventdata, handles)
% hObject    handle to sampleNum (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of sampleNum as text
%        str2double(get(hObject,'String')) returns contents of sampleNum as a double


% --- Executes during object creation, after setting all properties.
function sampleNum_CreateFcn(hObject, eventdata, handles)
% hObject    handle to sampleNum (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function MBLoadTime_Callback(hObject, eventdata, handles)
% hObject    handle to MBLoadTime (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of MBLoadTime as text
%        str2double(get(hObject,'String')) returns contents of MBLoadTime as a double


% --- Executes during object creation, after setting all properties.
function MBLoadTime_CreateFcn(hObject, eventdata, handles)
% hObject    handle to MBLoadTime (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in PCDcontrol.
function PCDcontrol_Callback(hObject, eventdata, handles)
% hObject    handle to PCDcontrol (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global fgen

%% function generator initialization

SonicationDuration = 20;        % s, total time for sonication
Fs      = 40e6;             % Sampling Frequency
freq    = str2num(get(handles.frequency,'String'));      %frequency [MHz]
cycle   = str2num(get(handles.BurstCount,'String'));        %burst count
PRF     = str2num(get(handles.PRF,'String'));        %burst period [s]
PL      = 1/freq*cycle/1e6;  % Pulse length, s
volt    = 80;    %voltage [mVppk]
offset  = 0;
depth = str2num(get(handles.sampleNum,'String'));      %Sampling frequency * pulse length
Vstep = 1;
SC_range = (24915:25418); %forth harmonic
IC_range = (2895:3398); %sub-harmonic


directory= get(handles.directory,'String');
if directory(length(directory)) ~= '\'
    directory(length(directory)+1) = '\';
end
filename = ['PCDcontrol_',get(handles.studyID,'String')];


%% Pico initialization
PS5000aConfig;
assignin('base','ps5000aConfigInfo',ps5000aConfigInfo);
assignin('base','ps5000aEnuminfo',ps5000aEnuminfo);
assignin('base','ps5000aMethodinfo',ps5000aMethodinfo);
assignin('base','ps5000aStructs',ps5000aStructs);
assignin('base','ps5000aThunkLibName',ps5000aThunkLibName);

ps5000aDeviceObj = Pico_initialize;
[status.setChB]=invoke(ps5000aDeviceObj,'ps5000aSetChannel',1,0,1,8,0.0); % Turn off channel B
blockGroupObj = get(ps5000aDeviceObj, 'Block');
blockGroupObj = blockGroupObj(1);

Timebase = 1/Fs * 125e6+ 2; % Reverse formula to calculate timebase inside picoscope
set(ps5000aDeviceObj, 'timebase', Timebase);
[status.getTimebase, timeIntervalNanoSeconds, maxSamples] = invoke(ps5000aDeviceObj, 'ps5000aGetTimebase', Timebase, 0);

set(ps5000aDeviceObj, 'numPreTriggerSamples', offset);
set(ps5000aDeviceObj, 'numPostTriggerSamples', depth);


%% data acquisition and FFT setup
realFs = 1/(double(timeIntervalNanoSeconds)*1e-9);             % Sampling Frequency
NFFT =2^nextpow2(depth);%real_depth/4;%how to define real depth
F = realFs.*(0:(NFFT/2))/NFFT;%x axils

cd(directory)


%% Get  and set Figure handle
cla(handles.realtimeSCplot)
cla(handles.realtimeICplot)
cla(handles.realtimeVplot)

pulse = 1;
axes(handles.FFT_plot);
FFTax = gca;
axes(handles.Signal_plot);
Signalax = gca;
axes(handles.realtimeSCplot);
SCplotax = gca;
axes(handles.realtimeICplot);
ICplotax = gca;
axes(handles.realtimeVplot);
Vplotax = gca;



%% Start data acquisition
fgen_excute_UTSW(freq,volt,0,cycle,1/PRF);
fprintf(fgen,'OUTPut ON');
SonicationTimeLeft  = SonicationDuration;
SonicationStart = tic;

PulseCount = 1;

while  SonicationTimeLeft > 0
    
    BlockrecordTimeStart = tic;
    [runBlock] = invoke(blockGroupObj, 'runBlock', 0);
    [numSamples, overflow, chA, ~] = invoke(blockGroupObj, 'getBlockData', 0, 0, 1, 0);
    datamat(pulse,:) = chA;
    BlockrecordTime(pulse) = toc(BlockrecordTimeStart);
    
    set(handles.PulseNum,'String',['Before MB injection; pulse #',num2str(pulse)]);
    PCD_data = chA;
    tt = (0:(length(PCD_data)-1)).*double(timeIntervalNanoSeconds).*1e-9.*1e6;
    plot(Signalax,tt,PCD_data)
    set(Signalax, 'FontSize', 9,'FontWeight','bold');
    title(Signalax, 'Time Signal');
    xlabel(Signalax, 'Time (micro second)')
    ylabel(Signalax, 'Amplitude')      
    
    X = abs(fft(PCD_data,NFFT));
    Y = X(1:(NFFT/2)+1);
    semilogy(FFTax,F/1e6,Y);
    set(FFTax, 'XLim', [0 5]);
    set(FFTax, 'FontSize', 9,'FontWeight','bold');
    title(FFTax, 'FFT');
    xlabel(FFTax, 'Frequency (MHz)')
    
    %% Baseline acquisition
    Vbaselinerealtime(pulse) = volt;  
    RampSC_baseline(pulse) = sum(Y(SC_range));
    RampIC_baseline(pulse) = sum(Y(IC_range));
    fprintf(fgen, sprintf('VOLTage %3g mVPP', volt));
    
    %% Plot realtime SC, IC, V
    hold(SCplotax,'on')
    plot(SCplotax,pulse,RampSC_baseline(pulse),'b.')
    set(SCplotax, 'FontSize', 9,'FontWeight','bold');
    set(SCplotax, 'XLim', [0 SonicationDuration*2+5]);
    title(SCplotax, 'SC');
    xlabel(SCplotax, 'Pulse #')
    
    hold(ICplotax,'on')
    plot(ICplotax,pulse,RampIC_baseline(pulse),'b.')
    set(ICplotax, 'FontSize', 9,'FontWeight','bold');
    set(ICplotax, 'XLim', [0 SonicationDuration*PRF+5]);
    title(ICplotax, 'IC');
    xlabel(ICplotax, 'Pulse #')
    
    hold(Vplotax,'on')
    plot(Vplotax,pulse,Vbaselinerealtime(pulse),'bo')
    set(Vplotax, 'FontSize', 9,'FontWeight','bold');
    set(Vplotax, 'YLim', [0 50]);
    set(Vplotax, 'XLim', [0 SonicationDuration*PRF+5]);
    title(Vplotax, 'Voltage');
    xlabel(Vplotax, 'Pulse #')
    drawnow
    
    pulse = pulse + 1;
    %% Time remaining
    SonicationTimeLeft  = (SonicationDuration - toc(SonicationStart))
    if SonicationTimeLeft<=0
        fprintf(fgen,'OUTPut OFF');
    end
end

guidata(hObject, handles);

[stopBlock] = invoke(ps5000aDeviceObj, 'ps5000astop');
disconnect(ps5000aDeviceObj);
delete(ps5000aDeviceObj);
% Set times for saving file name
date_format = 'yyyy-mm-dd';
date_string = datestr(now, date_format);
time_format = 'HHMMSS';
time_string = datestr(now, time_format);
filename = sprintf([filename, '_', date_string,'_',time_string]);
save([directory,'NoMB_', filename '.mat']);
clear datamat BlockrecordTime tempSC_baseline tempIC_baseline RampSC_baseline RampIC_baseline stdSC_baseline stdIC_baseline;




% --- Executes on button press in Sonication.
function Sonication_Callback(hObject, eventdata, handles)
% hObject    handle to Sonication (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%% Sonication Parameters Definition
global fgen
BaselineDuration = 5; %5s, 10pulses
BaselineVolt = 43; %mVpp
ICeventMax = 1000; %Set to be 1000 for first pig in order to turn off this function
SonicationDuration = str2num(get(handles.duration,'String'));        % s, total time for sonication
Fs      = 40e6;             % Sampling Frequency
freq    = str2num(get(handles.frequency,'String'));      %frequency [MHz]
cycle   = str2num(get(handles.BurstCount,'String'));        %burst count
PRF     = str2num(get(handles.PRF,'String'));        %burst period [s]
PL      = 1/freq*cycle/1e6;  % Pulse length, s
volt    = str2num(get(handles.voltage,'String'));    %voltage [Vppk]
offset  = 0;
depth = str2num(get(handles.sampleNum,'String'));      %samples
Vstep = 10;
SC_range = (24915:25418); %forth harmonic
IC_range = (2895:3398); %sub-harmonic
RampSC=0;

ICthreshold_dB = 5;
TGT = str2num(get(handles.ControllerTarget,'String'));
OriTGT = TGT;
VrampFlag = true;
maxInputV = str2num(get(handles.MaxV,'String')); % 60mVpp = 0.834MPa for 1.5MHz

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Pico initialization
ps5000aDeviceObj = Pico_initialize;

[status.setChB]=invoke(ps5000aDeviceObj,'ps5000aSetChannel',1,0,1,8,0.0); % Turn off channel B
blockGroupObj = get(ps5000aDeviceObj, 'Block');
blockGroupObj = blockGroupObj(1);

Timebase = 1/Fs * 125e6+ 2;  % Reverse formula to calculate timebase inside picoscope
set(ps5000aDeviceObj, 'timebase', Timebase);
[status.getTimebase, timeIntervalNanoSeconds, maxSamples] = invoke(ps5000aDeviceObj, 'ps5000aGetTimebase', Timebase, 0);


set(ps5000aDeviceObj, 'numPreTriggerSamples', offset);
set(ps5000aDeviceObj, 'numPostTriggerSamples', depth);


%% data acquisition and FFT setup
realFs = 1/(double(timeIntervalNanoSeconds)*1e-9);             % Sampling Frequency
NFFT =2^nextpow2(depth);%real_depth/4;%how to define real depth
F = realFs*(0:(NFFT/2))/NFFT;%x axils
filename=get(handles.directory,'string');
if filename(length(filename)) ~= '\'
    filename(length(filename)+1) = '\';
end
studyIDName = get(handles.studyID,'String');


%% Get and Set Figure handles
cla(handles.realtimeSCplot)
cla(handles.realtimeICplot)
cla(handles.realtimeVplot)

pulse = 1;
axes(handles.FFT_plot);
FFTax = gca;
axes(handles.Signal_plot);
Signalax = gca;
axes(handles.realtimeSCplot);
SCplotax = gca;
axes(handles.realtimeICplot);
ICplotax = gca;
axes(handles.realtimeVplot);
Vplotax = gca;

%% Clear variables
clear tempSC_baseline tempIC_baseline Vbaselinerealtime


%% Start baseline acquisition
fgen_excute_UTSW(freq,BaselineVolt,0,cycle,1/PRF);
SonicationStart = tic;
MBLoadTime = str2num(get(handles.MBLoadTime,'String'));
SonicationDuration = BaselineDuration+MBLoadTime;
SonicationTimeLeft  = SonicationDuration;
FUSonFlag = true;
firstFUSon=1;
while  SonicationTimeLeft > 0
    if (toc(SonicationStart)>=MBLoadTime)&&(FUSonFlag)
        fprintf(fgen,'OUTPut ON');
        FUSonFlag=false;
        firstFUSon = pulse;
    end
    [runBlock] = invoke(blockGroupObj, 'runBlock', 0);
    [numSamples, overflow, chA, ~] = invoke(blockGroupObj, 'getBlockData', 0, 0, 1, 0);
    datamat(pulse,:) = chA;
    
    set(handles.PulseNum,'String',['Baseline acquisition: pulse #',num2str(pulse)]);
    PCD_data = chA;
    tt = (0:(length(PCD_data)-1)).*double(timeIntervalNanoSeconds).*1e-9.*1e6;
    plot(Signalax,tt,PCD_data)
    set(Signalax, 'FontSize', 9,'FontWeight','bold');
    title(Signalax, 'Time Signal');
    xlabel(Signalax, 'Time (micro second)')
    ylabel(Signalax, 'Amplitude')   
    
    X = abs(fft(PCD_data,NFFT));
    Y = X(1:(NFFT/2)+1);
    semilogy(FFTax,F/1e6,Y);
    set(FFTax, 'XLim', [0 5]);
    set(FFTax, 'FontSize', 9,'FontWeight','bold');
    title(FFTax, 'FFT');
    xlabel(FFTax, 'Frequency (MHz)')
    
    % Baseline acquisition
    tempSC_baseline(pulse) = sum(Y(SC_range));
    tempIC_baseline(pulse) = sum(Y(IC_range));
    Vrealtime(pulse) = BaselineVolt;  
    % Plot realtime SC, IC, V
    hold(SCplotax,'on')
    plot(SCplotax,pulse,tempSC_baseline(pulse),'b.')
    set(SCplotax, 'FontSize', 9,'FontWeight','bold');
    set(SCplotax, 'XLim', [0 60]);
    title(SCplotax, 'SC');
    xlabel(SCplotax, 'Pulse #')

    hold(ICplotax,'on')
    plot(ICplotax,pulse,tempIC_baseline(pulse),'b.')
    set(ICplotax, 'FontSize', 9,'FontWeight','bold');
    set(ICplotax, 'XLim', [0 60]);
    title(ICplotax, 'IC');
    xlabel(ICplotax, 'Pulse #')

    hold(Vplotax,'on')
    plot(Vplotax,pulse,Vrealtime(pulse),'bo')
    set(Vplotax, 'FontSize', 9,'FontWeight','bold');
    set(Vplotax, 'YLim', [0 50]);
    set(Vplotax, 'XLim', [0 60]);
    title(Vplotax, 'Voltage');
    xlabel(Vplotax, 'Pulse #')
    drawnow
    
    pulse = pulse + 1;
    % Time remaining
    SonicationTimeLeft  = (SonicationDuration - toc(SonicationStart))
    if SonicationTimeLeft<=0
        fprintf(fgen,'OUTPut OFF');
    end
end
stdSC_baseline = std(tempSC_baseline(firstFUSon:end));
stdIC_baseline = std(tempIC_baseline(firstFUSon:end));
RampSC_baseline = mean(tempSC_baseline(firstFUSon:end));
RampIC_baseline = mean(tempIC_baseline(firstFUSon:end));
OriSCdesired = 10^(log10(RampSC_baseline*(10^(TGT/10))));
SCdesired = 10^(log10(RampSC_baseline*(10^(TGT/10))));


%% Start treatment acquisition
fgen_excute_UTSW(freq,volt,0,cycle,1/PRF);
fprintf(fgen,'OUTPut ON');
SonicationStart = tic;
SonicationDuration = str2num(get(handles.duration,'String'));        % s, total time for sonication
SonicationTimeLeft  = SonicationDuration;
sumRampSC = 0;
ICevent = 0;
while  (SonicationTimeLeft > 0)

    [runBlock] = invoke(blockGroupObj, 'runBlock', 0);
    [numSamples, overflow, chA, ~] = invoke(blockGroupObj, 'getBlockData', 0, 0, 1, 0);
    datamat(pulse,:) = chA;
    
    set(handles.PulseNum,'String',['During MB injection; pulse #',num2str(pulse)]);
    PCD_data = chA;
    tt = (0:(length(PCD_data)-1)).*double(timeIntervalNanoSeconds).*1e-9.*1e6;
    plot(Signalax,tt,PCD_data)
    set(Signalax, 'FontSize', 9,'FontWeight','bold');
    title(Signalax, 'Time Signal');
    xlabel(Signalax, 'Time (micro second)')
    
    
    X = abs(fft(PCD_data,NFFT));
    Y = X(1:(NFFT/2)+1); 
    semilogy(FFTax,F/1e6,Y); 
    set(FFTax, 'XLim', [0 5]);
    set(FFTax, 'FontSize', 9,'FontWeight','bold');
    title(FFTax, 'FFT');
    xlabel(FFTax, 'Frequency (MHz)')

    
    %% Feedback control processing
    Vrealtime(pulse) = volt;
    RampSC(pulse) = sum(Y(SC_range));%-handles.RampSC_baseline(volt);
    RampIC(pulse) = sum(Y(IC_range));%-handles.RampIC_baseline(volt);
    
    sumRampSC = sum(RampSC(firstFUSon:end));
    if ICevent<ICeventMax
        if VrampFlag      
            if ((RampSC(pulse)>= SCdesired)&&(firstFUSon~=pulse))
                FeedbackEvent = pulse;
                VrampFlag = false;
                Tolcoeff = 0.4;
                tolerance_posrange = 10^(log10(RampSC_baseline*(10^((TGT+Tolcoeff)/10))));
                tolerance_negrange = 10^(log10(RampSC_baseline*(10^((TGT-Tolcoeff)/10))));

            else
                volt = volt+Vstep;
                if volt>maxInputV, volt = maxInputV; end
            end

        else
            step = 1;
            if (RampSC(pulse)>tolerance_posrange)
                volt = volt-step;
            elseif (RampSC(pulse)<tolerance_negrange)
                volt = volt+step;
                if volt>maxInputV, volt = maxInputV; end
            end
        end
        if RampIC(pulse)>(10^(log10(RampIC_baseline*(10^(ICthreshold_dB/10)))))
            ICevent = ICevent+1;
        end
        fprintf(fgen, sprintf('VOLTage %3g mVPP', volt));
        FgenStopEvent = pulse+1;
    else
        TGT = TGT - 0.5;
        if TGT>0
            SCdesired = 10^(log10(RampSC_baseline*(10^(TGT/10))));
            ICevent = 0;   
        else
            TGT = 0.4;
            fprintf(fgen,'OUTPut OFF');
            volt = 0;
        end
        
    end
    
    
    %% Plot realtime SC, IC, V
    hold(SCplotax,'on')
    plot(SCplotax,pulse,RampSC(pulse),'b.')
    set(SCplotax, 'FontSize', 9,'FontWeight','bold');
    set(SCplotax, 'XLim', [0 SonicationDuration*PRF+30]);
    title(SCplotax, 'SC');
    xlabel(SCplotax, 'Pulse #')
    
    hold(ICplotax,'on')
    plot(ICplotax,pulse,RampIC(pulse),'b.')
    set(ICplotax, 'FontSize', 9,'FontWeight','bold');
    set(ICplotax, 'XLim', [0 SonicationDuration*PRF+30]);
    title(ICplotax, 'IC');
    xlabel(ICplotax, 'Pulse #')
    
    hold(Vplotax,'on')
    plot(Vplotax,pulse,Vrealtime(pulse),'bo')
    set(Vplotax, 'FontSize', 9,'FontWeight','bold');
    set(Vplotax, 'YLim', [0 maxInputV]);
    set(Vplotax, 'XLim', [0 SonicationDuration*PRF+30]);
    title(Vplotax, 'Voltage');
    xlabel(Vplotax, 'Pulse #')
    drawnow
    
    %% Time remaining
    SonicationTimeLeft  = (SonicationDuration - toc(SonicationStart))%(SonicationDuration - toc)/60;
    TimeRecord(pulse) = SonicationTimeLeft;
    if (SonicationTimeLeft<=0)
        fprintf(fgen,'OUTPut OFF');
    end
    pulse = pulse + 1;
end

[stopBlock] = invoke(ps5000aDeviceObj, 'ps5000astop');
disconnect(ps5000aDeviceObj);
delete(ps5000aDeviceObj);
date_format = 'yyyy-mm-dd';
date_string = datestr(now, date_format);
time_format = 'HHMMSS';
time_string = datestr(now, time_format);
studyIDName = sprintf([studyIDName, '_', date_string,'_',time_string]);
save([filename studyIDName '.mat']);
clear datamat BlockrecordTime Vrealtime RampSC RampIC TimeRecord



function ControllerTarget_Callback(hObject, eventdata, handles)
% hObject    handle to ControllerTarget (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ControllerTarget as text
%        str2double(get(hObject,'String')) returns contents of ControllerTarget as a double


% --- Executes during object creation, after setting all properties.
function ControllerTarget_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ControllerTarget (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function MaxV_Callback(hObject, eventdata, handles)
% hObject    handle to MaxV (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of MaxV as text
%        str2double(get(hObject,'String')) returns contents of MaxV as a double


% --- Executes during object creation, after setting all properties.
function MaxV_CreateFcn(hObject, eventdata, handles)
% hObject    handle to MaxV (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end




% --- Executes on button press in stop.
function stop_Callback(hObject, eventdata, handles)
% hObject    handle to stop (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global fgen
fprintf(fgen,'OUTPut OFF');



function edit40_Callback(hObject, eventdata, handles)
% hObject    handle to edit40 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit40 as text
%        str2double(get(hObject,'String')) returns contents of edit40 as a double


% --- Executes during object creation, after setting all properties.
function edit40_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit40 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in IniFgen.
function IniFgen_Callback(hObject, eventdata, handles)
% hObject    handle to IniFgen (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% 
global fgen

%% function generator initialization

% fgen_inital_Verasonics
freq    = str2num(get(handles.frequency,'String'));      %frequency [MHz]
cycle   = str2num(get(handles.BurstCount,'String'));        %burst count
PRF     = str2num(get(handles.PRF,'String'));        %burst period [s]
volt    = str2num(get(handles.voltage,'String'));    %voltage [mVppk]
fgen_initialize_UTSW(freq,volt,0,cycle,1/PRF);


% --- Executes on button press in IniDis.
function IniDis_Callback(hObject, eventdata, handles)
% hObject    handle to IniDis (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.MotordisX = 0;
handles.MotordisY = 0;
handles.MotordisZ = 0;
guidata(hObject, handles);
fprintf('Initialize Displacement Count \n')


% --- Executes on button press in CountDis.
function CountDis_Callback(hObject, eventdata, handles)
% hObject    handle to CountDis (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
fprintf(['\n\n=========================\n'])
fprintf(['X: ',num2str(handles.MotordisX),' mm \n'])
fprintf(['Y: ',num2str(handles.MotordisY),' mm \n'])
fprintf(['Z: ',num2str(handles.MotordisZ),' mm \n'])
fprintf(['=========================\n\n'])
