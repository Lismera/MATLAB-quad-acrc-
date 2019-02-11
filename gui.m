function varargout = gui(varargin)
% GUI MATLAB code for gui.fig
%      GUI, by itself, creates a new GUI or raises the existing
%      singleton*.
%
%      H = GUI returns the handle to a new GUI or the handle to
%      the existing singleton*.
%
%      GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in GUI.M with the given input arguments.
%
%      GUI('Property','Value',...) creates a new GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before gui_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to gui_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help gui

% Last Modified by GUIDE v2.5 11-Dec-2018 00:15:57

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @gui_OpeningFcn, ...
                   'gui_OutputFcn',  @gui_OutputFcn, ...
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


% --- Executes just before gui is made visible.
function gui_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to gui (see VARARGIN)

% Choose default command line output for gui
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes gui wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = gui_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double


% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO) 
global plot_clicks;
global Ang_rot;
global Xst; 
global Yst; 
global Xrct; 
global Yrct; 
global Xrot; 
global Yrot;
global rct_hd;
global cir_hd;
global arc_hd;
plot_clicks = plot_clicks + 1;
rct_hd=0;
cir_hd=0;
arc_hd=0;


axes(handles.axes2); % Make averSpec the current axes.
%cla reset; % Do a complete and total reset of the axes.

x_center = get(handles.edit1, 'string');
x_center = str2double(x_center);

y_center = get(handles.edit2, 'string');
y_center = str2double(y_center);

a = get(handles.edit3, 'string');
a = str2double(a)/2;

b = get(handles.edit6, 'string');
b = str2double(b)/2;

a_rot = get(handles.edit7, 'string'); 
a_rot = str2double(a_rot);
Ang_rot(plot_clicks, : ) = a_rot;

Xrct(plot_clicks, : ) = [x_center-a, x_center-a, x_center+a,x_center+a,x_center-a];
Yrct(plot_clicks, : ) = [y_center-b,y_center+b,y_center+b,y_center-b,y_center-b];


hold on
%plot (Xrct, Yrct)
axis equal

k = ((a-b)*(a + 3*b + sqrt(a^2 + 6*a*b + b^2)))/(4*b);
h = (a-b)* (a+b+sqrt(a^2+6*a*b+b^2))/(a-b+sqrt(a^2+6*a*b+b^2));



%возвращаем вниз
Xst(plot_clicks, : ) = Xrct(plot_clicks, : ) - x_center
Yst(plot_clicks, : ) = Yrct(plot_clicks, : ) - y_center;

%plot (Xst, Yst)

%крутим
Xrot(plot_clicks, : ) = cosd(a_rot)*Xst(plot_clicks, :)-sind(a_rot)*Yst(plot_clicks, :);
Yrot(plot_clicks, : ) = sind(a_rot)*Xst(plot_clicks, :)+cosd(a_rot)*Yst(plot_clicks, :);

%plot (Xrot, Yrot)

%возвращаем обратно
Xrot(plot_clicks, : ) = Xrot(plot_clicks, : )+x_center;
Yrot(plot_clicks, : ) = Yrot(plot_clicks, : )+y_center;

plot (Xrot(plot_clicks, :), Yrot(plot_clicks, :), 'b')

rad1 = b+k;
rad2 = a-h;

bkj = atand(h/k);
j = 90-bkj;

%ротирование дуг на угол
angle1 = (j+a_rot):1/1000:(180-j+a_rot); 
angle2 = (a_rot+180+j):1/1000:(360-j+a_rot);
angle3 = (a_rot+180-j):1/1000:(a_rot+180+j);
angle4 = (a_rot-j):1/1000:(j+a_rot);

x1_circ = x_center + k*sind(a_rot); %смещение это синус * к
y1_circ = y_center - k + (k - k*cosd(a_rot));

x2_circ = x_center - k*sind(a_rot);
y2_circ = y_center + k - (k - k*cosd(a_rot));

x3_circ = x_center - h + (h - h*cosd(a_rot));
y3_circ = y_center - h*sind(a_rot);

x4_circ = x_center + h - (h - h*cosd(a_rot));
y4_circ = y_center + h*sind(a_rot);

x1 = rad1*cosd(angle1)+ x1_circ;
y1 = rad1*sind(angle1)+ y1_circ;

x2 = rad1*cosd(angle2) + x2_circ;
y2 = rad1*sind(angle2) + y2_circ;

x3 = rad2*cosd(angle3) + x3_circ;
y3 = rad2*sind(angle3) + y3_circ;

x4 = rad2*cosd(angle4) + x4_circ;
y4 = rad2*sind(angle4) + y4_circ;

plot (x1, y1, 'b')
plot (x2, y2, 'b')
plot (x3, y3, 'b')
plot (x4, y4, 'b')
 
rectangle('Position',[x1_circ-rad1 y1_circ-rad1 rad1*2 rad1*2],'Curvature',[1 1],'LineStyle', ['-.'], 'EdgeColor','b');
hold on
rectangle('Position',[x2_circ-rad1 y2_circ-rad1 rad1*2 rad1*2],'Curvature',[1 1],'LineStyle', ['-.'],'EdgeColor','b');
hold on
rectangle('Position',[x3_circ-rad2 y3_circ-rad2 rad2*2 rad2*2],'Curvature',[1 1],'LineStyle', [':'],'EdgeColor','b');
hold on
rectangle('Position',[x4_circ-rad2 y4_circ-rad2 rad2*2 rad2*2],'Curvature',[1 1], 'LineStyle', [':'],'EdgeColor','b');
hold on
%axis equal;


% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



function edit2_Callback(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

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



function edit3_Callback(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit3 as text
%        str2double(get(hObject,'String')) returns contents of edit3 as a double


% --- Executes during object creation, after setting all properties.
function edit3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
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



function edit7_Callback(hObject, eventdata, handles)
% hObject    handle to edit7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit7 as text
%        str2double(get(hObject,'String')) returns contents of edit7 as a double


% --- Executes during object creation, after setting all properties.
function edit7_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in togglebutton1.
function togglebutton1_Callback(hObject, eventdata, handles)
if x1_circ == x1 || x1_circ == x2
    plot (x1, y1, 'r')
end    
% hObject    handle to togglebutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of togglebutton1


% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global plot_clicks;
global Ang_rot;
global Xst; 
global Yst; 
global Xrct; 
global Yrct; 
global Xrot; 
global Yrot; 
global rct_hd;
global cir_hd;
global arc_hd;
rct_hd=0;
clnumber = length(Xrot(:,1));

for iii = (1:clnumber)

axes(handles.axes2); % Make averSpec the current axes.
%cla reset; % Do a complete and total reset of the axes.
a = (Xrct(iii, 3)-Xrct(iii, 2))/2;
b = (Yrct(iii, 2)-Yrct(iii, 1))/2;
x_center = Xrct(iii, 4) - a;
y_center = Yrct(iii, 4) + b;
a_rot =  Ang_rot(iii);

k = ((a-b)*(a + 3*b + sqrt(a^2 + 6*a*b + b^2)))/(4*b);
h = (a-b)* (a+b+sqrt(a^2+6*a*b+b^2))/(a-b+sqrt(a^2+6*a*b+b^2));



plot (Xrot(iii, :), Yrot(iii, :), 'b')

axis equal

rad1 = b+k;
rad2 = a-h;

bkj = atand(h/k);
j = 90-bkj;

%ротирование дуг на угол
angle1 = (j+a_rot):1/1000:(180-j+a_rot); 
angle2 = (a_rot+180+j):1/1000:(360-j+a_rot);
angle3 = (a_rot+180-j):1/1000:(a_rot+180+j);
angle4 = (a_rot-j):1/1000:(j+a_rot);

x1_circ = x_center + k*sind(a_rot); %смещение это синус * к
y1_circ = y_center - k + (k - k*cosd(a_rot));

x2_circ = x_center - k*sind(a_rot);
y2_circ = y_center + k - (k - k*cosd(a_rot));

x3_circ = x_center - h + (h - h*cosd(a_rot));
y3_circ = y_center - h*sind(a_rot);

x4_circ = x_center + h - (h - h*cosd(a_rot));
y4_circ = y_center + h*sind(a_rot);

x1 = rad1*cosd(angle1)+ x1_circ;
y1 = rad1*sind(angle1)+ y1_circ;

x2 = rad1*cosd(angle2) + x2_circ;
y2 = rad1*sind(angle2) + y2_circ;

x3 = rad2*cosd(angle3) + x3_circ;
y3 = rad2*sind(angle3) + y3_circ;

x4 = rad2*cosd(angle4) + x4_circ;
y4 = rad2*sind(angle4) + y4_circ;

if arc_hd==0
plot (x1, y1, 'b')
plot (x2, y2, 'b')
plot (x3, y3, 'b')
plot (x4, y4, 'b')
end 


if cir_hd==0
rectangle('Position',[x1_circ-rad1 y1_circ-rad1 rad1*2 rad1*2],'Curvature',[1 1],'LineStyle', ['-.'], 'EdgeColor','b');
hold on
rectangle('Position',[x2_circ-rad1 y2_circ-rad1 rad1*2 rad1*2],'Curvature',[1 1],'LineStyle', ['-.'],'EdgeColor','b');
hold on
rectangle('Position',[x3_circ-rad2 y3_circ-rad2 rad2*2 rad2*2],'Curvature',[1 1],'LineStyle', [':'],'EdgeColor','b');
hold on
rectangle('Position',[x4_circ-rad2 y4_circ-rad2 rad2*2 rad2*2],'Curvature',[1 1], 'LineStyle', [':'],'EdgeColor','b');
hold on
end

end

% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton5.
function pushbutton5_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton6.
function pushbutton6_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton7.
function pushbutton7_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global plot_clicks;
global Ang_rot;
global Xst; 
global Yst; 
global Xrct; 
global Yrct; 
global Xrot; 
global Yrot; 
global rct_hd;
global cir_hd;
global arc_hd;
rct_hd=1;
clnumber = length(Xrot(:,1));

for iii = (1:clnumber)

axes(handles.axes2); % Make averSpec the current axes.
%cla reset; % Do a complete and total reset of the axes.
a = (Xrct(iii, 3)-Xrct(iii, 2))/2;
b = (Yrct(iii, 2)-Yrct(iii, 1))/2;
x_center = Xrct(iii, 4) - a;
y_center = Yrct(iii, 4) + b;
a_rot =  Ang_rot(iii);

k = ((a-b)*(a + 3*b + sqrt(a^2 + 6*a*b + b^2)))/(4*b);
h = (a-b)* (a+b+sqrt(a^2+6*a*b+b^2))/(a-b+sqrt(a^2+6*a*b+b^2));



plot (Xrot(iii, :), Yrot(iii, :), 'w')

axis equal

rad1 = b+k;
rad2 = a-h;

bkj = atand(h/k);
j = 90-bkj;

%ротирование дуг на угол
angle1 = (j+a_rot):1/1000:(180-j+a_rot); 
angle2 = (a_rot+180+j):1/1000:(360-j+a_rot);
angle3 = (a_rot+180-j):1/1000:(a_rot+180+j);
angle4 = (a_rot-j):1/1000:(j+a_rot);

x1_circ = x_center + k*sind(a_rot); %смещение это синус * к
y1_circ = y_center - k + (k - k*cosd(a_rot));

x2_circ = x_center - k*sind(a_rot);
y2_circ = y_center + k - (k - k*cosd(a_rot));

x3_circ = x_center - h + (h - h*cosd(a_rot));
y3_circ = y_center - h*sind(a_rot);

x4_circ = x_center + h - (h - h*cosd(a_rot));
y4_circ = y_center + h*sind(a_rot);

x1 = rad1*cosd(angle1)+ x1_circ;
y1 = rad1*sind(angle1)+ y1_circ;

x2 = rad1*cosd(angle2) + x2_circ;
y2 = rad1*sind(angle2) + y2_circ;

x3 = rad2*cosd(angle3) + x3_circ;
y3 = rad2*sind(angle3) + y3_circ;

x4 = rad2*cosd(angle4) + x4_circ;
y4 = rad2*sind(angle4) + y4_circ;

if arc_hd==0
plot (x1, y1, 'b')
plot (x2, y2, 'b')
plot (x3, y3, 'b')
plot (x4, y4, 'b')
end 

if cir_hd==0
rectangle('Position',[x1_circ-rad1 y1_circ-rad1 rad1*2 rad1*2],'Curvature',[1 1],'LineStyle', ['-.'], 'EdgeColor','b');
hold on
rectangle('Position',[x2_circ-rad1 y2_circ-rad1 rad1*2 rad1*2],'Curvature',[1 1],'LineStyle', ['-.'],'EdgeColor','b');
hold on
rectangle('Position',[x3_circ-rad2 y3_circ-rad2 rad2*2 rad2*2],'Curvature',[1 1],'LineStyle', [':'],'EdgeColor','b');
hold on
rectangle('Position',[x4_circ-rad2 y4_circ-rad2 rad2*2 rad2*2],'Curvature',[1 1], 'LineStyle', [':'],'EdgeColor','b');
hold on
end

end


% --- Executes on button press in pushbutton8.
function pushbutton8_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global plot_clicks;
global Ang_rot;
global Xst; 
global Yst; 
global Xrct; 
global Yrct; 
global Xrot; 
global Yrot; 
global rct_hd;
global cir_hd;
global arc_hd;
cir_hd=0;
clnumber = length(Xrot(:,1));

for iii = (1:clnumber)

axes(handles.axes2); % Make averSpec the current axes.
%cla reset; % Do a complete and total reset of the axes.
a = (Xrct(iii, 3)-Xrct(iii, 2))/2;
b = (Yrct(iii, 2)-Yrct(iii, 1))/2;
x_center = Xrct(iii, 4) - a;
y_center = Yrct(iii, 4) + b;
a_rot =  Ang_rot(iii);

k = ((a-b)*(a + 3*b + sqrt(a^2 + 6*a*b + b^2)))/(4*b);
h = (a-b)* (a+b+sqrt(a^2+6*a*b+b^2))/(a-b+sqrt(a^2+6*a*b+b^2));



axis equal

rad1 = b+k;
rad2 = a-h;

bkj = atand(h/k);
j = 90-bkj;

%ротирование дуг на угол
angle1 = (j+a_rot):1/1000:(180-j+a_rot); 
angle2 = (a_rot+180+j):1/1000:(360-j+a_rot);
angle3 = (a_rot+180-j):1/1000:(a_rot+180+j);
angle4 = (a_rot-j):1/1000:(j+a_rot);

x1_circ = x_center + k*sind(a_rot); %смещение это синус * к
y1_circ = y_center - k + (k - k*cosd(a_rot));

x2_circ = x_center - k*sind(a_rot);
y2_circ = y_center + k - (k - k*cosd(a_rot));

x3_circ = x_center - h + (h - h*cosd(a_rot));
y3_circ = y_center - h*sind(a_rot);

x4_circ = x_center + h - (h - h*cosd(a_rot));
y4_circ = y_center + h*sind(a_rot);

x1 = rad1*cosd(angle1)+ x1_circ;
y1 = rad1*sind(angle1)+ y1_circ;

x2 = rad1*cosd(angle2) + x2_circ;
y2 = rad1*sind(angle2) + y2_circ;

x3 = rad2*cosd(angle3) + x3_circ;
y3 = rad2*sind(angle3) + y3_circ;

x4 = rad2*cosd(angle4) + x4_circ;
y4 = rad2*sind(angle4) + y4_circ;


 
rectangle('Position',[x1_circ-rad1 y1_circ-rad1 rad1*2 rad1*2],'Curvature',[1 1],'LineStyle', ['-.'], 'EdgeColor','b');
hold on
rectangle('Position',[x2_circ-rad1 y2_circ-rad1 rad1*2 rad1*2],'Curvature',[1 1],'LineStyle', ['-.'],'EdgeColor','b');
hold on
rectangle('Position',[x3_circ-rad2 y3_circ-rad2 rad2*2 rad2*2],'Curvature',[1 1],'LineStyle', [':'],'EdgeColor','b');
hold on
rectangle('Position',[x4_circ-rad2 y4_circ-rad2 rad2*2 rad2*2],'Curvature',[1 1], 'LineStyle', [':'],'EdgeColor','b');
hold on

if rct_hd==0
plot (Xrot(iii, :), Yrot(iii, :), 'b')
end

if arc_hd==0
plot (x1, y1, 'b')
plot (x2, y2, 'b')
plot (x3, y3, 'b')
plot (x4, y4, 'b')
end

end

% --- Executes on button press in pushbutton9.
function pushbutton9_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global plot_clicks;
global Ang_rot;
global Xst; 
global Yst; 
global Xrct; 
global Yrct; 
global Xrot; 
global Yrot; 
global rct_hd;
global cir_hd;
global arc_hd;
cir_hd=1;
clnumber = length(Xrot(:,1));
for iii = (1:clnumber)

axes(handles.axes2); % Make averSpec the current axes.
%cla reset; % Do a complete and total reset of the axes.
a = (Xrct(iii, 3)-Xrct(iii, 2))/2;
b = (Yrct(iii, 2)-Yrct(iii, 1))/2;
x_center = Xrct(iii, 4) - a;
y_center = Yrct(iii, 4) + b;
a_rot =  Ang_rot(iii);

k = ((a-b)*(a + 3*b + sqrt(a^2 + 6*a*b + b^2)))/(4*b);
h = (a-b)* (a+b+sqrt(a^2+6*a*b+b^2))/(a-b+sqrt(a^2+6*a*b+b^2));



axis equal

rad1 = b+k;
rad2 = a-h;

bkj = atand(h/k);
j = 90-bkj;

%ротирование дуг на угол
angle1 = (j+a_rot):1/1000:(180-j+a_rot); 
angle2 = (a_rot+180+j):1/1000:(360-j+a_rot);
angle3 = (a_rot+180-j):1/1000:(a_rot+180+j);
angle4 = (a_rot-j):1/1000:(j+a_rot);

x1_circ = x_center + k*sind(a_rot); %смещение это синус * к
y1_circ = y_center - k + (k - k*cosd(a_rot));

x2_circ = x_center - k*sind(a_rot);
y2_circ = y_center + k - (k - k*cosd(a_rot));

x3_circ = x_center - h + (h - h*cosd(a_rot));
y3_circ = y_center - h*sind(a_rot);

x4_circ = x_center + h - (h - h*cosd(a_rot));
y4_circ = y_center + h*sind(a_rot);

x1 = rad1*cosd(angle1)+ x1_circ;
y1 = rad1*sind(angle1)+ y1_circ;

x2 = rad1*cosd(angle2) + x2_circ;
y2 = rad1*sind(angle2) + y2_circ;

x3 = rad2*cosd(angle3) + x3_circ;
y3 = rad2*sind(angle3) + y3_circ;

x4 = rad2*cosd(angle4) + x4_circ;
y4 = rad2*sind(angle4) + y4_circ;

 
rectangle('Position',[x1_circ-rad1 y1_circ-rad1 rad1*2 rad1*2],'Curvature',[1 1],'LineStyle', ['-.'], 'EdgeColor','w');
hold on
rectangle('Position',[x2_circ-rad1 y2_circ-rad1 rad1*2 rad1*2],'Curvature',[1 1],'LineStyle', ['-.'],'EdgeColor','w');
hold on
rectangle('Position',[x3_circ-rad2 y3_circ-rad2 rad2*2 rad2*2],'Curvature',[1 1],'LineStyle', [':'],'EdgeColor','w');
hold on
rectangle('Position',[x4_circ-rad2 y4_circ-rad2 rad2*2 rad2*2],'Curvature',[1 1], 'LineStyle', [':'],'EdgeColor','w');
hold on

if rct_hd==0
plot (Xrot(iii, :), Yrot(iii, :), 'b')
end

if arc_hd==0
plot (x1, y1, 'b')
plot (x2, y2, 'b')
plot (x3, y3, 'b')
plot (x4, y4, 'b')
end

end
% --- Executes on button press in pushbutton10.
function pushbutton10_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global plot_clicks;
global Ang_rot;
global Xst; 
global Yst; 
global Xrct; 
global Yrct; 
global Xrot; 
global Yrot; 
global rct_hd;
global cir_hd;
global arc_hd;
arc_hd=0;
clnumber = length(Xrot(:,1));
for iii = (1:clnumber)

axes(handles.axes2); % Make averSpec the current axes.
%cla reset; % Do a complete and total reset of the axes.
a = (Xrct(iii, 3)-Xrct(iii, 2))/2;
b = (Yrct(iii, 2)-Yrct(iii, 1))/2;
x_center = Xrct(iii, 4) - a;
y_center = Yrct(iii, 4) + b;
a_rot =  Ang_rot(iii);

k = ((a-b)*(a + 3*b + sqrt(a^2 + 6*a*b + b^2)))/(4*b);
h = (a-b)* (a+b+sqrt(a^2+6*a*b+b^2))/(a-b+sqrt(a^2+6*a*b+b^2));



axis equal

rad1 = b+k;
rad2 = a-h;

bkj = atand(h/k);
j = 90-bkj;

%ротирование дуг на угол
angle1 = (j+a_rot):1/1000:(180-j+a_rot); 
angle2 = (a_rot+180+j):1/1000:(360-j+a_rot);
angle3 = (a_rot+180-j):1/1000:(a_rot+180+j);
angle4 = (a_rot-j):1/1000:(j+a_rot);

x1_circ = x_center + k*sind(a_rot); %смещение это синус * к
y1_circ = y_center - k + (k - k*cosd(a_rot));

x2_circ = x_center - k*sind(a_rot);
y2_circ = y_center + k - (k - k*cosd(a_rot));

x3_circ = x_center - h + (h - h*cosd(a_rot));
y3_circ = y_center - h*sind(a_rot);

x4_circ = x_center + h - (h - h*cosd(a_rot));
y4_circ = y_center + h*sind(a_rot);

x1 = rad1*cosd(angle1)+ x1_circ;
y1 = rad1*sind(angle1)+ y1_circ;

x2 = rad1*cosd(angle2) + x2_circ;
y2 = rad1*sind(angle2) + y2_circ;

x3 = rad2*cosd(angle3) + x3_circ;
y3 = rad2*sind(angle3) + y3_circ;

x4 = rad2*cosd(angle4) + x4_circ;
y4 = rad2*sind(angle4) + y4_circ;

plot (x1, y1, 'b')
plot (x2, y2, 'b')
plot (x3, y3, 'b')
plot (x4, y4, 'b')

if rct_hd==0
plot (Xrot(iii, :), Yrot(iii, :), 'b')
end

if cir_hd==0
rectangle('Position',[x1_circ-rad1 y1_circ-rad1 rad1*2 rad1*2],'Curvature',[1 1],'LineStyle', ['-.'], 'EdgeColor','b');
hold on
rectangle('Position',[x2_circ-rad1 y2_circ-rad1 rad1*2 rad1*2],'Curvature',[1 1],'LineStyle', ['-.'],'EdgeColor','b');
hold on
rectangle('Position',[x3_circ-rad2 y3_circ-rad2 rad2*2 rad2*2],'Curvature',[1 1],'LineStyle', [':'],'EdgeColor','b');
hold on
rectangle('Position',[x4_circ-rad2 y4_circ-rad2 rad2*2 rad2*2],'Curvature',[1 1], 'LineStyle', [':'],'EdgeColor','b');
hold on
end

end

% --- Executes on button press in pushbutton11.
function pushbutton11_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global plot_clicks;
global Ang_rot;
global Xst; 
global Yst; 
global Xrct; 
global Yrct; 
global Xrot; 
global Yrot; 
global rct_hd;
global cir_hd;
global arc_hd;
arc_hd=1;
clnumber = length(Xrot(:,1));
for iii = (1:clnumber)

axes(handles.axes2); % Make averSpec the current axes.
%cla reset; % Do a complete and total reset of the axes.
a = (Xrct(iii, 3)-Xrct(iii, 2))/2;
b = (Yrct(iii, 2)-Yrct(iii, 1))/2;
x_center = Xrct(iii, 4) - a;
y_center = Yrct(iii, 4) + b;
a_rot =  Ang_rot(iii);

k = ((a-b)*(a + 3*b + sqrt(a^2 + 6*a*b + b^2)))/(4*b);
h = (a-b)* (a+b+sqrt(a^2+6*a*b+b^2))/(a-b+sqrt(a^2+6*a*b+b^2));


axis equal

rad1 = b+k;
rad2 = a-h;

bkj = atand(h/k);
j = 90-bkj;

%ротирование дуг на угол
angle1 = (j+a_rot):1/1000:(180-j+a_rot); 
angle2 = (a_rot+180+j):1/1000:(360-j+a_rot);
angle3 = (a_rot+180-j):1/1000:(a_rot+180+j);
angle4 = (a_rot-j):1/1000:(j+a_rot);

x1_circ = x_center + k*sind(a_rot); %смещение это синус * к
y1_circ = y_center - k + (k - k*cosd(a_rot));

x2_circ = x_center - k*sind(a_rot);
y2_circ = y_center + k - (k - k*cosd(a_rot));

x3_circ = x_center - h + (h - h*cosd(a_rot));
y3_circ = y_center - h*sind(a_rot);

x4_circ = x_center + h - (h - h*cosd(a_rot));
y4_circ = y_center + h*sind(a_rot);

x1 = rad1*cosd(angle1)+ x1_circ;
y1 = rad1*sind(angle1)+ y1_circ;

x2 = rad1*cosd(angle2) + x2_circ;
y2 = rad1*sind(angle2) + y2_circ;

x3 = rad2*cosd(angle3) + x3_circ;
y3 = rad2*sind(angle3) + y3_circ;

x4 = rad2*cosd(angle4) + x4_circ;
y4 = rad2*sind(angle4) + y4_circ;



plot (x1, y1, 'w')
plot (x2, y2, 'w')
plot (x3, y3, 'w')
plot (x4, y4, 'w')

if cir_hd==0
rectangle('Position',[x1_circ-rad1 y1_circ-rad1 rad1*2 rad1*2],'Curvature',[1 1],'LineStyle', ['-.'], 'EdgeColor','b');
hold on
rectangle('Position',[x2_circ-rad1 y2_circ-rad1 rad1*2 rad1*2],'Curvature',[1 1],'LineStyle', ['-.'],'EdgeColor','b');
hold on
rectangle('Position',[x3_circ-rad2 y3_circ-rad2 rad2*2 rad2*2],'Curvature',[1 1],'LineStyle', [':'],'EdgeColor','b');
hold on
rectangle('Position',[x4_circ-rad2 y4_circ-rad2 rad2*2 rad2*2],'Curvature',[1 1], 'LineStyle', [':'],'EdgeColor','b');
hold on
end

if rct_hd==0
plot (Xrot(iii, :), Yrot(iii, :), 'b')
end

end
