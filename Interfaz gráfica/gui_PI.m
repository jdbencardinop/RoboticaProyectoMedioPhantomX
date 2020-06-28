function varargout = gui_PI(varargin)
% GUI_PI MATLAB code for gui_PI.fig
%      GUI_PI, by itself, creates a new GUI_PI or raises the existing
%      singleton*.
%
%      H = GUI_PI returns the handle to a new GUI_PI or the handle to
%      the existing singleton*.
%
%      GUI_PI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in GUI_PI.M with the given input arguments.
%
%      GUI_PI('Property','Value',...) creates a new GUI_PI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before gui_PI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to gui_PI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help gui_PI

% Last Modified by GUIDE v2.5 27-Jun-2020 09:54:18

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @gui_PI_OpeningFcn, ...
                   'gui_OutputFcn',  @gui_PI_OutputFcn, ...
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


% --- Executes just before gui_PI is made visible.
function gui_PI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to gui_PI (see VARARGIN)

% Choose default command line output for gui_PI
handles.output = hObject;

% UIWAIT makes gui_PI wait for user response (see UIRESUME)
% uiwait(handles.figure1);

L0 = 0.094375;
L1 = 0.0415;
L2 = 0.107;
L3 = 0.107;
L4 = 0.0905;
q1 = 0;
q2 = 0;
q3 = 0;
q4 = 0;

handles.l = [L0+L1 L2 L3 L4];
handles.q0 = [q1 q2 q3 q4];
handles.qcur = [q1 q2 q3 q4];

handles.L(1) = Link('revolute','a',0,'alpha',0,'d',handles.l(1),'offset',0,'modified');
handles.L(2) = Link('revolute','a',0,'alpha',-pi/2,'d',0,'offset',-pi/2,'modified');
handles.L(3) = Link('revolute','a',handles.l(2),'alpha',0,'d',0,'offset',0,'modified');
handles.L(4) = Link('revolute','a',handles.l(3),'alpha',0,'d',0,'offset',0,'modified');

handles.pincher = SerialLink(handles.L,'name','PhantomX Pincher');
handles.pincher.base = transl(0,0,0);

handles.T0 = handles.pincher.fkine([0 0 0 0]); 
handles.T04 = handles.pincher.fkine(handles.q0);
handles.T4T = rt2tr(rotz(-90,'deg')*rotx(-90,'deg'),[handles.l(4) 0 0]');%Herramienta respecto al marco 4
handles.pincher.tool = handles.T4T;

handles.T0T = handles.T04*handles.T4T; %Herramienta respecto a base
axes(handles.axes1)
hold off
handles.pincher.plot(handles.q0,'noa','workspace',[-0.4 0.4 -0.4 0.4 -0.1 0.65],'view',[60 30]);

handles.Estado.String = 'apagado';
handles.GripperG.String = '';
handles.ObjetosClasificados.String = '0';

handles.Q1g.String = num2str(handles.q0(1));
handles.Q2g.String = num2str(handles.q0(2));
handles.Q3g.String = num2str(handles.q0(3));
handles.Q4g.String = num2str(handles.q0(4));

handles.XG.String = '0';
handles.YG.String = '0';
handles.ZG.String = num2str(L0 + L1 + L2 +L3 + L4);
handles.PitchG.String = '0';

handles.roson = false;
handles.operando = false;

handles.angle = 30;
handles.T = sequenceMaker(handles.angle);


% Update handles structure
guidata(hObject, handles);





% --- Outputs from this function are returned to the command line.
function varargout = gui_PI_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in Inicio.
function Inicio_Callback(hObject, eventdata, handles)
% hObject    handle to Inicio (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.operando = true;
guidata(hObject, handles);
if(handles.ModoDeOperacion.Value == 2)
    display(handles.angle,'oprimido');
    display(handles.T(21:24,:),'ultima');   
    handles = guidata(hObject);
    poseSequence(handles.pincher, handles.T, handles.l, handles.qcur,1,handles, hObject);
    guidata(hObject, handles);
else
    handles = guidata(hObject);
    disp(handles.ModoDeOperacion.Value,'Modo MANUAL');
    joystick(hObject,handles);
    guidata(hObject, handles);
end


    


% --- Executes on button press in Parada.
function Parada_Callback(hObject, eventdata, handles)
% hObject    handle to Parada (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% handles.Inicio.Value = 0;
%stop = warndlg('Sistema Parado Presione Ok para Continuar','Warning');
%waitfor(stop);
handles.operando = false;
guidata(hObject, handles);





% --- Executes on selection change in ModoDeOperacion.
function ModoDeOperacion_Callback(hObject, eventdata, handles)
%handles.ModoOp = handles.ModoDeOperacion.Value;


% --- Executes during object creation, after setting all properties.
function ModoDeOperacion_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ModoDeOperacion (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in Trayectoria.
function Trayectoria_Callback(hObject, eventdata, handles)
% hObject    handle to Trayectoria (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns Trayectoria contents as cell array
%        contents{get(hObject,'Value')} returns selected item from Trayectoria
if get(hObject,'Value') == 1
    handles.angle = 30;
elseif get(hObject,'Value') == 2
    handles.angle = -30;
elseif get(hObject,'Value') == 3
    handles.angle = -90;
else
    handles.angle = 90;
end
display(handles.angle, 'Hangle');
handles.T = sequenceMaker(handles.angle);
guidata(hObject, handles);




% --- Executes during object creation, after setting all properties.
function Trayectoria_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Trayectoria (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in ROS.
function ROS_Callback(hObject, eventdata, handles)
% hObject    handle to ROS (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns ROS contents as cell array
%        contents{get(hObject,'Value')} returns selected item from ROS
if get(hObject,'Value') == 2
    try
        handles.operando = false;
        rosinit
        
        %Subscribers init
        
        handles.joystick_sub = rossubscriber('/joy', @get_joystick_msg);
        sub = rossubscriber('/phantomx_pincher/joint_states');

        handles.r = rosrate(50); 
        
        %Publishsers init
        handles.pubJoint1 = rospublisher('/phantomx_pincher/joint_1_position_controller/command');
        handles.pubJoint2 = rospublisher('/phantomx_pincher/joint_2_position_controller/command');
        handles.pubJoint3 = rospublisher('/phantomx_pincher/joint_3_position_controller/command');
        handles.pubJoint4 = rospublisher('/phantomx_pincher/joint_4_position_controller/command');
        handles.pubFinger1 = rospublisher('/phantomx_pincher/joint_finger_1_position_controller/command'); 
        handles.pubFinger2 = rospublisher('/phantomx_pincher/joint_finger_2_position_controller/command'); 

        %Messages init
        handles.msgJoint1 = rosmessage(pubJoint1); 
        handles.msgJoint2 = rosmessage(pubJoint2); 
        handles.msgJoint3 = rosmessage(pubJoint3); 
        handles.msgJoint4 = rosmessage(pubJoint4); 
        handles.msgFinger1 = rosmessage(pubFinger1);
        handles.msgFinger2 = rosmessage(pubFinger2);
        handles.roson = true;
    catch
        disp('Algo falló')
        rosshutdown
        handles.roson = false;
    end
else
    rosshutdown
    handles.roson = false;
    handles.operando = false;
end
guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function ROS_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ROS (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

%%
%Funciones
function T = sequenceMaker(angle)
    
radio = 0.22;
altura_recogida = 0.035;
altura_traslacion = 0.13;
% Orientaciones
Rini = trotz(90, 'deg');
R = trotz(angle, 'deg');

%    transl( X0', 'Y0' , Z0' en metros )*R_left;
T1 = Rini*transl(radio,0,altura_traslacion)*troty(90,'deg')*trotz(180,'deg');
T2 = Rini*transl(radio,0,altura_recogida)*troty(90,'deg')*trotz(180,'deg');
T3 = R*transl(radio,0,altura_traslacion)*troty(90,'deg')*trotz(180,'deg');
T4 = R*transl(radio,0,altura_recogida)*troty(90,'deg')*trotz(180,'deg');

display(angle,'Interno');
T = [T1;T2;T1;T3;T4;T3]; 

    
%% Funciones 

%Función para el control automático del gripper
function poseSequence(pincher, T, l, q0, init, handles, hObject) 
    q = q0;  %Posición inicial de la secuencia
    handles.Estado.String = 'prendido';
    for i=init:6
        
        handles = guidata(hObject);
        if ~handles.operando
            break
        end
        
        %Casos para abrir o cerrar el gripper
        if handles.roson
           if i==3 
                gripper = 0.01; % metros
                moveGripper(gripper); % para gripper
                handles.GripperG.String = 'ON';
            end
            if i==6 
                gripper = 0; % metros
                moveGripper(gripper); % para gripper  
                handles.GripperG.String = '';
            end 
        end
        
        %Casos de moviemiento convencional
        Taux = T(4*i-3:4*i,:);
        prev = q;
        q = inverseX(Taux,l);
        [Q,~,~] = jtraj(prev, q, 20); % 20 puntos intermedios
        for j=1:size(Q,1) 
            handles = guidata(hObject);
            if ~handles.operando
                q = Q(j,:);
                break
            end
            
            movePincher(pincher, Q(j,:), Taux, handles)
            if handles.roson
               moveAllJoints(Q(j,:)); 
            end
            if handles.roson
                joint_states = readJointStates(handles.sub);
            else
                joint_states = Q(j,:);
            end
            updatetexts(joint_states,handles);
        end 
        
        handles = guidata(hObject);
        if ~handles.operando
            break
        end
        
        pause(0.1)  
    end
    %Acabar el ciclo
    handles = guidata(hObject);
    if ~handles.operando
        handles.qcur = q;
        display(handles.qcur,'Parado!');
        handles.Estado.String = 'apagado';
    else
        %Posición de arranque siguiente ciclo
        % Por defecto está en home
        handles.qcur = handles.q0;
    end
    handles.Estado.String = 'apagado';
    guidata(hObject,handles);

function qcodoarriba = inverseX(T,dims)
    wx = T(1,4) - dims(4)*T(1,3);
    wy = T(2,4) - dims(4)*T(2,3);
    wz = T(3,4) - dims(4)*T(3,3);
    
    q(1,1) = atan2(wy,wx);
    q(2,1) = atan2(wy,wx);
    cosq = ((wz-dims(1))^2 + wy^2 + wx^2 - dims(2)^2 - dims(3)^2 )/(2*dims(2)*dims(3));
    if(abs(cosq) <= 1)
        q(1,3) = atan2(sqrt(1-cosq^2),cosq);
        q(2,3) = atan2(-sqrt(1-cosq^2),cosq);
        q(1,2) = atan2(wz-dims(1),sqrt(wy^2 + wx^2)) + atan2(dims(3)*sin(q(1,3)),dims(2) + dims(3)*cos(q(1,3)));
        q(1,2) = pi/2 - q(1,2);
        q(2,2) = atan2(wz-dims(1),sqrt(wy^2 + wx^2)) + atan2(dims(3)*sin(q(2,3)),dims(2) + dims(3)*cos(q(2,3)));
        q(2,2) = pi/2 - q(2,2);
        q(1,4) = atan2(T(3,1),T(3,3)) - q(1,2) - q(1,3);
        q(2,4) = atan2(T(3,1),T(3,3)) - q(2,2) - q(2,3);
    end
    qcodoarriba= q(1,:);
    %qcodoabajo = q(2,:);
       
function moveAllJoints(pubJoint1,msgJoint1,pubJoint2,msgJoint2,pubJoint3,msgJoint3,pubJoint4,msgJoint4,Q) % deg
    moveJoint(pubJoint3,msgJoint3,Q(3))
    moveJoint(pubJoint2,msgJoint2,Q(2))
    moveJoint(pubJoint4,msgJoint4,Q(4))
    moveJoint(pubJoint1,msgJoint1,Q(1))


function moveJoint(pubJoint,msgJoint,angle) % deg
    msgJoint.Data = angle; 
    send(pubJoint,msgJoint);
%     pause(1/100)


function moveGripper(pubFinger1,msgFinger1,pubFinger2,msgFinger2,distance) % deg
    msgFinger1.Data = distance; 
    msgFinger2.Data = distance; 
    send(pubFinger1,msgFinger1);
    send(pubFinger2,msgFinger2);
%     pause(1/100)


function data = readJointStates(sub) % metros
    data = deg2rad(sub.LatestMessage.Position);


function movePincher(pincher, Q, ~, handles)
    axes(handles.axes1)
    hold off
    pincher.plot(Q,'noa','workspace',[-0.4 0.4 -0.4 0.4 -0.1 0.65],'view',[60 30]);
%     hold on
%     trplot(T,'length',0.1,'rgb');


function updatetexts(joint_states, handles)
        handles.Q1g.String = num2str(rad2deg(joint_states(1)));
        handles.Q2g.String = num2str(rad2deg(joint_states(2)));
        handles.Q3g.String = num2str(rad2deg(joint_states(3)));
        handles.Q4g.String = num2str(rad2deg(joint_states(4)));
        
        mat = handles.pincher.fkine(joint_states(1:4));
        
        handles.XG.String = num2str(mat(1,4));
        handles.YG.String = num2str(mat(2,4));
        handles.ZG.String = num2str(mat(3,4));
        handles.PitchG.String = rad2deg(sum(joint_states(2:4)));

function get_joystick_msg(~, message)
    global joystick_msg;
    joystick_msg = message;

function joystick(hObject,handles)
        
    js_deadzone = 0.3;

    step_rho = 0.01; % metros
    step_z = 0.01; % metros
    step_roll = deg2rad(10); % rad
    step_pitch = deg2rad(10); % rad

    % Home
    Q_home = handles.q0; %this may have to change to start at the last know position
    [R_home,t_home] = tr2rt(handles.T0T);
    rpy_home = tr2rpy(R_home, 'zyx'); % rad

    % Act
    Q_act = handles.qcur;
%     t_act = t_home;
    [R_act, t_act] = tr2rt(handles.pincher.fkine(Q_act));
    rpy_act = tr2rpy(R_act, 'zyx'); % rad

%     roll_aux = Q_home(1); % rad
%     pitch_aux = Q_home(4); % rad
    roll_aux = Q_act(1); % rad
    pitch_aux = Q_act(4); % rad
    rho_act = t_home(1)/cos(roll_aux); % metros
    z_act = t_home(3); % metros

    pub_index = 0;
    pub_state = ["OFF", "ON"];

    distante_index = 0;
    distance_value = [0 0.01];
    distance_state = ["Opened", "Closed"];
    
    handles = guidata(hObject);
    while handles.ModoDeOperacion.Value == 1 && handles.roson && handles.operando
        handles.Estado.String = 'prendido';
        
        axes = joystick_msg.Axes;
        buttons = joystick_msg.Buttons;

        if buttons(2)==1 % Home position - B
            roll_aux = Q_home(1);
            pitch_aux = Q_home(4);
            rho_act = t_home(1)/cos(roll_aux);
            z_act = t_home(3);
            disp('Home position')
        elseif buttons(4)==1
            pub_index = not(pub_index);
            disp('Pub state: '+pub_state(pub_index+1))
        elseif buttons(1)==1    
            distante_index = not(distante_index);
            disp('Dist state: '+distance_state(distante_index+1))
            moveGripper(handles.pubFinger1,handles.msgFinger1,handles.pubFinger2,handles.msgFinger2,distance_value(distante_index+1));
        end

        if abs(axes(2)) > js_deadzone 
            z_act = z_act + step_z*axes(2);
        end

        if axes(6) < 0 
            rho_act = rho_act - step_rho*axes(6);    
        elseif axes(3) < 0 
            rho_act = rho_act + step_rho*axes(3);    
        end

        if abs(axes(4)) > js_deadzone 
            roll_aux = roll_aux + step_roll*axes(4);
        elseif abs(axes(5)) > js_deadzone 
            pitch_aux = pitch_aux - step_pitch*axes(5);
        end

        t_act(1) = rho_act*cos(roll_aux);
        t_act(2) = rho_act*sin(roll_aux);
        t_act(3) = z_act;

        rpy_act(1) = rpy_home(1)+roll_aux;
        rpy_act(2) = rpy_home(2)+pitch_aux;
        rpy_act(3) = rpy_home(3);
        R_act = rpy2r(rpy_act,'zyx');

        T_new = double(rt2tr(R_act,t_act));
        axes(handles.axes1)
%         Q_act = handles.pincher.ikunc(T_new);
        Q_act = inverseX(T_new,handles.l);
        pincher.plot(Q_act,'noa','workspace', [-0.4 0.4 -0.4 0.4 -0.1 0.65],'view',[60 30]);

        if pub_index==1 
            moveAllJoints(handles.pubJoint1,handles.msgJoint1,handles.pubJoint2,handles.msgJoint2,handles.pubJoint3,handles.msgJoint3,handles.pubJoint4,handles.msgJoint4,Q_act);    
        end

        [~,t_act] = tr2rt(handles.pincher.fkine(Q_act));
        roll_aux = Q_act(1);
        rho_act = t_act(1)/cos(roll_aux);
        z_act = t_act(3);
        
        handles = guidata(hObject);
        if ~handles.operando
            break
        end
        
        guidata(hObject,handles);
        waitfor(r);
        handles = guidata(hObject);
    end
    %Finalizar el ciclo
    if ~handles.operando
        handles.qcur = Q_act;
        handles.Estado.String = 'apagado';
    else
        %Posición de arranque siguiente ciclo
        % Por defecto está en home
        handles.qcur = handles.q0;
    end
    handles.Estado.String = 'apagado';
    guidata(hObject,handles);
    
