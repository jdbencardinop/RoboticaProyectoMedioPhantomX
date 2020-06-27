
clc
clear all
close all

% rosinit

%% Joystick Init

global joystick_msg;

joystick_sub = rossubscriber('/joy', @get_joystick_msg);

r = rosrate(50); 

pubJoint1 = rospublisher('/phantomx_pincher/joint_1_position_controller/command');
pubJoint2 = rospublisher('/phantomx_pincher/joint_2_position_controller/command');
pubJoint3 = rospublisher('/phantomx_pincher/joint_3_position_controller/command');
pubJoint4 = rospublisher('/phantomx_pincher/joint_4_position_controller/command');
pubFinger1 = rospublisher('/phantomx_pincher/joint_finger_1_position_controller/command'); 
pubFinger2 = rospublisher('/phantomx_pincher/joint_finger_2_position_controller/command'); 

msgJoint1 = rosmessage(pubJoint1); 
msgJoint2 = rosmessage(pubJoint2); 
msgJoint3 = rosmessage(pubJoint3); 
msgJoint4 = rosmessage(pubJoint4); 
msgFinger1 = rosmessage(pubFinger1);
msgFinger2 = rosmessage(pubFinger2);
    

%% Robot Parameters

q1 = 0; q2 = 0; q3 = 0; q4 = 0;
L0 = 0.094375 ; L1 = 0.0415; 
L2 = 0.107 ; L3 = 0.107; 
L4 = 0.0905 ; 

% Eslabones
L(1) = Link([q1, L0+L1,  0,     0, 0,     0], 'modified');
L(2) = Link([q2,     0,  0, -pi/2, 0, -pi/2], 'modified');
L(3) = Link([q3,     0, L2,     0, 0,     0], 'modified');
L(4) = Link([q4,     0, L3,     0, 0,     0], 'modified');

pincher = SerialLink(L,'name','PhantomX Pincher');
pincher.base = transl(0,0,0);

T04 = pincher.fkine([0 0 0 0]); 

T4T = rt2tr(rotz(-90,'deg')*rotx(-90,'deg'),[L4 0 0]'); % Tool respecto a {4}
pincher.tool = T4T;

T0T = T04*T4T; % Tool respecto a {0}

%% Joystick 

js_deadzone = 0.3;

step_rho = 0.01; % metros
step_z = 0.01; % metros
step_roll = deg2rad(10); % rad
step_pitch = deg2rad(10); % rad

% Home
Q_home = pincher.ikunc(T0T);
[R_home,t_home] = tr2rt(T0T);
rpy_home = tr2rpy(R_home, 'zyx'); % rad

% Act
t_act = t_home;
R_act = R_home;
rpy_act = rpy_home;

roll_aux = Q_home(1); % rad
pitch_aux = Q_home(4); % rad
rho_act = t_home(1)/cos(roll_aux); % metros
z_act = t_home(3); % metros

pub_index = 0;
pub_state = ["OFF", "ON"];

distante_index = 0;
distance_value = [0 0.01];
distance_state = ["Opened", "Closed"];

while 1
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
        moveGripper(pubFinger1,msgFinger1,pubFinger2,msgFinger2,distance_value(distante_index+1));
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
    Q_act = pincher.ikunc(T_new);
    pincher.plot(Q_act,'noa','workspace', [-0.4 0.4 -0.4 0.4 -0.1 0.65]);
    
    if pub_index==1 
        moveAllJoints(pubJoint1,msgJoint1,pubJoint2,msgJoint2,pubJoint3,msgJoint3,pubJoint4,msgJoint4,Q_act);    
    end
    
    [R_act,t_act] = tr2rt(pincher.fkine(Q_act));
    roll_aux = Q_act(1);
    rho_act = t_act(1)/cos(roll_aux);
    z_act = t_act(3);
    
    waitfor(r);
end


%% Funciones

function get_joystick_msg(~, message)
    global joystick_msg;
    joystick_msg = message;
end


function moveAllJoints(pubJoint1,msgJoint1,pubJoint2,msgJoint2,pubJoint3,msgJoint3,pubJoint4,msgJoint4,Q) % deg
    moveJoint(pubJoint3,msgJoint3,Q(3))
    moveJoint(pubJoint2,msgJoint2,Q(2))
    moveJoint(pubJoint4,msgJoint4,Q(4))
    moveJoint(pubJoint1,msgJoint1,Q(1))
end

function moveJoint(pubJoint,msgJoint,angle) % deg
    msgJoint.Data = angle; 
    send(pubJoint,msgJoint);
%     pause(1/100)
end

function moveGripper(pubFinger1,msgFinger1,pubFinger2,msgFinger2,distance) % deg
    msgFinger1.Data = distance; 
    msgFinger2.Data = distance; 
    send(pubFinger1,msgFinger1);
    send(pubFinger2,msgFinger2);
%     pause(1/100)
end

function data = readJointStates(sub) % metros
    data = deg2rad(sub.LatestMessage.Position);
end