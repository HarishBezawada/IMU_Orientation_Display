% -------------------------------------
% Test of IMU orientation in Real-Time
% ▏ Author: Harish Bezawada
% ▏ Agile Robotics Lab
% ▏ The University of Alabama
% -------------------------------------

% Get workspace ready
clear
clc
close all

% Read serial data
ard = serialport("COM9", 115200);

fig = figure('Name',char(1),"WindowState","maximized");
set(fig,'Visible','on')
tiledlayout (1,2)

% Inertial Frame Display
nexttile;
hold on
quiver3(0,0,0,1,0,0,'r-','LineWidth',5)
quiver3(0,0,0,0,1,0,'g-','LineWidth',5)
quiver3(0,0,0,0,0,1,'b-','LineWidth',5)
ch_x = quiver3(0,0,0,1,0,0,'r--','LineWidth',4,);
ch_y = quiver3(0,0,0,0,1,0,'g--','LineWidth',4);
ch_z = quiver3(0,0,0,0,0,1,'b--','LineWidth',4);
hold off
grid on
grid minor
xlabel('Inertial - x','FontSize',18,'FontWeight','bold')
ylabel('Inertial - y','FontSize',18,'FontWeight','bold')
zlabel('Inertial - z','FontSize',18,'FontWeight','bold')
xlim([-1.5 1.5]); ylim([-1.5 1.5]); zlim([-1.5 1.5])
view(45,45)

% Camera Frame
global is_C_fixed
is_C_fixed = false;
global R_SC
R_SC = [];
nexttile;
hold on
quiver3(0,0,0,1,0,0,'k-','LineWidth',5)
quiver3(0,0,0,0,1,0,'m-','LineWidth',5)
quiver3(0,0,0,0,0,1,'c-','LineWidth',5)
ct_x = quiver3(0,0,0,1,0,0,'k-','LineWidth',5);
ct_y = quiver3(0,0,0,0,1,0,'m--','LineWidth',5);
ct_z = quiver3(0,0,0,0,0,1,'c--','LineWidth',5);
hold off
grid on
grid minor
xlabel('Camera - x','FontSize',18,'FontWeight','bold')
ylabel('Camera - y','FontSize',18,'FontWeight','bold')
zlabel('Camera - z','FontSize',18,'FontWeight','bold')
xlim([-1.5 1.5]); ylim([-1.5 1.5]); zlim([-1.5 1.5])
view(45,45)

% Serial data callback function
configureCallback(ard,"terminator",@(src,evnt)readSerialData(src,evnt,ch_x,ch_y,ch_z,ct_x,ct_y,ct_z))

%% Functions
function readSerialData(src,evt,varargin)
    data = readline(src);
    fprintf(data);
    src.UserData = data;
    
    if data.startsWith("[")
        data_string = data.replace("],[","] [");
        data_string = data_string.split(" ");
        
        calib = str2num(data_string(1));
        Q = str2num(data_string(2));

        R = rotmat(quaternion(Q),'frame')*eye(3);

        ch_x = varargin{1};
        ch_y = varargin{2};
        ch_z = varargin{3};

        ch_x.UData = R(1,1);
        ch_x.VData = R(1,2);
        ch_x.WData = R(1,3);

        ch_y.UData = R(2,1);
        ch_y.VData = R(2,2);
        ch_y.WData = R(2,3);

        ch_z.UData = R(3,1);
        ch_z.VData = R(3,2);
        ch_z.WData = R(3,3);        
        
        global is_C_fixed
        if is_C_fixed
            global R_SC
            if isempty(R_SC)
               R_SC = R; 
            else
                ct_x = varargin{4};
                ct_y = varargin{5};
                ct_z = varargin{6};
                
                R_ST = R_SC' * R;
                ct_x.UData = R_ST(1,1);
                ct_x.VData = R_ST(1,2);
                ct_x.WData = R_ST(1,3);

                ct_y.UData = R_ST(2,1);
                ct_y.VData = R_ST(2,2);
                ct_y.WData = R_ST(2,3);

                ct_z.UData = R_ST(3,1);
                ct_z.VData = R_ST(3,2);
                ct_z.WData = R_ST(3,3);  
            end
        end
    end
end