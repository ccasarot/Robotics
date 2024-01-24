classdef Robo < handle
    %https://github.com/MonsisGit/MyRobot/blob/master/matlab/MyRobot.m
    % clc;
    % clear all;
    properties (Access = private)

        % [~, ~] = loadlibrary('dxl_x64_c', 'dynamixel_sdk.h', 'addheader',...
        % 'port_handler.h', 'addheader', 'packet_handler.h');
        ADDR_MX_TORQUE_ENABLE = 24;
        ADDR_MX_CW_COMPLIANCE_MARGIN = 26;
        ADDR_MX_CCW_COMPLIANCE_MARGIN = 27;
        ADDR_MX_CW_COMPLIANCE_SLOPE = 28;
        ADDR_MX_CCW_COMPLIANCE_SLOPE = 29;
        ADDR_MX_GOAL_POSITION = 30;
        ADDR_MX_MOVING_SPEED = 32;
        ADDR_MX_PRESENT_POSITION = 36;
        ADDR_MX_PUNCH = 48;
        PROTOCOL_VER = 1.0;
        DEVICENAME = 'COM6';
        BAUDRATE = 1000000;
        TORQUE_ENABLE = 1;
        TORQUE_DISABLE = 0;
        port_num = 0;
        lib_name = '';
    end

    properties (Access = public)
        DXL_IDS = [1,2,3,4];
        JOINT_OFFSET = [150 157-90 150 150];
        COMM_SUCCESS = 0;
        JOINT_ANG_ERR = [0 0 0 0];
        JOINT_ANG =[0 0 0 0];
        EE_SPEED = 30;
        JOINT_SPEED = [1 1 1 1]
        
        
        BUSY_FLAG = 0;
        MOTOR_ACCEL = [0 0 0 0];
        TRAJ_SMOOTHNESS = 3;
        DIST_POS = [0, 0, 0];


    end

    methods
        function self = Robo()


            if strcmp(computer, 'PCWIN')
                self.lib_name = 'dxl_x86_c';
            elseif strcmp(computer, 'PCWIN64')
                self.lib_name = 'dxl_x64_c';
            elseif strcmp(computer, 'GLNX86')
                self.lib_name = 'libdxl_x86_c';
            elseif strcmp(computer, 'GLNXA64')
                self.lib_name = 'libdxl_x64_c';
            elseif strcmp(computer, 'MACI64')
                self.lib_name = 'libdxl_mac_c';
            end

            % Load Libraries
            if ~libisloaded(self.lib_name)
                [notfound, warnings] = loadlibrary(self.lib_name, 'dynamixel_sdk.h', 'addheader', 'port_handler.h', 'addheader', 'packet_handler.h');
            end
            self.port_num = portHandler(self.DEVICENAME);
            packetHandler();
            openPort(self.port_num)
            setBaudRate(self.port_num, self.BAUDRATE)

            self.JOINT_SPEED = self.JOINT_SPEED*self.EE_SPEED; 

            for DXL_ID=self.DXL_IDS
                write1ByteTxRx(self.port_num, self.PROTOCOL_VER, DXL_ID, self.ADDR_MX_TORQUE_ENABLE, self.TORQUE_ENABLE);
                write2ByteTxRx(self.port_num, self.PROTOCOL_VER, DXL_ID, self.ADDR_MX_CW_COMPLIANCE_MARGIN, 0)
                write2ByteTxRx(self.port_num, self.PROTOCOL_VER, DXL_ID, self.ADDR_MX_CCW_COMPLIANCE_MARGIN, 0)
                write1ByteTxRx(self.port_num, self.PROTOCOL_VER, DXL_ID, self.ADDR_MX_CW_COMPLIANCE_SLOPE, 32)
                write1ByteTxRx(self.port_num, self.PROTOCOL_VER, DXL_ID, self.ADDR_MX_CCW_COMPLIANCE_SLOPE, 32)
                write2ByteTxRx(self.port_num, self.PROTOCOL_VER, DXL_ID, self.ADDR_MX_MOVING_SPEED, self.EE_SPEED)
            end
            self.JOINT_ANG = self.read_JOINT_ANG;
        end


        function disable_motors(self)

            % move_joints(0, 150-90, 150, 150+90);

            %self.move_joints(0, 150-90, 150, 150+90);

            for DXL_ID=self.DXL_IDS
                write1ByteTxRx(self.port_num, self.PROTOCOL_VER, DXL_ID, self.ADDR_MX_TORQUE_ENABLE, self.TORQUE_DISABLE);

            end
            closePort(self.port_num);
            unloadlibrary(self.lib_name);

        end


        function q = INV_KIN(self,o04,x04);

            % o04 = [o1,o2,o3];
            phi = atan2(-sqrt(1-cos(x04)^2),cos(x04)); %angle from origin to end effector position.
            L0 = 50;
            L1 = 93; %length of links (L1 = a2,L2 = a3.. etc. but cant be bothered)
            L2 = 93;
            L3 =50;
            % syms theta1 theta2 theta3
            x_3 = sqrt(o04(1)^2+o04(2)^2)-L3*cos(phi); %The distance from the origin of the end effector in the xy plane
            z_3 = o04(3)-L3*sin(phi)-L0; %the distance of the origin of the end effector i the z direction. ...
            % (The -50 is because we have moved coordinate origin 50mm up)
            % H = x_3^2+z_3^2;
            % c2 = (x_3^2+z_3^2-L1^2-L2^2)/(-2*L1*L2) ;

            theta2 = double(atan2(-sqrt(1-((x_3^2+z_3^2-L1^2-L2^2)/(-2*L1*L2))^2),-((x_3^2+z_3^2-L1^2-L2^2)/(-2*L1*L2))));
            theta1 = double(atan2(z_3,x_3)-atan2(L2*sin(theta2),(L1+L2*cos(theta2))));
            theta3 = double(phi -theta1 - theta2);
            t1 = theta1;
            t2 = theta2;
            t3 = theta3;

            % [t1 t2 t3] = solve([f1, f2 ,f3],[theta1, theta2, theta3]);

            t0 = atan2(o04(2),o04(1));
            q = [t0 double(t1) double(t2) double(t3)];

        end

        function move_cartesian(self,o04,x04)

            q = self.INV_KIN(o04,x04);
            self.move_joints(q);
        end

        function move_joints(self,q)
            new_angle = q*180/pi+self.JOINT_OFFSET;
            
            self.SMOOTH_MOVEMENT(self.JOINT_ANG,new_angle);
            self.JOINT_ANG = q*180/pi+self.JOINT_OFFSET;
            
            write2ByteTxRx(self.port_num, self.PROTOCOL_VER, self.DXL_IDS(1), self.ADDR_MX_GOAL_POSITION, self.degree_to_rotation(self.JOINT_ANG(1)));
            write2ByteTxRx(self.port_num, self.PROTOCOL_VER, self.DXL_IDS(2), self.ADDR_MX_GOAL_POSITION, self.degree_to_rotation(self.JOINT_ANG(2)));
            write2ByteTxRx(self.port_num, self.PROTOCOL_VER, self.DXL_IDS(3), self.ADDR_MX_GOAL_POSITION, self.degree_to_rotation(self.JOINT_ANG(3)));
            write2ByteTxRx(self.port_num, self.PROTOCOL_VER, self.DXL_IDS(4), self.ADDR_MX_GOAL_POSITION, self.degree_to_rotation(self.JOINT_ANG(4)));
            
            while 1
                self.read_JOINT_ANG();
                if abs(self.JOINT_ANG_ERR)<2;
                    break;
                end
            end
        end

        function rot2 = rad_to_rotation(self,rad)
            rot2 = rad*(180/pi)*1/0.29;
        end

        function rot = degree_to_rotation(self,deg)
            rot = deg*1/0.29;
        end

        function deg = rotation_to_degree(self,rot)
            deg = rot*0.29;
        end

        function j_a = read_JOINT_ANG(self)

            j_a = zeros(4,1);
            for i=1:length(self.DXL_IDS)
                dxl_present_position = read2ByteTxRx(self.port_num, self.PROTOCOL_VER, self.DXL_IDS(i), self.ADDR_MX_PRESENT_POSITION);
                dxl_comm_result = getLastTxRxResult(self.port_num, self.PROTOCOL_VER);
                dxl_error = getLastRxPacketError(self.port_num, self.PROTOCOL_VER);
                if dxl_comm_result ~= self.COMM_SUCCESS
                    fprintf('%s\n', getTxRxResult(self.PROTOCOL_VER, dxl_comm_result));
                elseif dxl_error ~= 0
                    fprintf('%s\n', getRxPacketError(self.PROTOCOL_VER, dxl_error));
                else
                    j_a(i) = self.rotation_to_degree(dxl_present_position);% + self.JOINT_OFFSET(i);
                    
                    self.JOINT_ANG_ERR(i) = j_a(i)-self.JOINT_ANG(i);
                    disp(self.JOINT_ANG_ERR(i))
                end
            end
        end

        function  TRAJ_ARR = TrajectorMoverRRRR(self,coor_arr, knot_points)
            syms theta_1 theta_2 theta_3 theta_4 t
            t0 = 0;
            v = self.EE_SPEED;
            thetasym = [theta_1,theta_2,theta_3,theta_4];
            t1 = self.timesolve(coor_arr(1,:),coor_arr(2,:));
            t1_sec = t1/(knot_points-1);
            tt = t0:0.5:t1_sec;
            oA = zeros(3,length(tt));
            points = (knot_points-1);
            % q_circle = qcircleArr(points);
            qt = zeros(size(tt,1),4);
            dqt = zeros(size(tt,1),4);
            ddqt = zeros(size(tt,1),4);
            q_arr = zeros(size(coor_arr,1),4);
            for i=1:size(coor_arr,1)
                q_arr(i,:) = self.INV_KIN(coor_arr(i,:),pi/2);
            end

            J=[-sin(theta_1)*(50*cos(theta_2 + theta_3 + theta_4) + 93*cos(theta_2 + theta_3) + 93*cos(theta_2)), -cos(theta_1)*(50*sin(theta_2 + theta_3 + theta_4) + 93*sin(theta_2 + theta_3) + 93*sin(theta_2)), -cos(theta_1)*(50*sin(theta_2 + theta_3 + theta_4) + 93*sin(theta_2 + theta_3)), -50*sin(theta_2 + theta_3 + theta_4)*cos(theta_1);
                cos(theta_1)*(50*cos(theta_2 + theta_3 + theta_4) + 93*cos(theta_2 + theta_3) + 93*cos(theta_2)), -sin(theta_1)*(50*sin(theta_2 + theta_3 + theta_4) + 93*sin(theta_2 + theta_3) + 93*sin(theta_2)), -sin(theta_1)*(50*sin(theta_2 + theta_3 + theta_4) + 93*sin(theta_2 + theta_3)), -50*sin(theta_2 + theta_3 + theta_4)*sin(theta_1);
                sym(0), 50*cos(theta_2 + theta_3 + theta_4) + 93*cos(theta_2 + theta_3) + 93*cos(theta_2), 50*cos(theta_2 + theta_3 + theta_4) + 93*cos(theta_2 + theta_3), 50*cos(theta_2 + theta_3 + theta_4);
                sym(0), sin(theta_1), sin(theta_1), sin(theta_1);
                sym(0), -cos(theta_1), -cos(theta_1), -cos(theta_1);
                sym(1), sym(0), sym(0), sym(0)]; %symbollic jacobian 
            % figure
            % hold on
            for j = 0:(knot_points-2)


                q0 = q_arr(j+1,:)';
                q1 = q_arr(j+2,:)';


                % syms sdq1 sdq2 sdq3 sdq4
                % dq = [sdq1; sdq2;sdq3;sdq4];
                Jdq0 = double(subs(J,thetasym,q0')); %Jacobian for the start angles
                Jdq1 = double(subs(J,thetasym,q1')); %Jacobian for the end angles

                Jdq0inv = ((transpose(Jdq0)*Jdq0))^(-1)*transpose(Jdq0); %inverse jacobian
                Jdq1inv = ((transpose(Jdq1)*Jdq1))^(-1)*transpose(Jdq1); %inverse jacobian

                Jdq0 = Jdq0inv;
                Jdq1 = Jdq1inv;
                syms s01 s02 s03 s04  s11 s12 s13 s14 s1 s0

              
                if j == 0
                    v_arr0 = [0;0;0];
                else
                    v_arr0 = [0; -v*sin(2*pi/(knot_points-1)*j); v*cos(2*pi/(knot_points-1)*j) ];
                end
               
                dq0 = Jdq0(:,1:3)*v_arr0;  %initial motor speeds

               

                if 2*pi/(knot_points-1)*(j+1) == 2*pi
                    v_arr1 = [0;0;0];
                else
                    v_arr1 = [0; -v*sin(2*pi/(knot_points-1)*(j+1)); v*cos(2*pi/(knot_points-1)*(j+1)) ];
                end

             
                dq1 = Jdq1(:,1:3)*v_arr1; %final motor speeds
                ddq0 =[0;0;0;0]; %accelerations
                ddq1 = ddq0;
                [TrajFunc, TrajSpeed, TrajAccel] = self.TrajectoryPlannerRRRR(t0,t1,q0,q1,dq0,dq1,ddq0,ddq1); % Calculating the trajectory function given the boundary conditions.
                tend = 0.001; 
                dt = 0.001;
                tic;
                tstart = toc; 
                while 1

                     tstart = toc;

                     if tstart > 1
                     self.setSpeed(double(subs(TrajSpeed,t,tend)));
                     self.move_joints(double(subs(TrajFunc,t,tend+1)));
                     disp('hej')
                     tic;
                     end
                 
                     % tend =  toc;
                 
                   
                end

                % for i = 1:length(tt) %creating arrays to plot by using the forward kinematics
                %     qt(i,:) = double(subs(TrajFunc,t,tt(i)));
                %     dqt(i,:) = double(subs(TrajSpeed,t,tt(i)));
                %     ddqt(i,:) = double(subs(TrajAccel,t,tt(i)));
                % end
                %     self.move_joints(qt(1,1),qt(1,2),qt(1,3),qt(1,4));
                % for i = 2:length(tt)
                %     self.setSpeed(dqt(i-1,:))
                %     self.move_joints(qt(i,1),qt(i,2),qt(i,3),qt(i,4));
                % end

                % plot(oA(2,:),oA(3,:))
            end
        end

        function [TrajFunc, TrajSpeed, TrajAccel] = TrajectoryPlannerRRRR(self,t0, t1, q0,q1, dq0, dq1, ddq0, ddq1)
            syms t A15 A14 A13 A12 A11 A10 A25 A24 A23 A22 A21 A20 A35 A34 A33 A32 A31 A30 A45 A44 A43 A42 A41 A40

            qA = [A15*t^5+A14*t^4+A13*t^3+A12*t^2+A11*t+A10;
                A25*t^5+A24*t^4+A23*t^3+A22*t^2+A21*t+A20;
                A35*t^5+A34*t^4+A33*t^3+A32*t^2+A31*t+A30;
                A45*t^5+A44*t^4+A43*t^3+A42*t^2+A41*t+A40];
            dqA= diff(qA,t);
            ddqA= diff(qA,t,2);
            f1 = subs(qA,t,t0) == q0;
            f2 = subs(qA,t,t1) == q1;
            f3 = subs(dqA,t,t0) == dq0;
            f4 = subs(dqA,t,t1) == dq1;
            f5 = subs(ddqA,t,t0) == ddq0;
            f6 = subs(ddqA,t,t1) == ddq1;
            f = [f1,f2,f3, f4, f5, f6];
            A = [A15 A14 A13 A12 A11 A10 A25 A24 A23 A22 A21 A20 A35 A34 A33 A32 A31 A30 A45 A44 A43 A42 A41 A40];


            [B15 B14 B13 B12 B11 B10 B25 B24 B23 B22 B21 B20 B35 B34 B33 B32 B31 B30 B45 B44 B43 B42 B41 B40] = solve(f,A);

            B = eval([B15 B14 B13 B12 B11 B10 B25 B24 B23 B22 B21 B20 B35 B34 B33 B32 B31 B30 B45 B44 B43 B42 B41 B40]);

            TrajFunc =  subs(qA,A,B);
            TrajSpeed =  subs(dqA,A,B);
            TrajAccel = subs(ddqA,A,B);
        end
        function dt = timesolve(self,co1, co2)
            targetDist = sqrt((co1(1)-co2(1))^2+(co1(2)-co2(2))^2+(co1(3)-co2(3))^2);

            dt = targetDist/self.EE_SPEED;

        end

        function setSpeed(self,speed)
            disp(speed)
            speed = abs(speed)*180/pi;
            for DXL_ID=self.DXL_IDS
                write2ByteTxRx(self.port_num, self.PROTOCOL_VER, DXL_ID, self.ADDR_MX_MOVING_SPEED, speed(DXL_ID))
            end
        end

        function SMOOTH_MOVEMENT(self,qstart,qend)
            rot_dist = qend-qstart; 
            self.JOINT_SPEED = [1 1 1 1]*self.EE_SPEED;
            max_dist = max(abs(rot_dist));
            speed_per_deg = max_dist/100; 
                  if speed_per_deg~=0
                new_speeds = abs(rot_dist/speed_per_deg)*0.01;
                for i=1:length(self.JOINT_SPEED)
                    if new_speeds(i)==0
                        new_speeds(i)=self.JOINT_SPEED(i);
                    else
                        new_speeds(i)=new_speeds(i)*self.JOINT_SPEED(i);
                    end
                end
                self.setSpeed(new_speeds);
            end


        end
    end


end



