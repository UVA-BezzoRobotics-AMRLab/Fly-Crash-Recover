%% 
clear
clc
close all

bags = dir('*.bag'); % this calls/lists all bag files in current dir

bagNum = 5; %index of bag file in current directory

bag = rosbag(bags(bagNum).name);



viconMsgSelect = select(bag,'Topic','/vicon/crazyflie/crazyflie'); %vicon topic
viconMsgsStructs = readMessages(viconMsgSelect,'DataFormat','struct');
if ~isempty(viconMsgsStructs)
    Vicon(:,1) = cellfun(@(m) (double(m.Header.Stamp.Sec) + double(m.Header.Stamp.Nsec)*10^(-9) - double(viconMsgsStructs{1,1}.Header.Stamp.Sec) - double(viconMsgsStructs{1,1}.Header.Stamp.Nsec)*10^(-9)),viconMsgsStructs)';
    Vicon(:,2) = cellfun(@(m) (m.Transform.Translation.X),viconMsgsStructs)';
    Vicon(:,3) = cellfun(@(m) (m.Transform.Translation.Y),viconMsgsStructs)';
    Vicon(:,4) = cellfun(@(m) (m.Transform.Translation.Z),viconMsgsStructs)';
    vQ(:,2) = cellfun(@(m) (m.Transform.Rotation.X),viconMsgsStructs)';
    vQ(:,3) = cellfun(@(m) (m.Transform.Rotation.Y),viconMsgsStructs)';
    vQ(:,4) = cellfun(@(m) (m.Transform.Rotation.Z),viconMsgsStructs)';
    vQ(:,1) = cellfun(@(m) (m.Transform.Rotation.W),viconMsgsStructs)';

    Vicon(:,5:7) = flip(quat2eul(vQ),2); %flip to get ZYX to XYZ
end



cmdVelMsgSelect = select(bag,'Topic','/crazyflie/cmd_vel'); %cmdVel topic
cmdVelMsgsStructs = readMessages(cmdVelMsgSelect,'DataFormat','struct');
if ~isempty(cmdVelMsgsStructs)
    CmdVel(:,1) = table2array(cmdVelMsgSelect.MessageList(:,1))- double(viconMsgsStructs{1,1}.Header.Stamp.Sec) - double(viconMsgsStructs{1,1}.Header.Stamp.Nsec)*10^(-9);
    CmdVel(:,2) = cellfun(@(m) (m.Linear.X),cmdVelMsgsStructs)';
    CmdVel(:,3) = cellfun(@(m) (m.Linear.Y),cmdVelMsgsStructs)';
    CmdVel(:,4) = cellfun(@(m) (m.Linear.Z),cmdVelMsgsStructs)';
end



imuMsgSelect = select(bag,'Topic','/crazyflie/imu');
imuMsgsStructs = readMessages(imuMsgSelect,'DataFormat','struct');
if ~isempty(imuMsgsStructs)
    IMU(:,1) = cellfun(@(m) (double(m.Header.Stamp.Sec) + double(m.Header.Stamp.Nsec)*10^(-9)) - double(viconMsgsStructs{1,1}.Header.Stamp.Sec) - double(viconMsgsStructs{1,1}.Header.Stamp.Nsec)*10^(-9),imuMsgsStructs)';
    IMU(:,2) = cellfun(@(m) (m.LinearAcceleration.X),imuMsgsStructs)';
    IMU(:,3) = cellfun(@(m) (m.LinearAcceleration.Y),imuMsgsStructs)';
    IMU(:,4) = cellfun(@(m) (m.LinearAcceleration.Z),imuMsgsStructs)';
    IMU(:,5) = cellfun(@(m) (m.AngularVelocity.X),imuMsgsStructs)';
    IMU(:,6) = cellfun(@(m) (m.AngularVelocity.Y),imuMsgsStructs)';
    IMU(:,7) = cellfun(@(m) (m.AngularVelocity.Z),imuMsgsStructs)';
end



myDataMsgSelect = select(bag,'Topic','/crazyflie/my_data');
myDataMsgsStructs = readMessages(myDataMsgSelect,'DataFormat','struct');
if ~isempty(myDataMsgsStructs)
    MyData(:,1) = table2array(myDataMsgSelect.MessageList(:,1)) - double(viconMsgsStructs{1,1}.Header.Stamp.Sec) - double(viconMsgsStructs{1,1}.Header.Stamp.Nsec)*10^(-9);
    MyData(:,2) = cellfun(@(m) (m.Linear.X),myDataMsgsStructs)';
    MyData(:,3) = cellfun(@(m) (m.Linear.Y),myDataMsgsStructs)';
    MyData(:,4) = cellfun(@(m) (m.Linear.Z),myDataMsgsStructs)';
    MyData(:,5) = cellfun(@(m) (m.Angular.X),myDataMsgsStructs)';
    MyData(:,6) = cellfun(@(m) (m.Angular.Y),myDataMsgsStructs)';
    MyData(:,7) = cellfun(@(m) (m.Angular.Z),myDataMsgsStructs)';
end



joyMsgSelect = select(bag,'Topic','/crazyflie/joy');
joyMsgStructs = readMessages(joyMsgSelect,'DataFormat','struct');
if ~isempty(joyMsgStructs)
    Joy(:,1) = table2array(joyMsgSelect.MessageList(:,1)) - double(viconMsgsStructs{1,1}.Header.Stamp.Sec) - double(viconMsgsStructs{1,1}.Header.Stamp.Nsec)*10^(-9);
    Joy(:,2) = cellfun(@(m) (m.Buttons(1)),joyMsgStructs)';
end


% PIDVals1MsgSelect = select(bag,'Topic','/crazyflie/PIDVals1');
% PIDVals1MsgsStructs = readMessages(myDataMsgSelect,'DataFormat','struct');
% if ~isempty(PIDVals1MsgsStructs)
%     MyData(:,1) = table2array(PIDVals1MsgsStructs.MessageList(:,1)) - double(viconMsgsStructs{1,1}.Header.Stamp.Sec) - double(viconMsgsStructs{1,1}.Header.Stamp.Nsec)*10^(-9);
%     MyData(:,2) = cellfun(@(m) (m.Linear.X),PIDVals1MsgsStructs)';
%     MyData(:,3) = cellfun(@(m) (m.Linear.Y),PIDVals1MsgsStructs)';
%     MyData(:,4) = cellfun(@(m) (m.Linear.Z),PIDVals1MsgsStructs)';
%     MyData(:,2) = cellfun(@(m) (m.Angular.X),PIDVals1MsgsStructs)';
%     MyData(:,3) = cellfun(@(m) (m.Angular.Y),PIDVals1MsgsStructs)';
%     MyData(:,4) = cellfun(@(m) (m.Angular.Z),PIDVals1MsgsStructs)';
% end


% PIDVals2MsgSelect = select(bag,'Topic','/crazyflie/PIDVals1');
% PIDVals2MsgsStructs = readMessages(myDataMsgSelect,'DataFormat','struct');
% if ~isempty(PIDVals2MsgsStructs)
%     MyData(:,1) = table2array(PIDVals2MsgsStructs.MessageList(:,1)) - double(viconMsgsStructs{1,1}.Header.Stamp.Sec) - double(viconMsgsStructs{1,1}.Header.Stamp.Nsec)*10^(-9);
%     MyData(:,2) = cellfun(@(m) (m.Linear.X),PIDVals2MsgsStructs)';
%     MyData(:,3) = cellfun(@(m) (m.Linear.Y),PIDVals2MsgsStructs)';
%     MyData(:,4) = cellfun(@(m) (m.Linear.Z),PIDVals2MsgsStructs)';
%     MyData(:,2) = cellfun(@(m) (m.Angular.X),PIDVals2MsgsStructs)';
%     MyData(:,3) = cellfun(@(m) (m.Angular.Y),PIDVals2MsgsStructs)';
%     MyData(:,4) = cellfun(@(m) (m.Angular.Z),PIDVals2MsgsStructs)';
% end

%% Phase shifting section

shiftedCmdVelZ = shiftVectorBack(CmdVel(:,1),CmdVel(:,4),IMU(:,1),0.08);

IMUCmdDelta(:,1) = shiftedCmdVelZ(:,1);
IMUCmdDelta(:,2) = IMU(:,4)-(shiftedCmdVelZ(:,2)*18.5/60000-2.2);


%% Detection Plot
close all

% Not done yet
figure(1)
hold on
plot(Vicon(:,1),Vicon(:,4),'Linewidth',1.5) % Z-height
plot(Vicon(:,1),Vicon(:,6)*5,'Linewidth',1.5) % Pitch
%plot(IMU(:,1),IMU(:,6),'Linewidth',1.5) % Related IMU Pitch
plot(CmdVel(:,1),CmdVel(:,3)/20,'Linewidth',1.5) % Command signal
%plot(IMU(:,1),recreatedOrientation*5,'Linewidth',1.5) % Recreated


hold off
legend('Z height','Roll or Pitch','Corresponding IMU Angular Vel','CmdVel')


%% Linear Acceleration Plots
close all

figure(1)
hold on
plot(IMU(:,1),IMU(:,2),'Linewidth',1.5) % X-Linear Acceleration
plot(IMU(:,1),IMU(:,3),'Linewidth',1.5) % Y-Linear Acceleration
plot(IMU(:,1),IMU(:,4),'Linewidth',1.5) % Z-Linear Acceleration
hold off

legend('X Accel','Y Accel','Z Accel')

%% Angular Rate Plots
close all

figure(1)
hold on
plot(IMU(:,1),IMU(:,5),'Linewidth',1.5) % X-Angular Rate
plot(IMU(:,1),IMU(:,6),'Linewidth',1.5) % Y-Angular Rate
plot(IMU(:,1),IMU(:,7),'Linewidth',1.5) % Z-Angular Rate
hold off

legend('X Rate','Y Rate','Z Rate')

%% Attiude Plots
close all

figure(1)
hold on
plot(Vicon(:,1),Vicon(:,5),'Linewidth',1.5) % Roll
plot(Vicon(:,1),Vicon(:,6),'Linewidth',1.5) % Pitch
plot(Vicon(:,1),Vicon(:,7),'Linewidth',1.5) % Yaw
hold off

legend('Roll','Pitch','Yaw')

%% z accel filtering
testshiftedCmdVelZ = shiftVectorBack(CmdVel(:,1),CmdVel(:,4),IMU(:,1),-0.08);

testData(:,1) = testshiftedCmdVelZ(:,1);
testData(:,2) = (testshiftedCmdVelZ(:,2)*18.5/60000-2.2);
subplot(2,1,1)
plot(testData(:,1), testData(:,2), 'b');
hold on
plot(IMU(:,1), IMU(:,4), 'r');
xlim([4.5,12])
legend('Z Command Acceleration','Z Acceleration')
subplot(2,1,2)
plot(IMU(:,1),IMU(:,4)-testData(:,2),'b');
xlim([4.5,12])
legend('Commanded - Measured')



%% Function Definitions

function [interpVec] = shiftVectorBack(vectorToShiftTime,vectorToShiftData,vectorToShiftToTime,shiftTime) 

shiftedVec(:,1) = vectorToShiftTime(:) - shiftTime;
shiftedVec(:,2) = vectorToShiftData(:);

interpVec(:,1) = vectorToShiftToTime(:);
interpVec(:,2) = interp1(shiftedVec(:,1),shiftedVec(:,2),vectorToShiftToTime(:));

end


%% Unrelated other stuff

% Stuff to look at recreating positioning from IMU
% for i=1:size(IMU(:,1))
%     recreatedOrientation(i) = trapz(IMU(1:i,1),IMU(1:i,6),1);
% end

