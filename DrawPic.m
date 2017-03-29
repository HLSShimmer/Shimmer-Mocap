clear all; clc; close all;
%% 生成mocap的动画
load DataBase_KneeFlexion_mocap ankle_position_mocap knee_position_mocap thigh_position_mocap mocap_time triggerBound_mocap
ankle_position_mocap = ankle_position_mocap(triggerBound_mocap(1):end,:)/1000;
knee_position_mocap = knee_position_mocap(triggerBound_mocap(1):end,:)/1000;
thigh_position_mocap = thigh_position_mocap(triggerBound_mocap(1):end,:)/1000;
dataNum = size(ankle_position_mocap,1);
buffSize = 1000;
buffAnkle = [];
buffThigh = [];
buffKnee = [];
fHandle1 = figure(1);
XPoints = [ankle_position_mocap(1,1),knee_position_mocap(1,1),thigh_position_mocap(1,1)];
YPoints = [ankle_position_mocap(1,2),knee_position_mocap(1,2),thigh_position_mocap(1,2)];
ZPoints = [ankle_position_mocap(1,3),knee_position_mocap(1,3),thigh_position_mocap(1,3)];
segmentHandle1 = line(XPoints(1:2),YPoints(1:2),ZPoints(1:2),'color','r','LineWidth',2.5);
segmentHandle2 = line(XPoints(2:3),YPoints(2:3),ZPoints(2:3),'color','b','LineWidth',2.5);
pointsHandle = line(XPoints,YPoints,ZPoints,'Marker','.','MarkerSize',20,'LineStyle','none','Color','c');
axis equal
box on
grid on
axisValue = [0 1 0 2 0 1.2];
viewPoint = [50 20];
set(fHandle1,'Position',[403 20 1120 840])
xlabel('X')
ylabel('Y')
zlabel('Z')
fHandle2 = figure(2);
set(fHandle2,'Position',[900 20 1120 840])
subplot(311)
positionThighHandle1 = line(mocap_time(1),thigh_position_mocap(1,1),'LineStyle','-','Color','r');
positionThighHandle2 = line(mocap_time(1),thigh_position_mocap(1,2),'LineStyle','-','Color','b');
positionThighHandle3 = line(mocap_time(1),thigh_position_mocap(1,3),'LineStyle','-','Color','y');
xlabel('t (s)')
ylabel('Thigh')
subplot(312)
positionKneeHandle1 = line(mocap_time(1),knee_position_mocap(1,1),'LineStyle','-','Color','r');
positionKneeHandle2 = line(mocap_time(1),knee_position_mocap(1,2),'LineStyle','-','Color','b');
positionKneeHandle3 = line(mocap_time(1),knee_position_mocap(1,2),'LineStyle','-','Color','y');
xlabel('t (s)')
ylabel('Knee')
subplot(313)
positionAnkleHandle1 = line(mocap_time(1),ankle_position_mocap(1,1),'LineStyle','-','Color','r');
positionAnkleHandle2 = line(mocap_time(1),ankle_position_mocap(1,2),'LineStyle','-','Color','b');
positionAnkleHandle3 = line(mocap_time(1),ankle_position_mocap(1,3),'LineStyle','-','Color','y');
xlabel('t (s)')
ylabel('Ankle')
for i=1:dataNum
    if mod(i,8)==1
        figure(fHandle1)
        XPoints = [ankle_position_mocap(i,1),knee_position_mocap(i,1),thigh_position_mocap(i,1)];
        YPoints = [ankle_position_mocap(i,2),knee_position_mocap(i,2),thigh_position_mocap(i,2)];
        ZPoints = [ankle_position_mocap(i,3),knee_position_mocap(i,3),thigh_position_mocap(i,3)];
        set(segmentHandle1,'XData',XPoints(1:2),'YData',YPoints(1:2),'ZData',ZPoints(1:2));
        set(segmentHandle2,'XData',XPoints(2:3),'YData',YPoints(2:3),'ZData',ZPoints(2:3));
        set(pointsHandle,'XData',XPoints,'YData',YPoints,'ZData',ZPoints);
        axis(axisValue)
        view(viewPoint)
        f1 = getframe;
        f1=frame2im(f1);
        [X,map]=rgb2ind(f1,256);
        if i==1
            imwrite(X,map,'motion3D.gif','Loopcount',inf,'DelayTime',0.05);
        else
            imwrite(X,map,'motion3D.gif','WriteMode','Append','DelayTime',0.05);
        end
    end
    
    figure(fHandle2)
    if size(buffAnkle,1)<buffSize
        buffAnkle = [buffAnkle;ankle_position_mocap(i,:)];
        buffKnee = [buffKnee;knee_position_mocap(i,:)];
        buffThigh = [buffThigh;thigh_position_mocap(i,:)];
    else
        buffAnkle(1:end-1,:) = buffAnkle(2:end,:);
        buffKnee(1:end-1,:) = buffKnee(2:end,:);
        buffThigh(1:end-1,:) = buffThigh(2:end,:);
        buffAnkle(end,:) = ankle_position_mocap(i,:);
        buffKnee(end,:) = knee_position_mocap(i,:);
        buffThigh(end,:) = thigh_position_mocap(i,:);
    end
    subplot(311)
    set(positionThighHandle1,'XData',mocap_time(i-size(buffAnkle,1)+1:i),'YData',buffThigh(:,1));
    set(positionThighHandle2,'XData',mocap_time(i-size(buffAnkle,1)+1:i),'YData',buffThigh(:,2));
    set(positionThighHandle3,'XData',mocap_time(i-size(buffAnkle,1)+1:i),'YData',buffThigh(:,3));
    f1 = getframe;
    f1=frame2im(f1);
    [X1,map1]=rgb2ind(f1,256);
    subplot(312)
    set(positionKneeHandle1,'XData',mocap_time(i-size(buffKnee,1)+1:i),'YData',buffKnee(:,1));
    set(positionKneeHandle2,'XData',mocap_time(i-size(buffKnee,1)+1:i),'YData',buffKnee(:,2));
    set(positionKneeHandle3,'XData',mocap_time(i-size(buffKnee,1)+1:i),'YData',buffKnee(:,3));
    f2 = getframe;
    f2=frame2im(f2);
    [X2,map2]=rgb2ind(f2,256);
    subplot(313)
    set(positionAnkleHandle1,'XData',mocap_time(i-size(buffThigh,1)+1:i),'YData',buffAnkle(:,1));
    set(positionAnkleHandle2,'XData',mocap_time(i-size(buffThigh,1)+1:i),'YData',buffAnkle(:,2));
    set(positionAnkleHandle3,'XData',mocap_time(i-size(buffThigh,1)+1:i),'YData',buffAnkle(:,3));
    f3 = getframe;
    f3=frame2im(f3);
    [X3,map3]=rgb2ind(f3,256);
    if mod(i,8)==1
        if i==1
            imwrite(X1,map1,'motionSignals1.gif', 'Loopcount',inf,'DelayTime',0.05);
            imwrite(X2,map2,'motionSignals2.gif', 'Loopcount',inf,'DelayTime',0.05);
            imwrite(X3,map3,'motionSignals3.gif', 'Loopcount',inf,'DelayTime',0.05);
        else
            imwrite(X1,map1,'motionSignals1.gif','WriteMode','Append','DelayTime',0.05);
            imwrite(X2,map2,'motionSignals2.gif','WriteMode','Append','DelayTime',0.05);
            imwrite(X3,map3,'motionSignals3.gif','WriteMode','Append','DelayTime',0.05);
        end
    end
end