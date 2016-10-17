% roslaunch trakstar trakstar.launch
clear; clc;
sub=rossubscriber('/matlab_sensor1_msg');
pause(0.5);
x=sub.LatestMessage.Translation.X;
y=sub.LatestMessage.Translation.Y;
z=sub.LatestMessage.Translation.Z;
quat=[sub.LatestMessage.Rotation.W sub.LatestMessage.Rotation.X sub.LatestMessage.Rotation.Y sub.LatestMessage.Rotation.Z];

rotationmatrix=eye(3);
vec(1,:)=[quat2rotm(quat)*rotationmatrix*[1;0;0]]';

fig=quiver3(x,y,z,vec(1),vec(2),vec(3));
fig.MaxHeadSize=1;
fig.LineWidth=2;
xlabel('X Axis');
ylabel('Y Axis');
zlabel('Z Axis');
axis equal
count=1;
%axis([-1 1 -1 1 -1 1]);
%axis([0 1 0 1 0 1]);
view(135,35);
%%
while(1)
x(count)=-sub.LatestMessage.Translation.X;
y(count)=sub.LatestMessage.Translation.Y;
z(count)=sub.LatestMessage.Translation.Z;
quat=[sub.LatestMessage.Rotation.W sub.LatestMessage.Rotation.X sub.LatestMessage.Rotation.Y sub.LatestMessage.Rotation.Z];

vec(count,:)=[quat2rotm(quat)*rotationmatrix*[1;0;0]]';

set(fig,'XData',x','YData',y','ZData',z','UData',vec(:,1),'VData',vec(:,2),'WData',vec(:,3));
pause(0.01);
count=count+1;
end