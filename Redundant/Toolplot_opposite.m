% roslaunch trakstar trakstar.launch
clear; clc;
sub=rossubscriber('/matlab_sensor1_msg');
pause(0.1);
num=1000;
x=sub.LatestMessage.Translation.X;
y=sub.LatestMessage.Translation.Y;
z=-sub.LatestMessage.Translation.Z;
quat=[sub.LatestMessage.Rotation.W sub.LatestMessage.Rotation.X sub.LatestMessage.Rotation.Y sub.LatestMessage.Rotation.Z];

emmat=zeros(4,4);
emmat(1:3,1:3)=quat2rotm(quat);
emmat(1:3,4)=[x;y;z];
emmat(4,4)=1;

Tmat=zeros(4,4);
Tmat(1:3,1:3)=eye(3);
Tmat(4,4)=1;
Tmat(1,4)=-0.101;
Fmat=emmat*Tmat;

xt=Fmat(1,4);
yt=Fmat(2,4);
zt=Fmat(3,4);

vec=[Fmat(1:3,1:3)*[1;0;0]]';

fig=quiver3(xt,yt,zt,vec(1),vec(2),vec(3),'b');
hold on;
fig2=quiver3(x,y,z,emmat(1,1),emmat(2,1),emmat(3,1),'r');

xlabel('X Axis');
ylabel('Y Axis');
zlabel('Z Axis');
axis equal;grid on;
view(113,35);
%axis([-0.5 0 -0.5 0 -1 1]);
count=1;
%%
while(1)
x(count,1)=sub.LatestMessage.Translation.X;
y(count,1)=sub.LatestMessage.Translation.Y;
z(count,1)=-sub.LatestMessage.Translation.Z;
quat=[sub.LatestMessage.Rotation.W sub.LatestMessage.Rotation.X sub.LatestMessage.Rotation.Y sub.LatestMessage.Rotation.Z];

emmat(1:3,1:3)=quat2rotm(quat);
emmat(1:3,4)=[x(count);y(count);z(count)];
emvec(count,:)=[emmat(1:3,1)]';

Fmat=emmat*Tmat;

xt(count,1)=Fmat(1,4);
yt(count,1)=Fmat(2,4);
zt(count,1)=Fmat(3,4);

vec(count,:)=[Fmat(1:3,1:3)*[1;0;0]]';

set(fig,'XData',xt,'YData',yt,'ZData',zt,'UData',vec(:,1),'VData',vec(:,2),'WData',vec(:,3));
set(fig2,'XData',x,'YData',y,'ZData',z,'UData',emvec(:,1),'VData',emvec(:,2),'WData',emvec(:,3));
pause(0.01);
count=count+1;
end
%%
while(count>=num)
x(count)=sub.LatestMessage.Translation.X;
y(count)=sub.LatestMessage.Translation.Y;
z(count)=-sub.LatestMessage.Translation.Z;
quat=[sub.LatestMessage.Rotation.W sub.LatestMessage.Rotation.X sub.LatestMessage.Rotation.Y sub.LatestMessage.Rotation.Z];

emmat(1:3,1:3)=quat2rotm(quat);
emmat(1:3,4)=[x(count);y(count);z(count)];

vec(count,:)=[Tmat*emmat*[1;0;0;1]]';

set(fig,'XData',x(count-num+1:end)','YData',y(count-num+1:end)','ZData',z(count-num+1:end)','UData',vec((count-num+1:end),1),'VData',vec((count-num+1:end),2),'WData',vec((count-num+1:end),3));
pause(0.01);
count=count+1;
end