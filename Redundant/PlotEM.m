% roslaunch trakstar trakstar.launch
sub=rossubscriber('/matlab_sensor1_msg');
clear x y z;
pause(0.1);
x1=sub.LatestMessage.Translation.X*1000;
y1=sub.LatestMessage.Translation.Y*1000;
z1=sub.LatestMessage.Translation.Z*1000;

[x,y,z]=fetchXYZEM(sub);

fig=scatter3(x,y,z,'.','b');
hold on;
fig2=scatter3(x1,y1,z1,'.','r');
xlabel('X Axis')
ylabel('Y Axis')
zlabel('Z Axis');
axis equal;
count=1;
num=300;

while(1)
set(fig,'XData',x,'YData',y,'ZData',z);
set(fig2,'XData',x1,'YData',y1,'ZData',z1);
x1(count)=sub.LatestMessage.Translation.X;
y1(count)=sub.LatestMessage.Translation.Y;
z1(count)=sub.LatestMessage.Translation.Z;
[x(count),y(count),z(count)]=fetchXYZEM(sub);
pause(0.01);
count=count+1;
end

while(count>=num)
set(fig,'XData',x(count-num+1:end),'YData',y(count-num+1:end),'ZData',z(count-num+1:end));
x(count)=sub.LatestMessage.Translation.X;
y(count)=sub.LatestMessage.Translation.Y;
z(count)=sub.LatestMessage.Translation.Z;
pause(0.01);
count=count+1;
end