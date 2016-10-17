clear;clc
optoSub=rossubscriber('/WrenchData');
pause(0.1);
bias=optoSub.LatestMessage.Wrench.Force;
meanz=-0.0863;

sub=rossubscriber('/matlab_sensor1_msg');
pause(0.1);
count=1;

fig=scatter(0,0);
%
while(1)
fvec.X=(optoSub.LatestMessage.Wrench.Force.X-bias.X)/1000;
fvec.Y=(optoSub.LatestMessage.Wrench.Force.Y-bias.Y)/1000;
fvec.Z=(optoSub.LatestMessage.Wrench.Force.Z-bias.Z)/1000;
optoMag=sqrt((fvec.X)^2+(fvec.Y)^2+(fvec.Z)^2);
if optoMag>0.1
    %DO STUFF HERE
    force(count)=optoMag;
    x(count)=-sub.LatestMessage.Translation.X;
    y(count)=-sub.LatestMessage.Translation.Y;
    z(count)=-sub.LatestMessage.Translation.Z
    count=count+1;
    set(fig,'XData',z-meanz,'YData',force);
end
pause(0.01);
end