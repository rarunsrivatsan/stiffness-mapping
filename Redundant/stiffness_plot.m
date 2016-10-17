clear;clc;close all;
optoSub=rossubscriber('/WrenchData');
pause(0.5);
bias=optoSub.LatestMessage.Wrench.Force;
meanz=-0.0796;

sub=rossubscriber('/matlab_sensor1_msg');
pause(0.1);
count=1;

fig=quiver3(0,0,0,0,0,0);
xlabel('X Axis');
ylabel('Y Axis');
zlabel('Stiffness');
grid on;
view(-74,-48);
%%
while(1)
    fvec.X=(optoSub.LatestMessage.Wrench.Force.X-bias.X)/1000;
    fvec.Y=(optoSub.LatestMessage.Wrench.Force.Y-bias.Y)/1000;
    fvec.Z=(optoSub.LatestMessage.Wrench.Force.Z-bias.Z)/1000;
    %optoMag=sqrt((fvec.X)^2+(fvec.Y)^2+(fvec.Z)^2);
    optoMag=abs(fvec.Z);
    if optoMag>0.15
        % DO STUFF HERE
        force(count)=optoMag;
        [x(count),y(count),z(count)]=fetchXYZ(sub);
        %stiffness(count)=force(count)./-(z(count)-meanz);
        stiffness(count)=force(count);
        set(fig,'XData',x,'YData',y,'ZData',zeros(1,count),'UData',zeros(1,count),'VData',zeros(1,count),'WData',abs(stiffness));
        count=count+1;
    end
    pause(0.01);
end