clear;clc;close all;
optoSub=rossubscriber('/WrenchData');
pause(0.1);
bias=optoSub.LatestMessage.Wrench.Force;
meanz= -0.1920;

sub=rossubscriber('/matlab_sensor1_msg');
pause(0.1);
count=1;

xpoints=[1 -1 1 -1];
ypoints=[-1 1 1 -1];

triInit=delaunay(xpoints,ypoints);
fig=trisurf(triInit,xpoints,ypoints,zeros(4,1));
% hold on;
% fig2=plot3(0,0,0,'.','MarkerSize',15);
xlabel('X Axis');
ylabel('Y Axis');
zlabel('Stiffness');
grid on;
view(-74,-48);

%% Section
while(1)
    fvec.X=(optoSub.LatestMessage.Wrench.Force.X-bias.X)/1000;
    fvec.Y=(optoSub.LatestMessage.Wrench.Force.Y-bias.Y)/1000;
    fvec.Z=(optoSub.LatestMessage.Wrench.Force.Z-bias.Z)/1000;
    %optoMag=sqrt((fvec.X)^2+(fvec.Y)^2+(fvec.Z)^2);
    optoMag=fvec.Z;
    if optoMag>0.1
        % DO STUFF HERE
        force(count)=optoMag;
        [x(count),y(count),z(count)]=fetchXYZ(sub);
        stiffness(count)=force(count)./-(z(count)-meanz);
        if count>15
            tri=delaunay(x,y);
            trisurf(tri,x,y,abs(stiffness));
            xlabel('X Axis');
            ylabel('Y Axis');
            zlabel('Stiffness');
            shading interp
            grid on;
            view(-114,53);
        end
        count=count+1;
    end
    pause(0.01);
end