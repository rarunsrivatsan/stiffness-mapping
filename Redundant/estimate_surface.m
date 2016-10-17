clear;clc;close all;

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
        % DO STUFF HERE
        %[x(count),y(count),z(count)]=fetchXYZ(sub);
        x(count,1)=sub.LatestMessage.Translation.X;
        y(count,1)=sub.LatestMessage.Translation.Y;
        z(count,1)=-sub.LatestMessage.Translation.Z;
%         if count>15
%             tri=delaunay(x,y);
%             trisurf(tri,x,y,z);
%             xlabel('X Axis');
%             ylabel('Y Axis');
%             zlabel('Z axis');
%             shading interp
%             grid on;
%             view(-114,53);
%         end
        count=count+1;
end