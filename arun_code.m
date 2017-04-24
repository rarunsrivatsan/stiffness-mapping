%% Open Ros
clear;clc;
try
    rosinit
catch
end
atiSub=rossubscriber('/ft_sensor/netft_data');
emSub=rossubscriber('/matlab_sensor1_msg');

%% detect force
%start saving force and position
%stop recording once force decreases below threshold
tic;
flag=0;ii=1;probe=[];force=[];position=[];nocontact=1;
stopProbe=0;
while(stopProbe==0)
    x=receive(atiSub);
    y=fetchXYZEM(emSub);
    temp=x.Wrench.Force;
    forcet=[temp.X,temp.Y,temp.Z];
    
    if norm(forcet)>0.4
        if nocontact==1
            flag=flag+1;
            nocontact=0;
        end
        
        probe{flag}.force(ii,:)=forcet;
        probe{flag}.position(ii,:)=y(1:3,4)'*1000;
        probe{flag}.orientation(ii,:)=rotm2quat(y(1:3,1:3));
        ii=ii+1;
    else
         nocontact=1;
      
    end
    if flag==2
        stopProbe=1;
    end
end
%%
newp=[];
    for ii=1:length(probe{1}.orientation)
        newp(ii,:)=probe{1}.position(ii,:)+transpose(quat2rotm(probe{1}.orientation(ii,:))*[0;0;0]);

        
        

scatter3(probe{1}.position(:,1),probe{1}.position(:,2),probe{1}.position(:,3),'r','fill');
axis equal;
hold on
scatter3(newp(:,1),newp(:,2),newp(:,3),'b','fill');
%%
ii=1;stopProbe=0; force=[];position=[];orientation=[];
while(stopProbe==0)
    x=receive(atiSub);
    y=fetchXYZEM(emSub);
    temp=x.Wrench.Force;
    forcet=[temp.X,temp.Y,temp.Z];
    force(ii,:)=forcet;
    position(ii,:)=y(1:3,4)'*1000;
    orientation(ii,:)=rotm2quat(y(1:3,1:3));
  ii=ii+1
    if ii>2000
        stopProbe=1;
    end
end
%%

scatter3(position(:,1),position(:,2),position(:,3),'r','fill');
axis equal