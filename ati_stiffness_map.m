%% Open Ros
clear;clc;
try
    rosinit
catch
end
atiSub=rossubscriber('/ft_sensor/netft_data');
emSub=rossubscriber('/matlab_sensor1_msg');
%%
tic;
while(toc<60)
    x=fetchXYZEM(emSub);
    %x=showdetails(receive(emSub))
    display(x(1:3,4)'.*1000)
    
end
%%
tic;
while(toc<60)
    x=receive(atiSub);
    temp=x.Wrench.Force;
        forcet=[temp.X,temp.Y,temp.Z];
    display(norm(forcet));

end

%%
force_tmp=[];
t=0;
while(t<120)
    x=receive(atiSub);
    temp=x.Wrench.Force;
    forcet=[temp.X,temp.Y,temp.Z];
    display(norm(forcet));
    force_tmp=[force_tmp,(norm(forcet))];
  t=t+1;
end
min_force=max(force_tmp);
%%
%detect force
%start saving force and position
%stop recording once force decreases below threshold
tic;
flag=0;probe=[];force=[];position=[];nocontact=1;
stopProbe=0;
while(stopProbe==0)
    x=receive(atiSub);
    y=fetchXYZEM(emSub);
    temp=x.Wrench.Force;
    forcet=[temp.X,temp.Y,temp.Z];
    display(norm(forcet));
    if norm(forcet)>min_force
        if nocontact==1
            flag=flag+1;
            ii=1;
            nocontact=0;
        end
        
        if ii>=2
            if norm( forcet)>norm( probe{flag}.force(ii-1,:))
                probe{flag}.force(ii,:)=forcet;
                probe{flag}.position(ii,:)=y(1:3,4)'*1000;
                ii=ii+1;
            end
        else
            probe{flag}.force(ii,:)=forcet;
            probe{flag}.position(ii,:)=y(1:3,4)'*1000;
        end
        
    else
        nocontact=1;
    end
    if flag==10
        stopProbe=1;
    end
end

%%
disp=[];forcemag=[];
for jj=1:flag-1
    tt=1;
    for ii=1:length(probe{jj}.force)-1
        probe{jj}.disp(tt)=norm(probe{jj}.position(ii+1,:)-probe{jj}.position(1,:));
        probe{jj}.forcemag(tt)=norm(probe{jj}.force(ii+1,:)-probe{jj}.force(1,:));
        tt=tt+1;
    end
end
%%
for jj=1:flag-1
scatter(probe{jj}.disp,probe{jj}.forcemag);
waitforbuttonpress;
end
%%
for jj=1:flag-1
scatter3(probe{jj}.position(:,1),probe{jj}.position(:,2),probe{jj}.position(:,3),'r','fill');
axis equal;
hold on
end