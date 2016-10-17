clear variables;
close all;
sub=rossubscriber('/matlab_sensor1_msg');
f = figure(1);

while 1
    
    if ~ishghandle(f)
    break;
    end;
    trans_vector = [sub.LatestMessage.Translation.X; sub.LatestMessage.Translation.Y; sub.LatestMessage.Translation.Z];
    
    quat=[sub.LatestMessage.Rotation.W sub.LatestMessage.Rotation.X sub.LatestMessage.Rotation.Y sub.LatestMessage.Rotation.Z];

    em=zeros(4,4);
    em(4,4)=1;

    em(1:3,1:3)=quat2rotm(quat);
    em(1:3,4)=[trans_vector];
    x = em(1,4) * 1000;
    y = em(2,4) * 1000;
    z = em(3,4) * 1000;

    scatter3(x, y, z, 'o','b');

    hold on;


    T=zeros(4,4);
    T(1:3,1:3)=eye(3);
    T(4,4)=1;
    T(1,4)=4.5 * 10^-3;
    
    final = em* T;
    x = final(1,4) * 1000;
    y = final(2,4) * 1000;
    z = final(3,4) * 1000;

    scatter3(x, y, z, '.','r');
    pause(0.01);
end