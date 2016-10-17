
sub=rossubscriber('/matlab_sensor1_msg');
figure(1);
hold on;

idx = 1;
x(idx) = sub.LatestMessage.Translation.X * 1000;
y(idx) = sub.LatestMessage.Translation.Y * 1000;
z(idx) = sub.LatestMessage.Translation.Z * 1000;
h = scatter(x, y, '.','k');


idx = 1;


while 1
    
    idx = idx + 1;
    
    figure(1);
%     set(h,'XData',x,'YData',y,'ZData',z);
    set(h,'XData',x,'YData',y,'ZData',z);

    x = sub.LatestMessage.Translation.X * 1000;
    y = sub.LatestMessage.Translation.Y * 1000;
    z = sub.LatestMessage.Translation.Z * 1000;
    
    scatter(x, y, '.','b');
    pause(1)
end