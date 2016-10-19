function [xt,yt,zt]=fetchXYZtool(sub)

Tmat=zeros(4,4);
Tmat(1:3,1:3)=eye(3);
Tmat(4,4)=1;
Tmat(1,4)=122*10^-3;

x=sub.LatestMessage.Translation.X;
y=sub.LatestMessage.Translation.Y;
z=sub.LatestMessage.Translation.Z;
quat=[sub.LatestMessage.Rotation.W sub.LatestMessage.Rotation.X sub.LatestMessage.Rotation.Y sub.LatestMessage.Rotation.Z];

emmat=zeros(4,4);
emmat(4,4)=1;
emmat(1:3,1:3)=quat2rotm(quat);
emmat(1:3,4)=[x;y;z];

Fmat=emmat*Tmat;

xt=Fmat(1,4)*1000;
yt=Fmat(2,4)*1000;
zt=Fmat(3,4)*1000;

end