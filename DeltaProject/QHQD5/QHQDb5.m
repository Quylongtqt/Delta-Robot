function [vitri_P,vantoc_P,giatoc_P,theta,thetadot] =QHQDb5(A,B,time)
k=0;

C = [B(1);B(2);B(3)+100];

AC = paramsPoly([A,C],time*0.7,0);

for i=0:0.02:time*0.7
    k = k+1;
    [vitri_P(:,k),vantoc_P(:,k),giatoc_P(:,k)] = pathPoly(AC,i);  
end

CB = paramsPoly([C,B],time*0.3,0);
for i=0.1:0.02:time*0.3
    k = k+1;
    [vitri_P(:,k),vantoc_P(:,k),giatoc_P(:,k)] = pathPoly(CB,i);
end

for i=1:1:k
   [vail, theta(:,i)] = Inverse(vitri_P(:,i));
end

for i=1:1:k
    thetadot(:,i) = InvJ(vantoc_P(:,i),vitri_P(:,i),theta(:,i))*180/pi;

end


figure('Name','XYZ trajec');
plot3(vitri_P(1,:),vitri_P(2,:),vitri_P(3,:));
grid on;

figure('Name','XYZ velocity');
plot(vantoc_P(1,:));hold on;plot(vantoc_P(2,:));plot(vantoc_P(3,:));
legend('Xdot','Ydot','Zdot');
grid on;

figure('Name','XYZ acceleration');
plot(giatoc_P(1,:));hold on;plot(giatoc_P(2,:));plot(giatoc_P(3,:));
legend('Xdotdot','Ydotdot','Zdotdot');
grid on;

figure('Name','join angle');
plot(theta(1,:)*180/pi);hold on;plot(theta(2,:)*180/pi);plot(theta(3,:)*180/pi);
legend('theta1','theta2','theta3');
grid on;

figure('Name','join velocity');
plot(thetadot(1,:));hold on;plot(thetadot(2,:));plot(thetadot(3,:));
legend('theta1dot','theta2dot','theta3dot');
grid on;

end