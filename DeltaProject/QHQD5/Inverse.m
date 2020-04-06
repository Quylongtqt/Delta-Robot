function [valid , theta] = Inverse(P)%theta 
alpha = [0;2*pi/3;4*pi/3];
rA = 110;
rB = 50;
r = rA-rB;
La = 202.5;
Lb = 625;
A = zeros(3,1);
B = zeros(3,1);
C = zeros(3,1);
t = zeros(3,1);
validP = zeros(3,1);

for i=1:3
   A(i,1) = P(1,1)^2+P(2,1)^2+P(3,1)^2+r^2+La^2-Lb^2-2*r*(P(1,1)*cos(alpha(i,1))+P(2,1)*sin(alpha(i,1)));
   B(i,1) = -2*La*(-r+P(1,1)*cos(alpha(i,1))+P(2,1)*sin(alpha(i,1)));
   C(i,1) = -2*La*P(3,1);
end

for i=1:3
    k = 4*C(i,1)^2 -4*(A(i,1)^2-B(i,1)^2);
    if k>=0
       l = 2*atan((-2*C(i,1) + sqrt(k))/(2*(A(i,1)-B(i,1))));
       n = 2*atan((-2*C(i,1) - sqrt(k))/(2*(A(i,1)-B(i,1))));
       if abs(l)<=(pi/3)
           t(i,1) = l;
           validP(i,1)= 1;
       else if abs(n)<=(pi/3)
           t(i,1) = n;
           validP(i,1)= 1;
           end
       end
    else
    t(i,1) = 0;
    validP(i,1)= 0;
    end
end
valid = validP;
theta(1,1) = t(1,1)*180/pi
theta(2,1) = t(2,1)*180/pi
theta(3,1) = t(3,1)*180/pi

end
