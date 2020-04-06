function [position,speed,acceleration]=pathPoly(a,t)
position = a(:,1) + a(:,2)*t + a(:,3)*t^2 + a(:,4)*t^3+ a(:,5)*t^4 + a(:,6)*t^5;
speed = a(:,2)+ 2*a(:,3)*t + 3*a(:,4)*t^2+4*a(:,5)*t^3 + 5*a(:,6)*t^4;
acceleration = 2*a(:,3)+ 6*a(:,4)*t + 12*a(:,5)*t^2+20*a(:,6)*t^3;
end
