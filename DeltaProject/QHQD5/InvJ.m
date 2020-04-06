function [thetadot] = InvJ(veloP,P,theta)
la = 200;
lb = 516;
r = 60; %r=rA-rB=110-50
Jp = zeros(3,3);
Jt = zeros(3,3);

Jp = [P(1)-(r+la*cos(theta(1))), P(2), P(3)-la*sin(theta(1));...
      P(1)-(r+la*cos(theta(2)))*cos(2*pi/3), P(2)-(r+la*cos(theta(2)))*sin(2*pi/3), P(3)-la*sin(theta(2));...
      P(1)-(r+la*cos(theta(3)))*cos(4*pi/3), P(2)-(r+la*cos(theta(3)))*sin(4*pi/3), P(3)-la*sin(theta(3))];
  
Jt = [-la*P(1)*sin(theta(1)) + la*P(3)*cos(theta(1)) + la*r*sin(theta(1)), 0, 0;...
    0, 0.5*la*P(1)*sin(theta(2))-sin(2*pi/3)*la*P(2)*sin(theta(2))+la*P(3)*cos(theta(2))+la*r*sin(theta(2)), 0;...
    0,0, 0.5*la*P(1)*sin(theta(3))-sin(4*pi/3)*la*P(2)*sin(theta(3))+la*P(3)*cos(theta(3))+la*r*sin(theta(3))];

thetadot = inv(Jt)*Jp*veloP;
end
