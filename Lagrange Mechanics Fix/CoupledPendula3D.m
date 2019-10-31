% Simulation of coupled pendulum by Lagrangian mechanics
% see also http://www.eng.auburn.edu/~marghitu/MECH6710/06_Fall/MATLAB/C6_analytical/C6_MATLAB.pdf
close all
clear all
% generalized coordinates==================================================
syms t real
theta=sym('theta(t)');
phi=sym('phi(t)');
zeta=sym('zeta(t)');
xi=sym('xi(t)');
% constants, length, mass, g, geometry=====================================
J1=[-3;-1;0];
J2=[2;3;0];
L1=7;
L2=6;
m1=4;
m2=3;
g=9.81;
J1J2=norm(J1-J2); % distance between joints
d0=4; % rest length spring
% positions and velocities as function of the generalized coordinates======
x1=J1(1)+L1*sin(theta)*cos(phi);
y1=J1(2)+L1*sin(theta)*sin(phi);
z1=J1(3)-L1*cos(theta);
x2=J2(1)+L2*sin(zeta)*cos(xi);
y2=J2(2)+L2*sin(zeta)*sin(xi);
z2=J2(3)-L2*cos(zeta);
x1dot=diff(x1,t);
y1dot=diff(y1,t);
z1dot=diff(z1,t);
x2dot=diff(x2,t);
y2dot=diff(y2,t);
z2dot=diff(z2,t);
% kinetic and potential energy=============================================
T=m1/2*(x1dot^2+y1dot^2+z1dot^2)+m2/2*(x2dot^2+y2dot^2+z2dot^2);
k=0.5;
V=m1*g*z1+m2*g*z2+1/2*k*(sqrt((x2-x1)^2+(y2-y1)^2+(z2-z1)^2)-d0)^2;
% Lagrangian ==============================================================
L=T-V;
% dL/d(qdot)===============================================================
dummy=sym('dummy');
dLthetadot = subs(diff(subs(L,diff(theta,t),dummy),dummy),dummy,diff(theta,t));
dLphidot = subs(diff(subs(L,diff(phi,t),dummy),dummy),dummy,diff(phi,t));
dLzetadot = subs(diff(subs(L,diff(zeta,t),dummy),dummy),dummy,diff(zeta,t));
dLxidot = subs(diff(subs(L,diff(xi,t),dummy),dummy),dummy,diff(xi,t));
% dL/dq====================================================================
dLdtheta=subs(diff(subs(L,theta,dummy),dummy),dummy,theta);
dLdphi=subs(diff(subs(L,phi,dummy),dummy),dummy,phi);
dLdzeta=subs(diff(subs(L,zeta,dummy),dummy),dummy,zeta);
dLdxi=subs(diff(subs(L,xi,dummy),dummy),dummy,xi);
% dFdq=====================================================================
k=0.5; % dissipation constant
F=1/2*k*(x1dot^2+y1dot^2+z1dot^2+x2dot^2+y2dot^2+z2dot^2);
dFdthetadot=subs(diff(subs(F,diff(theta,t),dummy),dummy),dummy,diff(theta,t));
dFdphidot=subs(diff(subs(F,diff(phi,t),dummy),dummy),dummy,diff(phi,t));
dFdzetadot=subs(diff(subs(F,diff(zeta,t),dummy),dummy),dummy,diff(zeta,t));
dFdxidot=subs(diff(subs(F,diff(xi,t),dummy),dummy),dummy,diff(xi,t));
% generalized equations of motion==========================================
differentialequation1=diff(dLthetadot,t)-dLdtheta+dFdthetadot;
differentialequation2=diff(dLphidot,t)-dLdphi+dFdphidot;
differentialequation3=diff(dLzetadot,t)-dLdzeta+dFdzetadot;
differentialequation4=diff(dLxidot,t)-dLdxi+dFdxidot;
% abbreviation of variables================================================
variables={theta,phi,zeta,xi,diff(theta,t),diff(phi,t),diff(zeta,t),diff(xi,t),...
           diff(theta,t,2),diff(phi,t,2),diff(zeta,t,2),diff(xi,t,2)};    
variablesshort={'x(1)','x(2)','x(3)','x(4)','x(5)','x(6)','x(7)','x(8)',...
          'thetaddot','phiddot','zetaddot','xiddot'};
DifferentialEquation1=subs(differentialequation1,variables,variablesshort);
DifferentialEquation2=subs(differentialequation2,variables,variablesshort);
DifferentialEquation3=subs(differentialequation3,variables,variablesshort);
DifferentialEquation4=subs(differentialequation4,variables,variablesshort);
% solve for thetaddot, phiddot, zetaddot, xiddot===========================
solution=solve(DifferentialEquation1,DifferentialEquation2,DifferentialEquation3,DifferentialEquation4,...
              'thetaddot','phiddot','zetaddot','xiddot');
THETADDOT=solution.thetaddot;
PHIDDOT=solution.phiddot;
ZETADDOT=solution.zetaddot;
XIDDOT=solution.xiddot;
% solve non linear ode system==============================================
time=linspace(0,60,1000);
% initial conditions [theta,phi,zeta,xi,thetadot,phidot,zetadot,xidot]=====
x0=[pi/5 pi/4 -pi/6 -pi/6 0 0 0 0]; 
str=['xdot=@(t,x)[x(5);x(6);x(7);x(8);',char(THETADDOT),';',char(PHIDDOT),';',char(ZETADDOT),';',char(XIDDOT),'];'];
eval(str);
[t,q]=ode23(xdot,time,x0);
% Calculute positions as function of generalized coordinates===============
X1=J1(1)+L1*sin(q(:,1)).*cos(q(:,2));
Y1=J1(2)+L1*sin(q(:,1)).*sin(q(:,2));
Z1=J1(3)-L1*cos(q(:,1));
X2=J2(1)+L2*sin(q(:,3)).*cos(q(:,4));
Y2=J2(2)+L2*sin(q(:,3)).*sin(q(:,4));
Z2=J2(3)-L2*cos(q(:,3));
% plot solution============================================================
set(gcf,'color','w')
set(gca,'Projection','perspective')
view(-140,10)
axis vis3d
axis on
box on
hold on
colormap([0,0,1])
[xm,ym,zm]=sphere(20);
axis([-5,5,-5,5,-8,0]);
for i=1:numel(time)
    cla;
    plot3([J1(1),X1(i)],[J1(2),Y1(i)],[J1(3),Z1(i)],'b','linewidth',4);
    plot3([J2(1),X2(i)],[J2(2),Y2(i)],[J2(3),Z2(i)],'b','linewidth',4);
    surf(X1(i)+0.15*m1*xm,Y1(i)+0.15*m1*ym,Z1(i)+0.15*m1*zm);
    surf(X2(i)+0.15*m2*xm,Y2(i)+0.15*m2*ym,Z2(i)+0.15*m2*zm);
    DrawSpring3D([X1(i);Y1(i);Z1(i)],[X2(i);Y2(i);Z2(i)],12,0.5)
    camlight headlight
    shading flat
    drawnow
 end
