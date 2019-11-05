% Simulation of coupled pendulum by Lagrangian mechanics
% see also http://www.eng.auburn.edu/~marghitu/MECH6710/06_Fall/MATLAB/C6_analytical/C6_MATLAB.pdf
close all
clear all
% generalized coordinates==================================================
syms t real
syms theta(t)
syms phi(t)
% constants, length, mass, g, geometry=====================================
L1=7.5;
L2=6;
m1=4;
m2=3;
g=9.81;
J1J2=10; % distance between joints
d0=6; % rest length spring
% positions and velocities as function of the generalized coordinates======
x1=-J1J2/2+L1*sin(theta);
y1=-L1*cos(theta);
x2=J1J2/2+L2*sin(phi); 
y2=-L2*cos(phi); 
x1dot=diff(x1,t);
x2dot=diff(x2,t);
y1dot=diff(y1,t);
y2dot=diff(y2,t);
% kinetic and potential energy=============================================
T=m1/2*(x1dot^2+y1dot^2)+m2/2*(x2dot^2+y2dot^2);
k=0.5;
V=m1*g*y1+m2*g*y2+1/2*k*(sqrt((x2-x1)^2+(y2-y1)^2)-d0)^2;
% determine for which theta=alpha and phi=beta the system is at rest
alpha=sym('alpha');
beta=sym('beta');
v=subs(V,{theta,phi},{alpha,beta});
S=vpasolve([diff(v,alpha),diff(v,beta)],[alpha,beta]);
alpha=double(S.alpha);
beta=double(S.beta);
% Lagrangian ==============================================================
L=T-V;
% dL/d(qdot)===============================================================
syms dummy
dLthetadot = subs(diff(subs(L,diff(theta,t),dummy),dummy),dummy,diff(theta,t));
dLphidot =   subs(diff(subs(L,diff(phi,t),dummy),dummy),dummy,diff(phi,t));
% dL/dq====================================================================
dLdtheta=subs(diff(subs(L,theta,dummy),dummy),dummy,theta);
dLdphi=subs(diff(subs(L,phi,dummy),dummy),dummy,phi);
% dFdq============================a=========================================
k=0.25; % dissipation constant
F=1/2*k*(x1dot^2+y1dot^2+x2dot^2+y2dot^2);
dFdthetadot=subs(diff(subs(F,diff(theta,t),dummy),dummy),dummy,diff(theta,t));
dFdphidot=subs(diff(subs(F,diff(phi,t),dummy),dummy),dummy,diff(phi,t));
% generalized equations of motion==========================================
differentialequation1=diff(dLthetadot,t)-dLdtheta+dFdthetadot;
differentialequation2=diff(dLphidot,t)-dLdphi+dFdphidot;
% abbreviation of variables================================================
variables={theta,phi,diff(theta,t),diff(phi,t),diff(theta,t,2),diff(phi,t,2)};    
x = sym('x', [4, 1]);assume(x,'real');
syms thetaddot phiddot
variablesshort={x(1),x(2),x(3),x(4),thetaddot,phiddot};
% variablesshort=str2sym(variablesshort);
DifferentialEquation1=subs(differentialequation1,variables,variablesshort);
DifferentialEquation2=subs(differentialequation2,variables,variablesshort);
% solve for thetaddot, phiddot=============================================
solution=solve(DifferentialEquation1,DifferentialEquation2,thetaddot,phiddot);
THETADDOT=solution.thetaddot;
PHIDDOT=solution.phiddot;
% solve non linear ode system==============================================
time=linspace(0,60,2000);
% initial conditions [theta,phi,thetadot,phidot]===========================
x0=[pi/12 -pi/6 0 0]; 
str=['xdot=@(t,x)[x(3);x(4);',char(THETADDOT),';',char(PHIDDOT),'];'];
str = strrep(str, 'x1','x(1)');
str = strrep(str, 'x2','x(2)');
str = strrep(str, 'x3','x(3)');
str = strrep(str, 'x4','x(4)');

eval(str);


[t,q]=ode23(xdot,time,x0);
% Calculute positions as function of generalized coordinates===============
X1=-J1J2/2+L1*sin(q(:,1));
Y1=-L1*cos(q(:,1));
X2=J1J2/2+L2*sin(q(:,2));
Y2=-L2*cos(q(:,2));
% plot solution============================================================
set(gcf,'color',[1 1 1])
hold on
box on
axis equal
for i=1:numel(time)
    cla;
    plot([-J1J2/2,X1(i)],[0,Y1(i)],'k','linewidth',4);
    plot(X1(i),Y1(i),'o','MarkerFaceColor','k','MarkerEdgeColor','k','MarkerSize',4*m1);
    plot([J1J2/2,X2(i)],[0,Y2(i)],'k','linewidth',4);
    plot(X2(i),Y2(i),'o','MarkerFaceColor','k','MarkerEdgeColor','k','MarkerSize',4*m2);
    axis([-12,12,-10,0]);
    DrawSpring2D([X1(i);Y1(i)],[X2(i);Y2(i)],12,0.25)
    drawnow
end
