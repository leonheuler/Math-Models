function J = J(t,q1,q2,q3,L1,L2,L3)
%J
%    J = J(T,Q1,Q2,Q3,L1,L2,L3)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    15-Nov-2019 23:00:26

t2 = q1-q2+q3;
t3 = cos(t2);
t4 = (L3.*t3)./2.0;
t5 = q1-q2;
t6 = cos(t5);
t7 = L2.*t6;
t8 = sin(t2);
t9 = sin(t5);
t10 = (L3.*t8)./2.0;
J = reshape([t4+t7+L1.*cos(q1),-L2.*t9-(L3.*t8)./2.0-L1.*sin(q1),-t4-t7,t10+L2.*t9,t4,-t10],[2,3]);
