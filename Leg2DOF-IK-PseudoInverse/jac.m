function J = jac(in1)
%JAC
%    J = JAC(IN1)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    03-Aug-2019 20:47:53

q1 = in1(1,:);
q2 = in1(2,:);
t2 = q1-q2;
t3 = cos(t2);
t4 = t3./2.0;
t5 = sin(t2);
t6 = t5./2.0;
J = reshape([t4+cos(q1)./2.0,t6+sin(q1)./2.0,-t4,-t6],[2,2]);
