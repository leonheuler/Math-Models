function dotJ = dotJ(in1,in2)
%DOTJ
%    DOTJ = DOTJ(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    03-Oct-2019 23:46:29

q1 = in1(1,:);
q2 = in1(2,:);
q_dot1 = in2(1,:);
q_dot2 = in2(2,:);
t2 = q1+q2;
t3 = cos(t2);
t4 = q_dot1+q_dot2;
t5 = sin(t2);
dotJ = reshape([t3.*t4.*(-1.0./2.0)-(q_dot1.*cos(q1))./2.0,t4.*t5.*(-1.0./2.0)-(q_dot1.*sin(q1))./2.0,t3.*t4.*(-1.0./2.0),t4.*t5.*(-1.0./2.0)],[2,2]);
