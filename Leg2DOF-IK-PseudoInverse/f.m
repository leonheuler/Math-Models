function F = f(in1)
%F
%    F = F(IN1)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    03-Aug-2019 20:47:49

q1 = in1(1,:);
q2 = in1(2,:);
t2 = q1-q2;
F = [sin(q1)./2.0+sin(t2)./2.0;cos(q1).*(-1.0./2.0)-cos(t2)./2.0];
