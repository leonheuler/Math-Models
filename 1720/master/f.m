function F = f(in1)
%F
%    F = F(IN1)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    28-Dec-2018 20:24:56

q1 = in1(1,:);
q2 = in1(2,:);
q3 = in1(3,:);
q4 = in1(4,:);
q5 = in1(5,:);
q6 = in1(6,:);
q7 = in1(7,:);
q8 = in1(8,:);
q9 = in1(9,:);
q10 = in1(10,:);
q11 = in1(11,:);
t2 = sign(q10);
t3 = sin(q1);
t4 = sin(q2);
t5 = cos(q3);
t6 = cos(q1);
t7 = cos(q5);
t8 = cos(q4);
t9 = cos(q2);
t10 = cos(q6);
t11 = cos(q10);
t12 = t2.*2.215e-1;
t13 = cos(q11);
t14 = sin(q3);
t15 = sin(q5);
t16 = sin(q10);
t17 = sin(q11);
t18 = t2.*t16.*2.215e-1;
t19 = t11.*2.215e-1;
t20 = sin(q4);
t21 = sin(q6);
F = [q7+t12-t3.*t5.*9.254237288135593e-2-t3.*t7.*2.817796610169492e-2-t4.*t8.*9.254237288135593e-2-t2.*t11.*2.215e-1-t4.*t10.*2.817796610169492e-2;q8-t13.*(2.7e1./1.0e2)+t14.*9.254237288135593e-2+t15.*2.817796610169492e-2+t20.*9.254237288135593e-2+t21.*2.817796610169492e-2+2.7e1./5.9e1;q9-t17.*(2.7e1./1.0e2)+t18-t5.*t6.*9.254237288135593e-2-t6.*t7.*2.817796610169492e-2-t8.*t9.*9.254237288135593e-2-t9.*t10.*2.817796610169492e-2;q7+t12+t19-t3.*t5.*(2.1e1./5.0e1)-t3.*t7.*(1.9e1./4.0e1)-t2.*t11.*2.215e-1;q8-t13.*(2.7e1./1.0e2)+t14.*(2.1e1./5.0e1)+t15.*(1.9e1./4.0e1)+2.7e1./5.0e1;q9-t16.*2.215e-1-t17.*(2.7e1./1.0e2)+t18-t5.*t6.*(2.1e1./5.0e1)-t6.*t7.*(1.9e1./4.0e1);q7+t12-t19-t4.*t8.*(2.1e1./5.0e1)-t2.*t11.*2.215e-1-t4.*t10.*(1.9e1./4.0e1);q8-t13.*(2.7e1./1.0e2)+t20.*(2.1e1./5.0e1)+t21.*(1.9e1./4.0e1)+2.7e1./5.0e1;q9+t16.*2.215e-1-t17.*(2.7e1./1.0e2)+t18-t8.*t9.*(2.1e1./5.0e1)-t9.*t10.*(1.9e1./4.0e1)];
