function F = f(in1)
%F
%    F = F(IN1)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    23-Jan-2019 01:45:30

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
t2 = q1+q3+q7;
t3 = q2+q4+q7;
t4 = q1-q3-q5+q7;
t5 = q2-q4-q6+q7;
t6 = q1-q3+q7;
t7 = q2-q4+q7;
t8 = q1+q3+q5+q7;
t9 = q2+q4+q6+q7;
t10 = q4+q6;
t11 = sin(q2);
t12 = cos(t10);
t13 = cos(q4);
t14 = sin(t10);
t15 = sin(q4);
t16 = sin(q7);
t17 = t11.*t13.*(2.1e1./5.0e1);
t18 = t11.*t12.*(1.9e1./4.0e1);
t19 = t17+t18-2.215e-1;
t20 = cos(q2);
t21 = cos(q7);
t22 = t12.*(1.9e1./4.0e1);
t23 = t13.*(2.1e1./5.0e1);
t24 = t22+t23;
t25 = q3+q5;
t26 = sin(q1);
t27 = cos(t25);
t28 = cos(q3);
t29 = sin(t25);
t30 = sin(q3);
t31 = t26.*t28.*(2.1e1./5.0e1);
t32 = t26.*t27.*(1.9e1./4.0e1);
t33 = t31+t32+2.215e-1;
t34 = cos(q1);
t35 = t27.*(1.9e1./4.0e1);
t36 = t28.*(2.1e1./5.0e1);
t37 = t35+t36;
F = [q8-sin(t2).*4.627118644067797e-2-sin(t3).*4.627118644067797e-2-sin(t4).*1.408898305084746e-2-sin(t5).*1.408898305084746e-2-sin(t6).*4.627118644067797e-2-sin(t7).*4.627118644067797e-2-sin(t8).*1.408898305084746e-2-sin(t9).*1.408898305084746e-2;q9+t14.*2.817796610169492e-2+t15.*9.254237288135593e-2+t29.*2.817796610169492e-2+t30.*9.254237288135593e-2+1.876271186440678e-1;q10-cos(t2).*4.627118644067797e-2-cos(t3).*4.627118644067797e-2-cos(t4).*1.408898305084746e-2-cos(t5).*1.408898305084746e-2-cos(t6).*4.627118644067797e-2-cos(t7).*4.627118644067797e-2-cos(t8).*1.408898305084746e-2-cos(t9).*1.408898305084746e-2;q8-t19.*t21-t16.*t20.*t24;q9+t14.*(1.9e1./4.0e1)+t15.*(2.1e1./5.0e1)+2.7e1./1.0e2;q10+t16.*t19-t20.*t21.*t24;q8-t21.*t33-t16.*t34.*t37;q9+t29.*(1.9e1./4.0e1)+t30.*(2.1e1./5.0e1)+2.7e1./1.0e2;q10+t16.*t33-t21.*t34.*t37];
