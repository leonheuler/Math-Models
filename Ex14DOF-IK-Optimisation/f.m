function F = f(in1)
%F
%    F = F(IN1)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    11-Aug-2019 15:22:03

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
q12 = in1(12,:);
q13 = in1(13,:);
q14 = in1(14,:);
t2 = cos(q14);
t3 = cos(q13);
t4 = sin(q14);
t5 = cos(q12);
t6 = cos(q3);
t7 = cos(q1);
t8 = sin(q13);
t9 = t2.*t8;
t10 = sin(q12);
t11 = t3.*t4.*t10;
t12 = t9+t11;
t13 = t7.*t12;
t14 = sin(q1);
t15 = t4.*t5.*t14;
t16 = t13+t15;
t17 = sin(q3);
t18 = sin(q2);
t19 = t12.*t14;
t27 = t4.*t5.*t7;
t20 = t19-t27;
t21 = t18.*t20;
t22 = cos(q2);
t23 = t2.*t3;
t28 = t4.*t8.*t10;
t24 = t23-t28;
t25 = t22.*t24;
t26 = t21+t25;
t29 = cos(q4);
t30 = t4.*t8;
t34 = t2.*t3.*t10;
t31 = t30-t34;
t32 = t7.*t31;
t33 = t32-t2.*t5.*t14;
t35 = t14.*t31;
t36 = t2.*t5.*t7;
t37 = t35+t36;
t38 = t18.*t37;
t39 = t3.*t4;
t40 = t2.*t8.*t10;
t41 = t39+t40;
t42 = t22.*t41;
t43 = t38+t42;
t44 = sin(q4);
t45 = t7.*t10;
t46 = t3.*t5.*t14;
t47 = t45+t46;
t48 = t18.*t47;
t49 = t48-t5.*t8.*t22;
t50 = t10.*t14;
t51 = t50-t3.*t5.*t7;
t52 = cos(q7);
t53 = cos(q5);
t54 = t12.*t53;
t55 = sin(q5);
t56 = t4.*t5.*t55;
t57 = t54+t56;
t58 = sin(q7);
t59 = sin(q6);
t60 = t12.*t55;
t66 = t4.*t5.*t53;
t61 = t60-t66;
t62 = t59.*t61;
t63 = cos(q6);
t64 = t24.*t63;
t65 = t62+t64;
t67 = t4.*t8.*t10.*(3.0./1.0e1);
t68 = t2.*t5.*(3.0./2.0e1);
t69 = cos(q8);
t70 = t31.*t53;
t71 = t70-t2.*t5.*t55;
t72 = t31.*t55;
t73 = t2.*t5.*t53;
t74 = t72+t73;
t75 = t59.*t74;
t76 = t41.*t63;
t77 = t75+t76;
t78 = sin(q8);
t79 = t10.*(3.0./2.0e1);
t80 = t5.*t8.*(3.0./1.0e1);
t81 = t10.*t53;
t82 = t3.*t5.*t55;
t83 = t81+t82;
t84 = t59.*t83;
t85 = t84-t5.*t8.*t63;
t86 = t10.*t55;
t87 = t86-t3.*t5.*t53;
F = [q9+t67-t2.*t3.*(3.0./1.0e1)-t4.*t5.*(3.0./2.0e1)-t6.*t16.*4.165e-1-t17.*t26.*4.165e-1-t29.*(t6.*t16+t17.*t26).*4.182e-1-t44.*(t18.*t24-t20.*t22).*4.182e-1;q10+t68-t3.*t4.*(3.0./1.0e1)-t6.*t33.*4.165e-1-t17.*t43.*4.165e-1-t29.*(t6.*t33+t17.*t43).*4.182e-1-t44.*(t18.*t41-t22.*t37).*4.182e-1-t2.*t8.*t10.*(3.0./1.0e1);q11+t79+t80+t44.*(t22.*t47+t5.*t8.*t18).*4.182e-1+t6.*t51.*4.165e-1-t17.*t49.*4.165e-1+t29.*(t6.*t51-t17.*t49).*4.182e-1;q9-t67+t2.*t3.*(3.0./1.0e1)-t4.*t5.*(3.0./2.0e1)-t52.*t57.*4.165e-1-t58.*t65.*4.165e-1-t78.*(t24.*t59-t61.*t63).*4.182e-1-t69.*(t52.*t57+t58.*t65).*4.182e-1;q10+t68+t3.*t4.*(3.0./1.0e1)-t52.*t71.*4.165e-1-t58.*t77.*4.165e-1-t78.*(t41.*t59-t63.*t74).*4.182e-1-t69.*(t52.*t71+t58.*t77).*4.182e-1+t2.*t8.*t10.*(3.0./1.0e1);q11+t79-t80+t78.*(t63.*t83+t5.*t8.*t59).*4.182e-1+t52.*t87.*4.165e-1-t58.*t85.*4.165e-1+t69.*(t52.*t87-t58.*t85).*4.182e-1];