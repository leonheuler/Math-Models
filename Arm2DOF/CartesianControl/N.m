function N = N(in1,in2)
%N
%    N = N(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    03-Oct-2019 23:46:39

q1 = in1(1,:);
q2 = in1(2,:);
q_dot1 = in2(1,:);
q_dot2 = in2(2,:);
t2 = q1+q2;
t3 = q_dot1.^2;
t4 = q1.*2.0;
t5 = q2+t4;
t6 = sin(t5);
t7 = q2.*2.0;
t8 = q2.*3.0;
t9 = q_dot2.^2;
t10 = cos(t7);
t11 = q2.*4.0;
t12 = t4+t7;
t13 = cos(t12);
t14 = t4+t8;
t15 = sin(t12);
t16 = sin(t14);
t17 = cos(t11);
t18 = sin(q2);
t19 = 1.0./t18.^4;
t20 = cos(t2);
t21 = q1-t7;
t22 = sin(t21);
t23 = q1+t7;
t24 = sin(t23);
t25 = q1+t11;
t26 = sin(t25);
t27 = sin(q1);
t28 = cos(q2);
t29 = cos(t5);
t30 = cos(t8);
t31 = cos(t14);
t32 = sin(t2);
t33 = q1-q2;
t34 = sin(t33);
t35 = t34.*1.7658e4;
t36 = q1-t8;
t37 = sin(t36);
t38 = q1+t8;
t39 = sin(t38);
t40 = t39.*5.886e3;
t41 = t26.*2.943e3;
t42 = t27.*8.829e3;
t43 = t3.*t6.*2.4e3;
t44 = t6.*t9.*1.2e3;
t45 = t3.*t10.*2.4e3;
t46 = t3.*t30.*1.2e3;
t47 = t9.*t10.*1.8e3;
t48 = t9.*t17.*1.5e2;
t49 = t3.*t15.*4.8e3;
t50 = t3.*t16.*2.4e3;
t51 = t9.*t15.*2.4e3;
t52 = t9.*t16.*1.2e3;
t53 = q_dot1.*q_dot2.*t6.*2.4e3;
t54 = q_dot1.*q_dot2.*t10.*3.6e3;
t55 = q_dot1.*q_dot2.*t17.*3.0e2;
t56 = q_dot1.*q_dot2.*t15.*4.8e3;
t57 = q_dot1.*q_dot2.*t16.*2.4e3;
t58 = t3.*-4.8e3-t9.*4.35e3-t22.*2.943e3-t24.*8.829e3-t32.*1.7658e4+t35-t37.*5.886e3+t40+t41+t42+t43+t44+t45+t46+t47+t48+t49+t50+t51+t52+t53+t54+t55+t56+t57-q_dot1.*q_dot2.*8.7e3-t3.*t13.*2.4e3-t9.*t13.*2.4e3-t3.*t28.*3.6e3-t3.*t29.*1.2e3-t3.*t31.*1.2e3-q_dot1.*q_dot2.*t13.*4.8e3;
t59 = t22.*3.67875e2;
t60 = t24.*1.103625e3;
t61 = t3.*t29.*3.0e2;
t62 = t9.*t29.*1.5e2;
t63 = t3.*t6.*1.5e2;
t64 = t3.*t10.*5.25e2;
t65 = t3.*t30.*3.0e2;
t66 = t9.*t10.*3.0e2;
t67 = t3.*t17.*(7.5e1./4.0);
t68 = t9.*t30.*1.5e2;
t69 = t3.*t13.*6.0e2;
t70 = t3.*t31.*3.0e2;
t71 = t9.*t13.*3.0e2;
t72 = t9.*t31.*1.5e2;
t73 = t3.*t15.*3.0e2;
t74 = t3.*t16.*1.5e2;
t75 = t9.*t15.*3.0e2;
t76 = q_dot1.*q_dot2.*t29.*3.0e2;
t77 = q_dot1.*q_dot2.*t10.*6.0e2;
t78 = q_dot1.*q_dot2.*t30.*3.0e2;
t79 = q_dot1.*q_dot2.*t13.*6.0e2;
t80 = q_dot1.*q_dot2.*t31.*3.0e2;
t81 = q_dot1.*q_dot2.*t15.*6.0e2;
t82 = t3.*(-1.14375e3)-t9.*6.0e2-t26.*3.67875e2-t27.*1.103625e3+t59+t60+t61+t62+t63+t64+t65+t66+t67+t68+t69+t70+t71+t72+t73+t74+t75+t76+t77+t78+t79+t80+t81-q_dot1.*q_dot2.*1.2e3-t3.*t28.*9.0e2-t9.*t28.*4.5e2-q_dot1.*q_dot2.*t28.*9.0e2;
N = [t19.*t20.*t58.*(-1.0./8.0e2)-(t19.*t82.*(t20+cos(q1)))./1.0e2;t19.*t82.*(t27+t32).*(-1.0./1.0e2)-(t19.*t32.*t58)./8.0e2];
