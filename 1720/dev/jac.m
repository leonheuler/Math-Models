function J = jac(in1)
%JAC
%    J = JAC(IN1)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    23-Jan-2019 01:45:30

q1 = in1(1,:);
q2 = in1(2,:);
q3 = in1(3,:);
q4 = in1(4,:);
q5 = in1(5,:);
q6 = in1(6,:);
q7 = in1(7,:);
t2 = q1+q3+q7;
t3 = cos(t2);
t4 = q1-q3-q5+q7;
t5 = cos(t4);
t6 = q1-q3+q7;
t7 = cos(t6);
t8 = q1+q3+q5+q7;
t9 = cos(t8);
t10 = q2+q4+q7;
t11 = cos(t10);
t12 = q2-q4-q6+q7;
t13 = cos(t12);
t14 = q2-q4+q7;
t15 = cos(t14);
t16 = q2+q4+q6+q7;
t17 = cos(t16);
t18 = t5.*1.408898305084746e-2;
t19 = t13.*1.408898305084746e-2;
t20 = t7.*4.627118644067797e-2;
t21 = t15.*4.627118644067797e-2;
t22 = q3+q5;
t23 = cos(t22);
t24 = t23.*2.817796610169492e-2;
t25 = q4+q6;
t26 = cos(t25);
t27 = t26.*2.817796610169492e-2;
t28 = sin(t2);
t29 = t28.*4.627118644067797e-2;
t30 = sin(t4);
t31 = t30.*1.408898305084746e-2;
t32 = sin(t6);
t33 = t32.*4.627118644067797e-2;
t34 = sin(t8);
t35 = t34.*1.408898305084746e-2;
t36 = sin(t10);
t37 = t36.*4.627118644067797e-2;
t38 = sin(t12);
t39 = t38.*1.408898305084746e-2;
t40 = sin(t14);
t41 = t40.*4.627118644067797e-2;
t42 = sin(t16);
t43 = t42.*1.408898305084746e-2;
t44 = cos(q4);
t45 = q2+q7;
t46 = sin(t45);
t47 = sin(t25);
t48 = sin(q2);
t49 = t26.*(1.9e1./4.0e1);
t50 = t44.*(2.1e1./5.0e1);
t51 = t49+t50;
t52 = t26.*9.5e1;
t53 = t44.*8.4e1;
t54 = t52+t53;
t55 = cos(t45);
t56 = t47.*9.5e1;
t57 = sin(q4);
t58 = t57.*8.4e1;
t59 = t56+t58;
t60 = cos(q7);
t61 = t44.*t48.*(2.1e1./5.0e1);
t62 = t26.*t48.*(1.9e1./4.0e1);
t63 = t61+t62-2.215e-1;
t64 = cos(q2);
t65 = sin(q7);
t66 = cos(q3);
t67 = q1+q7;
t68 = sin(t67);
t69 = sin(t22);
t70 = sin(q1);
t71 = t23.*(1.9e1./4.0e1);
t72 = t66.*(2.1e1./5.0e1);
t73 = t71+t72;
t74 = t23.*9.5e1;
t75 = t66.*8.4e1;
t76 = t74+t75;
t77 = cos(t67);
t78 = t69.*9.5e1;
t79 = sin(q3);
t80 = t79.*8.4e1;
t81 = t78+t80;
t82 = t66.*t70.*(2.1e1./5.0e1);
t83 = t23.*t70.*(1.9e1./4.0e1);
t84 = t82+t83+2.215e-1;
t85 = cos(q1);
J = reshape([t3.*(-4.627118644067797e-2)-t5.*1.408898305084746e-2-t7.*4.627118644067797e-2-t9.*1.408898305084746e-2,0.0,t29+t31+t33+t35,0.0,0.0,0.0,t76.*t77.*(-1.0./2.0e2),0.0,(t68.*t76)./2.0e2,t11.*(-4.627118644067797e-2)-t13.*1.408898305084746e-2-t15.*4.627118644067797e-2-t17.*1.408898305084746e-2,0.0,t37+t39+t41+t43,t54.*t55.*(-1.0./2.0e2),0.0,(t46.*t54)./2.0e2,0.0,0.0,0.0,t3.*(-4.627118644067797e-2)-t9.*1.408898305084746e-2+t18+t20,t24+t66.*9.254237288135593e-2,t29-t31-t33+t35,0.0,0.0,0.0,(t68.*t81)./2.0e2,t73,(t77.*t81)./2.0e2,t11.*(-4.627118644067797e-2)-t17.*1.408898305084746e-2+t19+t21,t27+t44.*9.254237288135593e-2,t37-t39-t41+t43,(t46.*t59)./2.0e2,t51,(t55.*t59)./2.0e2,0.0,0.0,0.0,t9.*(-1.408898305084746e-2)+t18,t24,-t31+t35,0.0,0.0,0.0,t68.*t69.*(1.9e1./4.0e1),t71,t69.*t77.*(1.9e1./4.0e1),t17.*(-1.408898305084746e-2)+t19,t27,-t39+t43,t46.*t47.*(1.9e1./4.0e1),t49,t47.*t55.*(1.9e1./4.0e1),0.0,0.0,0.0,t3.*(-4.627118644067797e-2)-t9.*1.408898305084746e-2-t11.*4.627118644067797e-2-t17.*1.408898305084746e-2-t18-t19-t20-t21,0.0,t29+t31+t33+t35+t37+t39+t41+t43,t63.*t65-t51.*t60.*t64,0.0,t60.*t63+t51.*t64.*t65,t65.*t84-t60.*t73.*t85,0.0,t60.*t84+t65.*t73.*t85,1.0,0.0,0.0,1.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0,0.0,0.0,1.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0,0.0,0.0,1.0,0.0,0.0,1.0],[9,10]);
