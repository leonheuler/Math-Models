function J = jac(in1)
%JAC
%    J = JAC(IN1)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    11-Aug-2019 15:22:07

q1 = in1(1,:);
q2 = in1(2,:);
q3 = in1(3,:);
q4 = in1(4,:);
q5 = in1(5,:);
q6 = in1(6,:);
q7 = in1(7,:);
q8 = in1(8,:);
q12 = in1(12,:);
q13 = in1(13,:);
q14 = in1(14,:);
t2 = sin(q14);
t3 = cos(q3);
t4 = sin(q1);
t5 = cos(q14);
t6 = sin(q13);
t7 = t5.*t6;
t8 = cos(q13);
t9 = sin(q12);
t10 = t2.*t8.*t9;
t11 = t7+t10;
t12 = t4.*t11;
t13 = cos(q1);
t14 = cos(q12);
t22 = t2.*t13.*t14;
t15 = t12-t22;
t16 = t11.*t13;
t17 = t2.*t4.*t14;
t18 = t16+t17;
t19 = sin(q2);
t20 = sin(q3);
t21 = cos(q2);
t23 = sin(q4);
t24 = t5.*t8;
t28 = t2.*t6.*t9;
t25 = t24-t28;
t26 = cos(q4);
t27 = t15.*t21;
t33 = t19.*t25;
t29 = t27-t33;
t30 = t15.*t19;
t31 = t21.*t25;
t32 = t30+t31;
t34 = t9.*t13.*t19;
t35 = t4.*t8.*t14.*t19;
t64 = t6.*t14.*t21;
t36 = t34+t35-t64;
t37 = t4.*t9;
t65 = t8.*t13.*t14;
t38 = t37-t65;
t39 = t11.*t21;
t40 = t39-t4.*t19.*t25;
t41 = t2.*t6;
t45 = t5.*t8.*t9;
t42 = t41-t45;
t43 = t13.*t42;
t55 = t4.*t5.*t14;
t44 = t43-t55;
t46 = t4.*t42;
t47 = t5.*t13.*t14;
t48 = t46+t47;
t49 = t19.*t48;
t50 = t2.*t8;
t51 = t5.*t6.*t9;
t52 = t50+t51;
t53 = t21.*t52;
t54 = t49+t53;
t56 = t21.*t48;
t58 = t19.*t52;
t57 = t56-t58;
t59 = t26.*2.46e2;
t60 = t59+2.45e2;
t61 = t3.*t44;
t62 = t20.*t54;
t63 = t61+t62;
t66 = t9.*t13.*t21;
t67 = t6.*t14.*t19;
t68 = t4.*t8.*t14.*t21;
t69 = t66+t67+t68;
t70 = t21.*t42;
t71 = t70-t4.*t19.*t52;
t72 = t3.*t18;
t73 = t20.*t32;
t74 = t72+t73;
t75 = t9.*t13;
t76 = t4.*t8.*t14;
t77 = t75+t76;
t78 = t21.*t77;
t79 = t67+t78;
t80 = t64-t19.*t77;
t81 = t13.*t14;
t83 = t4.*t8.*t9;
t82 = t81-t83;
t84 = t19.*t82;
t85 = t6.*t9.*t21;
t86 = t84+t85;
t87 = t4.*t14;
t88 = t8.*t9.*t13;
t89 = t87+t88;
t90 = t8.*t14.*t21;
t91 = t4.*t6.*t14.*t19;
t92 = t90+t91;
t93 = cos(q7);
t94 = sin(q5);
t95 = t11.*t94;
t96 = cos(q5);
t104 = t2.*t14.*t96;
t97 = t95-t104;
t98 = t11.*t96;
t99 = t2.*t14.*t94;
t100 = t98+t99;
t101 = sin(q6);
t102 = sin(q7);
t103 = cos(q6);
t105 = sin(q8);
t106 = cos(q8);
t107 = t25.*t101;
t108 = t97.*t101;
t109 = t25.*t103;
t110 = t108+t109;
t153 = t97.*t103;
t111 = t107-t153;
t112 = t2.*t9.*(3.0./2.0e1);
t113 = t9.*t96.*t101;
t114 = t8.*t14.*t94.*t101;
t141 = t6.*t14.*t103;
t115 = t113+t114-t141;
t116 = t9.*t94;
t142 = t8.*t14.*t96;
t117 = t116-t142;
t118 = t2.*t6.*t14.*(3.0./1.0e1);
t119 = t5.*t6.*(3.0./1.0e1);
t120 = t11.*t103;
t121 = t120-t25.*t94.*t101;
t122 = t2.*t8.*t9.*(3.0./1.0e1);
t123 = t2.*t8.*(3.0./1.0e1);
t124 = t42.*t96;
t133 = t5.*t14.*t94;
t125 = t124-t133;
t126 = t42.*t94;
t127 = t5.*t14.*t96;
t128 = t126+t127;
t129 = t101.*t128;
t130 = t52.*t103;
t131 = t129+t130;
t132 = t5.*t6.*t9.*(3.0./1.0e1);
t134 = t52.*t101;
t135 = t106.*2.46e2;
t136 = t135+2.45e2;
t137 = t93.*t125;
t138 = t102.*t131;
t139 = t137+t138;
t140 = t134-t103.*t128;
t143 = t9.*t96.*t103;
t144 = t6.*t14.*t101;
t145 = t8.*t14.*t94.*t103;
t146 = t143+t144+t145;
t147 = t2.*t6.*(3.0./1.0e1);
t148 = t42.*t103;
t149 = t148-t52.*t94.*t101;
t150 = t93.*t100;
t151 = t102.*t110;
t152 = t150+t151;
t154 = t2.*t6.*t9.*(3.0./1.0e1);
t155 = t9.*t96;
t156 = t8.*t14.*t94;
t157 = t155+t156;
t158 = t103.*t157;
t159 = t144+t158;
t160 = t141-t101.*t157;
t161 = t14.*(3.0./2.0e1);
t162 = t14.*t96;
t164 = t8.*t9.*t94;
t163 = t162-t164;
t165 = t101.*t163;
t166 = t6.*t9.*t103;
t167 = t165+t166;
t168 = t14.*t94;
t169 = t8.*t9.*t96;
t170 = t168+t169;
t171 = t8.*t14.*(3.0./1.0e1);
t172 = t8.*t14.*t103;
t173 = t6.*t14.*t94.*t101;
t174 = t172+t173;
J = reshape([t26.*(t3.*t15-t18.*t19.*t20).*4.182e-1+t3.*t15.*4.165e-1-t18.*t19.*t20.*4.165e-1+t18.*t21.*t23.*4.182e-1,t26.*(t3.*t48-t19.*t20.*t44).*4.182e-1+t3.*t48.*4.165e-1-t19.*t20.*t44.*4.165e-1+t21.*t23.*t44.*4.182e-1,t26.*(t3.*t77+t19.*t20.*t38).*4.182e-1+t3.*t77.*4.165e-1+t19.*t20.*t38.*4.165e-1-t21.*t23.*t38.*4.182e-1,0.0,0.0,0.0,t20.*t29.*(-4.165e-1)-t23.*t32.*4.182e-1-t20.*t26.*t29.*4.182e-1,t20.*t57.*(-4.165e-1)-t23.*t54.*4.182e-1-t20.*t26.*t57.*4.182e-1,t20.*t79.*(-4.165e-1)+t23.*t80.*4.182e-1-t20.*t26.*t79.*4.182e-1,0.0,0.0,0.0,t60.*(-t3.*t5.*t8.*t21+t2.*t4.*t14.*t20+t5.*t6.*t13.*t20-t3.*t4.*t5.*t6.*t19+t2.*t3.*t6.*t9.*t21+t2.*t3.*t13.*t14.*t19+t2.*t8.*t9.*t13.*t20-t2.*t3.*t4.*t8.*t9.*t19).*1.7e-3,t60.*(t2.*t3.*t8.*t21-t2.*t6.*t13.*t20+t4.*t5.*t14.*t20+t2.*t3.*t4.*t6.*t19+t3.*t5.*t6.*t9.*t21+t3.*t5.*t13.*t14.*t19+t5.*t8.*t9.*t13.*t20-t3.*t4.*t5.*t8.*t9.*t19).*(-1.7e-3),t60.*(t4.*t9.*t20-t3.*t6.*t14.*t21+t3.*t9.*t13.*t19-t8.*t13.*t14.*t20+t3.*t4.*t8.*t14.*t19).*(-1.7e-3),0.0,0.0,0.0,t26.*t29.*4.182e-1+t23.*t74.*4.182e-1,t26.*t57.*4.182e-1+t23.*t63.*4.182e-1,t26.*t79.*4.182e-1-t23.*(t3.*t38+t20.*t80).*4.182e-1,0.0,0.0,0.0,0.0,0.0,0.0,t106.*(t93.*t97-t100.*t101.*t102).*4.182e-1+t93.*t97.*4.165e-1-t100.*t101.*t102.*4.165e-1+t100.*t103.*t105.*4.182e-1,t106.*(t93.*t128-t101.*t102.*t125).*4.182e-1+t93.*t128.*4.165e-1-t101.*t102.*t125.*4.165e-1+t103.*t105.*t125.*4.182e-1,t106.*(t93.*t157+t101.*t102.*t117).*4.182e-1+t93.*t157.*4.165e-1+t101.*t102.*t117.*4.165e-1-t103.*t105.*t117.*4.182e-1,0.0,0.0,0.0,t102.*t111.*4.165e-1-t105.*t110.*4.182e-1+t102.*t106.*t111.*4.182e-1,t105.*t131.*(-4.182e-1)+t102.*t140.*4.165e-1+t102.*t106.*t140.*4.182e-1,t102.*t159.*(-4.165e-1)+t105.*t160.*4.182e-1-t102.*t106.*t159.*4.182e-1,0.0,0.0,0.0,t136.*(t5.*t6.*t96.*t102-t5.*t8.*t93.*t103+t2.*t14.*t94.*t102+t2.*t6.*t9.*t93.*t103+t2.*t8.*t9.*t96.*t102-t5.*t6.*t93.*t94.*t101+t2.*t14.*t93.*t96.*t101-t2.*t8.*t9.*t93.*t94.*t101).*1.7e-3,t136.*(-t2.*t6.*t96.*t102+t2.*t8.*t93.*t103+t5.*t14.*t94.*t102+t5.*t6.*t9.*t93.*t103+t5.*t8.*t9.*t96.*t102+t2.*t6.*t93.*t94.*t101+t5.*t14.*t93.*t96.*t101-t5.*t8.*t9.*t93.*t94.*t101).*(-1.7e-3),t136.*(t9.*t94.*t102-t6.*t14.*t93.*t103-t8.*t14.*t96.*t102+t9.*t93.*t96.*t101+t8.*t14.*t93.*t94.*t101).*(-1.7e-3),0.0,0.0,0.0,t106.*t111.*(-4.182e-1)+t105.*t152.*4.182e-1,t105.*t139.*4.182e-1-t106.*t140.*4.182e-1,t106.*t159.*4.182e-1-t105.*(t93.*t117+t102.*t160).*4.182e-1,1.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0,0.0,0.0,1.0,t112+t118+t26.*(t2.*t3.*t38-t2.*t20.*t36).*4.182e-1+t2.*t3.*t38.*4.165e-1-t2.*t20.*t36.*4.165e-1+t2.*t23.*t69.*4.182e-1,t5.*t9.*(-3.0./2.0e1)-t26.*(t3.*t5.*t38-t5.*t20.*t36).*4.182e-1-t5.*t6.*t14.*(3.0./1.0e1)-t3.*t5.*t38.*4.165e-1+t5.*t20.*t36.*4.165e-1-t5.*t23.*t69.*4.182e-1,t161+t23.*(t21.*t82-t6.*t9.*t19).*4.182e-1-t6.*t9.*(3.0./1.0e1)+t3.*t89.*4.165e-1-t20.*t86.*4.165e-1+t26.*(t3.*t89-t20.*t86).*4.182e-1,t112-t118+t106.*(t2.*t93.*t117-t2.*t102.*t115).*4.182e-1+t2.*t93.*t117.*4.165e-1-t2.*t102.*t115.*4.165e-1+t2.*t105.*t146.*4.182e-1,t5.*t9.*(-3.0./2.0e1)-t106.*(t5.*t93.*t117-t5.*t102.*t115).*4.182e-1+t5.*t6.*t14.*(3.0./1.0e1)-t5.*t93.*t117.*4.165e-1+t5.*t102.*t115.*4.165e-1-t5.*t105.*t146.*4.182e-1,t161+t105.*(t103.*t163-t6.*t9.*t101).*4.182e-1+t6.*t9.*(3.0./1.0e1)+t93.*t170.*4.165e-1-t102.*t167.*4.165e-1+t106.*(t93.*t170-t102.*t167).*4.182e-1,t119+t122+t23.*(t11.*t19+t4.*t21.*t25).*4.182e-1+t26.*(t20.*t40-t3.*t13.*t25).*4.182e-1+t20.*t40.*4.165e-1-t3.*t13.*t25.*4.165e-1,t147+t23.*(t19.*t42+t4.*t21.*t52).*4.182e-1+t26.*(t20.*t71-t3.*t13.*t52).*4.182e-1+t20.*t71.*4.165e-1-t5.*t8.*t9.*(3.0./1.0e1)-t3.*t13.*t52.*4.165e-1,t171+t23.*(t8.*t14.*t19-t4.*t6.*t14.*t21).*4.182e-1+t20.*t92.*4.165e-1+t26.*(t20.*t92+t3.*t6.*t13.*t14).*4.182e-1+t3.*t6.*t13.*t14.*4.165e-1,-t119-t122+t105.*(t11.*t101+t25.*t94.*t103).*4.182e-1+t106.*(t102.*t121-t25.*t93.*t96).*4.182e-1+t102.*t121.*4.165e-1-t25.*t93.*t96.*4.165e-1,-t147+t105.*(t42.*t101+t52.*t94.*t103).*4.182e-1+t106.*(t102.*t149-t52.*t93.*t96).*4.182e-1+t102.*t149.*4.165e-1+t5.*t8.*t9.*(3.0./1.0e1)-t52.*t93.*t96.*4.165e-1,-t171+t105.*(t8.*t14.*t101-t6.*t14.*t94.*t103).*4.182e-1+t102.*t174.*4.165e-1+t106.*(t102.*t174+t6.*t14.*t93.*t96).*4.182e-1+t6.*t14.*t93.*t96.*4.165e-1,t123+t132-t5.*t14.*(3.0./2.0e1)+t3.*t44.*4.165e-1+t20.*t54.*4.165e-1-t23.*t57.*4.182e-1+t26.*t63.*4.182e-1,t154-t5.*t8.*(3.0./1.0e1)-t2.*t14.*(3.0./2.0e1)-t3.*t18.*4.165e-1-t20.*t32.*4.165e-1+t23.*t29.*4.182e-1-t26.*t74.*4.182e-1,0.0,-t123-t132-t5.*t14.*(3.0./2.0e1)+t93.*t125.*4.165e-1+t102.*t131.*4.165e-1+t105.*t140.*4.182e-1+t106.*t139.*4.182e-1,-t154+t5.*t8.*(3.0./1.0e1)-t2.*t14.*(3.0./2.0e1)-t93.*t100.*4.165e-1-t102.*t110.*4.165e-1-t105.*t111.*4.182e-1-t106.*t152.*4.182e-1,0.0],[6,14]);
