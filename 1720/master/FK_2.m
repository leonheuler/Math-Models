function FK_2()

    clc;

    function ret = Rx(a)
        ret = [1,    0,        0;
             0,   cos(a), -sin(a);
             0,   sin(a),  cos(a)];
    end

    function ret = Ry(a)
        ret = [cos(a),    0,        sin(a);
             0,         1,        0;
             -sin(a),   0,        cos(a)];
    end

    function ret = Rz(a)
        ret = [cos(a),    -sin(a),        0;
               sin(a),     cos(a),        0;
                    0,          0,        1];
    end    
 
    function ret = Rotm(v,q)
        
        ret = [cos(q)+(1-cos(q))*(v(1)^2), (1-cos(q))*v(1)*v(2)-sin(q)*v(3), (1-cos(q))*v(1)*v(3)+sin(q)*v(2);
                (1-cos(q))*v(2)*v(1)+sin(q)*v(3), cos(q)+(1-cos(q))*(v(2)^2), (1-cos(q))*v(2)*v(3)-sin(q)*v(1);
                (1-cos(q))*v(3)*v(1)-sin(q)*v(2),   (1-cos(q))*v(3)*v(2)+sin(q)*v(1), cos(q)+(1-cos(q))*(v(3)^2)];
    end



    for i = -pi:pi/12:pi
    
    L0 = 0.4;
    L1 = 0.2215;  L2 = L1;
    L3 = 0.270;  L4 = L3;
    L5 = 0.420;   L6 = L5;
    L7 = 0.475;  L8 = L7;
    L9 = 0.227;   L10 = L9;
    L11 = 0.160;   L12 = L11;    
        
        % Left 
    q1 = 0;        % bedro vo frontalnoy
    q3 = 0;        % bedro v sagital'noi
    q5 = 0;        % koleno 
    q7 = 0;        % pod'em stupni
    q9 = 0;        % naklon stupni
    
        % Right
    q2 = 0;        % bedro vo frontalnoy
    q4 = 0;        % bedro v sagital'noi
    q6 = 0;        % koleno 
    q8 = 0;        % pod'em stupni
    q10 = 0;       % naklon stupni
    
    q11 = 0;        % Rz
    q12 = pi/4;        % Ry
    q13 = 0;        % Rx
    
    q14 = 0.2; q15 = 0.2; q16 = 0.2;    % x y z 
    
    T_0 = Rx(q13)*Ry(q12)*Rz(q11);
    
    r0 = -L0*T_0(:,3) + [q14 q15 q16]';
    
    r1 = L1*T_0(:,1) + r0;
    r2 = -L2*T_0(:,1) + r0;
    
    r3 = L3*T_0(:,2) + r1;
    r4 = L4*T_0(:,2) + r2;
    

    T_3_5 = Rotm( T_0(:,2), q1 )*Rotm( T_0(:,1), q3 ) * T_0;
    T_4_6 = Rotm( T_0(:,2), q2 )*Rotm( T_0(:,1), q4 ) * T_0;
    r5 =  -L5*T_3_5(:,3) + r3;
    r6 =  -L6*T_4_6(:,3) + r4;      
    
    T_5_7 = Rotm( T_3_5(:,1) , q5 )*T_3_5;
    T_6_8 = Rotm( T_4_6(:,1) , q6 )*T_4_6;
    r7 = -L7*T_5_7(:,3) + r5;
    r8 = -L8*T_6_8(:,3) + r6;
    
    T_7_9 = Rotm( T_5_7(:,1) , q7 ) * T_5_7;
    T_8_10 = Rotm( T_6_8(:,1) , q8 ) * T_6_8;
    r9 = L9*T_7_9(:,2) + r7;
    r10 = L10*T_8_10(:,2) + r8;
    
    T_7_11 = Rotm( T_7_9(:,2) , q9 ) * T_7_9;
    T_8_12 = Rotm( T_8_10(:,2) , q10 ) * T_8_10;
    r11 = -L11*T_7_11(:,1) + r7;
    r12 = L12*T_8_12(:,1) + r8;
    
    r13 = r7 + ( (r9-r7) + (r11-r7) ); 
    r14 = r8 + ( (r10-r8) + (r12-r8) ); 

    disp('    L0        L1        L3        L5        L7');
    norms = [norm(r0 - [q14 q15 q16]'), norm(r1-r0), norm(r3-r1), norm(r5-r3), norm(r7-r5)];
    disp(norms);
    
    disp('    L0        L2        L4        L6        L8');
    norms = [norm(r0 - [q14 q15 q16]'), norm(r2-r0), norm(r4-r2), norm(r6-r4), norm(r8-r6)];
    disp(norms);
    

    l = animatedline('LineWidth',1); 
    axis([-1, 1, -1, 1, -1.25, 1]);
%     axis equal;
    grid on;
    xlabel('X'); ylabel('Y'); zlabel('Z');
        
    addpoints(l, q14, q15, q16);
        
       
    addpoints(l, r0(1), r0(2), r0(3));
    addpoints(l, r1(1), r1(2), r1(3));
    addpoints(l, r3(1), r3(2), r3(3));
    addpoints(l, r5(1), r5(2), r5(3));    
    addpoints(l, r7(1), r7(2), r7(3));
    addpoints(l, r9(1), r9(2), r9(3));
    addpoints(l, r13(1), r13(2), r13(3));
    addpoints(l, r11(1), r11(2), r11(3));
    addpoints(l, r7(1), r7(2), r7(3));

    r = animatedline('LineWidth',1);  
    addpoints(r, r0(1), r0(2), r0(3));
    addpoints(r, r2(1), r2(2), r2(3));
    addpoints(r, r4(1), r4(2), r4(3));
    addpoints(r, r6(1), r6(2), r6(3));
    addpoints(r, r8(1), r8(2), r8(3));    
    addpoints(r, r10(1), r10(2), r10(3));   
    addpoints(r, r14(1), r14(2), r14(3)); 
    addpoints(r, r12(1), r12(2), r12(3));   
    addpoints(r, r8(1), r8(2), r8(3));
    drawnow;
    end


end