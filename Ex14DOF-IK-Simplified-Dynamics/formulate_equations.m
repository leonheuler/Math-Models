function formulate_equations()

    q = sym('q', [12, 1]);
    assume(q, 'real');

    function ret = Rx(a)
        ret = [1,    0,        0    0;
               0,   cos(a), -sin(a)   0;
               0,   sin(a),  cos(a)   0;
               0,   0,       0,       1];
    end

    function ret = Ry(a)
        ret = [cos(a),    0,        sin(a)   0;
               0,         1,        0        0;
              -sin(a),    0,        cos(a)   0;
                0,        0,        0,       1];
    end

    function ret = Rz(a)
        ret = [cos(a),    -sin(a),        0     0;
               sin(a),     cos(a),        0     0;
                    0,          0,        1     0;
                    0,          0,        0,    1];
    end

    function ret = T(a,b,c)
        ret = [1,    0,        0     a;
               0,    1,        0     b;
               0,    0,        1     c;
               0,    0,        0,    1];
    end
    
    H = 1.70;
    L_hip = H*0.2450;
    L_shin = H*0.2460;
    L_toe = 0.0577*H;
    L_pelvis = 0.2;
    
    M0 = T(q(10),q(11),q(12));
    M1 = M0*Rx(q(1))*T(0,0,L_shin);
    M2 = M1*Rx(q(2))*T(0,0,L_hip);
    M3 = M2*Ry(q(3))*T(0,0,L_hip/10);
    M4 = M3*Rz(q(4))*T(0,0,L_hip/20);
    M5 = M4*Rx(q(5))*T(0,-L_pelvis/4,0);
    M6 = M5*T(-L_pelvis, 0, 0);
    M7 = M6*T(0,L_pelvis/4,0);
    M8 = M7*Rx(q(6))*T(0,0,-L_hip/20);
    M9 = M8*Rz(q(7))*T(0,0,-L_hip/10);
    M10 = M9*Ry(q(8))*T(0,0,-L_hip);
    M11 = M10*Rx(q(9))*T(0,0,-L_shin);
    M12 = M11*T(0,L_toe,0);

    
    
    rAL = M11(:,4);
    

    F1 = [ rAL(1); rAL(2); rAL(3) ];
    
    fprintf("simplifying F1..\n");
    F1 = simplify(F1);
    fprintf("F1 simplify completed\n");

    J1 = jacobian(F1, q);
    fprintf("simplifying J1..\n");
    J1 = simplify(J1);
    fprintf("J1 simplify completed\n");
    
    
    matlabFunction(F1,'File','f1','Vars', {q});
    matlabFunction(J1,'File','jac1','Vars', {q});

    
    M0 = T(q(10),q(11),q(12));
    M1 = M0*Rx(q(1))*T(0,0,L_shin);
    M2 = M1*Rx(q(2))*T(0,0,L_hip);
    M3 = M2*Ry(q(3))*T(0,0,L_hip/10);
    M4 = M3*Rz(q(4))*T(0,0,L_hip/20);
    M5 = M4*Rx(q(5))*T(0,-L_pelvis/4,0);
    M6 = M5*T(L_pelvis, 0, 0);
    M7 = M6*T(0,L_pelvis/4,0);
    M8 = M7*Rx(q(6))*T(0,0,-L_hip/20);
    M9 = M8*Rz(q(7))*T(0,0,-L_hip/10);
    M10 = M9*Ry(q(8))*T(0,0,-L_hip);
    M11 = M10*Rx(q(9))*T(0,0,-L_shin);
    M12 = M11*T(0,L_toe,0);
    
    
    rAL = M11(:,4);

    F2 = [ rAL(1); rAL(2); rAL(3) ];
    
    fprintf("simplifying F2..\n");
    F2 = simplify(F2);
    fprintf("F2 simplify completed\n");

    J2 = jacobian(F2, q);
    fprintf("simplifying J2..\n");
    J2 = simplify(J2);
    fprintf("J2 simplify completed\n");
    
    
    matlabFunction(F2,'File','f2','Vars', {q});
    matlabFunction(J2,'File','jac2','Vars', {q});
    

end
