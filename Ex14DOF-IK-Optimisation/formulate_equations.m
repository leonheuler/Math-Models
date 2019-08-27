function formulate_equations()
    tic
    q = sym('q', [14, 1]);
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
    L_pelvis = 0.6;
    
    
    M0 = T(q(9), q(10), q(11))*Rz(q(14))*Rx(q(12))*Ry(q(13));
    ML1 = M0*T(-L_pelvis/2, 0, 0);
    ML2 = ML1*T(0, L_pelvis/4, 0);
    ML3 = ML2*Rx(q(1));
    ML4 = ML3*Rz(q(2));
    ML5 = ML4*Ry(q(3))*T(0,0,-L_hip);
    ML6 = ML5*Rx(q(4))*T(0,0,-L_shin);
    ML7 = ML6*T(0,L_toe,0);
    
    rB = M0(:,4);
    rLH1 = ML1(:,4);
    rLH2 = ML2(:,4);
    rLH3 = ML3(:,4);
    rLH4 = ML4(:,4);
    rLK = ML5(:,4);
    rLA = ML6(:,4);
    rLT = ML7(:,4);
 
    MR1 = M0*T(L_pelvis/2, 0, 0);
    MR2 = MR1*T(0, L_pelvis/4, 0);
    MR3 = MR2*Rx(q(5));
    MR4 = MR3*Rz(q(6));
    MR5 = MR4*Ry(q(7))*T(0,0,-L_hip);
    MR6 = MR5*Rx(q(8))*T(0,0,-L_shin);
    MR7 = MR6*T(0,L_toe,0);  
    
    rRH1 = MR1(:,4);
    rRH2 = MR2(:,4);
    rRH3 = MR3(:,4);
    rRH4 = MR4(:,4);
    rRK = MR5(:,4);
    rRA = MR6(:,4);
    rRT = MR7(:,4);
    
    
    
    F = [rLA(1) rLA(2) rLA(3) rRA(1) rRA(2) rRA(3)]';
    
    fprintf("simplifying F..\n");
    F = simplify(F);
    fprintf("F simplify completed\n");
    disp(F);
    
    J = jacobian(F, q);
    fprintf("simplifying J..\n");
    J = simplify(J);
    fprintf("J simplify completed\n");
    disp(J);
    
    matlabFunction(F,'File','f','Vars', {q});
    matlabFunction(J,'File','jac','Vars', {q});

    toc
end
