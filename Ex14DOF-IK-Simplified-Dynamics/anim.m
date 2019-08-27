function Anim(q, mode, color, phase)
	%%

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

    %%
%     fig1 = figure(1);
    % clf('reset');

    axis equal
    
    h = animatedline();
    h.Marker = 'o';
    h.LineWidth = 2;
    h.Color = color;
    
    %%

    H = 1.70;
    L_hip = H*0.2450;
    L_shin = H*0.2460;
    L_toe = 0.0577*H;
    L_pelvis = 0.2;
    
    rG = [ 0 0 0 1]';
    %% legs
    q1 = q(1);
    q2 = q(2);
    q3 = q(3);
    q4 = q(4);
    q5 = q(5);
    q6 = q(6);
    q7 = q(7);
    q8 = q(8);
    q9 = q(9);
    
    %% FK
    M0 = T(q(10),q(11),q(12));    
    M1 = M0*Rx(q1)*T(0,0,L_shin);
    M2 = M1*Rx(q2)*T(0,0,L_hip);
    M3 = M2*Ry(q3)*T(0,0,L_hip/10);
    M4 = M3*Rz(q4)*T(0,0,L_hip/20);
    M5 = M4*Rx(q5)*T(0,-L_pelvis/4,0);
    switch phase
        case 'rs'
            M6 = M5*T(-L_pelvis, 0, 0);
        case 'ls'
            M6 = M5*T(L_pelvis, 0, 0);
    end
    M7 = M6*T(0,L_pelvis/4,0);
    M8 = M7*Rx(q6)*T(0,0,-L_hip/20);
    M9 = M8*Rz(q7)*T(0,0,-L_hip/10);
    M10 = M9*Ry(q8)*T(0,0,-L_hip);
    M11 = M10*Rx(q9)*T(0,0,-L_shin);
    M12 = M11*T(0,L_toe,0);
    
    
    rKR = M1(:,4); 
    rHR1 = M2(:,4);
    rHR2 = M3(:,4);
    rHR3 = M4(:,4);
    rHR4 = M5(:,4);
    rHL1 = M6(:,4);
    rHL2 = M7(:,4);
    rHL3 = M8(:,4);
    rHL4 = M9(:,4);
    rKL = M10(:,4);
    rAL = M11(:,4);
    rTL = M12(:,4); 
    
    addpoints(h, rG(1), rG(2)+L_toe, rG(3));
    addpoints(h, rG(1), rG(2), rG(3));
    addpoints(h, rKR(1), rKR(2), rKR(3));
    addpoints(h, rHR1(1), rHR1(2), rHR1(3));
    addpoints(h, rHR2(1), rHR2(2), rHR2(3));
    addpoints(h, rHR3(1), rHR3(2), rHR3(3));
    addpoints(h, rHR4(1), rHR4(2), rHR4(3));
    addpoints(h, rHL1(1), rHL1(2), rHL1(3));   
    addpoints(h, rHL2(1), rHL2(2), rHL2(3)); 
    addpoints(h, rHL3(1), rHL3(2), rHL3(3)); 
    addpoints(h, rHL4(1), rHL4(2), rHL4(3)); 
    addpoints(h, rKL(1), rKL(2), rKL(3)); 
    addpoints(h, rAL(1), rAL(2), rAL(3));     
    addpoints(h, rTL(1), rTL(2), rTL(3));     
    
    addpoints(h, rAL(1), rAL(2), rAL(3));    
    addpoints(h, rKL(1), rKL(2), rKL(3));    
    addpoints(h, rHL4(1), rHL4(2), rHL4(3)); 
    addpoints(h, rHL3(1), rHL3(2), rHL3(3));     
    addpoints(h, rHL2(1), rHL2(2), rHL2(3));     
    addpoints(h, rHL1(1), rHL1(2), rHL1(3));
    addpoints(h, rHR4(1), rHR4(2), rHR4(3));
    addpoints(h, rHR3(1), rHR3(2), rHR3(3));
    addpoints(h, rHR2(1), rHR2(2), rHR2(3));
    addpoints(h, rHR1(1), rHR1(2), rHR1(3));
    addpoints(h, rKR(1), rKR(2), rKR(3));
    addpoints(h, rG(1), rG(2), rG(3));
  
    
    drawnow

    if (mode == 0)
        clearpoints(h);

    end

end