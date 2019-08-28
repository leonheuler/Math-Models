function Anim(q, mode)
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
    fig1 = figure(1);
    % clf('reset');
    fig1.Position = [ 700 250 600 600 ];
    axis([-2 2 -2 2 -2 2]);
    axis equal
    fig1.CurrentAxes.XDir = 'Reverse';
    fig1.CurrentAxes.YDir = 'Reverse';
    grid on
    h = animatedline();
    h.Marker = 'o';
    h.LineWidth = 2;

    %%
    L_shin = 0.4;
    L_hip = 0.4;
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
    T_shinR = Rx(q1)*T(0,0,L_shin);
    T_hipR1 = T_shinR*Rx(q2)*T(0,0,L_hip);
    T_hipR2 = T_hipR1*Ry(q3)*T(0,0,L_hip/10);
    T_hipR3 = T_hipR2*Rz(q4);
    T_hipR4 = T_hipR3*Rx(q5)*T(0,-L_pelvis/4,0);
    T_hipL1 = T_hipR4*T(-L_pelvis, 0, 0);
    T_hipL2 = T_hipL1*T(0,L_pelvis/4,0);
    T_hipL3 = T_hipL2*Rx(q6)*T(0,0,-L_hip/10);
    T_hipL4 = T_hipL3*Rz(q7);
    T_shinL = T_hipL4*Ry(q8)*T(0,0,-L_hip);
    T_ankleL = T_shinL*Rx(q9)*T(0,0,-L_shin);
    T_toeL = T_ankleL*T(0,0.1,0);    
    
    rKR = T_shinR(:,4); 
    rHR1 = T_hipR1(:,4);
    rHR2 = T_hipR2(:,4);
    rHR3 = T_hipR4(:,4);
    rHL1 = T_hipL1(:,4);
    rHL2 = T_hipL2(:,4);
    rHL3 = T_hipL3(:,4);
    rKL = T_shinL(:,4);
    rAL = T_ankleL(:,4);
    rTL = T_toeL(:,4);   
    
    addpoints(h, rG(1), rG(2), rG(3));
    addpoints(h, rG(1), rG(2)+0.1, rG(3));
    addpoints(h, rG(1), rG(2), rG(3));
    addpoints(h, rKR(1), rKR(2), rKR(3));
    addpoints(h, rHR1(1), rHR1(2), rHR1(3));
    addpoints(h, rHR2(1), rHR2(2), rHR2(3));
    addpoints(h, rHR3(1), rHR3(2), rHR3(3));
    addpoints(h, rHL1(1), rHL1(2), rHL1(3));   
    addpoints(h, rHL2(1), rHL2(2), rHL2(3)); 
    addpoints(h, rHL3(1), rHL3(2), rHL3(3)); 
    addpoints(h, rKL(1), rKL(2), rKL(3)); 
    addpoints(h, rAL(1), rAL(2), rAL(3));     
    addpoints(h, rTL(1), rTL(2), rTL(3));     
    
    addpoints(h, rAL(1), rAL(2), rAL(3));    
    addpoints(h, rKL(1), rKL(2), rKL(3));     
    addpoints(h, rHL3(1), rHL3(2), rHL3(3));     
    addpoints(h, rHL2(1), rHL2(2), rHL2(3));     
    addpoints(h, rHL1(1), rHL1(2), rHL1(3)); 
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