

fig1 = figure(1);
% clf('reset');
fig1.Position = [ 700 250 600 600 ];
axis([-2 2 -2 2 -2 2]);
axis equal
set(gca, 'YDir', 'reverse');
set(gca, 'XDir', 'reverse');

grid on
h = animatedline();
h.Marker = 'o';
h.LineWidth = 2;

rG = [ 0 0 0 1]';

H = 1.70;
L_hip = H*0.2450;
L_shin = H*0.2460;
L_toe = 0.0577*H;
L_pelvis = 0.2;

for t = -pi/8:0.1:pi/8

    q1 = 0; q2  = 0; q3 = 0; q4 = 0; 
    q5 = q1; q6 = q2; q7 = q3; q8 = q4; 
    
    q9 = 0; q10 = 0; q11 = 0; 
    q12 = pi/12; q13 = pi/4; q14 = pi/6;
    
%       Lhx Lhz Lhy Lkx Rhx Rhz Rhy Rkx   x  y   z   Rx  Ry  Rz                                         
    q = [q1 q2  q3  q4  q5  q6  q7  q8    q9 q10 q11 q12 q13 q14]';
    
    M0 = T(q(9), q(10), q(11))*Rz(q(14))*Rx(q(12))*Ry(q(13));
    ML1 = M0*T(-L_pelvis/2, 0, 0);
    ML2 = ML1*T(0, L_pelvis/4, 0);;
    ML3 = ML2*Rx(q(1));
    ML4 = ML3*Rz(q(2));
    ML5 = ML4*Ry(q(3))*T(0,0,-L_hip);
    ML6 = ML5*Rx(q(4))*T(0,0,-L_shin);
    ML7 = ML6*T(0,L_toe,0);
    
    rB = tform2trvec(M0); 
    rLH1 =  tform2trvec(ML1);
    rLH2 = tform2trvec(ML2);
    rLH3 = tform2trvec(ML3);
    rLH4 = tform2trvec(ML4);
    rLK = tform2trvec(ML5);
    rLA = tform2trvec(ML6);
    rLT = tform2trvec(ML7);
    

    
    addpoints(h, rB(1), rB(2), rB(3));
    addpoints(h, rLH1(1), rLH1(2), rLH1(3));
    addpoints(h, rLH2(1), rLH2(2), rLH2(3));
    addpoints(h, rLH3(1), rLH3(2), rLH3(3));
    addpoints(h, rLH4(1), rLH4(2), rLH4(3));
    addpoints(h, rLK(1), rLK(2), rLK(3));
    addpoints(h, rLA(1), rLA(2), rLA(3));
    addpoints(h, rLT(1), rLT(2), rLT(3));

    addpoints(h, rLA(1), rLA(2), rLA(3));
    addpoints(h, rLK(1), rLK(2), rLK(3));    
    addpoints(h, rLH4(1), rLH4(2), rLH4(3));
    addpoints(h, rLH3(1), rLH3(2), rLH3(3));    
    addpoints(h, rLH2(1), rLH2(2), rLH2(3));
    addpoints(h, rLH1(1), rLH1(2), rLH1(3));   
    addpoints(h, rB(1), rB(2), rB(3));
       
    MR1 = M0*T(L_pelvis/2, 0, 0);
    MR2 = MR1*T(0, L_pelvis/4, 0);;
    MR3 = MR2*Rx(q(5));
    MR4 = MR3*Rz(q(6));
    MR5 = MR4*Ry(q(7))*T(0,0,-L_hip);
    MR6 = MR5*Rx(q(8))*T(0,0,-L_shin);
    MR7 = MR6*T(0,L_toe,0);   
    
    rRH1 =  tform2trvec(MR1);
    rRH2 = tform2trvec(MR2);
    rRH3 = tform2trvec(MR3);
    rRH4 = tform2trvec(MR4);
    rRK = tform2trvec(MR5);
    rRA = tform2trvec(MR6);
    rRT = tform2trvec(MR7);
    
    addpoints(h, rB(1), rB(2), rB(3));
    addpoints(h, rRH1(1), rRH1(2), rRH1(3));
    addpoints(h, rRH2(1), rRH2(2), rRH2(3));
    addpoints(h, rRH3(1), rRH3(2), rRH3(3));
    addpoints(h, rRH4(1), rRH4(2), rRH4(3));
    addpoints(h, rRK(1), rRK(2), rRK(3));
    addpoints(h, rRA(1), rRA(2), rRA(3));
    addpoints(h, rRT(1), rRT(2), rRT(3));

    addpoints(h, rRA(1), rRA(2), rRA(3));
    addpoints(h, rRK(1), rRK(2), rRK(3));    
    addpoints(h, rRH4(1), rRH4(2), rRH4(3));
    addpoints(h, rRH3(1), rRH3(2), rRH3(3));    
    addpoints(h, rRH2(1), rRH2(2), rRH2(3));
    addpoints(h, rRH1(1), rRH1(2), rRH1(3));   
    addpoints(h, rB(1), rB(2), rB(3));
    
    drawnow;
end

% fprintf("L_shin = %.1f, L_hip = %.1f\n", norm(rKR-rG), norm(rHRy-rKR));


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



