
fig1 = figure(1);
clf('reset');
fig1.Position = [ 1300 350 600 600 ];
axis([-2 2 -2 2 -2 2]);
axis equal
set(gca, 'YDir', 'reverse');
set(gca, 'XDir', 'reverse');
grid on

trS = animatedline();
trS_m = animatedline();

trD = animatedline();
trD.Color = 'red';
trD_m = animatedline();

trHL = animatedline();
trHL.LineWidth = 2;
trHR = animatedline();
trHR.LineWidth = 2;

q = zeros(12,1);

H = 1.70;
L_hip = H*0.2450;
L_shin = H*0.2460;
L_toe = 0.0577*H;
L_pelvis = 0.2;

q(1) = -pi/72;   q(6) = pi/72;
q(2) = 0;   q(7) = 0;
q(3) = 0;   q(8) = 0;
q(4) = pi/72;   q(9) = 0;
q(5) = -pi/72; 

q(10) = 0; q(11) = 0; q(12) = 0;
q10_0 = q(10); q11_0 = q(11); q12_0 = q(12); 
%%

Time = 0.4;
dt = 0.01;
Count = floor(Time/ dt);

global Data_q Data_t
Data_q = zeros(Count+1, 12);
Data_t = zeros( 1, Count+1); 

s_0 = f1(q);
s = s_0;
d_0 = s_0;
d = s;
q1_0 = q(1);
q2_0 = q(2);


newton_euler_iterations = 1;     
tic

%%
% d = s + [ 0 ; 0.4  ; 0.05];

K = [
    1    0   0   0   0   0   0   0   0  0   0   0;
    0    1   0   0   0   0   0   0   0  0   0   0;
    0    0   1   0   0   0   0   0   0  0   0   0;
    0    0   0   1   0   0   0   0   0  0   0   0;
    0    0   0   0   1   0   0   0   0  0   0   0;
    0    0   0   0   0   1   0   0   0  0   0   0;
    0    0   0   0   0   0   1   0   0  0   0   0;
    0    0   0   0   0   0   0   1   0  0   0   0;
    0    0   0   0   0   0   0   0   1  0   0   0;
    0    0   0   0   0   0   0   0   0  1   0   0;
    0    0   0   0   0   0   0   0   0  0   1   0;
    0    0   0   0   0   0   0   0   0  0   0   1;
    ];

%%
p5y = s_poli5(Time, 0.2, 0, 0, 0, 0);
p6z = s_poli6(Time, 0.05, 0, 0, 0, 0);
%%
IK_solver = 1;

for i = 0:Count+50

    t = i*dt;
   
    s = f1(q);
    
    if (i <= Count)
        d(2) = d_0(2) + get_poli5_val(p5y, t); 
        d(3) = d_0(3) + get_poli6_val(p6z, t);
    end
    
    F = (d - s);
%     s = s + F;
    
%     q = q + pinv(jac1(q))*(s - f1(q));
    q_dot = K*transpose(jac1(q))*F;
    
    q = q + q_dot;
    
    if (q(1) > q1_0)
        q(1) = q1_0;
    end
    if (q(2) ~= q2_0)
       q(2) = q2_0;
    end
   
    if (q(9) > 0)
       q(9) = 0;
    end
    
    q(10) = q10_0;
    q(11) = q11_0;
    q(12) = q12_0;
    
    anim(q,0, 'black',  'rs');
    addpoints(trS, s(1), s(2), s(3));
    addpoints(trD, d(1), d(2), d(3));

    
    Data_q(i+1,:) = q';
    Data_t(i+1) = t;

    M1 = Rx(q(1))*T(0,0,L_shin);
    M2 = M1*Rx(q(2))*T(0,0,L_hip);
    M3 = M2*Ry(q(3))*T(0,0,L_hip/10);
    M4 = M3*Rz(q(4))*T(0,0,L_hip/20);
    M5 = M4*Rx(q(5))*T(0,-L_pelvis/4,0);
    M6 = M5*T(-L_pelvis, 0, 0);
    M7 = M6*T(0,L_pelvis/4,0);
    M8 = M7*Rx(q(6))*T(0,0,-L_hip/10);

    
    rKR = M1(:,4); 
    rHR1 = M2(:,4);
    rHR2 = M3(:,4);
    rHR3 = M4(:,4);
    rHR4 = M5(:,4);
    rHL1 = M6(:,4);
    rHL2 = M7(:,4);
    rHL3 = M8(:,4);

    addpoints(trHR, rHR3(1), rHR3(2), rHR3(3));
    addpoints(trHL, rHL2(1), rHL2(2), rHL2(3));
    
end

s = f1(q);
q(10) = s(1); q(11) = s(2); q(12) = s(3);


s = f2(q);
d = s;
d_0 = d;

for i = 0:Count+50

    t = i*dt;
   
    s = f2(q);
    
    if (i <= Count)
        d(2) = d_0(2) + get_poli5_val(p5y, t); 
        d(3) = d_0(3) + get_poli6_val(p6z, t);
    end
    
    F = (d - s);
%     s = s + F;
    
%     q = q + pinv(jac(q))*(s - f(q));
    q_dot = K*transpose(jac2(q))*F;
    
    q = q + q_dot;
    
%     if (q(1) > q1_0)
%         q(1) = q1_0;
%     end
%     if (q(2) ~= q2_0)
%        q(2) = q2_0;
%     end
%    
    if (q(9) > 0)
       q(9) = 0;
    end
    
    q(10) = q10_0;
    q(11) = q11_0;
    q(12) = q12_0;

    anim(q,0, 'black',  'ls');
    addpoints(trS, s(1), s(2), s(3));
    addpoints(trD, d(1), d(2), d(3));

    
    Data_q((Count+50)+i+1,:) = q';
    Data_t((Count+50)+i+1) = t;

    M1 = Rx(q(1))*T(0,0,L_shin);
    M2 = M1*Rx(q(2))*T(0,0,L_hip);
    M3 = M2*Ry(q(3))*T(0,0,L_hip/10);
    M4 = M3*Rz(q(4))*T(0,0,L_hip/20);
    M5 = M4*Rx(q(5))*T(0,-L_pelvis/4,0);
    M6 = M5*T(-L_pelvis, 0, 0);
    M7 = M6*T(0,L_pelvis/4,0);
    M8 = M7*Rx(q(6))*T(0,0,-L_hip/10);

    
    rKL = M1(:,4); 
    rHL1 = M2(:,4);
    rHL2 = M3(:,4);
    rHL3 = M4(:,4);
    rHL4 = M5(:,4);
    rHR1 = M6(:,4);
    rHR2 = M7(:,4);
    rHR3 = M8(:,4);

    addpoints(trHR, rHR3(1), rHR3(2), rHR3(3));
    addpoints(trHL, rHL2(1), rHL2(2), rHL2(3));
    
end



% anim(Data_q(1,:)',  1,  'red', '-');
% anim(Data_q(30,:)',   1,  'green', '-');
% anim(Data_q(Count+50,:)',   1,  'blue', '-');


fig3 = figure(3);
clf('reset');
[x,y,z] = getpoints(trHR);
plot(y,z);
hold on;
[x,y,z] = getpoints(trHL);
plot(y,z);


showQ();

function ret = get_poli5_val(p,t)

    ret = p(1)*t^5 + p(2)*t^4 + p(3)*t^3 + p(4)*t^2 + p(5)*t;
    
end

function ret = get_poli6_val(p,t)

    ret = p(1)*t^6 + p(2)*t^5 + p(3)*t^4 + p(4)*t^3 + p(5)*t^2 + p(6)*t;
    
end

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