
fig1 = figure(1);
clf('reset');
fig1.Position = [ 1300 350 600 600 ];
axis([-2 2 -2 2 -2 2]);
axis equal
set(gca, 'YDir', 'reverse');
set(gca, 'XDir', 'reverse');
grid on

trSR = animatedline();
trSL = animatedline();

trDL = animatedline();
trDL.Color = 'red';
trDR = animatedline();
trDR.Color = 'red';

trHL = animatedline();
trHL.LineWidth = 2;
trHR = animatedline();
trHR.LineWidth = 2;

trB = animatedline();
trB.LineWidth = 2;

q = zeros(14,1);

H = 1.70;
L_hip = H*0.2450;
L_shin = H*0.2460;
L_toe = 0.0577*H;
L_pelvis = 0.6;

q(1) = -5*(pi/180); q(2) = 0; q(3) = 0; q(4) = 0;
q(5) = -5*(pi/180); q(6) = 0; q(7) = 0; q(8) = 0;

q(9) = 0; q(10) = 0; q(11) = 0;
q(12) = 0; q(13) = 0; q(14) = 0;
%%

Time1 = 0.2;
Time2 = 0.4;
Time3 = 0.4;
Time4 = 0.4;
Time5 = 0.2;

dt = 0.01;

Count1 = floor(Time1 / dt);
Count2 = floor(Time2 / dt);
Count3 = floor(Time3 / dt);
Count4 = floor(Time4 / dt);
Count5 = floor(Time5 / dt);

global Data_q Data_t
Data_q = zeros(Count1 + Count2 + Count3 + Count4 + Count5 + 1, 14);
Data_t = zeros( 1, Count1 + Count2 + Count3 + Count4 + Count5 + 1); 


%%

K = diag(ones(1,14)./3);
k_p = 0.5;

s = f(q);
q_0 = q;
sL = [s(1) s(2) s(3)]';
sR = [s(4) s(5) s(6)]';

sLen = 0.4;
sHeight = 0.02;
sTwist = 0.00;

dL = sL;
dR = sR;
dL_0 = dL;
dR_0 = dR;

%%
tic
for i = 0:Count1 + Count2 + Count3 + Count4 + Count5

    t = i*dt;
   
    s = f(q);
    
    sL = [s(1) s(2) s(3)]';
    sR = [s(4) s(5) s(6)]';
    
%   Trajectory config
    if (i <= Count1)
        
        dL = [
                  Poli4H(0, Time1, dL_0(1), dL_0(1), 0, 0, sTwist, t);
                  Poli3(0, Time1, dL_0(2), dL_0(2) + sLen/2, 0, 0, t);
                  Poli4H(0, Time1, dL_0(3), dL_0(3), 0, 0, -sHeight, t);
                ];
        dL_01 = dL;
        
    elseif (i <= Count1 + Count2)
        
        
        dR = [
                  Poli4H(Time1+dt, Time1+Time2, dR_0(1), dR_0(1), 0, 0, -sTwist, t);
                  Poli3(Time1+dt, Time1+Time2, dR_0(2), dR_0(2) + sLen, 0, 0, t);
                  Poli4H(Time1+dt, Time1+Time2, dR_0(3), dR_0(3), 0, 0, -sHeight, t);
                ];
        dR_01 = dR;
         
    elseif (i <= Count1 + Count2 + Count3)
        
         dL = [
                  Poli4H(Time1+Time2+dt, Time1+Time2+Time3, dL_01(1), dL_01(1), 0, 0, sTwist, t);
                  Poli3(Time1+Time2+dt, Time1+Time2+Time3, dL_01(2), dL_01(2) + sLen, 0, 0, t);
                  Poli4H(Time1+Time2+dt, Time1+Time2+Time3, dL_01(3), dL_01(3), 0, 0, -sHeight, t);
                ];       
        dL_02 = dL;
        
    elseif (i <= Count1 + Count2 + Count3 + Count4)
        
         dR = [
                  Poli4H(Time1+Time2+Time3+dt, Time1+Time2+Time3+Time4, dR_01(1), dR_01(1), 0, 0, -sTwist, t);
                  Poli3(Time1+Time2+Time3+dt, Time1+Time2+Time3+Time4, dR_01(2), dR_01(2) + sLen, 0, 0, t);
                  Poli4H(Time1+Time2+Time3+dt, Time1+Time2+Time3+Time4, dR_01(3), dR_01(3), 0, 0, -sHeight, t);
                ];       
        dR_02 = dR;    
        
    elseif (i <= Count1 + Count2 + Count3 + Count4 + Count5)
        
         dL = [
                  Poli4H(Time1+Time2+Time3+Time4+dt, Time1+Time2+Time3+Time4+Time5, dL_02(1), dL_02(1), 0, 0, sTwist, t);
                  Poli3(Time1+Time2+Time3+Time4+dt, Time1+Time2+Time3+Time4+Time5, dL_02(2), dL_02(2) + sLen/2, 0, 0, t);
                  Poli4H(Time1+Time2+Time3+Time4+dt, Time1+Time2+Time3+Time4+Time5, dL_02(3), dL_02(3), 0, 0, -sHeight, t);
                ];       
        dL_03 = dL;        
            
    end
       
    d = [dL; dR];

 
%   Solver config
    fun = @(q)(norm(d-f(q)));
    q0 = q;   
    
    lb = [0;    %q1
          -5;
          -5;
          -8;
          0;    %q5
          -5;
          -5;
          -8;
          -5*(180/pi);  %q9
          -5*(180/pi);
          -0.1*(180/pi);
          -15;  %q12
          -10;
          -1;
          ]*(pi/180); 
      
      ub = [10; %q1
          5;
          5;
          0;
          10;   %q5
          5;
          5;
          0;
          5*(180/pi);   %q9
          5*(180/pi);
          0.06*(180/pi);
          -5;       %q12
          10;
          1;
          ]*(pi/180); 
      
      
    options = optimoptions('fmincon','Display','off','Algorithm', 'sqp');

    q = fmincon(fun,q0,[],[],[],[],lb,ub,[],options);


%   Animation
    
    anim(q,0, 'black');
    addpoints(trSL, s(1), s(2), s(3));
    addpoints(trSR, s(4), s(5), s(6));
    
    addpoints(trDL, d(1), d(2), d(3));
    addpoints(trDR, d(4), d(5), d(6));
    
    Data_q(i+1,:) = q';
    Data_t(i+1) = t;

    M0 = T(q(9), q(10), q(11))*Rz(q(14))*Rx(q(12))*Ry(q(13));
    
    ML1 = M0*T(-L_pelvis/2, 0, 0);
    ML2 = ML1*T(0, L_pelvis/4, 0);;
    MR1 = M0*T(L_pelvis/2, 0, 0);
    MR2 = MR1*T(0, L_pelvis/4, 0);;
    
    rB = M0(:,4);
    rLH2 = ML2(:,4);    
    rRH2 = MR2(:,4);

    addpoints(trHR, rLH2(1), rLH2(2), rLH2(3));
    addpoints(trHL, rRH2(1), rRH2(2), rRH2(3));
    addpoints(trB, rB(1), rB(2), rB(3));
     
    drawnow limitrate;
end
%%


anim(Data_q(1,:)',  1,  'red');
anim(Data_q(Count1,:)',   1,  'green');
anim(Data_q(Count1+Count2+1,:)',   1,  [0 0.4470 0.7410]);
anim(Data_q(Count1+Count2+Count3+1,:)',   1,  [0 0.4470 0.7410]);
anim(Data_q(Count1+Count2+Count3+Count4+1,:)',   1,  [0 0.4470 0.7410]);
anim(Data_q(Count1+Count2+Count3+Count4+Count5+1,:)',   1,  [0 0.4470 0.7410]);


fig4 = figure(4);
clf('reset');
[x,y,z] = getpoints(trHR);
plot(y,z);
hold on;
[x,y,z] = getpoints(trHL);
plot(y,z);



showQ();



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