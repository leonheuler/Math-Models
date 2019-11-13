clear all
%% Parameters
global m1 m2 m3 L1 L2 L3 g

m = 70;
H = 1.7;
L2 = 0.2450*H;
L1 = 0.2460*H;
L3 = H - (L1 + L2);

m1 = 0.0475*m;
m2 = 0.105*m;
m3 = 0.551*m;

g = 9.81;

fig3 = figure(3);
clf('reset');
global tau1p tau2p tau3p
tau1p = animatedline('Color','red');
tau2p = animatedline('Color','blue');
tau3p = animatedline('Color','black');
legend('tau1','tau2','tau3'); grid on;


%% Solver config  
fig1 = figure(1);
clf('reset');
opts_1 = odeset('Stats','on','OutputFcn',@odeplot);
opts_2 = odeset('RelTol',1e-3,'AbsTol',1e-5);
opts_3 = odeset('Mass',@MassMatrixFcn,'MStateDependence','strong');
opts = odeset(opts_1,opts_3);

q0 = [0 0 0 0 0 0 ]';

tic; 
tspan = [0 10];
[t, q] = ode23t(@sys, tspan, q0,opts);
% tspan = 0:0.01:10;
% q = ode4(@sys, tspan, q0);
toc;

%%

X1 = L1*sin(q(:,1));
Y1 = L1*cos(q(:,1));
X2 = L1*sin(q(:,1)) + L2*sin(q(:,2)+q(:,1));
Y2 = L1*cos(q(:,1)) + L2*cos(q(:,2)+q(:,1));
X3 = L1*sin(q(:,1)) + L2*sin(q(:,2)+q(:,1)) + L3*sin(q(:,3)+q(:,2)+q(:,1));
Y3 = L1*cos(q(:,1)) + L2*cos(q(:,2)+q(:,1)) + L3*cos(q(:,3)+q(:,2)+q(:,1));


rC = (m1*[X1;Y1] + m2*[X2;Y2] + m3*[X3;Y3]) / (m1 + m2 + m3);


% Animation
fig2 = figure(2);
clf('reset');
h = animatedline('Marker','o','LineWidth',1.5);
tr = animatedline();
cp = animatedline('Marker','o','MarkerSize',1,'Color','red');
c1p = animatedline('Marker','o','Color','blue');
c2p = animatedline('Marker','o','Color','blue');
c3p = animatedline('Marker','o','Color','blue');
axis equal;


for i=1:numel(q(:,1))-1

    addpoints(h, 0, 0);
    addpoints(h, X1(i), Y1(i));
    addpoints(h, X2(i), Y2(i));
    addpoints(h, X3(i), Y3(i));
    addpoints(tr, X3(i), Y3(i));
    
    rC1 = [X1(i)/2;Y1(i)/2];
    rC2 = [(X1(i)+X2(i))/2; (Y1(i)+Y2(i))/2];
    rC3 = [(X2(i)+X3(i))/2; (Y2(i)+Y3(i))/2];
    rC = (m1*[X1(i)/2;Y1(i)/2] + m2*[(X2(i)+X1(i))/2;(Y2(i)+Y1(i))/2] + m3*[(X2(i)+X3(i))/2;(Y3(i)+Y2(i))/2]) / (m1 + m2 + m3);
    
    addpoints(c1p, rC1(1), rC1(2));
    addpoints(c2p, rC2(1), rC2(2));
    addpoints(c3p, rC3(1), rC3(2));
    
    addpoints(cp, rC(1), rC(2));
    
    drawnow;
%     pause(0.05);
    clearpoints(c1p);
    clearpoints(c2p);
    clearpoints(c3p);
    clearpoints(h);
end

last = numel(q(:,1));
addpoints(h, 0, 0);
addpoints(h, X1(last), Y1(last));
addpoints(h, X2(last), Y2(last));
addpoints(h, X3(last), Y3(last));

%% ODE system in non-linear state-space representation
function dx = sys(t,x)
    
    % Non-linear state-space formulation
    % Generic
    global m1 m2 m3 L1 L2 L3 g
    global tau1p tau2p tau3p
    
%     matrixM = M(t, g, x(1),x(2),x(3),m1,m2,m3,L1,L2,L3);
%     invM = inv(M(t, g, x(1),x(2),x(3),m1,m2,m3,L1,L2,L3));
    vectorN = N(t,g,x(1),x(2),x(3),x(4),x(5),x(6),m1,m2,m3,L1,L2,L3);
%     Jtranspose = transpose(J(t,x(1),x(2),x(3),L1,L2,L3));
    vectorG = G(t, g, x(1),x(2),x(3),m1,m2,m3,L1,L2,L3);
    
%     if (t < 5)
%     q1d = pi/4; q2d = -pi/2; q3d = pi/4;
%     q1dotd = 0; q2dotd = 0; q3dotd = 0;
%     elseif (t >= 5)
%     q1d = pi/12; q2d = -pi/6; q3d = pi/12;
%     q1d = 0; q2d = 0; q3d = 0;
    q1dotd = 0; q2dotd = 0; q3dotd = 0;    
%     end
    
    q1d = abs(pi/4*sin(t));
    q2d = -abs(pi/2*sin(t));  
    q3d = abs(pi/4*sin(t));
    
    w = 100;
    Kp = diag([w^2 w^2 w^2]);
    Kv = diag([2*w 2*w 2*w]);
    
    e = [ q1d - x(1);
          q2d - x(2);
          q3d - x(3) ];
    
    e_dot = [ q1dotd - x(4);
              q2dotd - x(5);
              q3dotd - x(6) ];
          
    
%     tau = matrixM*(Kp*e + Kv*e_dot) + vectorN;
    tau = Kp*e + Kv*e_dot + vectorG;
    
    TAU_MAX = 200;
    
%     tau(1) = constrain(tau(1),-TAU_MAX,TAU_MAX);
    tau(2) = constrain(tau(2),-TAU_MAX,TAU_MAX);
%     tau(3) = constrain(tau(3),-TAU_MAX,TAU_MAX);
    
    addpoints(tau1p, t,tau(1));
    addpoints(tau2p, t,tau(2));
    addpoints(tau3p, t,tau(3));
    
    
%     dx = [ x(4); x(5); x(6); -invM*vectorN] + [zeros(3); invM]*tau;
    dx = [ x(4); x(5); x(6); -vectorN] + [zeros(3,1); tau];
%     Ftip = [0; 0];             
%     dx = [ x(4); x(5); x(6); -invM*vectorN] + [zeros(3); invM]*tau + [zeros(3,1); -matrixM \ (Jtranspose*Ftip)];

    % Generalized coordinates only
%     dx = [ x(4); x(5); x(6); -M(x(1),x(2),x(3)) \ N(x(1),x(2),x(3),x(4),x(5),x(6)) ];
                                  
end



function ret = constrain(val, min, max)
    
    if (val < min)
        ret = min;
    elseif (val > max)
        ret = max;
    else
        ret = val;
    end

end









