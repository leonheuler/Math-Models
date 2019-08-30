


fig1 = figure(1);
clf('reset');
h = animatedline();
h.LineWidth = 2;
h.Marker = 'o';
axis equal
grid on



% Model props
L1 = 0.5;
L2 = 0.5;

% Initial condtitions
phi3 = 0*pi/180;
phi2 = 0*pi/180;


rA_0 = [
         L1*sin(phi3) - L2*sin(phi2 - phi3);
        -L1*cos(phi3) - L2*cos(phi2 - phi3)
     ];

% Simulation props
 Time = 2*3.14;
 dt = 0.01;
 Iterations = Time / dt;
 
 % Trajectory params
 L = 0.65;
 H = 0.05;
 
for It = 0:Iterations

    t = It*dt;
    
%  Trajectory rA
%     rA = [
%             Poli5(0, Time, rA_0(1), rA_0(1) + L, 0, 0, 0, 0, t);
%             Poli6H(0, Time, rA_0(2), rA_0(2), 0, 0, 0, 0, H, t);
%          ];
   rA = rA_0 + [0.05*sin(t); -0.05*cos(t)];
     
%  IK
    s = rA;
    q = [phi3; phi2];
    q = q + pinv(jac(q))*(s - f(q));
    phi3 = q(1);
    phi2 = q(2);
    if (phi2 < 0)
        phi2 = 0;
    end
%  Animation

     rC = [
        L1*sin(phi3);
        -L1*cos(phi3)
        ];
    
    disp(norm(rC-rA));
    
%     rA = [
%          L1*sin(phi3) - L2*sin(phi2 - phi3);
%         -L1*cos(phi3) - L2*cos(phi2 - phi3)
%          ];
 
 
    addpoints(h, 0, 0);
    addpoints(h, rC(1), rC(2));
    addpoints(h, rA(1), rA(2));
    drawnow;
    clearpoints(h);
end

addpoints(h, 0, 0);
addpoints(h, rC(1), rC(2));
addpoints(h, rA(1), rA(2));
drawnow;




