fig1 = figure(1);
fig1.Position = [200 200 500 500];
clf('reset');

h = animatedline();
h.Marker = 'o';
h.LineWidth = 2;
axis equal;
grid on;

tr = animatedline();
tr.Color = 'red';


fig2 = figure(2);
fig2.Position = [750 200 500 500];
clf('reset');
subplot(3,1,1);
q1_graph = animatedline();
q2_graph = animatedline('Color','red');
grid on;
legend('q1','q2'); ylabel('deg'); title('Joint Angles');

subplot(3,1,2);
w1_graph = animatedline();
w2_graph = animatedline('Color','red');
grid on;
legend('w1','w2'); ylabel('deg/s'); title('Joint Velocities');


subplot(3,1,3);
e1_graph = animatedline();
e2_graph = animatedline('Color','red');
grid on;
legend('e1','e2');  ylabel('deg/s^2'); title('Joint Accelerations');

fig3 = figure(3);
fig3.Position = [1300 200 500 500];
clf('reset');
subplot(2,1,1);
tau1_graph = animatedline();
grid on; ylabel('H*m'); title('Torque 1');
subplot(2,1,2);
tau2_graph = animatedline();
grid on; ylabel('H*m'); title('Torque 2');

%% Model Parameters
global m1 m2 L1 L2 g

L1 = 0.5;   m1 = 1;
L2 = 0.5;   m2 = 1;

g = 9.81; dt = 0.01;
%% Trajectory 

q = [0 0]';
q_d = [0 0]';

t0 = 0;
t1 = 0;

while 1
    
    fprintf("1 - Move\n2 - Clear\n3 - Plots\n4 - Exit\n");
    cmd = input('Specify command..');
    
    switch cmd
        case 1
            task = input('Specify task.. [q1_target, q2_target,  T]: ');
        case 2
            clearpoints(tr);
            clearpoints(q1_graph);
            clearpoints(q2_graph);
            clearpoints(w1_graph);
            clearpoints(w2_graph);
            clearpoints(e1_graph);
            clearpoints(e2_graph);
            clearpoints(tau1_graph);
            clearpoints(tau2_graph);
            t0 = 0;
            continue;
        case 3 
            [Data_t, Data_q1] = getpoints(q1_graph); figure; plot(Data_t, Data_q1);
            [Data_t, Data_q2] = getpoints(q2_graph); hold on; plot(Data_t, Data_q2);
            [Data_t, Data_q1_d] = getpoints(q1_d_graph);  plot(Data_t, Data_q1_d,'--');
            [Data_t, Data_q2_d] = getpoints(q2_d_graph);  plot(Data_t, Data_q2_d,'--');
        case 4
            break;  
    end
    
    
    q_d = [ task(1);
            task(2)];
    
    T = task(3);
    
    
    [a1, a2, a3, a4, a5, a6] = Poli5(q(1), q_d(1), 0, 0, 0, 0, T);
    [b1, b2, b3, b4, b5, b6] = Poli5(q(2), q_d(2), 0, 0, 0, 0, T);
       
    t1 = t0 + T;
    
    tic;
    for t = t0:dt:t1
        

        q = [a1 + (t-t0)*a2 + (t-t0)^2*a3 + (t-t0)^3*a4 + (t-t0)^4*a5 + (t-t0)^5*a6;
                b1 + (t-t0)*b2 + (t-t0)^2*b3 + (t-t0)^3*b4 + (t-t0)^4*b5 + (t-t0)^5*b6];
        
        q_dot = [ a2 + 2*(t-t0)*a3 + 3*(t-t0)^2*a4 + 4*(t-t0)^3*a5 + 5*(t-t0)^4*a6;
                    b2 + 2*(t-t0)*b3 + 3*(t-t0)^2*b4 + 4*(t-t0)^3*b5 + 5*(t-t0)^4*b6];
            
        q_dot_dot = [ 2*a3 + 6*(t-t0)*a4 + 12*(t-t0)^2*a5 + 20*(t-t0)^3*a6;
                        2*b3 + 6*(t-t0)*b4 + 12*(t-t0)^2*b5 + 20*(t-t0)^3*b6];
            
        
        tau = M(q)*q_dot_dot + N(q, q_dot);
        
        
        %% Forward Kinematics
        r1 = [ L1*cos(q(1));
               L1*sin(q(1))];
        r2 = [ L1*cos(q(1)) + L2*cos(q(1) + q(2));
               L1*sin(q(1)) + L2*sin(q(1) + q(2))];


        %% Animation

        addpoints(h, 0, 0, 0);
        addpoints(h, r1(1), r1(2));
        addpoints(h, r2(1), r2(2));
        addpoints(tr, r2(1), r2(2));


        drawnow %limitrate;
        clearpoints(h);

        addpoints(q1_graph, t, q(1));
        addpoints(q2_graph, t, q(2));
        addpoints(w1_graph, t, q_dot(1));
        addpoints(w2_graph, t, q_dot(2));
        addpoints(e1_graph, t, q_dot_dot(1));
        addpoints(e2_graph, t, q_dot_dot(2));
        addpoints(tau1_graph, t, tau(1));
        addpoints(tau2_graph, t, tau(2));


    end

    addpoints(h, 0, 0, 0);
    addpoints(h, r1(1), r1(2));
    addpoints(h, r2(1), r2(2));

    drawnow;
    
    t0 = t1;
    q1_0 = q(1);
    q2_0 = q(2);
    
    fprintf("Task completed in %.2f sec!\n", toc);
    
end



function [a,  b,  c,  d] = CIP(q_0,q_1, w_0, w_1, T)
%CUBIC_INTERPOLATING_POLYNOMIAL 
%   Returns coefficients of cubic polynomial


    a = q_0;
    b = w_0;
    c = ( 3*(q_1-q_0)- T*(2*w_0+w_1) ) / T^2;
    d = ( 2*(q_0-q_1) - T*(w_0+w_1) )/ T^3; 

end


    
function [a,  b,  c,  d, e, f] = Poli5( q_0, q_1, w_0, w_1, e_0, e_1, T)

    a = q_0; b = w_0; c = e_0/2;
    d = -(20*(q_0) - 20*(q_1) + 12*T*(w_0) + 8*T*(w_1) + 3*T^2*(e_0) - T^2*(e_1))/(2*T^3);
    e = (30*(q_0) - 30*(q_1) + 16*T*(w_0) + 14*T*(w_1) + 3*T^2*(e_0) - 2*T^2*(e_1))/(2*T^4);
    f = -(12*(q_0) - 12*(q_1) + 6*T*(w_0) + 6*T*(w_1) + T^2*(e_0) - T^2*(e_1))/(2*T^5);

end     
    
    

function ret = M(q)

    global m1 m2 L1 L2
    
    ret = [ (m1+m2)*L1^2 + m2*L2^2 + 2*m2*L1*L2*cos(q(2)),      m2*L2^2+m2*L1*L2*cos(q(2));
            m2*L2^2+m2*L1*L2*cos(q(2)),                         m2*L2^2];
    
end

function ret = V(q, q_dot)
    
    global m1 m2 L1 L2 
    ret = [ -m2*L1*L2*(2*q_dot(1)*q_dot(2)+(q_dot(2)^2))*sin(q(2));
            m2*L1*L2*(q_dot(1)^2)*sin(q(2))];
        
end

function ret = G(q)
    
    global m1 m2 L1 L2 g
    ret = [(m1+m2)*g*L1*cos(q(1)) + m2*g*L2*cos(q(1)+q(2));
            m2*g*L2*cos(q(1)+q(2))];

end


function ret = F(q_dot)
    
    Fv1 = 0.0;
    Fv2 = 0.0;

    Fv  = [Fv1 0;
           0   Fv2];

    ret = Fv*q_dot;
        
end


function ret = N(q,q_dot)

    ret = V(q,q_dot) + F(q_dot) + G(q);
          
end
    
    
    
    
