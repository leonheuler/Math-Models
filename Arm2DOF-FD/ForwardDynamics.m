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

L1 = 0.5;   m1 = 1;
L2 = 0.5;   m2 = 1;

g = 9.81; dt = 0.01;
%% Trajectory 

q1_0 = -pi/2;    w1_0 = 0;
                w1_1 = 0;

q2_0 = 0;    w2_0 = 0;
             w2_1 = 0;

t0 = 0;
t1 = 0;

while 1
    
    fprintf("1 - Move\n2 - Clear\n3 - Plots\n4 - Exit\n");
    cmd = input('Specify command..');
    
    switch cmd
        case 1
            task = input('Specify task.. [q1_target, w1_0, w1_1, q2_target, w2_0, w2_1, T]: ');
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
            [Data_t, Data_q1] = getpoints(q1_graph);
            [Data_t, Data_q2] = getpoints(q2_graph);
        case 4
            break;  
    end
    
    q1_target = task(1)*(pi/180);
    w1_0 = task(2);
    w1_1 = task(3);


    q2_target = task(4)*(pi/180);
    w2_0 = task(5);
    w2_1 = task(6);
    
    if (q1_target == q1_0 && q2_target == q2_0)
        continue;
    end
    
    T = task(7);
    
    [a1, b1, c1, d1] = CIP(q1_0, q1_target, w1_0, w1_1, T);
    [a2, b2, c2, d2] = CIP(q2_0, q2_target, w2_0, w2_1, T);
    
    t1 = t0 + T;
    
    tic;
    for t = t0:dt:t1
        
        

        q1 = a1 + (t-t0)*b1 + (t-t0)^2*c1 + (t-t0)^3*d1;
        q2 = a2 + (t-t0)*b2 + (t-t0)^2*c2 + (t-t0)^3*d2;

        w1 = b1 + 2*(t-t0)*c1+3*(t-t0)^2*d1;
        w2 = b2 + 2*(t-t0)*c2+3*(t-t0)^2*d2;

        e1 = 2*c1 + 6*(t-t0)*d1;
        e2 = 2*c2 + 6*(t-t0)*d2;

        tau1 = ((m1+m2)*L1^2 + m2*L2^2 + 2*m2*L1*L2*cos(q2))*e1 + (m2*L2^2+m2*L1*L2*cos(q2))*e2 - m2*L1*L2*(2*w1*w2 + w2^2)*sin(q2) + (m1+m2)*g*L1*cos(q1) + m2*g*L2*cos(q1+q2);
        tau2 = (m2*L2^2 + m2*L1*L2*cos(q2))*e1 + m2*L2^2*e2 + m2*L1*L2*w1^2*sin(q2) + m2*g*L2*cos(q1+q2);


        %% Forward Kinematics
        r1 = [ L1*cos(q1);
               L1*sin(q1)];
        r2 = [ L1*cos(q1) + L2*cos(q1 + q2);
               L1*sin(q1) + L2*sin(q1 + q2)];


        %% Animation

        addpoints(h, 0, 0, 0);
        addpoints(h, r1(1), r1(2));
        addpoints(h, r2(1), r2(2));
        addpoints(tr, r2(1), r2(2));


        drawnow %limitrate;
        clearpoints(h);

        addpoints(q1_graph, t, q1*(180/pi));
        addpoints(q2_graph, t, q2*(180/pi));
        addpoints(w1_graph, t, w1*(180/pi));
        addpoints(w2_graph, t, w2*(180/pi));
        addpoints(e1_graph, t, e1*(180/pi));
        addpoints(e2_graph, t, e2*(180/pi));
        addpoints(tau1_graph, t, tau1);
        addpoints(tau2_graph, t, tau2);


    end

    addpoints(h, 0, 0, 0);
    addpoints(h, r1(1), r1(2));
    addpoints(h, r2(1), r2(2));

    drawnow;
    
    t0 = t1;
    q1_0 = q1_target;
    q2_0 = q2_target;
    
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


    
    
    
    
    
    
    
