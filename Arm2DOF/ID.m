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

L1 = 1;   m1 = 1;
L2 = 1;   m2 = 1;

g = 9.81; dt = 0.01; t0 = 0;
%% Trajectory 


while 1
    

    fprintf("1 - Move\n2 - Clear\n3 - Plots\n4 - Exit\n");
    cmd = input('Specify command..');
    
    switch cmd
        case 1
            task = input('Specify initial conditions.. [q1, q2, q1dot, q2dot, tau1, tau2, T]: ');
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
            continue;
        case 4
            break;  
    end
    
    
    q = [ task(1);
          task(2)];
    
    q_dot = [ task(3);
              task(4)];
    
    tau = [ task(5);
            task(6)];
    
    T = task(7);

    t1 = t0 + T;
    
    for t = t0:dt:t1

        
        if (t >= t0 + 1)
            tau = [0; 0];
        end  
        
        q_dot_dot = inv(M(q))*(tau - V(q,q_dot) - G(q));
        
        q_dot = q_dot + q_dot_dot.*dt;
        
        q = q + q_dot.*dt + q_dot_dot.*(dt^2/2);
            

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
        addpoints(w1_graph, t, q_dot(1)*(180/pi));
        addpoints(w2_graph, t, q_dot(2)*(180/pi));
        addpoints(e1_graph, t, q_dot_dot(1)*(180/pi));
        addpoints(e2_graph, t, q_dot_dot(2)*(180/pi));
        addpoints(tau1_graph, t, tau(1));
        addpoints(tau2_graph, t, tau(2));

    end
    
    addpoints(h, 0, 0, 0);
    addpoints(h, r1(1), r1(2));
    addpoints(h, r2(1), r2(2));

    drawnow;
    
    t0 = t1;

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

            
            
            
            
            
            
            
            
            
            
            
            
            

    
    
    
    
    
    
    
