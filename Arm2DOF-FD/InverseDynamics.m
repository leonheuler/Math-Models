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

I1 = m1*L1^2;
I2 = m2*L2^2;

while 1
    

    task = input('Specify task.. [q1_0, q2_0, q1_dot_0, q2_dot_0 tau1, tau2, T]: ');
    
    if (task == 'q')
        fprintf("Quitting..");
        break;
    end
    
    q  = [task(1);
        task(2)];
    
    q1 = q(1);
    q2 = q(2);
    
    q_dot = [task(3);
             task(4)];
    q1_dot = q_dot(1);
    q2_dot = q_dot(2);
         
    tau = [task(5);
           task(6)];
       
    T = task(7);    
       
    for t = 0:dt:T
        
        t1 = M(q2);
        t2 = inv(M(q2))*tau;
        t3 = V(q2, q1_dot, q2_dot);
        t4 = G(q1,q2);
        t5 = inv(M(q2))*V(q2, q1_dot, q2_dot) + G(q1,q2);
        
        
        q_dot_dot = inv(M(q2))*tau - inv(M(q2))*(V(q2, q1_dot, q2_dot) + G(q1,q2));
        
        q_dot = q_dot + q_dot_dot.*dt;
        q = q + q_dot.*dt + q_dot_dot.*(dt^2/2);
        
        q1 = q(1); q2 = q(2);
        w1 = q_dot(1); w2 = q_dot(2);    
        e1 = q_dot_dot(1); e2 = q_dot_dot(2);
        tau1 = I1*e1;
        tau2=  I2*e2;
        
        %% Forward Kinematics
        A1 = [
                cos(q1)  -sin(q1)  0   L1*cos(q1);
                sin(q1)  cos(q1)   0   L1*sin(q1);
                0       0          1   0;
                0       0          0   1];


        A2 = [
                cos(q2)  -sin(q2)  0   L2*cos(q2);
                sin(q2)  cos(q2)   0   L2*sin(q2);
                0       0          1   0;
                0       0          0   1
                ];

        T1 = A1;
        T2 = A1*A2;

        r1 = tform2trvec(T1);
        r2 = tform2trvec(T2);

        %% Animation

        addpoints(h, 0, 0, 0);
        addpoints(h, r1(1), r1(2), r1(3));
        addpoints(h, r2(1), r2(2), r2(3));
        addpoints(tr, r2(1), r2(2), r2(3));


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
    addpoints(h, r1(1), r1(2), r1(3));
    addpoints(h, r2(1), r2(2), r2(3));

    drawnow;
    
end

function ret = M(q2)
    
    global m1 m2 L1 L2
    ret = [ (m1+m2)*L1^2 + m2*L2^2 + 2*m2*L1*L2*cos(q2),    m2*L2^2 + m2*L1*L2*cos(q2);
            m2*L2^2 + m2*L1*L2*cos(q2),                     m2*L2^2];
        
end

function ret = V(q2, q1_dot, q2_dot)
    
    global m2 L1 L2
    ret = [-m2*L1*L2*(2*q1_dot*q2_dot + q2_dot^2)*sin(q2);
            m2*L1*L2*(q1_dot^2)*sin(q2)];

end

function ret = G(q1, q2)

    global m1 m2 L1 L2 g
    ret = [(m1+m2)*g*L1*cos(q1) + m2*g*L2*cos(q1+q2);
            m2*g*L2*cos(q1+q2)];     
end



