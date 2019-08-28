function Solver()

clear;
clc;
close all;

%% inital conditions

q1 = 0;
q2 = 0;
q3 = 0 * (pi/180);
q4 = 0 * (pi/180);
q5 = -4 * (pi/180);
q6 = -4 * (pi/180);

q7 = 0;
q8 = 0;
q9 = 0;

x = 1;
y = 0.2;
z = 1;

q = [q1 q2 q3 q4 q5 q6 q7 q8 q9 x y z]';
%%

Time = 2;
dt = 0.01;
Count = floor(Time/ dt);
                                                                                            
 
s_0 = f(q);
s = s_0;

newton_euler_iterations = 3;     
tic

%%

for i = 1:Count

    for it=1:newton_euler_iterations
        t = i*dt;
        
        s(1) = Poli3(0, Time, s_0(1), s_0(1) - 0.1, 0, 0, t);
        s(2) = Poli3(0, Time, s_0(2), s_0(2) + 0.15, 0, 0, t);
        s(3) = Poli3(0, Time, s_0(3), s_0(3) + 0.04, 0, 0, t);
        
        s(5) = Poli3(0, Time, s_0(5), s_0(5) + 0.31, 0, 0, t);
        s(6) = Poli3(0, Time, s_0(6), s_0(6) + 0.2, 0, 0, t);
        
        q = q + pinv(jac(q)) * (s - f(q)); 
    
    end

    Anim(q,0);

end

s_0 = s;

for i = 1:Count

    for it=1:newton_euler_iterations
        t = i*dt;
        
        s(1) = Poli3(0, Time, s_0(1), s_0(1) + 0.1, 0, 0, t);
        s(2) = Poli3(0, Time, s_0(2), s_0(2) - 0.15, 0, 0, t);
        s(3) = Poli3(0, Time, s_0(3), s_0(3) - 0.04, 0, 0, t);
        
        s(5) = Poli3(0, Time, s_0(5), s_0(5) - 0.31, 0, 0, t);
        s(6) = Poli3(0, Time, s_0(6), s_0(6) - 0.2, 0, 0, t);
        
        q = q + pinv(jac(q)) * (s - f(q)); 
    
    end

    Anim(q,0);

end


%%

Anim(q,1);

%%
toc  

end


