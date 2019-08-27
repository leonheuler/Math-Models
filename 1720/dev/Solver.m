

clear all;
clc;
close all;


q_i = 0;
q_x = [];
q1_y = [];
q2_y = [];
q3_y = [];
q4_y = [];
q5_y = [];
q6_y = [];
q7_y = [];
%% inital conditions

q1 = 0;
q2 = 0;
q3 = -8 * (pi/180);
q4 = -8 * (pi/180);
q5 = -1 * (pi/180);
q6 = -1 * (pi/180);

q7 = 0;

x = 1;
y = 0.2;
z = 1;

q = [q1 q2 q3 q4 q5 q6 q7  x y z]';
%%

Time = 0.4;
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
        
        s(1) = Poli3(0, Time, s_0(1), s_0(1) - 0.07, 0, 0, t);
%         s(2) = Poli3(0, Time, s_0(2), s_0(2) + 0.01, 0, 0, t);
        s(3) = Poli3(0, Time, s_0(3), s_0(3) - 0.005, 0, 0, t);
        
%         s(5) = Poli3(0, Time, s_0(5), s_0(5) + 0.31, 0, 0, t);
%         s(6) = Poli3(0, Time, s_0(6), s_0(6) + 0.2, 0, 0, t);
        
        q = q + pinv(jac(q)) * (s - f(q)); 
    
    end
    q_i = q_i + 1;    
    q_x(q_i) = q_i*dt;
    
    q1_y(q_i) = q(1);
    q2_y(q_i) = q(2);
    q3_y(q_i) = q(3);
    q4_y(q_i) = q(4);
    q5_y(q_i) = q(5);
    q6_y(q_i) = q(6);
    q7_y(q_i) = q(7);
    
    Anim(q,0);

end
%%
s_0 = s;
%%
for i = 1:Count

    for it=1:newton_euler_iterations
        t = i*dt;
        
%         s(1) = Poli3(0, Time, s_0(1), s_0(1) + 0.1, 0, 0, t);
        s(2) = Poli3(0, Time, s_0(2), s_0(2) - 0.02, 0, 0, t);
        s(3) = Poli3(0, Time, s_0(3), s_0(3) + 0.02, 0, 0, t);
%         
%         s(5) = Poli3(0, Time, s_0(5), s_0(5) + 0.07, 0, 0, t);
        s(6) = Poli3(0, Time, s_0(6), s_0(6) + 0.05, 0, 0, t);
        
        q = q + pinv(jac(q)) * (s - f(q)); 
    
    end
    q_i = q_i + 1;    
    q_x(q_i) = q_i*dt;
    q1_y(q_i) = q(1);
    q2_y(q_i) = q(2);
    q3_y(q_i) = q(3);
    q4_y(q_i) = q(4);
    q5_y(q_i) = q(5);
    q6_y(q_i) = q(6);
    q7_y(q_i) = q(7);

    Anim(q,0);

end

%%
s_0 = s;
%%
for i = 1:Count

    for it=1:newton_euler_iterations
        t = i*dt;
        
%         s(1) = Poli3(0, Time, s_0(1), s_0(1) + 0.1, 0, 0, t);
        s(2) = Poli3(0, Time, s_0(2), s_0(2) + 0.04, 0, 0, t);
%         s(3) = Poli3(0, Time, s_0(3), s_0(3) + 0.02, 0, 0, t);
%         
        s(5) = Poli3(0, Time, s_0(5), s_0(5) + 0.1, 0, 0, t);
%         s(6) = Poli3(0, Time, s_0(6), s_0(6) + 0.05, 0, 0, t);
        
        q = q + pinv(jac(q)) * (s - f(q)); 
    
    end
    q_i = q_i + 1;
    q_x(q_i) = q_i*dt;
    q1_y(q_i) = q(1);
    q2_y(q_i) = q(2);
    q3_y(q_i) = q(3);
    q4_y(q_i) = q(4);
    q5_y(q_i) = q(5);
    q6_y(q_i) = q(6);
    q7_y(q_i) = q(7);
    
    Anim(q,0);

end
%%
s_0 = s;
%%
for i = 1:Count

    for it=1:newton_euler_iterations
        t = i*dt;
        
%         s(1) = Poli3(0, Time, s_0(1), s_0(1) + 0.1, 0, 0, t);
        s(2) = Poli3(0, Time, s_0(2), s_0(2) + 0.025, 0, 0, t);
        s(3) = Poli3(0, Time, s_0(3), s_0(3) - 0.03, 0, 0, t);
%         
%         s(5) = Poli3(0, Time, s_0(5), s_0(5) + 0.07, 0, 0, t);
        s(6) = Poli3(0, Time, s_0(6), s_0(6) - 0.05, 0, 0, t);
        
        q = q + pinv(jac(q)) * (s - f(q)); 
    
    end
    q_i = q_i + 1;
    q_x(q_i) = q_i*dt;
    q1_y(q_i) = q(1);
    q2_y(q_i) = q(2);
    q3_y(q_i) = q(3);
    q4_y(q_i) = q(4);
    q5_y(q_i) = q(5);
    q6_y(q_i) = q(6);
    q7_y(q_i) = q(7);
    
    Anim(q,0);

end
%%
s_0 = s;
%%
for i = 1:Count

    for it=1:newton_euler_iterations
        t = i*dt;
        
        s(1) = Poli3(0, Time, s_0(1), s_0(1) + 0.15, 0, 0, t);
        s(2) = Poli3(0, Time, s_0(2), s_0(2) + 0.08, 0, 0, t);
        s(3) = Poli3(0, Time, s_0(3), s_0(3) - 0.01, 0, 0, t);
%         
%         s(5) = Poli3(0, Time, s_0(5), s_0(5) + 0.07, 0, 0, t);
%         s(6) = Poli3(0, Time, s_0(6), s_0(6) - 0.05, 0, 0, t);
        
        q = q + pinv(jac(q)) * (s - f(q)); 
    
    end
    q_i = q_i + 1;
    q_x(q_i) = q_i*dt;
    q1_y(q_i) = q(1);
    q2_y(q_i) = q(2);
    q3_y(q_i) = q(3);
    q4_y(q_i) = q(4);
    q5_y(q_i) = q(5);
    q6_y(q_i) = q(6);
    q7_y(q_i) = q(7);
    
    Anim(q,0);

end
%%
s_0 = s;
for i = 1:Count

    for it=1:newton_euler_iterations
        t = i*dt;
        
%         s(1) = Poli3(0, Time, s_0(1), s_0(1) + 0.1, 0, 0, t);
        s(2) = Poli3(0, Time, s_0(2), s_0(2) + 0.1, 0, 0, t);
        s(3) = Poli3(0, Time, s_0(3), s_0(3) + 0.02, 0, 0, t);
%         
%         s(5) = Poli3(0, Time, s_0(5), s_0(5) + 0.07, 0, 0, t);
%         s(6) = Poli3(0, Time, s_0(6), s_0(6) + 0.05, 0, 0, t);
        
        s(8) = Poli3(0, Time, s_0(8), s_0(8) + 0.2, 0, 0, t);
        s(9) = Poli3(0, Time, s_0(9), s_0(9) + 0.05, 0, 0, t);

        q = q + pinv(jac(q)) * (s - f(q)); 
    
    end
    q_i = q_i + 1;
    q_x(q_i) = q_i*dt;
    q1_y(q_i) = q(1);
    q2_y(q_i) = q(2);
    q3_y(q_i) = q(3);
    q4_y(q_i) = q(4);
    q5_y(q_i) = q(5);
    q6_y(q_i) = q(6);
    q7_y(q_i) = q(7);
    
    Anim(q,0);

end

%%
s_0 = s;
for i = 1:Count

    for it=1:newton_euler_iterations
        t = i*dt;
        
%         s(1) = Poli3(0, Time, s_0(1), s_0(1) + 0.1, 0, 0, t);
%         s(2) = Poli3(0, Time, s_0(2), s_0(2) + 0.1, 0, 0, t);
        s(3) = Poli3(0, Time, s_0(3), s_0(3) - 0.015, 0, 0, t);
%         
%         s(5) = Poli3(0, Time, s_0(5), s_0(5) + 0.07, 0, 0, t);
%         s(6) = Poli3(0, Time, s_0(6), s_0(6) + 0.05, 0, 0, t);
        
        s(8) = Poli3(0, Time, s_0(8), s_0(8) + 0.05, 0, 0, t);
        s(9) = Poli3(0, Time, s_0(9), s_0(9) - 0.05, 0, 0, t);

        q = q + pinv(jac(q)) * (s - f(q)); 
    
    end
    q_i = q_i + 1;
    q_x(q_i) = q_i*dt;
    q1_y(q_i) = q(1);
    q2_y(q_i) = q(2);
    q3_y(q_i) = q(3);
    q4_y(q_i) = q(4);
    q5_y(q_i) = q(5);
    q6_y(q_i) = q(6);
    q7_y(q_i) = q(7);
    
    Anim(q,0);

end

%%
s_0 = s;
for i = 1:Count

    for it=1:newton_euler_iterations
        t = i*dt;
        
        s(1) = Poli3(0, Time, s_0(1), s_0(1) - 0.15, 0, 0, t);
        s(2) = Poli3(0, Time, s_0(2), s_0(2) + 0.05, 0, 0, t);
        s(3) = Poli3(0, Time, s_0(3), s_0(3) - 0.015, 0, 0, t);
%         
%         s(5) = Poli3(0, Time, s_0(5), s_0(5) + 0.07, 0, 0, t);
%         s(6) = Poli3(0, Time, s_0(6), s_0(6) + 0.05, 0, 0, t);
        
%         s(8) = Poli3(0, Time, s_0(8), s_0(8) + 0.05, 0, 0, t);
%         s(9) = Poli3(0, Time, s_0(9), s_0(9) - 0.05, 0, 0, t);

        q = q + pinv(jac(q)) * (s - f(q)); 
    
    end
    q_i = q_i + 1;
    q_x(q_i) = q_i*dt;
    q1_y(q_i) = q(1);
    q2_y(q_i) = q(2);
    q3_y(q_i) = q(3);
    q4_y(q_i) = q(4);
    q5_y(q_i) = q(5);
    q6_y(q_i) = q(6);
    q7_y(q_i) = q(7);
    
    Anim(q,0);

end

%%
s_0 = s;
for i = 1:Count

    for it=1:newton_euler_iterations
        t = i*dt;
        
%         s(1) = Poli3(0, Time, s_0(1), s_0(1) + 0.1, 0, 0, t);
        s(2) = Poli3(0, Time, s_0(2), s_0(2) - 0.01, 0, 0, t);
        s(3) = Poli3(0, Time, s_0(3), s_0(3) + 0.04, 0, 0, t);
% %         
% %         s(5) = Poli3(0, Time, s_0(5), s_0(5) + 0.07, 0, 0, t);
% %         s(6) = Poli3(0, Time, s_0(6), s_0(6) + 0.05, 0, 0, t);
%         
        s(5) = Poli3(0, Time, s_0(5), s_0(5) + 0.2, 0, 0, t);
        s(6) = Poli3(0, Time, s_0(6), s_0(6) + 0.05, 0, 0, t);

        q = q + pinv(jac(q)) * (s - f(q)); 
    
    end
    q_i = q_i + 1;
    q_x(q_i) = q_i*dt;
    q1_y(q_i) = q(1);
    q2_y(q_i) = q(2);
    q3_y(q_i) = q(3);
    q4_y(q_i) = q(4);
    q5_y(q_i) = q(5);
    q6_y(q_i) = q(6);
    q7_y(q_i) = q(7);
    
    Anim(q,0);

end

%%
s_0 = s;
for i = 1:Count

    for it=1:newton_euler_iterations
        t = i*dt;
        
%         s(1) = Poli3(0, Time, s_0(1), s_0(1) + 0.1, 0, 0, t);
        s(2) = Poli3(0, Time, s_0(2), s_0(2) + 0.1, 0, 0, t);
        s(3) = Poli3(0, Time, s_0(3), s_0(3) - 0.02, 0, 0, t);
% %         
% %         s(5) = Poli3(0, Time, s_0(5), s_0(5) + 0.07, 0, 0, t);
% %         s(6) = Poli3(0, Time, s_0(6), s_0(6) + 0.05, 0, 0, t);
%         
        s(5) = Poli3(0, Time, s_0(5), s_0(5) + 0.2, 0, 0, t);
        s(6) = Poli3(0, Time, s_0(6), s_0(6) - 0.05, 0, 0, t);

        q = q + pinv(jac(q)) * (s - f(q)); 
    
    end
    q_i = q_i + 1;
    q_x(q_i) = q_i*dt;
    q1_y(q_i) = q(1);
    q2_y(q_i) = q(2);
    q3_y(q_i) = q(3);
    q4_y(q_i) = q(4);
    q5_y(q_i) = q(5);
    q6_y(q_i) = q(6);
    q7_y(q_i) = q(7);
    
    Anim(q,0);

end

%%
Anim(q,1);

%%
toc  
figure;
hold on;

plot(q_x, q1_y);
plot(q_x, q2_y);
plot(q_x, q3_y);
plot(q_x, q4_y);
plot(q_x, q5_y);
plot(q_x, q6_y);
plot(q_x, q7_y);



