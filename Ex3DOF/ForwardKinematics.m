

L1 = 0.5; L2 = 0.5; L3 = 0.75;
q1 = pi/16  ;
q2 = pi/12;
q3 = pi/12;

x_k = L1*sin(q1);
y_k = L1*cos(q1);

x_h = L1*sin(q1) - L2*sin(q2 - q1);
y_h = L1*cos(q1) + L2*cos(q2 - q1);

x_b = L1*sin(q1) - L2*sin(q2 - q1) - L3*sin(-q3 + q2 - q1);
y_b = L1*cos(q1) + L2*cos(q2 - q1) + L3*cos(-q3 + q2 - q1);

h = animatedline('Marker','o','LineWidth',2);
axis equal; grid on;

addpoints(h, 0, 0);
addpoints(h, x_k, y_k);
addpoints(h, x_h, y_h);
addpoints(h, x_b, y_b);

drawnow;