% Anim
fig2 = figure(2);
clf('reset');
h = animatedline('LineWidth',2,'Marker','o');
axis equal; grid on;
tr = animatedline('Color','red');
x = x(:,1);
y = x(:,2);

trd = animatedline('Color','red','LineStyle','--');

for i = 1:length(t)
        
    r1 = [ L1*cos(q1(i)); 
           L1*sin(q1(i))];
    r2 = [ L1*cos(q1(i)) + L2*cos(q1(i) + q2(i));
           L1*sin(q1(i)) + L2*sin(q1(i) + q2(i))];

    addpoints(h, 0, 0);
    addpoints(h, r1(1), r1(2));
    addpoints(h, r2(1), r2(2));

    addpoints(tr, r2(1), r2(2));

    drawnow;

    pause(0.01);

    clearpoints(h);
        
end

addpoints(h, 0, 0);
addpoints(h, r1(1), r1(2));
addpoints(h, r2(1), r2(2));

drawnow;