%%
% Animation
fig2 = figure(2);
clf('reset');
exo = animatedline('Marker','o','LineWidth',1.5);
tr = animatedline();
cp = animatedline('Marker','o','MarkerSize',1,'Color','red');
c1p = animatedline('Marker','o','Color','blue');
c2p = animatedline('Marker','o','Color','blue');
c3p = animatedline('Marker','o','Color','blue');
axis equal;

human = animatedline('Marker','o','LineWidth',1.5);

last = numel(q(:,1));
for i=1:last-1

    addpoints(exo, 0, 0);
    addpoints(exo, X1(i), Y1(i));
    addpoints(exo, X2(i), Y2(i));
    addpoints(exo, X3(i), Y3(i));
    addpoints(tr, X3(i), Y3(i));
    
    rC1 = [X1(i)/2;Y1(i)/2];
    rC2 = [(X1(i)+X2(i))/2; (Y1(i)+Y2(i))/2];
    rC3 = [(X2(i)+X3(i))/2; (Y2(i)+Y3(i))/2];
    rC = (m1*[X1(i)/2;Y1(i)/2] + m2*[(X2(i)+X1(i))/2;(Y2(i)+Y1(i))/2] + m3*[(X2(i)+X3(i))/2;(Y3(i)+Y2(i))/2]) / (m1 + m2 + m3);
    
    addpoints(c1p, rC1(1), rC1(2));
    addpoints(c2p, rC2(1), rC2(2));
    addpoints(c3p, rC3(1), rC3(2));
    addpoints(cp, rC(1), rC(2));
    
    drawnow limitrate;
    
    clearpoints(c1p);
    clearpoints(c2p);
    clearpoints(c3p);
    clearpoints(exo);
end
addpoints(exo, 0, 0);
addpoints(exo, X1(last), Y1(last));
addpoints(exo, X2(last), Y2(last));
addpoints(exo, X3(last), Y3(last));

