fig4 = figure(4);
clf('reset');
h = animatedline('Marker','o','LineWidth',1.5);
tr = animatedline();
axis equal;

for i=1:numel(q(:,1))-1
    
    addpoints(h, 0, 0);
    addpoints(h, X1(i), Y1(i));
    addpoints(h, X2(i), Y2(i));
    addpoints(h, X3(i), Y3(i));
    addpoints(tr, X3(i), Y3(i));
    drawnow ;
%     pause(0.05);
    clearpoints(h);
end

addpoints(h, 0, 0);
addpoints(h, X1(numel(q(:,1))), Y1(numel(q(:,1))));
addpoints(h, X2(numel(q(:,1))), Y2(numel(q(:,1))));
addpoints(h, X3(numel(q(:,1))), Y3(numel(q(:,1))));