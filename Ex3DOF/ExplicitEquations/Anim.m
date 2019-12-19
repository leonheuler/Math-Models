% % Animation
% fig2 = figure(2);
% clf('reset');
% h = animatedline('Marker','o','LineWidth',1.5);
% tr = animatedline();
% rCp = animatedline('Marker','o','MarkerSize',2,'Color','red');
% axis equal;
% 
% 
% for i=1:numel(q(:,1))-1
% 
%     addpoints(h, 0, 0);
%     addpoints(h, X1(i), Y1(i));
%     addpoints(h, X2(i), Y2(i));
%     addpoints(h, X3(i), Y3(i));
%     addpoints(tr, X3(i), Y3(i));
% 
%     rC = (m1*[X1(i);Y1(i)] + m2*[X2(i);Y2(i)] + m3*[X3(i);Y3(i)]) / (m1 + m2 + m3);
%     addpoints(rCp, rC(1), rC(2));
%     
%     drawnow;
% %     pause(0.05);
%     clearpoints(h);
% end
% 
% last = numel(q(:,1));
% addpoints(h, 0, 0);
% addpoints(h, X1(last), Y1(last));
% addpoints(h, X2(last), Y2(last));
% addpoints(h, X3(last), Y3(last));