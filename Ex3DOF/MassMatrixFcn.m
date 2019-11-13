function ret = MassMatrixFcn(t,x)

global m1 m2 m3 L1 L2 L3 g

% t1 = M(t, g, x(1),x(2),x(3),m1,m2,m3,L1,L2,L3);
% ret = M(t, g, x(1),x(2),x(3),m1,m2,m3,L1,L2,L3);
ret = cat(2, cat(1, M(t, g, x(1),x(2),x(3),m1,m2,m3,L1,L2,L3), zeros(3)), zeros(6,3));

end

