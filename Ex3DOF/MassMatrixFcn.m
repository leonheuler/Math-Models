function ret = MassMatrixFcn(t,x)

global m1 m2 m3 L1 L2 L3 g

ret = M(t, g, x(1),x(2),x(3),m1,m2,m3,L1,L2,L3);


end

