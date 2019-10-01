syms m1 m2 L1 L1 q1 q2 

M = [ (m1+m2)*L1^2 + m2*L2^2 + 2*m2*L1*L2*cos(q2),      m2*L2^2+m2*L1*L2*cos(q2);
        m2*L2^2+m2*L1*L2*cos(q2),                         m2*L2^2];

invM = inv(M);

invM = simplify(invM);







    

