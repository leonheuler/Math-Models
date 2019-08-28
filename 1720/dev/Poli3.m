function ret = Poli3(t0, tk, X0, Xk, V0, Vk, t)

    TT = [t0^3    t0^2  t0  1;
         tk^3    tk^2  tk  1;
         3*t0^2  2*t0  1   0;
         3*tk^2  2*tk  1   0];

    X = [X0; Xk; V0; Vk];
    A = TT^(-1)*X;  

    ret = A(4)+A(3)*t+A(2)*t^2+A(1)*t^3;    
end

