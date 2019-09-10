    

syms M 
syms T H V0 V A0 A
assume(T, 'real');
assume(H, 'real');
assume(V0, 'real');
assume(V, 'real');
assume(A0, 'real');
assume(A, 'real');

    M = [
           (T/2)^6         (T/2)^5          (T/2)^4          (T/2)^3        (T/2)^2    (T/2)  ;
           0                0               0                 0             0          1 ;
           6*T^5            5*T^4           4*T^3             3*T^2         2*T        1 ;
            0               0               0                 0             2          0;
           30*T^4           20*T^3          12*T^2            6^T           2          0  ;
           T^6              T^5             T^4             T^3             T^2        T ;
          ];
    
    b = [H; V0; V; A0; A; 0];
   
    x = M^(-1)*b;
    
    x = simplify(x);
    s_poli6 = x;
    matlabFunction(s_poli6,'File','s_poli6','Vars', {T, H, V0, V, A0, A});