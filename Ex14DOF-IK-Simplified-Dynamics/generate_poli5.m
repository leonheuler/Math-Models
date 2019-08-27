    

syms M 
syms T L V0 V A0 A
assume(T, 'real');
assume(L, 'real');
assume(V0, 'real');
assume(V, 'real');
assume(A0, 'real');
assume(A, 'real');

    M = [
           T^5         T^4          T^3          T^2         T  ;
           0            0           0           0           1 ;
           5*T^4       4*T^3        3*T^2        2*T         1  ;
            0            0           0           2            0;
           20*T^3      12*T^2       6*T          2            0  ;        
          ];
    
    b = [L; V0; V; A0; A];
   
    x = M^(-1)*b;
    
    x = simplify(x);
    s_poli5 = x;
    matlabFunction(s_poli5,'File','s_poli5','Vars', {T, L, V0, V, A0, A});