function ret = Poli6H(T0, T1, Y0, Y1, V0, V1, A0, A1, H, t)
    
    T_H = (T1-T0)/2;

    A = [
          T0^6        T0^5         T0^4          T0^3          T0^2         T0              1;
          T1^6        T1^5         T1^4          T1^3          T1^2         T1              1;
          T_H^6       T_H^5        T_H^4         T_H^3         T_H^2        T_H             1;
          6*T0^5      5*T0^4       4*T0^3        3*T0^2        2*T0         1               0;
          6*T1^5      5*T1^4       4*T1^3        3*T1^2        2*T1         1               0;
          30*T0^4     20*T0^3      12*T0^2       6*T0          2            0               0;
          30*T1^4     20*T1^3      12*T1^2       6*T1          2            0               0;
          ];
    
    b = [Y0; Y1; Y0+H; V0; V1; A0; A1];
    
    a = A^(-1)*b;
    
    ret = a(1) * t^6 + a(2)*t^5 + a(3)*t^4 + a(4)*t^3 + a(5)*t^2 + a(6)*t + a(7);

end

