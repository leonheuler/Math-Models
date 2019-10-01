syms T q0 q1 w0 w1 e0 e1 

    A = [   1 0 0 0 0 0;
            0 1 0 0 0 0;
            0 0 2   0     0     0;
            1 T T^2 T^3   T^4   T^5;
            0 1 2*T 3*T^2 4*T^3  5*T^4;
            0 0 2   6*T   12*T^2 20*T^3];
        
        
x = A \ [q0, w0, e0, q1, w1, e1]';

ret = simplify(x);

% syms T q0 q1 w0 w1  
% 
%     A = [   1 0 0 0 ;
%             0 1 0 0 ;
%             1 T T^2 T^3 ;
%             0 1 2*T 3*T^2];
%         
%         
% ret = A \ [q0, w0, q1, w1]';