function EG()

    tic


    function ret = Rx(a)
        ret = [1,    0,        0;
             0,   cos(a), -sin(a);
             0,   sin(a),  cos(a)];
    end

    function ret = Ry(a)
        ret = [cos(a),    0,        sin(a);
             0,         1,        0;
             -sin(a),   0,        cos(a)];
    end

    function ret = Rz(a)
        ret = [cos(a),    -sin(a),        0;
               sin(a),     cos(a),        0;
                    0,          0,        1];
    end

    L0 = 0.443;
    L1 = 0.270;
    L3 = 0.42;
    L5 = 0.475;
    L7 = 0.227;
    L9 = 0.16;
    
    L2 = L1;
    L4 = L3;
    L6 = L5;
    L8 = L7;
    L10 = L9;

  
    

M = [ 15;   % M(1)  %% спина
      3;    % M(2)  
      3;    % M(3)
      12;   % M(4)  %% бедро
      12;   % M(5)  %% бедро
      7;    % M(6)  %% голень
      7;    % M(7)  %% голень

     ];
  

   q = sym('q', [12, 1]);assume(q,'real');
    % положение локальной системы координат
% q(7) q(8) q(9)

    % обобщенные координаты правой ноги
% q(1) 
% q(3) 
% q(5) 



    % обобщенные координаты левой ноги
% q(2) 
% q(4) 
% q(6) 


    % наклон спины
% q(10) 
    % манипулятор на правой 
% q(11)
    % манипулятор на левой
% q(12)
    % при q(11)=q(12) - наклон спины

    % векторы левой стороны
    
    
    
    r0 = [q(7) q(8) q(9)]';
    
%     r0 = Rz(
    
    r01= Ry(q(10))*[L0/2 0 0]';
    rO = sign(q(10))*([L0/2 0 0]' - r01 );   %!
    r1 = [0 L1 0]';
    r3 = [0 0 -L3]';
    r5 = [0 0 -L5]';
    r7 = [0 L7 0]';
    r9 = [-L9 0 0]';

    r3_ = Ry(q(1))*( Rx(q(3))*r3 ); % / norm(Rz(q3)*[0 -L5 0]')% );
    r3 = L3*r3_/norm(r3_);
    
    r5_ = Ry(q(1))*( Rx(q(5))*r5 );
    r5 = L5*r5_/norm(r5_);
    
    % векторы правой стороны
    
    r02 = Ry(q(10))*[-L0/2 0 0]';
    r2 = [0 L2 0]';
    r4 = [0 0 -L4]';
    r6 = [0 0 -L6]';
    r8 = [0 L8 0]';
    r10 = [L10 0 0]';
    
    r4_ = Ry(q(2))*( Rx(q(4))*r4 ); % / norm(Rz(q3)*[0 -L5 0]')% );
    r4 = L4*r4_/norm(r4_);
    
    r6_ = Ry(q(2))*( Rx(q(6))*r6 );
    r6 = L6*r6_/norm(r6_); 

        
    
    O = r0 + rO + [0 L1-L1*cos(q(11)) L1*sin(-q(11))]';         %!
   
    O1 = O + r01;
    O3 = O1 + r1;
    O5 = O3 + r3;
    O7 = O5 + r5;  
    O9 = O7 + r7;
    O11 = O7 + r9;
  
    
    O2 = O + r02;
    O4 = O2 + r2;
    O6 = O4 + r4;
    O8 = O6 + r6;  % O8 = [q(7) q(8) q(9)]' + sign(q(10))*([L0/2 0 0]' - r01 ) + [0 L1-L1*cos(q(11)) L1*sin(-q(11))]' + Ry(q(10))*[-L0/2 0 0]' + r2 + r4 + r6
    O10 = O8 + r8;
    O12 = O8 + r10;

    C0 = (O2+O1)/2;
    C1 = (O3+O1)/2;
    C2 = (O4+O2)/2;
    C3 = (O5+O3)/2;
    C4 = (O6+O4)/2;
    C5 = (O7+O5)/2;
    C6 = (O8+O6)/2;
    
    C = (C0*M(1) + C1*M(2) + C2*M(3) + C3*M(4) + C4*M(5) + C5*M(6) + C6*M(7))/sum(M);
    C = simplify(C);
    
    C7 = O7;
    C8 = O8;                                                                       
    
%     q = [ q1, q2, q3, q4, q5, q6,  x, y, z ]';
    
    
    F = [C; C7; C8];
    F = simplify(F);
    J = jacobian(F, q);
    J = simplify(J);
    
    
    
    matlabFunction(F,'File','f','Vars', {q});
    matlabFunction(J,'File','jac','Vars', {q});

    toc
    
end





