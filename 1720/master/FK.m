function FK

%close all
clc
exo_color = [0.2 0.2 0.2];
plane_color = [0.4 0.4 0.4];

bodyR = animatedline('Marker', 'o', 'MarkerSize', 3, 'LineWidth', 1, 'Color', exo_color);
bodyL = animatedline('Marker', 'o', 'MarkerSize', 3, 'LineWidth', 1, 'Color', exo_color);
bodyMass = animatedline('Marker', 'o', 'MarkerSize', 1);
bodyPlanes = animatedline('Color', 'g', 'LineWidth', 1, 'Color', plane_color);
bodyPointO = animatedline('Marker', '.', 'MarkerSize', 1 );
bodyPointO7 = animatedline('Marker', '.', 'MarkerSize', 1);
bodyPointO8 = animatedline('Marker', '.', 'MarkerSize', 1);

% axis([-1,1,-1,1,-1,1]);
axis equal
xlabel('X');
ylabel('Y');
zlabel('Z');
grid on;

    function R = Rx(a)
        R = [1,    0,        0;
             0,   cos(a), -sin(a);
             0,   sin(a),  cos(a)];
    end

    function R = Ry(a)
        R = [cos(a),    0,        sin(a);
             0,         1,        0;
             -sin(a),   0,        cos(a)];
    end



    function ret = Rotm(v,q)
        
        ret = [cos(q)+(1-cos(q))*(v(1)^2), (1-cos(q))*v(1)*v(2)-sin(q)*v(3), (1-cos(q))*v(1)*v(3)+sin(q)*v(2);
                (1-cos(q))*v(2)*v(1)+sin(q)*v(3), cos(q)+(1-cos(q))*(v(2)^2), (1-cos(q))*v(2)*v(3)-sin(q)*v(1);
                (1-cos(q))*v(3)*v(1)-sin(q)*v(2),   (1-cos(q))*v(3)*v(2)+sin(q)*v(1), cos(q)+(1-cos(q))*(v(3)^2)];
    end

    function ret = Rz(a)
        ret = [cos(a),    -sin(a),        0;
               sin(a),     cos(a),        0;
                    0,          0,        1];
    end

M = [ 15;   % M(1)  %% спина
      3;    % M(2)  
      3;    % M(3)
      12;   % M(4)  %% бедро
      12;   % M(5)  %% бедро
      7;    % M(6)  %% голень
      7;    % M(7)  %% голень

     ];


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
    
 
    for i = -pi:+pi/12:pi
    
    q1 = i;
    q3 = +8 * (pi/180);
    q5 = q3 + 0;
    
    q2 = 0;
    q4 = +8 * (pi/180);
    q6 = q3 + 0;
    
    q7 = 0;
    q8 = 0;
    q9 = 0;
    
    q10 = 0;
    q11 = -pi/12;
    q12 = -pi/12;
    
    q13 = 0;
    
    r0 = [q7 q8 q9]';

    
    r01= Ry(q10)*[L0/2 0 0]';
    rO = sign(q10)*([L0/2 0 0]' - r01 );
    
    r1 = Rx(q11)*[0 L1 0]';
    r3 = [0 0 -L3]';
    r5 = [0 0 -L5]';
    r7 = [0 L7 0]';
    r9 = [-L9 0 0]';
    
%     r3_ = Ry(q1)*( Rx(q3)*r3 ); 
%     r3 = L3*r3_/norm(r3_);
% 
%     r5_ = Ry(q1)*( Rx(q5)*r5 );
%     r5 = L5*r5_/norm(r5_);
    
    r3_ = Rotm(r1,q1)*( Rx(q3)*r3 ); 
    r3 = L3*r3_/norm(r3_);
    
    r5_ = Rotm(r1,q1)*( Rx(q5)*r5 );
    r5 = L5*r5_/norm(r5_);

    
    r02 = Ry(q10)*[-L0/2 0 0]';  
%     r02 = -r01;
    r2 = Rx(q12)*[0 L2 0]';
    r4 = [0 0 -L4]';
    r6 = [0 0 -L6]';
    r8 = [0 L8 0]';
    r10 = [L10 0 0]';
    
%     r4_ = Ry(q2)*( Rx(q4)*r4 ); % / norm(Rz(q3)*[0 -L5 0]')% );
%     r4 = L4*r4_/norm(r4_);
%     
%     r6_ = Ry(q2)*( Rx(q6)*r6 );
%     r6 = L6*r6_/norm(r6_); 

    r4_ = Rotm(r2, q2)*( Rx(q4)*r4 ); % / norm(Rz(q3)*[0 -L5 0]')% );
    r4 = L4*r4_/norm(r4_);
    
    r6_ = Rotm(r2, q2)*( Rx(q6)*r6 );
    r6 = L6*r6_/norm(r6_);    
    

    
    %O = r0 + [ 0 0 0]';
    O = r0 + rO + [0 L1-L1*cos(q11) L1*sin(-q11)]';
    O1 = O + r01;
    O3 = O1 + r1;
    O5 = O3 + r3;
    O7 = O5 + r5;  
    O9 = O7 + r7;
    O11 = O7 + r9;
    O13 = O11 + r7;    %% anim
    
    O2 = O + r02;
    O4 = O2 + r2;
    O6 = O4 + r4;
    O8 = O6 + r6;  
    O10 = O8 + r8;
    O12 = O8 + r10;
    O14 = O12 + r8;
    
    C0 = (O2+O1)/2;
    C1 = (O3+O1)/2;
    C2 = (O4+O2)/2;
    C3 = (O5+O3)/2;
    C4 = (O6+O4)/2;
    C5 = (O7+O5)/2;
    C6 = (O8+O6)/2;
    
    C = (C0*M(1) + C1*M(2) + C2*M(3) + C3*M(4) + C4*M(5) + C5*M(6) + C6*M(7))/sum(M);
    
    disp ( acos(  sum( (O7-O5).*(O5-O3) ) / ( norm(O7-O5)*norm(O5-O3)) ) * 180/pi );
    disp ( acos(  sum( (O8-O6).*(O6-O4) ) / ( norm(O8-O6)*norm(O6-O4)) ) * 180/pi );
    
    disp ( 'lengths (R):');
    disp ( [norm(O3 - O1) norm(O5 - O3) norm(O7-O5) norm(O9-O7) norm(O11-O7)]);
    disp ( 'lengths (L):');
    disp ( [norm(O4 - O2) norm(O6 - O4) norm(O8-O6) norm(O10-O8) norm(O12-O8)]);
    disp ( 'lengths (back):');
    disp (norm(O2 - O1));
        
    disp ( 'step length:');
    disp ( abs(O8(2)-O7(2)) );
    disp ( 'step heignt:');    
    disp ( abs(O8(3)-O7(3)) );
    

    %%
    
    
    
    addpoints(bodyR, O(1), O(2), O(3));
    addpoints(bodyR, O1(1), O1(2), O1(3));
    addpoints(bodyR, O3(1), O3(2), O3(3));
    addpoints(bodyR, O5(1), O5(2), O5(3));
    addpoints(bodyR, O7(1), O7(2), O7(3));
    addpoints(bodyR, O9(1), O9(2), O9(3));
    addpoints(bodyR, O13(1), O13(2), O13(3));
    addpoints(bodyR, O11(1), O11(2), O11(3));
    addpoints(bodyR, O7(1), O7(2), O7(3));
    addpoints(bodyR, O5(1), O5(2), O5(3));
    addpoints(bodyR, O3(1), O3(2), O3(3));
    addpoints(bodyR, O1(1), O1(2), O1(3));
    addpoints(bodyR, O(1), O(2), O(3));
    
    
    addpoints(bodyL, O(1), O(2), O(3));
    addpoints(bodyL, O2(1), O2(2), O2(3));
    addpoints(bodyL, O4(1), O4(2), O4(3));
    addpoints(bodyL, O6(1), O6(2), O6(3));
    addpoints(bodyL, O8(1), O8(2), O8(3));
    addpoints(bodyL, O10(1), O10(2), O10(3));
    addpoints(bodyL, O14(1), O14(2), O14(3));
    addpoints(bodyL, O12(1), O12(2), O12(3));
    addpoints(bodyL, O8(1), O8(2), O8(3));
    addpoints(bodyL, O6(1), O6(2), O6(3));
    addpoints(bodyL, O4(1), O4(2), O4(3));
    addpoints(bodyL, O2(1), O2(2), O2(3));
    addpoints(bodyL, O(1), O(2), O(3));

    addpoints(bodyMass, C(1), C(2), C(3));
%    
%    addpoints(bodyPointO7, O7(1), O7(2), O7(3));
%    addpoints(bodyPointO8, O8(1), O8(2), O8(3));
%    addpoints(bodyPointO, O(1), O(2), O(3));
   
   addpoints(bodyPlanes, O7(1), O7(2), O7(3));
   addpoints(bodyPlanes, O1(1), O1(2), O1(3));
   addpoints(bodyPlanes, O2(1), O2(2), O2(3));
   addpoints(bodyPlanes, O8(1), O8(2), O8(3));
   addpoints(bodyPlanes, O2(1), O2(2), O2(3));
   addpoints(bodyPlanes, O1(1), O1(2), O1(3));
   addpoints(bodyPlanes, O7(1), O7(2), O7(3));

   
    drawnow;
    


    end
    
    
    



end
