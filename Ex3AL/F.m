% Friction
function ret = F(qdot)
   
    brkwy_trq = [250; 250; 250;];        
    brkwy_vel = [1; 1; 1];   
    Col_trq = [200; 200; 200];
    visc_coef = [0.01; 0.01; 0.01];
    
    static_scale = sqrt(2*exp(1)).*(brkwy_trq-Col_trq);
    static_thr = sqrt(2).*brkwy_vel;                     % Velocity threshold for static torque
    Col_thr = brkwy_vel./10;    
    
    ret = visc_coef .* qdot ...
         + static_scale .* (qdot./static_thr.*exp(-(qdot./static_thr).^2)) ...
         + Col_trq .* tanh(qdot./Col_thr); 
end