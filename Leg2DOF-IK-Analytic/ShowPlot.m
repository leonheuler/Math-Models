function  ShowPlot(command)
    
    global dt
    global Data_T 
    global Data_XC Data_YC
    global Data_XD Data_YD
    global Data_VX Data_VY
    global Data_V Data_dV
    global Data_phi3
    global Data_phi2

    
    %% V_A/B
    dData_XC = diff(Data_XC) / dt;
    dData_YC = diff(Data_YC) / dt;

    dData_XD = diff(Data_XD) / dt;
    dData_YD = diff(Data_YD) / dt;

    dData_A = (dData_XC.^2 + dData_YC.^2).^(1/2);
    dData_B = (dData_XD.^2 + dData_YD.^2).^(1/2);
    %% A
    ddData_XC = diff(dData_XC)/dt;
    ddData_XD = diff(dData_XD)/dt;
    ddData_YC = diff(dData_YC)/dt;
    ddData_YD = diff(dData_YD)/dt;
    ddData_A = (ddData_XC.^2 + ddData_YC.^2).^(1/2);
    ddData_B = (ddData_XD.^2 + ddData_YD.^2).^(1/2);
    
    %% Err
    Data_ErrX = Data_XD - Data_XC;
    Data_ErrY = Data_YD - Data_YC;
    Data_Err = (Data_ErrX.^2 + Data_ErrY.^2).^(1/2); 
    %% sigmaErr
    dErrX = (1./Data_T).*(cumtrapz(Data_T, Data_ErrX.^2));
    dErrY = (1./Data_T).*(cumtrapz(Data_T, Data_ErrY.^2));
    dErr = (dErrX.^2 + dErrY.^2).^(1/2);
    %%  dQ
    dData_phi3 = diff(Data_phi3) / dt;
    dData_phi2 = diff(Data_phi2) / dt;

    %% ddQ
    ddData_phi3 = diff(dData_phi3) / dt;
    ddData_phi2 = diff(dData_phi2) / dt;

    %%
    switch command 
        case 'P'
            %%
            %-----------------------------------------Position
            fig3 = figure(3);
            clf('reset');
            fig3.Name = 'Траектории';
            fig3.Position = [  0 500 550 500 ];

            % x(t)
            subplot(3,1,1);
            plot(Data_T, Data_XC);
            hold on;
            plot(Data_T, Data_XD);
            legend('X_A(t)','X_B(t)');
            grid on;
            xlabel('t, sec'); ylabel('X_A/X_B, m');
            title('Траектории');

            % y(t)
            subplot(3,1,2);
            plot(Data_T, Data_YC);
            hold on;
            plot(Data_T, Data_YD);
            legend('Y_A(t)','Y_B(t)');
            grid on;
            xlabel('t, sec'); ylabel('Y_A/Y_B, m');

            % y(x)
            subplot(3,1,3);
            plot(Data_XC, Data_YC);
            hold on;
            plot(Data_XD, Data_YD);
            legend('Y_A(X_A)', 'Y_B(X_B)');
            grid on;
            xlabel('X_A/X_B, m'); ylabel('Y_A/Y_B, m');
        
        case 'V'
            %%
             %-----------------------------------------Velocities
            fig4 = figure(4); 
            clf('reset');
            fig4.Name = 'Скорости';
            fig4.Position = [ 500 500 550 500 ];

            % v_x(t)
            subplot(3,1,1);
            plot(Data_T(:,1:length(dData_XC)), dData_XC);
            hold on;
            plot(Data_T(:,1:length(dData_XD)), dData_XD);
            legend('V_X_A(t)', 'V_X_B(t)');
            ylabel('V_X_A/V_X_B, m/s'); xlabel('t, sec');
            grid on;
            title('Скорости');

            % v_y(t)
            subplot(3,1,2);
            plot(Data_T(:,1:length(dData_YC)), dData_YC);
            hold on;
            plot(Data_T(:,1:length(dData_YD)), dData_YD);
            legend('V_Y_A(t)', 'V_Y_B(t)');
            ylabel('V_Y_A/V_Y_B, m/s'); xlabel('t, sec');
            grid on;

            % v_y(v_x)
            subplot(3,1,3);
            plot(Data_T(:,1:length(dData_A)), dData_A);
            hold on;
            plot(Data_T(:,1:length(dData_B)), dData_B);
            legend('V_A(t)', 'V_B(t)');
            ylabel('V_A/V_B, m/s'); xlabel('t, sec');
            grid on;
        case 'A'
            %%
             %-----------------------------------------Accelerations
            fig5 = figure(5); 
            clf('reset');
            fig5.Name = 'Ускорения';
            fig5.Position = [  1000 500 550 500 ];

            % a_x(t)
            subplot(3,1,1);
            plot(Data_T(:,1:length(ddData_XC)), ddData_XC);
            hold on;
            plot(Data_T(:,1:length(ddData_XD)), ddData_XD);
            legend('a_X_A(t)', 'a_X_B(t)');
            ylabel('a_X_A/a_X_B, m/s^2'); xlabel('t, sec');
            grid on;
            title('Ускорения');

            % a_y(t)
            subplot(3,1,2);
            plot(Data_T(:,1:length(ddData_YC)), ddData_YC);
            hold on;
            plot(Data_T(:,1:length(ddData_YD)), ddData_YD);
            legend('a_Y_A(t)', 'a_Y_B(t)');
            ylabel('a_Y_A/a_Y_B, m/s^2'); xlabel('t, sec');
            grid on;

            % a_y(a_x)
            subplot(3,1,3);
            plot(Data_T(:,1:length(ddData_A)), ddData_A);
            hold on;
            plot(Data_T(:,1:length(ddData_B)), ddData_B);
            legend('a_A(t)', 'a_B(t)');
            ylabel('a_A/a_B, m/s^2'); xlabel('t, sec');
            grid on;

        case 'F'
            %%
            %-----------------------------------------Force
            fig2 = figure(2); 
            clf('reset');
            fig2.Name = 'Сила V';
            fig2.Position = [ 20 300 550 500 ];

            % p_x(t)
            subplot(3,1,1);
            plot(Data_T, Data_VX);
            grid on;
            legend('V_X(t)');
            xlabel('t, sec'); ylabel('V_X, H');
            title('Сила V');

            % p_y(t)
            subplot(3,1,2);
            plot(Data_T, Data_VY);
            grid on;
            legend('V_Y(t)');
            xlabel('t, sec'); ylabel('V_Y, H');

            % |p|(t)
            subplot(3,1,3);
            plot(Data_T, Data_V);
            hold on;
            plot(Data_T(:,1:length(Data_dV)), Data_dV);
            grid on;
            legend('V(t)', 'dV(t)');
            xlabel('t, sec'); ylabel('V, H,   V'', H/s');
            
        case 'E'
            %%
            %-----------------------------------------Error
            fig6 = figure(6); 
            clf('reset');
            fig6.Name = 'Ошибка';
            fig6.Position = [ 570 300 550 500 ];

            subplot(3,1,1);
            plot(Data_T, Data_ErrX);
            grid on;
            title('Ошибка');
            ylabel('Err_X, m'); xlabel('t, sec');

            subplot(3,1,2);
            plot(Data_T, Data_ErrY);
            grid on;
            ylabel('Err_Y, m'); xlabel('t, sec');

            subplot(3,1,3);
            plot(Data_T, Data_Err);
            hold on;
            plot(Data_T, cumtrapz(Data_T, Data_Err));
            grid on;
            ylabel('Err, m'); xlabel('t, sec');
            legend('Err', 'I_E_r_r');
            disp(trapz(Data_T, Data_Err));


        case 'sigmaE'
            %%
            %-----------------------------------------SigmaError
            fig7 = figure(7); 
            clf('reset');
            fig7.Name = 'Ошибка2';
            fig7.Position = [ 1120 300 550 500 ];

            subplot(3,1,1);
            plot(Data_T, dErrX);
            grid on;
            title('SigmaErr');
            ylabel('Err_X'); xlabel('t, sec');

            subplot(3,1,2);
            plot(Data_T, dErrY);
            grid on;
            ylabel('Err_Y'); xlabel('t, sec');

            subplot(3,1,3);
            plot(Data_T, dErr);
            grid on;
            ylabel('Err'); xlabel('t, sec');
        case 'Q'
            %%
             %-----------------------------------------q1---q2---
            fig8 = figure(8); 
            clf('reset');
            fig8.Name = 'q1 q2';
            fig8.Position = [ 0 50 500 400 ];

            subplot(2,1,1);
            plot(Data_T, Data_phi3.*(180/pi));
            grid on;
            ylabel('q1, Deg'); xlabel('t, sec');

            subplot(2,1,2);
            plot(Data_T, Data_phi2.*(180/pi));
            grid on;
            ylabel('q2, Deg'); xlabel('t, sec');

        case 'W'
            %%
             %-----------------------------------------w1---w2---
            fig9 = figure(9); 
            clf('reset');
            fig9.Name = 'w1 w2';
            fig9.Position = [ 500 50 500 400 ];

            subplot(2,1,1);
            plot(Data_T(:,1:length(dData_phi3)), dData_phi3);
            grid on;
            ylabel('w1, Rad/s'); xlabel('t, sec');

            subplot(2,1,2);
            plot(Data_T(:,1:length(dData_phi2)), dData_phi2);
            grid on;
            ylabel('w2, Rad/s'); xlabel('t, sec');
            
        case 'dW'
            %%
             %-----------------------------------------w1---w2---
            fig10 = figure(10); 
            clf('reset');
            fig10.Name = 'dW1 dW2';
            fig10.Position = [ 1000 50 500 400 ];

            subplot(2,1,1);
            plot(Data_T(:,1:length(ddData_phi3)), ddData_phi3);
            grid on;
            ylabel('dW1, Rad/s^2'); xlabel('t, sec');

            subplot(2,1,2);
            plot(Data_T(:,1:length(ddData_phi2)), ddData_phi2);
            grid on;
            ylabel('dW2, Rad/s^2'); xlabel('t, sec');
            
     end
   %%
    

end

