function  ShowPlot(command)
    
    global dt
    global Data_T 
    global Data_XK Data_YK
    global Data_XA Data_YA
    global Data_r Data_r_dt
    global Data_q1
    global Data_q2

    
    %% V_A/B
    dData_XK = diff(Data_XK) / dt;
    dData_YK = diff(Data_YK) / dt;

    dData_XA = diff(Data_XA) / dt;
    dData_YA = diff(Data_YA) / dt;

    dData_B = (dData_XK.^2 + dData_YK.^2).^(1/2);
    dData_C = (dData_XA.^2 + dData_YA.^2).^(1/2);
    %% A
    ddData_XK = diff(dData_XK)/dt;
    ddData_YK = diff(dData_YK)/dt;
    ddData_XA = diff(dData_XA)/dt;
    ddData_YA = diff(dData_YA)/dt;
    ddData_B = (ddData_XK.^2 + ddData_YK.^2).^(1/2);
    ddData_C = (ddData_XA.^2 + ddData_YA.^2).^(1/2);
    
    %% Err
    Data_ErrX = Data_XA - Data_XK;
    Data_ErrY = Data_YA - Data_YK;
    Data_Err = (Data_ErrX.^2 + Data_ErrY.^2).^(1/2); 
    %% sigmaErr
    dErrX = (1./Data_T).*(cumtrapz(Data_T, Data_ErrX.^2));
    dErrY = (1./Data_T).*(cumtrapz(Data_T, Data_ErrY.^2));
    dErr = (dErrX.^2 + dErrY.^2).^(1/2);
    %%  dQ
    dData_q1 = diff(Data_q1) / dt;
    dData_q2 = diff(Data_q2) / dt;

    %% ddQ
    ddData_q1 = diff(dData_q1) / dt;
    ddData_q2 = diff(dData_q2) / dt;

    %%
    switch command 
        case 'P'
            %%
            %-----------------------------------------Position
            fig3 = figure(3);
            clf('reset');
            fig3.Name = '����������';
            fig3.Position = [  0 500 550 500 ];

            % x(t)
            subplot(3,1,1);
            plot(Data_T, Data_XK);
            hold on;
            plot(Data_T, Data_XA);
            legend('X_B(t)','X_C(t)');
            grid on;
            xlabel('t, sec'); ylabel('X_B/X_C, m');
            title('����������');

            % y(t)
            subplot(3,1,2);
            plot(Data_T, Data_YK);
            hold on;
            plot(Data_T, Data_YA);
            legend('Y_B(t)','Y_C(t)');
            grid on;
            xlabel('t, sec'); ylabel('Y_B/Y_C, m');

            % y(x)
            subplot(3,1,3);
            plot(Data_XK, Data_YK);
            hold on;
            plot(Data_XA, Data_YA);
            legend('Y_B(X_B)', 'Y_C(X_C)');
            grid on;
            xlabel('X_B/X_C, m'); ylabel('Y_B/Y_C, m');
        
        case 'V'
            %%
             %-----------------------------------------Velocities
            fig4 = figure(4); 
            clf('reset');
            fig4.Name = '��������';
            fig4.Position = [ 500 500 550 500 ];

            % v_x(t)
            subplot(3,1,1);
            plot(Data_T(:,1:length(dData_XK)), dData_XK);
            hold on;
            plot(Data_T(:,1:length(dData_XA)), dData_XA);
            legend('V_X_B(t)', 'V_X_C(t)');
            ylabel('V_X_B/V_X_C, m/s'); xlabel('t, sec');
            grid on;
            title('��������');

            % v_y(t)
            subplot(3,1,2);
            plot(Data_T(:,1:length(dData_YK)), dData_YK);
            hold on;
            plot(Data_T(:,1:length(dData_YA)), dData_YA);
            legend('V_Y_B(t)', 'V_Y_C(t)');
            ylabel('V_Y_B/V_Y_C, m/s'); xlabel('t, sec');
            grid on;

            % v_y(v_x)
            subplot(3,1,3);
            plot(Data_T(:,1:length(dData_B)), dData_B);
            hold on;
            plot(Data_T(:,1:length(dData_C)), dData_C);
            legend('V_B(t)', 'V_C(t)');
            ylabel('V_B/V_C, m/s'); xlabel('t, sec');
            grid on;
        case 'A'
            %%
             %-----------------------------------------Accelerations
            fig5 = figure(5); 
            clf('reset');
            fig5.Name = '���������';
            fig5.Position = [  1000 500 550 500 ];

            % a_x(t)
            subplot(3,1,1);
            plot(Data_T(:,1:length(ddData_XK)), ddData_XK);
            hold on;
            plot(Data_T(:,1:length(ddData_XA)), ddData_XA);
            legend('a_X_A(t)', 'a_X_B(t)');
            ylabel('a_X_A/a_X_B, m/s^2'); xlabel('t, sec');
            grid on;
            title('���������');

            % a_y(t)
            subplot(3,1,2);
            plot(Data_T(:,1:length(ddData_YK)), ddData_YK);
            hold on;
            plot(Data_T(:,1:length(ddData_YA)), ddData_YA);
            legend('a_Y_A(t)', 'a_Y_B(t)');
            ylabel('a_Y_A/a_Y_B, m/s^2'); xlabel('t, sec');
            grid on;

            % a_y(a_x)
            subplot(3,1,3);
            plot(Data_T(:,1:length(ddData_B)), ddData_B);
            hold on;
            plot(Data_T(:,1:length(ddData_B)), ddData_C);
            legend('a_B(t)', 'a_C(t)');
            ylabel('a_B/a_C, m/s^2'); xlabel('t, sec');
            grid on;

            
        case 'E'
            %%
            %-----------------------------------------Error
            fig6 = figure(6); 
            clf('reset');
            fig6.Name = '������';
            fig6.Position = [ 570 300 550 500 ];

            subplot(3,1,1);
            plot(Data_T, Data_ErrX);
            grid on;
            title('������');
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
            fig7.Name = '������2';
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
            plot(Data_T, Data_q1.*(180/pi));
            grid on;
            ylabel('q1, Deg'); xlabel('t, sec');

            subplot(2,1,2);
            plot(Data_T, Data_q2.*(180/pi));
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
            plot(Data_T(:,1:length(dData_q1)), dData_q1);
            grid on;
            ylabel('w1, Rad/s'); xlabel('t, sec');

            subplot(2,1,2);
            plot(Data_T(:,1:length(dData_q2)), dData_q2);
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
            plot(Data_T(:,1:length(ddData_q1)), ddData_q1);
            grid on;
            ylabel('dW1, Rad/s^2'); xlabel('t, sec');

            subplot(2,1,2);
            plot(Data_T(:,1:length(ddData_q2)), ddData_q2);
            grid on;
            ylabel('dW2, Rad/s^2'); xlabel('t, sec');
            
     end
   %%
    

end

