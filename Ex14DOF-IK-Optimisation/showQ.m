function showQ() 
    
    global Data_t Data_q
    
    fig2 = figure(2);
    fig2.Position = [ 600 50 700 700 ];
    clf('reset');
    subplot(3,3,1);
    plot(Data_t, Data_q(:,1).*(180/pi));
    grid on, title('q1(LH)');

    subplot(3,3,2);
    plot(Data_t, Data_q(:,2).*(180/pi));
    grid on, title('q2(LH_Z)');

    subplot(3,3,3);
    plot(Data_t, Data_q(:,3).*(180/pi));
    grid on, title('q3(LH_Y)');


    subplot(3,3,4);
    plot(Data_t, Data_q(:,4).*(180/pi));
    grid on, title('q4(LK)');

    subplot(3,3,5);
    plot(Data_t, Data_q(:,5).*(180/pi));
    grid on, title('q5(RH)');

    subplot(3,3,6);
    plot(Data_t, Data_q(:,6).*(180/pi));
    grid on, title('q6(RH_Z)');

    subplot(3,3,7);
    plot(Data_t, Data_q(:,7).*(180/pi));
    grid on, title('q7(RH_Y)');

    subplot(3,3,8);
    plot(Data_t, Data_q(:,8).*(180/pi));
    grid on, title('q8(RK)');

    
    fig3 = figure(3);
    fig3.Position = [ 1200 50 700 700 ];
    clf('reset');
    subplot(2,3,1);
    plot(Data_t, Data_q(:,9));
    grid on, title('q9(x)');
    
    subplot(2,3,2);
    plot(Data_t, Data_q(:,10));
    grid on, title('q10(y)');
    
    subplot(2,3,3);
    plot(Data_t, Data_q(:,11));
    grid on, title('q10(z)');  
    
    subplot(2,3,4);
    plot(Data_t, Data_q(:,12).*(180/pi));
    grid on, title('q12(Rx)');

    subplot(2,3,5);
    plot(Data_t, Data_q(:,13).*(180/pi));
    grid on, title('q13(Ry)');

    subplot(2,3,6);
    plot(Data_t, Data_q(:,14).*(180/pi));
    grid on, title('q14(Rz)');    
    
end