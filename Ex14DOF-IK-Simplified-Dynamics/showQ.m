function showQ() 
    
    global Data_t Data_q
    
    fig2 = figure(2);
    fig2.Position = [ 600 50 700 700 ];
    clf('reset');
    subplot(3,3,1);
    plot(Data_t, Data_q(:,1).*(180/pi));
    grid on, title('q1');

    subplot(3,3,2);
    plot(Data_t, Data_q(:,2).*(180/pi));
    grid on, title('q2(RK)');

    subplot(3,3,3);
    plot(Data_t, Data_q(:,3).*(180/pi));
    grid on, title('q3');


    subplot(3,3,4);
    plot(Data_t, Data_q(:,4).*(180/pi));
    grid on, title('q4');

    subplot(3,3,5);
    plot(Data_t, Data_q(:,5).*(180/pi));
    grid on, title('q5(RH)');

    subplot(3,3,6);
    plot(Data_t, Data_q(:,6).*(180/pi));
    grid on, title('q6(LH)');

    subplot(3,3,7);
    plot(Data_t, Data_q(:,7).*(180/pi));
    grid on, title('q7');

    subplot(3,3,8);
    plot(Data_t, Data_q(:,8).*(180/pi));
    grid on, title('q8');

    subplot(3,3,9);
    plot(Data_t, Data_q(:,9).*(180/pi));
    grid on, title('q9(LK)');
    
end