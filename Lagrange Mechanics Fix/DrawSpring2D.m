function[]=DrawSpring2D(A,B,w,r )
    % normalize vector AB
    n1=(B-A)/norm(B-A);
    a=A+0.75*n1;
    b=B-0.75*n1;
    t=linspace(0,1,500);
    phase=-pi+(2*w+1)*pi*t;
    y=2*r*sin(phase);
    x=r+(norm(b-a)-2*r)*t+r*cos(phase);
    p=[x;y];
    n2=[-n1(2);n1(1)];
    M=[n1,n2]; % rotation matrix
    P=M*p;
    plot([A(1),a(1)+P(1,:),B(1)],[A(2),a(2)+P(2,:),B(2)],'k')
end

