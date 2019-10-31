function[]=DrawSpring3D(A,B,w,r )
    % normalize vector AB
    n1=(B-A)/norm(B-A);
    a=A+0.75*n1;
    b=B-0.75*n1;
    t=linspace(0,1,500);
    h=[t*norm(b-a);r*cos(2*pi*w*t);r*sin(2*pi*w*t)];
    n2=[-n1(2);n1(1);0];
    n3=cross(n1,n2);
    M=[n1,n2,n3]; % rotation matrix
    P=M*h;
    plot3([A(1),a(1)],[A(2),a(2)],[A(3),a(3)],'k-','linewidth',1)
    plot3([a(1),a(1)+P(1,1)],[a(2),a(2)+P(2,1)],[a(3),a(3)+P(3,1)],'k-','linewidth',1)
    plot3([a(1)+P(1,end),b(1)],[a(2)+P(2,end),b(2)],[a(3)+P(3,end),b(3)],'k-','linewidth',1)
    plot3([b(1),B(1)],[b(2),B(2)],[b(3),B(3)],'k-','linewidth',1)
    plot3([a(1)+P(1,:)],[a(2)+P(2,:)],[a(3)+P(3,:)],'k','linewidth',1)
end

