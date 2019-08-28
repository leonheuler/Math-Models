function s_poli6 = s_poli6(T,H,V0,V,A0,A)
%S_POLI6
%    S_POLI6 = S_POLI6(T,H,V0,V,A0,A)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    25-Jul-2019 20:37:05

t2 = T.^2;
t3 = 6.0.^T;
t4 = T.*8.0;
t5 = t3-t4;
t6 = 1.0./t5;
t7 = H.*t3.*1.28e2;
s_poli6 = [1.0./T.^6.*t6.*(H.*t3.*3.2e1+V.*t2.*4.0+V0.*t2.*5.6e1-H.*T.*1.28e2-A.*T.*t2+A0.*T.*t2.*1.1e1+T.*V.*t3-T.*V0.*t3.*1.1e1-A0.*t2.*t3.*2.0).*2.0;-1.0./T.^5.*t6.*(t7+V.*t2.*3.6e1+V0.*t2.*2.16e2-H.*T.*3.84e2-A.*T.*t2.*5.0+A0.*T.*t2.*4.7e1+T.*V.*t3.*3.0-T.*V0.*t3.*4.7e1-A0.*t2.*t3.*9.0);(1.0./T.^4.*t6.*(t7+V.*t2.*8.0e1+V0.*t2.*1.6e2-A.*T.*t2.*8.0+A0.*T.*t2.*4.8e1+T.*V.*t3.*2.0-T.*V0.*t3.*5.2e1-A0.*t2.*t3.*1.1e1))./2.0;1.0./T.^2.*t6.*(H.*-1.28e2-T.*V.*1.2e1+T.*V0.*3.2e1+A.*t2+A0.*t2.*5.0);A0./2.0;V0];
