function trJ = trJ(in1)
%TRJ
%    TRJ = TRJ(IN1)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    03-Oct-2019 23:46:29

q1 = in1(1,:);
q2 = in1(2,:);
t2 = q1+q2;
t3 = sin(t2);
t4 = cos(t2);
t5 = t4./2.0;
trJ = reshape([t3.*(-1.0./2.0)-sin(q1)./2.0,t3.*(-1.0./2.0),t5+cos(q1)./2.0,t5],[2,2]);
