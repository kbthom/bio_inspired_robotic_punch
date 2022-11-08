function J = jacobian_foot(in1,in2)
%JACOBIAN_FOOT
%    J = JACOBIAN_FOOT(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 9.0.
%    08-Nov-2022 14:48:48

l_AC = in2(17,:);
l_DE = in2(18,:);
l_OB = in2(16,:);
th1 = in1(1,:);
th2 = in1(2,:);
t2 = cos(th1);
t3 = sin(th1);
t4 = th1+th2;
t5 = cos(t4);
t6 = sin(t4);
t7 = l_AC.*t5;
t8 = l_AC.*t6;
J = reshape([t7+l_DE.*t2+l_OB.*t2,t8+l_DE.*t3+l_OB.*t3,t7,t8],[2,2]);
