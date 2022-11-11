function E = energy_arm(in1,in2)
%ENERGY_ARM
%    E = ENERGY_ARM(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 9.0.
%    10-Nov-2022 20:41:44

I1 = in2(5,:);
I2 = in2(6,:);
I3 = in2(7,:);
I4 = in2(8,:);
Ir = in2(9,:);
N = in2(10,:);
dth1 = in1(3,:);
dth2 = in1(4,:);
g = in2(19,:);
l_AC = in2(17,:);
l_A_m3 = in2(13,:);
l_B_m2 = in2(12,:);
l_C_m4 = in2(14,:);
l_OA = in2(15,:);
l_OB = in2(16,:);
l_O_m1 = in2(11,:);
m1 = in2(1,:);
m2 = in2(2,:);
m3 = in2(3,:);
m4 = in2(4,:);
th1 = in1(1,:);
th2 = in1(2,:);
t2 = cos(th1);
t3 = cos(th2);
t4 = th1+th2;
t5 = N.^2;
t6 = dth1.^2;
t7 = dth2.^2;
t8 = l_AC.^2;
t9 = l_A_m3.^2;
t10 = l_B_m2.^2;
t11 = l_OA.^2;
t12 = l_OA.*t2;
t13 = cos(t4);
et1 = (I1.*t6)./2.0+(I2.*t6)./2.0+(I2.*t7)./2.0+(I3.*t6)./2.0+(I3.*t7)./2.0+(I4.*t6)./2.0+(Ir.*t6)./2.0+(l_C_m4.^2.*m4.*t6)./2.0+(l_OB.^2.*m2.*t6)./2.0+(l_O_m1.^2.*m1.*t6)./2.0-g.*m2.*(l_B_m2.*t13+l_OB.*t2)-g.*m3.*(t12+l_A_m3.*t13)+I2.*dth1.*dth2+I3.*dth1.*dth2-g.*m4.*(t12+l_AC.*t13+l_C_m4.*t2)+(Ir.*t5.*t6)./2.0+(Ir.*t5.*t7)./2.0+(m2.*t6.*t10)./2.0+(m3.*t6.*t9)./2.0+(m4.*t6.*t8)./2.0+(m2.*t7.*t10)./2.0+(m3.*t7.*t9)./2.0+(m4.*t7.*t8)./2.0+(m3.*t6.*t11)./2.0+(m4.*t6.*t11)./2.0+dth1.*dth2.*m2.*t10+dth1.*dth2.*m3.*t9+dth1.*dth2.*m4.*t8;
et2 = -g.*l_O_m1.*m1.*t2+l_C_m4.*l_OA.*m4.*t6+Ir.*N.*dth1.*dth2+l_AC.*l_C_m4.*m4.*t3.*t6+l_AC.*l_OA.*m4.*t3.*t6+l_A_m3.*l_OA.*m3.*t3.*t6+l_B_m2.*l_OB.*m2.*t3.*t6+dth1.*dth2.*l_AC.*l_C_m4.*m4.*t3+dth1.*dth2.*l_AC.*l_OA.*m4.*t3+dth1.*dth2.*l_A_m3.*l_OA.*m3.*t3+dth1.*dth2.*l_B_m2.*l_OB.*m2.*t3;
E = et1+et2;
