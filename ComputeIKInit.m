function qd = ComputeIKInit(xd,yd,zd,Ree,q_pre)
%q_pre is a column vector
%DH param
alpha = [pi/2,0,0,pi/2,-pi/2,0]; %[in rad]
a = [0,-0.425,-0.39225,0,0,0]; %[in m]
d = [0.08916,0,0,0.10915,0.09465,0.0823]; %[in m]

ee_pos_desired = [xd yd zd]; %desired position of the end effector
T60=[Ree ee_pos_desired';
    zeros(1,3) 1];
p50=T60*[0 0 -d(1,6) 1]'-[0 0 0 1]';
psi=atan2(p50(2,1),p50(1,1));
argacosphi=d(1,4)/(sqrt(p50(1,1)^2+p50(2,1)^2));
phi1=acos(max(min(argacosphi,1),-1)); %bound acos argument between -1 and 1
phi2=-acos(max(min(argacosphi,1),-1)); %bound acos argument between -1 and 1
Theta_11=psi+phi1+pi/2; %shoulder left
Theta_12=psi+phi2+pi/2; %shoulder right

p60=ee_pos_desired';
p61_z1=p60(1,1)*sin(Theta_11)-p60(2,1)*cos(Theta_11);
p61_z2=p60(1,1)*sin(Theta_12)-p60(2,1)*cos(Theta_12);
argacos51=(p61_z1-d(1,4))/d(1,6); %bound acos argument between -1 and 1
argacos52=(p61_z2-d(1,4))/d(1,6); %bound acos argument between -1 and 1
Theta_51=acos(max(min(argacos51,1),-1));
Theta_52=-acos(max(min(argacos51,1),-1));
Theta_53=acos(max(min(argacos52,1),-1));
Theta_54=-acos(max(min(argacos52,1),-1));

T10_1=[cos(Theta_11) -sin(Theta_11)*cos(alpha(1,1)) sin(Theta_11)*sin(alpha(1,1)) a(1,1)*cos(Theta_11);
    sin(Theta_11) cos(Theta_11)*cos(alpha(1,1)) -cos(Theta_11)*sin(alpha(1,1)) a(1,1)*sin(Theta_11);
    0 sin(alpha(1,1)) cos(alpha(1,1)) d(1,1);
    0 0 0 1];
T10_2=[cos(Theta_12) -sin(Theta_12)*cos(alpha(1,1)) sin(Theta_12)*sin(alpha(1,1)) a(1,1)*cos(Theta_12);
    sin(Theta_12) cos(Theta_12)*cos(alpha(1,1)) -cos(Theta_12)*sin(alpha(1,1)) a(1,1)*sin(Theta_12);
    0 sin(alpha(1,1)) cos(alpha(1,1)) d(1,1);
    0 0 0 1];

T16_1=inv(T60)*T10_1;
T16_2=inv(T60)*T10_2;
zy_1=T16_1(2,3);
zx_1=T16_1(1,3);
zy_2=T16_2(2,3);
zx_2=T16_2(1,3);
Theta_61=atan2(-zy_1/sin(Theta_51),zx_1/sin(Theta_51));
Theta_62=atan2(-zy_1/sin(Theta_52),zx_1/sin(Theta_52));
Theta_63=atan2(-zy_2/sin(Theta_53),zx_2/sin(Theta_53));
Theta_64=atan2(-zy_2/sin(Theta_54),zx_2/sin(Theta_54));


T61_1=inv(T16_1);
T61_2=inv(T16_2);
T54_1=[cos(Theta_51) -sin(Theta_51)*cos(alpha(1,5)) sin(Theta_51)*sin(alpha(1,5)) a(1,5)*cos(Theta_51);
    sin(Theta_51) cos(Theta_51)*cos(alpha(1,5)) -cos(Theta_51)*sin(alpha(1,5)) a(1,5)*sin(Theta_51);
    0 sin(alpha(1,5)) cos(alpha(1,5)) d(1,5);
    0 0 0 1];
T54_2=[cos(Theta_52) -sin(Theta_52)*cos(alpha(1,5)) sin(Theta_52)*sin(alpha(1,5)) a(1,5)*cos(Theta_52);
    sin(Theta_52) cos(Theta_52)*cos(alpha(1,5)) -cos(Theta_52)*sin(alpha(1,5)) a(1,5)*sin(Theta_52);
    0 sin(alpha(1,5)) cos(alpha(1,5)) d(1,5);
    0 0 0 1];
T54_3=[cos(Theta_53) -sin(Theta_53)*cos(alpha(1,5)) sin(Theta_53)*sin(alpha(1,5)) a(1,5)*cos(Theta_53);
    sin(Theta_53) cos(Theta_53)*cos(alpha(1,5)) -cos(Theta_53)*sin(alpha(1,5)) a(1,5)*sin(Theta_53);
    0 sin(alpha(1,5)) cos(alpha(1,5)) d(1,5);
    0 0 0 1];
T54_4=[cos(Theta_54) -sin(Theta_54)*cos(alpha(1,5)) sin(Theta_54)*sin(alpha(1,5)) a(1,5)*cos(Theta_54);
    sin(Theta_54) cos(Theta_54)*cos(alpha(1,5)) -cos(Theta_54)*sin(alpha(1,5)) a(1,5)*sin(Theta_54);
    0 sin(alpha(1,5)) cos(alpha(1,5)) d(1,5);
    0 0 0 1];

T65_1=[cos(Theta_61) -sin(Theta_61)*cos(alpha(1,6)) sin(Theta_61)*sin(alpha(1,6)) a(1,6)*cos(Theta_61);
    sin(Theta_61) cos(Theta_61)*cos(alpha(1,6)) -cos(Theta_61)*sin(alpha(1,6)) a(1,6)*sin(Theta_61);
    0 sin(alpha(1,6)) cos(alpha(1,6)) d(1,6);
    0 0 0 1];
T65_2=[cos(Theta_62) -sin(Theta_62)*cos(alpha(1,6)) sin(Theta_62)*sin(alpha(1,6)) a(1,6)*cos(Theta_62);
    sin(Theta_62) cos(Theta_62)*cos(alpha(1,6)) -cos(Theta_62)*sin(alpha(1,6)) a(1,6)*sin(Theta_62);
    0 sin(alpha(1,6)) cos(alpha(1,6)) d(1,6);
    0 0 0 1];
T65_3=[cos(Theta_63) -sin(Theta_63)*cos(alpha(1,6)) sin(Theta_63)*sin(alpha(1,6)) a(1,6)*cos(Theta_63);
    sin(Theta_63) cos(Theta_63)*cos(alpha(1,6)) -cos(Theta_63)*sin(alpha(1,6)) a(1,6)*sin(Theta_63);
    0 sin(alpha(1,6)) cos(alpha(1,6)) d(1,6);
    0 0 0 1];
T65_4=[cos(Theta_64) -sin(Theta_64)*cos(alpha(1,6)) sin(Theta_64)*sin(alpha(1,6)) a(1,6)*cos(Theta_64);
    sin(Theta_64) cos(Theta_64)*cos(alpha(1,6)) -cos(Theta_64)*sin(alpha(1,6)) a(1,6)*sin(Theta_64);
    0 sin(alpha(1,6)) cos(alpha(1,6)) d(1,6);
    0 0 0 1];



T41_1=T61_1/(T54_1*T65_1); %=T61*inv(T54*T65)
T41_2=T61_1/(T54_2*T65_2); %=T61*inv(T54*T65)
T41_3=T61_2/(T54_3*T65_3); %=T61*inv(T54*T65)
T41_4=T61_2/(T54_4*T65_4); %=T61*inv(T54*T65)


p31_1=T41_1*[0 -d(1,4) 0 1]'-[0 0 0 1]';
p31_2=T41_2*[0 -d(1,4) 0 1]'-[0 0 0 1]';
p31_3=T41_3*[0 -d(1,4) 0 1]'-[0 0 0 1]';
p31_4=T41_4*[0 -d(1,4) 0 1]'-[0 0 0 1]';

norm_p31_1=norm(p31_1);
norm_p31_2=norm(p31_2);
norm_p31_3=norm(p31_3);
norm_p31_4=norm(p31_4);

argacos3_1=(norm_p31_1^2-a(1,2)^2-a(1,3)^2)/(2*a(1,2)*a(1,3)); %bound acos argument between -1 and 1
argacos3_2=(norm_p31_2^2-a(1,2)^2-a(1,3)^2)/(2*a(1,2)*a(1,3)); %bound acos argument between -1 and 1
argacos3_3=(norm_p31_3^2-a(1,2)^2-a(1,3)^2)/(2*a(1,2)*a(1,3)); %bound acos argument between -1 and 1
argacos3_4=(norm_p31_4^2-a(1,2)^2-a(1,3)^2)/(2*a(1,2)*a(1,3)); %bound acos argument between -1 and 1

Theta_31=acos(max(min(argacos3_1,1),-1));
Theta_32=-acos(max(min(argacos3_1,1),-1));
Theta_33=acos(max(min(argacos3_2,1),-1));
Theta_34=-acos(max(min(argacos3_2,1),-1));
Theta_35=acos(max(min(argacos3_3,1),-1));
Theta_36=-acos(max(min(argacos3_3,1),-1));
Theta_37=acos(max(min(argacos3_4,1),-1));
Theta_38=-acos(max(min(argacos3_4,1),-1));

Theta_21=-atan2(p31_1(2,1),-p31_1(1,1))+asin((a(1,3)*sin(Theta_31))/norm_p31_1);
Theta_22=-atan2(p31_1(2,1),-p31_1(1,1))+asin((a(1,3)*sin(Theta_32))/norm_p31_1);
Theta_23=-atan2(p31_2(2,1),-p31_2(1,1))+asin((a(1,3)*sin(Theta_33))/norm_p31_2);
Theta_24=-atan2(p31_2(2,1),-p31_2(1,1))+asin((a(1,3)*sin(Theta_34))/norm_p31_2);
Theta_25=-atan2(p31_3(2,1),-p31_3(1,1))+asin((a(1,3)*sin(Theta_35))/norm_p31_3);
Theta_26=-atan2(p31_3(2,1),-p31_3(1,1))+asin((a(1,3)*sin(Theta_36))/norm_p31_3);
Theta_27=-atan2(p31_4(2,1),-p31_4(1,1))+asin((a(1,3)*sin(Theta_37))/norm_p31_4);
Theta_28=-atan2(p31_4(2,1),-p31_4(1,1))+asin((a(1,3)*sin(Theta_38))/norm_p31_4);

T21_1=[cos(Theta_21) -sin(Theta_21)*cos(alpha(1,2)) sin(Theta_21)*sin(alpha(1,2)) a(1,2)*cos(Theta_21);
    sin(Theta_21) cos(Theta_21)*cos(alpha(1,2)) -cos(Theta_21)*sin(alpha(1,2)) a(1,2)*sin(Theta_21);
    0 sin(alpha(1,2)) cos(alpha(1,2)) d(1,2);
    0 0 0 1];
T21_2=[cos(Theta_22) -sin(Theta_22)*cos(alpha(1,2)) sin(Theta_22)*sin(alpha(1,2)) a(1,2)*cos(Theta_22);
    sin(Theta_22) cos(Theta_22)*cos(alpha(1,2)) -cos(Theta_22)*sin(alpha(1,2)) a(1,2)*sin(Theta_22);
    0 sin(alpha(1,2)) cos(alpha(1,2)) d(1,2);
    0 0 0 1];
T21_3=[cos(Theta_23) -sin(Theta_23)*cos(alpha(1,2)) sin(Theta_23)*sin(alpha(1,2)) a(1,2)*cos(Theta_23);
    sin(Theta_23) cos(Theta_23)*cos(alpha(1,2)) -cos(Theta_23)*sin(alpha(1,2)) a(1,2)*sin(Theta_23);
    0 sin(alpha(1,2)) cos(alpha(1,2)) d(1,2);
    0 0 0 1];
T21_4=[cos(Theta_24) -sin(Theta_24)*cos(alpha(1,2)) sin(Theta_24)*sin(alpha(1,2)) a(1,2)*cos(Theta_24);
    sin(Theta_24) cos(Theta_24)*cos(alpha(1,2)) -cos(Theta_24)*sin(alpha(1,2)) a(1,2)*sin(Theta_24);
    0 sin(alpha(1,2)) cos(alpha(1,2)) d(1,2);
    0 0 0 1];
T21_5=[cos(Theta_25) -sin(Theta_25)*cos(alpha(1,2)) sin(Theta_25)*sin(alpha(1,2)) a(1,2)*cos(Theta_25);
    sin(Theta_25) cos(Theta_25)*cos(alpha(1,2)) -cos(Theta_25)*sin(alpha(1,2)) a(1,2)*sin(Theta_25);
    0 sin(alpha(1,2)) cos(alpha(1,2)) d(1,2);
    0 0 0 1];
T21_6=[cos(Theta_26) -sin(Theta_26)*cos(alpha(1,2)) sin(Theta_26)*sin(alpha(1,2)) a(1,2)*cos(Theta_26);
    sin(Theta_26) cos(Theta_26)*cos(alpha(1,2)) -cos(Theta_26)*sin(alpha(1,2)) a(1,2)*sin(Theta_26);
    0 sin(alpha(1,2)) cos(alpha(1,2)) d(1,2);
    0 0 0 1];
T21_7=[cos(Theta_27) -sin(Theta_27)*cos(alpha(1,2)) sin(Theta_27)*sin(alpha(1,2)) a(1,2)*cos(Theta_27);
    sin(Theta_27) cos(Theta_27)*cos(alpha(1,2)) -cos(Theta_27)*sin(alpha(1,2)) a(1,2)*sin(Theta_27);
    0 sin(alpha(1,2)) cos(alpha(1,2)) d(1,2);
    0 0 0 1];
T21_8=[cos(Theta_28) -sin(Theta_28)*cos(alpha(1,2)) sin(Theta_28)*sin(alpha(1,2)) a(1,2)*cos(Theta_28);
    sin(Theta_28) cos(Theta_28)*cos(alpha(1,2)) -cos(Theta_28)*sin(alpha(1,2)) a(1,2)*sin(Theta_28);
    0 sin(alpha(1,2)) cos(alpha(1,2)) d(1,2);
    0 0 0 1];



T32_1=[cos(Theta_31) -sin(Theta_31)*cos(alpha(1,3)) sin(Theta_31)*sin(alpha(1,3)) a(1,3)*cos(Theta_31);
    sin(Theta_31) cos(Theta_31)*cos(alpha(1,3)) -cos(Theta_31)*sin(alpha(1,3)) a(1,3)*sin(Theta_31);
    0 sin(alpha(1,3)) cos(alpha(1,3)) d(1,3);
    0 0 0 1];
T32_2=[cos(Theta_32) -sin(Theta_32)*cos(alpha(1,3)) sin(Theta_32)*sin(alpha(1,3)) a(1,3)*cos(Theta_32);
    sin(Theta_32) cos(Theta_32)*cos(alpha(1,3)) -cos(Theta_32)*sin(alpha(1,3)) a(1,3)*sin(Theta_32);
    0 sin(alpha(1,3)) cos(alpha(1,3)) d(1,3);
    0 0 0 1];
T32_3=[cos(Theta_33) -sin(Theta_33)*cos(alpha(1,3)) sin(Theta_33)*sin(alpha(1,3)) a(1,3)*cos(Theta_33);
    sin(Theta_33) cos(Theta_33)*cos(alpha(1,3)) -cos(Theta_33)*sin(alpha(1,3)) a(1,3)*sin(Theta_33);
    0 sin(alpha(1,3)) cos(alpha(1,3)) d(1,3);
    0 0 0 1];
T32_4=[cos(Theta_34) -sin(Theta_34)*cos(alpha(1,3)) sin(Theta_34)*sin(alpha(1,3)) a(1,3)*cos(Theta_34);
    sin(Theta_34) cos(Theta_34)*cos(alpha(1,3)) -cos(Theta_34)*sin(alpha(1,3)) a(1,3)*sin(Theta_34);
    0 sin(alpha(1,3)) cos(alpha(1,3)) d(1,3);
    0 0 0 1];
T32_5=[cos(Theta_35) -sin(Theta_35)*cos(alpha(1,3)) sin(Theta_35)*sin(alpha(1,3)) a(1,3)*cos(Theta_35);
    sin(Theta_35) cos(Theta_35)*cos(alpha(1,3)) -cos(Theta_35)*sin(alpha(1,3)) a(1,3)*sin(Theta_35);
    0 sin(alpha(1,3)) cos(alpha(1,3)) d(1,3);
    0 0 0 1];
T32_6=[cos(Theta_36) -sin(Theta_36)*cos(alpha(1,3)) sin(Theta_36)*sin(alpha(1,3)) a(1,3)*cos(Theta_36);
    sin(Theta_36) cos(Theta_36)*cos(alpha(1,3)) -cos(Theta_36)*sin(alpha(1,3)) a(1,3)*sin(Theta_36);
    0 sin(alpha(1,3)) cos(alpha(1,3)) d(1,3);
    0 0 0 1];
T32_7=[cos(Theta_37) -sin(Theta_37)*cos(alpha(1,3)) sin(Theta_37)*sin(alpha(1,3)) a(1,3)*cos(Theta_37);
    sin(Theta_37) cos(Theta_37)*cos(alpha(1,3)) -cos(Theta_37)*sin(alpha(1,3)) a(1,3)*sin(Theta_37);
    0 sin(alpha(1,3)) cos(alpha(1,3)) d(1,3);
    0 0 0 1];
T32_8=[cos(Theta_38) -sin(Theta_38)*cos(alpha(1,3)) sin(Theta_38)*sin(alpha(1,3)) a(1,3)*cos(Theta_38);
    sin(Theta_38) cos(Theta_38)*cos(alpha(1,3)) -cos(Theta_38)*sin(alpha(1,3)) a(1,3)*sin(Theta_38);
    0 sin(alpha(1,3)) cos(alpha(1,3)) d(1,3);
    0 0 0 1];


T43_1=inv(T21_1*T32_1)*T41_1;
T43_2=inv(T21_2*T32_2)*T41_1;
T43_3=inv(T21_3*T32_3)*T41_2;
T43_4=inv(T21_4*T32_4)*T41_2;
T43_5=inv(T21_5*T32_5)*T41_3;
T43_6=inv(T21_6*T32_6)*T41_3;
T43_7=inv(T21_7*T32_7)*T41_4;
T43_8=inv(T21_8*T32_8)*T41_4;


Theta_41=atan2(T43_1(2,1),T43_1(1,1));
Theta_42=atan2(T43_2(2,1),T43_2(1,1));
Theta_43=atan2(T43_3(2,1),T43_3(1,1));
Theta_44=atan2(T43_4(2,1),T43_4(1,1));
Theta_45=atan2(T43_5(2,1),T43_5(1,1));
Theta_46=atan2(T43_6(2,1),T43_6(1,1));
Theta_47=atan2(T43_7(2,1),T43_7(1,1));
Theta_48=atan2(T43_8(2,1),T43_8(1,1));

Solutions = [Theta_11 Theta_21 Theta_31 Theta_41 Theta_51 Theta_61;
    Theta_11 Theta_22 Theta_32 Theta_42 Theta_51 Theta_61;
    Theta_11 Theta_23 Theta_33 Theta_43 Theta_52 Theta_62;
    Theta_11 Theta_24 Theta_34 Theta_44 Theta_52 Theta_62;
    Theta_12 Theta_25 Theta_35 Theta_45 Theta_53 Theta_63;
    Theta_12 Theta_26 Theta_36 Theta_46 Theta_53 Theta_63;
    Theta_12 Theta_27 Theta_37 Theta_47 Theta_54 Theta_64;
    Theta_12 Theta_28 Theta_38 Theta_48 Theta_54 Theta_64];

error = zeros(8,1);
for i=1:8
    error(i)=norm(q_pre'-Solutions(i,:));
end
[~, index] = min(error);

qd = Solutions(index,:)';

end
