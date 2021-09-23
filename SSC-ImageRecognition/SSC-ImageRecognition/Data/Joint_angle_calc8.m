%% ������ɂ�����쓮���̈ʒu�C�p������֐ߊp�x�����߂�֐�
%���]�̈ʒu�p���ɂ����āC���[�����g�̒ނ荇�������藧�֐ߊp�x�̎Z�o
%kineto static inverse analysis
%11/28�Ƀ����N�t���[���̒�������C��
%�����N0r,0f�̐ݒ���@�̕ύX
%�����N�t���[���̒������O��̋��ʋ@�\�ł�����iDH�p�����[�^���Ⴄ�j
%


function [F,the_2r,the_3r,the_1r,the_2f,the_3f,the_1f] ...
   = Joint_angle_calc8(x,the_xr,the_yr,the_zr,the_xf,the_yf,the_zf,phi_nr1,phi_nr2,the_nr,phi_nf1,phi_nf2,the_nf,...
                     gamma1,a1,a2,a3,Lc,OrCr,OfCf,OrOf,g0,g2,g3,g4,g6,w0,w2,w3,w4,w6,mode)


   
    

    %% �����N3���猩���쓮���̎p���p�̓��o(p23~)
    OrCf = OrOf+OfCf;            %���쓮�ւ̐ڒn�_����O�쓮�ւ̌��_�Ɍ������x�N�g��
    CrCf = (OrCf - OrCr);        %���쓮�ւ̌��_����O�쓮�ւ̌��_�Ɍ������x�N�g��

    xb0 = CrCf/Lc;
    yb0 = cross([-sin(gamma1);0;cos(gamma1)],xb0)/norm(cross([-sin(gamma1);0;cos(gamma1)],xb0));
    zb0 = cross(xb0,yb0);
    Co_base0 = [xb0 yb0 zb0];    %base���W�n�ւ̍��W�ϊ��s��
    
    Co_base = Rod_R(xb0(1),xb0(2),xb0(3),x)*Co_base0; %xb0������ɂ�������]�����Cbase���W�n�ւ̍��W�ϊ��s������߂�D
    xb = Co_base(:,1);
    yb = Co_base(:,2);
    zb = Co_base(:,3);
    
    Co_0r_ = Rod_R(yb(1),yb(2),yb(3),-a3)*[xb yb zb];
    Co_0r = Rod_R(Co_0r_(1,3),Co_0r_(2,3),Co_0r_(3,3),pi/2)*Co_0r_;       %���͈͂��l���������W�n�̐ݒ�
    Co_0f_ = Rod_R(yb(1),yb(2),yb(3),a3)*[xb yb zb];
    Co_0f = Rod_R(Co_0f_(1,3),Co_0f_(2,3),Co_0f_(3,3),-pi/2)*Co_0f_;
    
    %�����N0�̍��W�n����쓮���i�����N3�̍��W�n�j�ւ̍��W�ϊ��s��
    M_0r = Co_0r\R("x",the_xr)*R("y",the_yr)*R("z",the_zr);              
    M_0f = Co_0f\R("x",the_xf)*R("y",the_yf)*R("z",the_zf);
  
    
    
    %�t�^���w���i��ցj
    if mode == 1
        the_2r = -acos((-cos(a1)*cos(a2)+M_0r(3,3))/(sin(a1)*sin(a2)));
    else
        the_2r =  acos((-cos(a1)*cos(a2)+M_0r(3,3))/(sin(a1)*sin(a2)));
    end
    
    Ar1 = -sin(a2)*sin(the_2r); Br1 =  cos(a1)*sin(a2)*cos(the_2r) - sin(a1)*cos(a2); Cr1 = M_0r(1,3); Dr1 = M_0r(2,3);
    Ar3 =  sin(a1)*sin(the_2r); Br3 = -sin(a1)*cos(a2)*cos(the_2r) + cos(a1)*sin(a2); Cr3 = M_0r(3,2); Dr3 = M_0r(3,1);
    
    the_1r = atan2(Ar1*Dr1-Br1*Cr1,Ar1*Cr1+Br1*Dr1); 
    the_3r = atan2(Ar3*Dr3-Br3*Cr3,Ar3*Cr3+Br3*Dr3);
    
    %�t�^���w���i�O�ցj
    if mode == 1
        the_2f =  acos((-cos(a1)*cos(a2)+M_0f(3,3))/(sin(a1)*sin(a2)));
    else
        the_2f = -acos((-cos(a1)*cos(a2)+M_0f(3,3))/(sin(a1)*sin(a2)));
    end
    
    Af1 = -sin(a2)*sin(the_2f); Bf1 = cos(a1)*sin(a2)*cos(the_2f) - sin(a1)*cos(a2); Cf1 = M_0f(1,3); Df1 = M_0f(2,3);
    Af3 = -sin(a1)*sin(the_2f); Bf3 = sin(a1)*cos(a2)*cos(the_2f) - cos(a1)*sin(a2); Cf3 = M_0f(3,2); Df3 = M_0f(3,1);
    
    the_1f = atan2(Af1*Df1-Bf1*Cf1,Af1*Cf1+Bf1*Df1); 
    the_3f = atan2(Af3*Df3-Bf3*Cf3,Af3*Cf3+Bf3*Df3);
    

    %% �d�S�v�Z�ithe_x�́j����

    %Or�,�P�[�u�����W�n(���쓮�ւƃP�[�u���̐ڐG�_)
    %�����g���b�L�[%�Ȃ��Ƃ��Ă���(�ŏI�I�ɂ͂����Ƃ������W�n�Ōv�Z���s���̂��]�܂������C�X���͔c���̂��߁C�e�����N�̏d�S�́C�΋�ʒu�Ɉ�v����Ɖ���)
    %(�����N�P�̏d�ʂ̓����N0�ɁC�����N5�̏d�ʂ�,�����N6�Ɋ܂܂��Ɖ��肷��)
    %(���Ȃ킿�����N3�ȊO�́C�d���̐����͂����������Ȃ�)

    m0 = cross((OrCr + R("x",the_xr)*R("y",the_yr)*R("z",the_zr)*g0),[-w0*sin(gamma1);0;w0*cos(gamma1)]);
    m2 = cross((OrCr + R("x",the_xr)*R("y",the_yr)*R("z",the_zr)*R("z",-the_3r+pi/2)*R("x",a2)*R("z",-the_2r)*g2),[-w2*sin(gamma1);0;w2*cos(gamma1)]);
    m3 = cross((OrCr + R("x",the_xr)*R("y",the_yr)*R("z",the_zr)*R("z",-the_3r+pi/2)*R("x",a2)*R("z",-the_2r)*R("x",-a1)*R("z",-the_1r)*R("z",-pi/2)*R("y",a3)*g3),[-w3*sin(gamma1);0;w3*cos(gamma1)]);
    m4 = cross((OrCf + R("x",the_xr)*R("y",the_yr)*R("z",the_zr)*R("z",-the_3r+pi/2)*R("x",a2)*R("z",-the_2r)*R("x",-a1)*R("z",-the_1r)*R("z",-pi/2)*R("y",2*a3)*R("z",-pi/2)*R("z",the_1f)*R("x",a1)*g4),[-w4*sin(gamma1);0;w4*cos(gamma1)]);
    m6 = cross((OrCf + R("x",the_xr)*R("y",the_yr)*R("z",the_zr)*R("z",-the_3r+pi/2)*R("x",a2)*R("z",-the_2r)*R("x",-a1)*R("z",-the_1r)*R("z",-pi/2)*R("y",2*a3)*R("z",-pi/2)*R("z",the_1f)*R("x",a1)*R("z",the_2f)*R("x",-a2)*R("z",the_3f + pi/2)*g6),[-w6*sin(gamma1);0;w6*cos(gamma1)]);
    
    %�]���֐��i���[�����g�j
    F = dot(m0,OrOf/norm(OrOf))+dot(m2,OrOf/norm(OrOf))+dot(m3,OrOf/norm(OrOf))+dot(m4,OrOf/norm(OrOf))+dot(m6,OrOf/norm(OrOf));

    %% ��]�s��
    function X = R(i,j)
        if i == "x"
            X = [1 0 0 ; 0 cos(j) -sin(j) ; 0 sin(j) cos(j)];
        elseif i == "y"
            X = [cos(j) 0 sin(j) ; 0 1 0 ; -sin(j) 0 cos(j)];
        elseif i == "z"
            X = [cos(j) -sin(j) 0 ; sin(j) cos(j) 0 ; 0 0 1];
        end    
    end
    %% ���h���Q�X�̉�]�s��
    function X = Rod_R(n1,n2,n3,the)
    
        X = [cos(the)+(1-cos(the))*n1^2      n1*n2*(1-cos(the))-n3*sin(the)   n1*n3*(1-cos(the))+n2*sin(the); ...
             sin(the)*n3+n2*n1*(1-cos(the))  cos(the)+(1-cos(the))*n2^2       n2*n3*(1-cos(the))-n1*sin(the); ...
            -sin(the)*n2+n3*n1*(1-cos(the))  n3*n2*(1-cos(the))+n1*sin(the)   cos(the)+(1-cos(the))*n3^2   ];
    
    end

end