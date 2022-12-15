%% ����݂̂ɓ��������v���O����
%�E���򍶕�������[�h�I���ɂ���ĕύX�ł���悤�ȃv���O������ڎw���D
%r,f��rear,front R,L��Righgt�CLeft
%�p���p��xyz�ŗ^���邱�Ƃɂ����i�{���I�ɂ͕ς��Ȃ����C�`�挋�ʂ��قȂ�̂ł�������̗p�j
%�����̎p���p�𕪊�O�̃P�[�u���ŗ^���邱�Ƃɂ�������

%% �����ݒ�
%�P�[�u������
    gamma1 = 10*pi/180;      %�P�[�u���Γx
    phi_n =  10*pi/180;      %�P�[�u�����ʂ̉�]�p
    the_nR = 60*pi/180;     %�E����p
    the_nL = 60*pi/180;    %������p

%���򃂁[�h
    mode = 1;   %�E����(1) ������(2)
    switching = 0;
%�f�[�^�̕ۑ�
    save = 0;
    animation = 1;
%% �萔
%�@�\�萔�Ȃ�
    Rw = 83/2;      %�^�C���̒��a�C���a
    rw = 22.185/2;
    d = 6;          %�P�[�u�����a
    phi_nw = atan(12/23.5);   %p88�^�C���̌`��ɂ���ăP�[�u���𔲂��p�x

    rm = 45/2;  %��~�����a�i���[�^�����j
    lm = 50;  %��~�������i���[�^�����j
    rj = 35; %�ڐG����p�̋��̔��a
    
    a1 = 79/180*pi;         %�˂���p
    a2 = 64/180*pi;
    a3 = 15/180*pi;         %1���ڂ̎��t���p�i�_���ł�a0�j  
    
    
%�e�����N�̏d���C�d�S�ʒu�C����
    Lc = 300;
    l2_min = (Lc-80)/2;
    l1 = 90;                %�e���[�^�[�̒��S�_����̋���
    l2 = l2_min/sin(a2);
    l3 = 185;
    l4 = 185;
    l5 = l2_min/sin(a2);
    l6 = 90;
    La = 215;
    
    w0 = 800;
    %w1 = 146.34;
    w2 = 154;
    %w3 = 635+865;
    w3 = 700+700;
    w4 = 154;   
    %w5 = 146.34;
    w6 = 800;
    
    g0 = [0;0;0];
    %g1 = [0;-19.248;91.207];
    g2 = [0;0;l2];
    g3 = [Lc/2;0;La];
    g4 = [0;0;l2];
    %g5 = [0;-83.118;42.198];
    g6 = [0;0;0];
    
%% data�֘A
    count1 = zeros(500,3);
    count2 = zeros(500,3);
    data_r = zeros(500,4);
    data_f = zeros(500,4);
    data = zeros(500,9);
    data_r_full = zeros(180,4);
    data_f_full = zeros(180,4);
    moment = zeros(180,2);
    data_collide_r = zeros(1000,2);
    data_collide_f = zeros(1000,2);
        



    %���X�̏�����
    buf1 = 0; buf2 = 0; buf3 = 0;
    t1 = 0; t2 = 0; t3 = 0;t4 = 0;t5 = 0;
    cnt = 0;
    L1 = Lc; %�P�[�u���̒���
    L2 = Lc;
    
  
%% �p���x�Ȃǂ̐ݒ�
    %�^�������
%   t_step = 0.0485; %50[ms] �L���������Ƌ��������������Ȃ�D48ms�ł���
    t_step = 1.6/34; %�L���̂��������ɂ��邽��(���������菭���͂��)
    r_v = 13.5; %�L�����a[mm]
    vel = 50; %���򎞂̌�ւ̈ړ����x[mm/s]
    L_gap = 40; %����_���O�Ŏ~�܂鋗��(��^�C�����a�̔���)
    
 
   
    %�]���I�Ɍ��܂���̂���
    t_bra = 2*L_gap/vel;                    %�쓮�ւ�����ɂ����鎞��(80/50=1.6[s])
    n_bra = round(t_bra/t_step);            %����ɂ�����J�E���g
    omega_c =vel/r_v*180/pi ;                 %[deg/s] �ʏ푖�s���̑��x    



 
    %% XYX�I�C���[�p�ƃN�H�[�^�j�I���ɂ�镪�򎞂̎p���\��

    %���򓮍�ɂ�����p
    E0_R = [-pi/2,0,0];               %�I�C���[�p�̏����l
    E1_R = [-pi/2-phi_n,-pi/2+the_nL,phi_nw];  %�T�u�P�[�u���𔲂��p�� 
    E2_R = [-pi/2-phi_n,-the_nR,atan((-sin(gamma1)*sin(the_nR) + cos(gamma1)*cos(the_nR)*sin(phi_n))/cos(phi_n)*cos(gamma1))]; 
    
    E0_L = [pi/2,0,0];               %�I�C���[�p�̏����l
    E1_L = [pi/2-phi_n,-pi/2+the_nR,-phi_nw];  %�T�u�P�[�u���𔲂��p�� 
    E2_L = [pi/2-phi_n,-the_nL,atan((sin(gamma1)*sin(the_nR) + cos(gamma1)*cos(the_nR)*sin(phi_n))/cos(phi_n)*cos(gamma1))]; 

    %XYX�I�C���[�p���N�H�[�^�j�I���ɕϊ�
    quat0_R = quaternion(E0_R,'euler','XYX','frame');   %�N�H�[�^�j�I���̏����l
    quat1_R = quaternion(E1_R,'euler','XYX','frame');   %�T�u�P�[�u���𔲂��p��
    quat2_R = quaternion(E2_R,'euler','XYX','frame');   %�����̎p��
    quat0_L = quaternion(E0_L,'euler','XYX','frame');   %�N�H�[�^�j�I���̏����l
    quat1_L = quaternion(E1_L,'euler','XYX','frame');   %�T�u�P�[�u���𔲂��p��
    quat2_L = quaternion(E2_L,'euler','XYX','frame');   %�����̎p��

    if mode == 1
        %�E���򃂁[�h
        quat0 = quaternion(E0_R,'euler','XYX','frame');   %�N�H�[�^�j�I���̏����l
        quat1 = quaternion(E1_R,'euler','XYX','frame');   %�T�u�P�[�u���𔲂��p��
        quat2 = quaternion(E2_R,'euler','XYX','frame');   %�����̎p��
        E2 = E2_R;
    else
        quat0 = quaternion(E0_L,'euler','XYX','frame');   %�N�H�[�^�j�I���̏����l
        quat1 = quaternion(E1_L,'euler','XYX','frame');   %�T�u�P�[�u���𔲂��p��
        quat2 = quaternion(E2_L,'euler','XYX','frame');   %�����̎p��
        E2 = E2_L;
    end    
        
    %���򎞂̖ڕW�p�x(��Ԍ�)
    [X_01,Y_01,Z_01] = slerp_xyz_euler(quat0,quat1,ceil(n_bra/2));
    [X_12,Y_12,Z_12] = slerp_xyz_euler(quat1,quat2,fix(n_bra/2));            %��������Ȃ�������D�D�D
    X = [X_01;X_12];
    Y = [Y_01;Y_12];   %�A��
    Z = [Z_01;Z_12];
    
 
    %[L_def,OrCr,OfCf,OrOf,OrOw1r,OrOw2r,OfOw1f,OfOw2f] =  node_cal1(0,0,0,X(n_bra),Y(n_bra),Z(n_bra),phi_n,the_n,0,Lc,Rw,rw,d,mode); %�O�֕��򒼌��Ln1�̒����i���ꂩ���ւ��ړ�����ׂ������C��Ԃ���Lc�ƈقȂ�j
    %t_const = (L_def-L_gap)/vel - t_bra;    %�葬���s����[s]
    %n_const = t_const/t_step;               %�葬���s�J�E���g
    
    if mode == 1
        the_x_init = -pi/2;
        the_n = the_nR;
    else
        the_x_init = pi/2;
        the_n = -the_nL;
    end
%% ���C�����[�v

flag = 0;
for t = 1:1000
    
    if switching == 0
        if flag == 0    %�O�ւ̕���J�n
            t1 = t1+1;
            %���C���P�[�u���ɑ΂��āC�ǂ̃P�[�u���ɏ���Ă���̂��̏��C
            phi_nr1 = 0/180*pi; the_nr = 0/180*pi ;phi_nr2 = 0/180*pi; 
            phi_nf1 = 0/180*pi; the_nf = 0/180*pi; phi_nf2 = 0/180*pi;

            the_xr = the_x_init; the_yr = 0/180*pi; the_zr = 0/180*pi;
            the_xf = X(t1); the_yf = Y(t1); the_zf = Z(t1);
            
            Ln2 = -L_gap + omega_c*((t1*t_step)^4/(2*t_bra^3) -(t1*t_step)^3/(t_bra^2)+(t1*t_step))*pi/180*r_v;
            [Ln1,OrCr,OfCf,OrOf,OrOw1r,OrOw2r,OfOw1f,OfOw2f] = node_cal1(the_xr,the_yr,the_zr,the_xf,the_yf,the_zf,phi_nr1,the_nr,phi_nf1,the_nf,Ln2,Lc,Rw,rw,d,mode);
            
            omega_r = omega_c*(2*(t1*t_step)^3/(t_bra^3) -3*(t1*t_step)^2/(t_bra^2)+1);
            omega_f = omega_r;
            
            if t1 == n_bra
                flag = 1;
                L_def = Ln1;
                t_const = (L_def-2*L_gap)/vel;    %�葬���s����[s]
                n_const = t_const/t_step;               %�葬���s�J�E���g
            end
            
        elseif flag == 1
            %�O�ւ̕���I���`����
            t2 = t2+1;
            %���C���P�[�u���ɑ΂��āC�ǂ̃P�[�u���ɏ���Ă���̂��̏��C
            phi_nr1 = 0/180*pi; the_nr = 0/180*pi ;phi_nr2 = 0/180*pi; 
            phi_nf1 = -phi_n; the_nf = the_n; phi_nf2 = E2(3);

            the_xr = the_x_init; the_yr = 0/180*pi; the_zr = 0/180*pi;
            the_xf = X(n_bra); the_yf = Y(n_bra); the_zf = Z(n_bra);
            
            Ln1 = L_def - omega_c*(-(t2*t_step)^4/(2*t_bra^3) +(t2*t_step)^3/(t_bra^2))*pi/180*r_v;
            [Ln2,OrCr,OfCf,OrOf,OrOw1r,OrOw2r,OfOw1f,OfOw2f] = node_cal2(the_xr,the_yr,the_zr,the_xf,the_yf,the_zf,phi_nr1,the_nr,phi_nf1,the_nf,Ln1,Lc,Rw,rw,d,mode);
            
            omega_r = omega_c*(-2*(t2*t_step)^3/(t_bra^3) +3*(t2*t_step)^2/(t_bra^2));
            omega_f = omega_r;
            
            if t2 == n_bra
                flag = 2;
                L_def = Ln1;
            end
        elseif flag == 2
            %�葬���
            t3 = t3+1;
            %���C���P�[�u���ɑ΂��āC�ǂ̃P�[�u���ɏ���Ă���̂��̏��C
            phi_nr1 = 0/180*pi; the_nr = 0/180*pi ;phi_nr2 = 0/180*pi; 
            phi_nf1 = -phi_n; the_nf = the_n; phi_nf2 = E2(3);

            the_xr = the_x_init; the_yr = 0/180*pi; the_zr = 0/180*pi;
            the_xf = X(n_bra); the_yf = Y(n_bra); the_zf = Z(n_bra);
            
            Ln1 = L_def - omega_c*pi/180*r_v*t3*t_step;
            [Ln2,OrCr,OfCf,OrOf,OrOw1r,OrOw2r,OfOw1f,OfOw2f] = node_cal2(the_xr,the_yr,the_zr,the_xf,the_yf,the_zf,phi_nr1,the_nr,phi_nf1,the_nf,Ln1,Lc,Rw,rw,d,mode);

            omega_r = omega_c;
            omega_f = omega_r;
            
            if t3 >= n_const-1
                flag = 3;
                L_def = Ln1;
            end
            
        elseif flag == 3
            %������ԁi��֕���J�n�j
            t4 = t4+1;
            %���C���P�[�u���ɑ΂��āC�ǂ̃P�[�u���ɏ���Ă���̂��̏��C
            phi_nr1 = 0/180*pi; the_nr = 0/180*pi ;phi_nr2 = 0/180*pi; 
            phi_nf1 = -phi_n; the_nf = the_n; phi_nf2 = E2(3);

            the_xr = X(t4); the_yr = Y(t4); the_zr = Z(t4);
            the_xf = X(n_bra); the_yf = Y(n_bra); the_zf = Z(n_bra);
            
            Ln1 = L_gap - omega_c*((t4*t_step)^4/(2*t_bra^3) -(t4*t_step)^3/(t_bra^2)+(t4*t_step))*pi/180*r_v;
            [Ln2,OrCr,OfCf,OrOf,OrOw1r,OrOw2r,OfOw1f,OfOw2f] = node_cal2(the_xr,the_yr,the_zr,the_xf,the_yf,the_zf,phi_nr1,the_nr,phi_nf1,the_nf,Ln1,Lc,Rw,rw,d,mode);

            omega_r = omega_c*(2*(t4*t_step)^3/(t_bra^3) -3*(t4*t_step)^2/(t_bra^2)+1);
            omega_f = omega_r;
            
            if t4 == n_bra
                flag = 4;
            end
        
        elseif flag == 4
            %�����(�������)
            t5 = t5+1;
            
            %���C���P�[�u���ɑ΂��āC�ǂ̃P�[�u���ɏ���Ă���̂��̏��C
            phi_nr1 = -phi_n; the_nr = the_n ;phi_nr2 = E2(3); 
            phi_nf1 = -phi_n; the_nf = the_n; phi_nf2 = E2(3);
            

            the_xr = X(n_bra); the_yr = Y(n_bra); the_zr = Z(n_bra);
            the_xf = X(n_bra); the_yf = Y(n_bra); the_zf = Z(n_bra);

            Ln1 = - omega_c*(-(t5*t_step)^4/(2*t_bra^3) +(t5*t_step)^3/(t_bra^2))*pi/180*r_v;
            [Ln2,OrCr,OfCf,OrOf,OrOw1r,OrOw2r,OfOw1f,OfOw2f] = node_cal2(the_xr,the_yr,the_zr,the_xf,the_yf,the_zf,phi_nr1,the_nr,phi_nf1,the_nf,Ln1,Lc,Rw,rw,d,mode);
            
            omega_r = omega_c*(-2*(t5*t_step)^3/(t_bra^3) +3*(t5*t_step)^2/(t_bra^2));
            omega_f = omega_r;
            
            if t5 == n_bra+1
               %�f�[�^�̏����o�� 
               text = gamma1*180/pi+ "_" + phi_n*180/pi+ "_" + the_nR*180/pi+ "_" + the_nL*180/pi+"_"+mode;
               T = t1+t2+t3+t4+t5-1;
               data(T+1:500,:)=[];
               if save ==1
                    writematrix(data,text+'.csv')
               end
               

               %�����o���f�[�^�̕`��
               figure(2)
               %�_�~�[�f�[�^�̍쐬
%                x0 = [1 1];
%                x1 = [n_count01 n_count01];
%                x2 = [t1 t1];                            
%                x3 = [t1+t2 t1+t2];
%                x4 = [t1+t2+n_count01 t1+t2+n_count01];                            
%                x5 = [t1+t2+t3 t1+t2+t3];
%                y = [-180 350];
                            
                            
               plot(data_r(1:T,1),data(1:T,1)*180/pi,'LineWidth',2)
               hold on
               plot(data_r(1:T,1),data(1:T,2)*180/pi,'LineWidth',2)
               plot(data_r(1:T,1),data(1:T,3)*180/pi,'LineWidth',2)
               plot(data_r(1:T,1),data(1:T,4)*180/pi,'LineWidth',2)
               plot(data_r(1:T,1),data(1:T,5)*180/pi,'LineWidth',2)
               plot(data_r(1:T,1),data(1:T,6)*180/pi,'LineWidth',2)
%                plot(x1,y,'--k','LineWidth',0.5)
%                plot(x2,y,'--k','LineWidth',0.5)
%                plot(x3,y,'--k','LineWidth',0.5)
%                plot(x4,y,'--k','LineWidth',0.5)
%                plot(x0,y,'--k','LineWidth',0.5)
%                plot(x5,y,'--k','LineWidth',0.5)
                            
               legend('1����(�O)','2����(�O)','3����(�O)','1����(��)','2����(��)','3����(��)','Location','NorthEast')
               ylim([-180 180]) 
               yticks(-180:30:180)
               xlabel('���ԁC�X�e�b�v')
               ylabel('�p�x[deg]')
                            
               figure(3)
               plot(data_r(1:T,1),data(1:T,7),'LineWidth',2)
               hold on
               plot(data_r(1:T,1),data(1:T,8),'LineWidth',2)
%                plot(x1,y,'--k','LineWidth',0.5)
%                plot(x2,y,'--k','LineWidth',0.5)
%                plot(x3,y,'--k','LineWidth',0.5)
%                plot(x4,y,'--k','LineWidth',0.5)*
%                plot(x0,y,'--k','LineWidth',0.5)
%                plot(x5,y,'--k','LineWidth',0.5)
               ylim([0 350])
               xlabel('���ԁC�X�e�b�v')
               ylabel('�p���x[deg/s]')
          
                break;
            end
            
        end

    end
 
    %% �e�֐ߊp�x�̌v�Z
        
    %�����v�Z
    fun = @(x)Joint_angle_calc8(x,the_xr,the_yr,the_zr,the_xf,the_yf,the_zf,phi_nr1,phi_nr2,the_nr,phi_nf1,phi_nf2,the_nf,...
                                       gamma1,a1,a2,a3,Lc,OrCr,OfCf,OrOf,g0,g2,g3,g4,g6,w0,w2,w3,w4,w6,mode);
    [x,fval,exitflag,output]= fzero(fun,0);
            
    %�����������ʂ�p���āC�֐ߊp�x���Z�o
    [F,the_2r,the_3r,the_1r,the_2f,the_3f,the_1f] ...
            = Joint_angle_calc8(x,the_xr,the_yr,the_zr,the_xf,the_yf,the_zf,phi_nr1,phi_nr2,the_nr,phi_nf1,phi_nf2,the_nf,...
                                gamma1,a1,a2,a3,Lc,OrCr,OfCf,OrOf,g0,g2,g3,g4,g6,w0,w2,w3,w4,w6,mode);
                              
%%                            
         %�֐ߊp�x�f�[�^�̕ۑ�
         data_r(t,:) = [t*t_step the_1r the_2r the_3r];
         data_f(t,:) = [t*t_step the_1f the_2f the_3f];
         data(t,:) = [the_1f the_2f the_3f the_1r the_2r the_3r omega_f omega_r t*t_step];
    if animation == 1
        %% �f�[�^�̕`��
           
            lw = 45;  %��~�������i�̂�y�����j
    
            %���[�^�̈ʒu����
            %����
            [Xm,Ym,Zm] = cylinder(rm);                  %��~�����W�i���[�^�����j
            Zm = (Zm - 0.5).*lm;                        %�~���̍����␳
            %Xm = Xm + 0.01*t; 
            m_top     = [Xm(1,:);Ym(1,:);Zm(1,:)];      %��~�����W�i�㉺�e�����j
            m_bottom  = [Xm(2,:);Ym(2,:);Zm(2,:)];
            m1_top    = [Xm(1,:);Ym(1,:);Zm(1,:)+l1];   %�ŏ��̃��[�^�̈ʒu
            m1_bottom = [Xm(2,:);Ym(2,:);Zm(2,:)+l1];
            m2_top    = [Xm(1,:);Ym(1,:);Zm(1,:)+l2];   %2�Ԗڂ̃��[�^�̊�ʒu
            m2_bottom = [Xm(2,:);Ym(2,:);Zm(2,:)+l2];
            m3_top    = [Xm(1,:);Ym(1,:);Zm(1,:)+l3];   %3�Ԗڂ̃��[�^�̊�ʒu
            m3_bottom = [Xm(2,:);Ym(2,:);Zm(2,:)+l3];
            m4_top    = [Xm(1,:);Ym(1,:);Zm(1,:)+l4];   %4�Ԗڂ̃��[�^�̊�ʒu
            m4_bottom = [Xm(2,:);Ym(2,:);Zm(2,:)+l4];
            m5_top    = [Xm(1,:);Ym(1,:);Zm(1,:)+l5];   %5�Ԗڂ̃��[�^�̊�ʒu
            m5_bottom = [Xm(2,:);Ym(2,:);Zm(2,:)+l5];
            m6_top    = [Xm(1,:);Ym(1,:);Zm(1,:)+l6];   %6�Ԗڂ̃��[�^�̊�ʒu
            m6_bottom = [Xm(2,:);Ym(2,:);Zm(2,:)+l6];
            
            %���[�^�̍��W����

            OrCf = OrOf+OfCf;
            m1_coord = R("y",gamma1)*([L1;0;0]+R("x",phi_nr1)*R("z",the_nr)*R("x",phi_nr2)*[-Ln1;0;0]+OrCr + R("x",the_xr)*R("y",the_yr)*R("z",the_zr)*R("z",-the_3r+pi/2)*horzcat(m1_top,m1_bottom));
            m2_coord = R("y",gamma1)*([L1;0;0]+R("x",phi_nr1)*R("z",the_nr)*R("x",phi_nr2)*[-Ln1;0;0]+OrCr + R("x",the_xr)*R("y",the_yr)*R("z",the_zr)*R("z",-the_3r+pi/2)*R("x",a2)*R("z",-the_2r)*horzcat(m2_top,m2_bottom));
            m3_coord = R("y",gamma1)*([L1;0;0]+R("x",phi_nr1)*R("z",the_nr)*R("x",phi_nr2)*[-Ln1;0;0]+OrCr + R("x",the_xr)*R("y",the_yr)*R("z",the_zr)*R("z",-the_3r+pi/2)*R("x",a2)*R("z",-the_2r)*R("x",-a1)*R("z",-the_1r)*R("z",-pi/2)*horzcat(m3_top,m3_bottom));
            m4_coord = R("y",gamma1)*([L1;0;0]+R("x",phi_nr1)*R("z",the_nr)*R("x",phi_nr2)*[-Ln1;0;0]+OrCf + R("x",the_xr)*R("y",the_yr)*R("z",the_zr)*R("z",-the_3r+pi/2)*R("x",a2)*R("z",-the_2r)*R("x",-a1)*R("z",-the_1r)*R("z",-pi/2)*R("y",2*a3)*R("z",-pi/2)*R("z",the_1f)*horzcat(m4_top,m4_bottom));
            m5_coord = R("y",gamma1)*([L1;0;0]+R("x",phi_nr1)*R("z",the_nr)*R("x",phi_nr2)*[-Ln1;0;0]+OrCf + R("x",the_xr)*R("y",the_yr)*R("z",the_zr)*R("z",-the_3r+pi/2)*R("x",a2)*R("z",-the_2r)*R("x",-a1)*R("z",-the_1r)*R("z",-pi/2)*R("y",2*a3)*R("z",-pi/2)*R("z",the_1f)*R("x",a1)*R("z",the_2f)*horzcat(m5_top,m5_bottom));
            m6_coord = R("y",gamma1)*([L1;0;0]+R("x",phi_nr1)*R("z",the_nr)*R("x",phi_nr2)*[-Ln1;0;0]+OrCf + R("x",the_xr)*R("y",the_yr)*R("z",the_zr)*R("z",-the_3r+pi/2)*R("x",a2)*R("z",-the_2r)*R("x",-a1)*R("z",-the_1r)*R("z",-pi/2)*R("y",2*a3)*R("z",-pi/2)*R("z",the_1f)*R("x",a1)*R("z",the_2f)*R("x",-a2)*R("z",the_3f+pi/2)*horzcat(m6_top,m6_bottom));

            
            %�^�C�������̈ʒu�����i�^�C���͂Ђ傤����^�j
            i = 1:11;
            [Xw,Yw,Zw] = cylinder((Rw+rw)/2+(Rw-rw)/2*cos(2*pi*(i-1)/10));
            Zw = (Zw - 0.5).*lw;
            
            for i = 1:11
                W(1:3,i+20*(i-1):i+20*i) = [Xw(i,:);Yw(i,:);Zw(i,:)];
            end
            W1 = W - [0;Rw;0];
            W2 = W + [0;Rw;0];
            
            %phi_nr1�Ƃ��͏��0�Ȃ̂Ŏ����I�ɉ������Ă��Ȃ��i������`�悵�����Ƃ��̂ݎg�p�j
            w1r_coord = R("y",gamma1)*([L1;0;0]+R("x",phi_nr1)*R("z",the_nr)*R("x",phi_nr2)*[-Ln1;0;0]+OrCr + R("x",the_xr)*R("y",the_yr)*R("z",the_zr)*W1);
            w2r_coord = R("y",gamma1)*([L1;0;0]+R("x",phi_nr1)*R("z",the_nr)*R("x",phi_nr2)*[-Ln1;0;0]+OrCr + R("x",the_xr)*R("y",the_yr)*R("z",the_zr)*W2);
            w1f_coord = R("y",gamma1)*([L1;0;0]+R("x",phi_nr1)*R("z",the_nr)*R("x",phi_nr2)*[-Ln1;0;0]+OrCf + R("x",the_xr)*R("y",the_yr)*R("z",the_zr)*R("z",-the_3r+pi/2)*R("x",a2)*R("z",-the_2r)*R("x",-a1)*R("z",-the_1r)*R("z",-pi/2)*R("y",2*a3)*R("z",-pi/2)*R("z",the_1f)*R("x",a1)*R("z",the_2f)*R("x",-a2)*R("z",the_3f+pi/2)*W2);
            w2f_coord = R("y",gamma1)*([L1;0;0]+R("x",phi_nr1)*R("z",the_nr)*R("x",phi_nr2)*[-Ln1;0;0]+OrCf + R("x",the_xr)*R("y",the_yr)*R("z",the_zr)*R("z",-the_3r+pi/2)*R("x",a2)*R("z",-the_2r)*R("x",-a1)*R("z",-the_1r)*R("z",-pi/2)*R("y",2*a3)*R("z",-pi/2)*R("z",the_1f)*R("x",a1)*R("z",the_2f)*R("x",-a2)*R("z",the_3f+pi/2)*W1);
           

            w1r_middle_bottom = (sum(w1r_coord(:,11*21-20:11*21),2)-w1r_coord(:,231))/20;
            w2r_middle_bottom = (sum(w2r_coord(:,11*21-20:11*21),2)-w2r_coord(:,231))/20;
            w1f_middle_bottom = (sum(w2f_coord(:,11*21-20:11*21),2)-w2f_coord(:,231))/20;
            w2f_middle_bottom = (sum(w1f_coord(:,11*21-20:11*21),2)-w1f_coord(:,231))/20;
            
            
            %�����̕`��f�[�^����(�����N����)
            m1_middle_top    = (sum(m1_coord(:,1:21),2)-m1_coord(:,1))/20;
            m1_middle_bottom = (sum(m1_coord(:,22:42),2)-m1_coord(:,22))/20;
            m2_middle_top    = (sum(m2_coord(:,1:21),2)-m2_coord(:,1))/20;
            m2_middle_bottom = (sum(m2_coord(:,22:42),2)-m2_coord(:,22))/20;
            m3_middle_top    = (sum(m3_coord(:,1:21),2)-m3_coord(:,1))/20;
            m3_middle_bottom = (sum(m3_coord(:,22:42),2)-m3_coord(:,22))/20;
            m4_middle_top    = (sum(m4_coord(:,1:21),2)-m4_coord(:,1))/20;
            m4_middle_bottom = (sum(m4_coord(:,22:42),2)-m4_coord(:,22))/20;
            m5_middle_top    = (sum(m5_coord(:,1:21),2)-m5_coord(:,1))/20;
            m5_middle_bottom = (sum(m5_coord(:,22:42),2)-m5_coord(:,22))/20;
            m6_middle_top    = (sum(m6_coord(:,1:21),2)-m6_coord(:,1))/20;
            m6_middle_bottom = (sum(m6_coord(:,22:42),2)-m6_coord(:,22))/20;
            
            
            c1w1 = horzcat(m1_middle_top,w1r_middle_bottom);
            c1w2 = horzcat(m1_middle_top,w2r_middle_bottom);
            c2_1 = horzcat(m2_middle_top,m1_middle_bottom);
            c3_2 = horzcat(m3_middle_top,m2_middle_bottom);
            c4_3 = horzcat(m4_middle_bottom,m3_middle_bottom);
            c4_5 = horzcat(m4_middle_top,m5_middle_bottom);
            c5_6 = horzcat(m5_middle_top,m6_middle_bottom);
            c6w1 = horzcat(m6_middle_top,w1f_middle_bottom);
            c6w2 = horzcat(m6_middle_top,w2f_middle_bottom);
            
            hold off
            clinder_plot(m1_coord(:,1:21),m1_coord(:,22:42))
            clinder_plot(m2_coord(:,1:21),m2_coord(:,22:42))
            clinder_plot(m3_coord(:,1:21),m3_coord(:,22:42))
            clinder_plot(m4_coord(:,1:21),m4_coord(:,22:42))
            clinder_plot(m5_coord(:,1:21),m5_coord(:,22:42))
            clinder_plot(m6_coord(:,1:21),m6_coord(:,22:42))
            gourd_plot(w1r_coord)
            gourd_plot(w2r_coord)
            gourd_plot(w1f_coord)
            gourd_plot(w2f_coord)
           
            
            line(c1w1(1,:)',c1w1(2,:)',c1w1(3,:)','LineWidth',1,'Color','k')
            line(c1w2(1,:)',c1w2(2,:)',c1w2(3,:)','LineWidth',1,'Color','k')
            line(c2_1(1,:)',c2_1(2,:)',c2_1(3,:)','LineWidth',1,'Color','k')
            line(c3_2(1,:)',c3_2(2,:)',c3_2(3,:)','LineWidth',1,'Color','k')
            line(c4_3(1,:)',c4_3(2,:)',c4_3(3,:)','LineWidth',1,'Color','k')
            line(c4_5(1,:)',c4_5(2,:)',c4_5(3,:)','LineWidth',1,'Color','k')
            line(c5_6(1,:)',c5_6(2,:)',c5_6(3,:)','LineWidth',1,'Color','k')
            line(c3_2(1,:)',c3_2(2,:)',c3_2(3,:)','LineWidth',1,'Color','k')
            line(c6w1(1,:)',c6w1(2,:)',c6w1(3,:)','LineWidth',1,'Color','k')
            line(c6w2(1,:)',c6w2(2,:)',c6w2(3,:)','LineWidth',1,'Color','k')
                       
             %�P�[�u���̕`��
            [Xc,Yc,Zc] = cylinder(3);
            Zc = (Zc - 0.5).*2*Lc;                        %�~���̍����␳
            
            c_top     = [Xc(1,:);Yc(1,:);Zc(1,:)];      %��~�����W�i�㉺�e�����j
            c_bottom  = [Xc(2,:);Yc(2,:);Zc(2,:)];
            c_top = R("y",gamma1)*R("y",pi/2)*c_top;
            c_bottom = R("y",gamma1)*R("y",pi/2)*c_bottom;
            cable_plot(c_top,c_bottom)
            
            %����P�[�u��(���C���E�T�u)
            [Xc2,Yc2,Zc2] = cylinder(3);
            Zc2 = Zc2*400;
            c_top2     = [Xc2(1,:);Yc2(1,:);Zc2(1,:)];      %��~�����W�i�㉺�e�����j
            c_bottom2  = [Xc2(2,:);Yc2(2,:);Zc2(2,:)];
            
            %�����͕`�悾���Ȃ̂œK���ɂ��Ă���
            c_top2h = R("y",gamma1)*([L1;0;0]+R("x",-phi_n)*R("z",the_nR)*R("y",pi/2)*c_top2);
            c_bottom2h = R("y",gamma1)*([L1;0;0]+R("x",-phi_n)*R("z",the_nR)*R("y",pi/2)*c_bottom2);
            c_top2s = R("y",gamma1)*([L1;0;0]+R("x",-phi_n)*R("z",-the_nL)*R("y",pi/2)*c_top2);
            c_bottom2s = R("y",gamma1)*([L1;0;0]+R("x",-phi_n)*R("z",-the_nL)*R("y",pi/2)*c_bottom2);
            
            cable_plot(c_top2h,c_bottom2h)
            cable_plot(c_top2s,c_bottom2s)
            %line([400;-400],[0;0],[0;0],'Color','k','LineWidth',6) 
        %     line([0;cable1(1)],[0;cable1(2)],[0;cable1(3)],'LineWidth',1)
        %     line([0;cable2(1)],[0;cable2(2)],[0;cable2(3)],'LineWidth',1)
            xlabel('x[mm]')
            ylabel('y[mm]')
            zlabel('z[mm]')
            
%% ����Ȃ�����          
    %
            %���W�̃g���[�X�ƐڐG����
            %���[�^�̒��S���W
%            Pr2 = R("y",gamma1)*([L1;0;0]+R("x",phi_nr1)*R("z",the_nr)*R("x",phi_nr2)*([-Ln1;0;0]+OrCr + R("y",the_yr)*R("x",the_xr)*R("z",the_zr)*R("z",-the_3r-pi/2)*R("x",a2)*[0;0;l2]));
%            Pf2 = R("y",gamma1)*([L1;0;0]+R("x",phi_nr1)*R("z",the_nr)*R("x",phi_nr2)*([-Ln1;0;0]+OrCf + R("y",the_yr)*R("x",the_xr)*R("z",the_zr)*R("z",-the_3r-pi/2)*R("x",a2)*R("z",-the_2r)*R("x",-a1)*R("z",-the_1r)*R("z",-pi)*R("y",2*a3)*R("z",the_1f)*R("x",a1)*[0;0;l2]));

%             [data_collide_r(t,1),data_collide_r(t,2)] = collide(gamma1,phi_n,the_nR,Pr2(1),Pr2(2),Pr2(3),rj,L1);
%             [data_collide_f(t,1),data_collide_f(t,2)] = collide(gamma1,phi_n,the_nR,Pf2(1),Pf2(2),Pf2(3),rj,L1);
% 
%             if (data_collide_r(t,1)+data_collide_r(t,2)+data_collide_f(t,1)+data_collide_f(t,2)) ~= 0
%                 flag_fin =1;     %�ڐG�����m������t���O�𗧂Ă�
%                 Data_sim(p,15) = 1;
% 
%             end
%
    
               
    
%             [Xs,Ys,Zs] = sphere;                  
%             Xs = Xs*rj; Ys = Ys*rj; Zs = Zs*rj;  
% %             surf(Xs+Pr2(1),Ys+Pr2(2),Zs+Pr2(3),'FaceColor','r','EdgeColor','none', 'FaceAlpha','0.5')
% %              surf(Xs+Pf2(1),Ys+Pf2(2),Zs+Pf2(3),'FaceColor','r','EdgeColor','none', 'FaceAlpha','0.5')
%        
%             if data_collide_r(t,1) == 1 || data_collide_r(t,2) == 1 
%                 surf(Xs+Pr2(1),Ys+Pr2(2),Zs+Pr2(3),'FaceColor','b','EdgeColor','none', 'FaceAlpha','0.5')
%             end    
%             if data_collide_f(t,1) == 1 || data_collide_f(t,2) == 1 
%                 surf(Xs+Pf2(1),Ys+Pf2(2),Zs+Pf2(3),'FaceColor','b','EdgeColor','none', 'FaceAlpha','0.5')
%             end
            
            %����ȉ~�̕`��
%             [e_r,M_ellipsoid_r] = manipulability_ellipsoid(a1,a2,the_2r,the_3r,phi_r,the_r,psi_r,phi_nr1,the_nr,phi_nr2,gamma1,L1,Ln1,OrCr,phi_nr1,the_nr,phi_nr2);
%             [e_f,M_ellipsoid_f] = manipulability_ellipsoid(a1,a2,the_2f,the_3f,phi_f,the_f,psi_f,phi_nr1,the_nr,phi_nr2,gamma1,L1,Ln1,OrCf,phi_nf1,the_nf,phi_nf2);
            
            %�K���}���l�����Ƃ��͂��������pi�񂷂������Ⴞ�߂��ۂ�
 %           M_R_ellipsoid_r = R("y",gamma1)*([L1;0;0]+R("x",phi_nr1)*R("z",the_nr)*R("x",phi_nr2)*([-Ln1;0;0]+OrCr)) + R("x",phi_nr1)*R("z",the_nr)*R("x",phi_nr2)*R("x",phi_r)*R("z",the_r)*R("y",psi_r)*R("z",pi)*R("x",-pi/2)*e_r*M_ellipsoid_r;
 %           M_R_ellipsoid_f = R("y",gamma1)*([L1;0;0]+R("x",phi_nr1)*R("z",the_nr)*R("x",phi_nr2)*([-Ln1;0;0]+OrCf)) + R("x",phi_nf1)*R("z",the_nf)*R("x",phi_nf2)*R("x",phi_f)*R("z",the_f)*R("y",psi_f)*R("x",-pi/2)*e_f*M_ellipsoid_f;
     
            %�K���}���l�����Ƃ��͂��������pi�񂷂������Ⴞ�߂��ۂ�
%             M_R_ellipsoid_r = R("y",gamma1)*([L1;0;0]+R("x",phi_nr1)*R("z",the_nr)*R("x",phi_nr2)*([-Ln1;0;0]+OrCr)) + R("y",gamma1)*R("x",phi_nr1)*R("z",the_nr)*R("x",phi_nr2)*R("y",the_yr)*R("x",the_xr)*R("z",the_zr)*e_r*M_ellipsoid_r;
%             M_R_ellipsoid_f = R("y",gamma1)*([L1;0;0]+R("x",phi_nr1)*R("z",the_nr)*R("x",phi_nr2)*([-Ln1;0;0]+OrCf)) + R("y",gamma1)*R("x",phi_nf1)*R("z",the_nf)*R("x",phi_nf2)*R("x",phi_f)*R("z",the_f)*R("y",psi_f)*R("x",-pi/2)*e_f*M_ellipsoid_f;
    
%              xr = reshape(M_R_ellipsoid_r(1,:),21,21);
%              yr = reshape(M_R_ellipsoid_r(2,:),21,21);
%              zr = reshape(M_R_ellipsoid_r(3,:),21,21);
%              xf = reshape(M_R_ellipsoid_f(1,:),21,21);
%              yf = reshape(M_R_ellipsoid_f(2,:),21,21);
%              zf = reshape(M_R_ellipsoid_f(3,:),21,21);

            
             % surf(xr, yr, zr,'FaceColor','b', 'FaceAlpha','0.3')
             % surf(xf, yf, zf,'FaceColor','b', 'FaceAlpha','0.3')
            
            %���[�N�X�y�[�X�̉���
%             [n_line,M_w_sphere] =  Work_space_sphere(a1,a2,l2);
%             M_w_sphere_r =R("y",gamma1)*([L1;0;0]+R("x",phi_nr1)*R("z",the_nr)*R("x",phi_nr2)*([-Ln1;0;0]+OrCr)) + R("y",gamma1)*R("x",phi_nr1)*R("z",the_nr)*R("x",phi_nr2)*R("y",the_yr)*R("x",the_xr)*R("z",the_zr)*R("z",-the_3r)*R("x",-a2)*R("z",-the_2r)*R("x",-a1)*R("z",-the_1r)*R("x",pi)*M_w_sphere;
%             M_w_sphere_f =R("y",gamma1)*([L1;0;0]+R("x",phi_nr1)*R("z",the_nr)*R("x",phi_nr2)*([-Ln1;0;0]+OrCf)) + R("y",gamma1)*R("x",phi_nf1)*R("z",the_nf)*R("x",phi_nf2)*R("y",the_yr)*R("x",the_xr)*R("z",the_zr)*R("z",-the_3r)*R("x",-a2)*R("z",-the_2r)*R("x",-a1)*R("z",-the_1r)*R("y",2*a3)*R("z",the_1f)*R("x",pi)*M_w_sphere;
%             
%             Xr = reshape(M_w_sphere_r(1,:),n_line,61);
%             Yr = reshape(M_w_sphere_r(2,:),n_line,61);
%             Zr = reshape(M_w_sphere_r(3,:),n_line,61);
%             Xf = reshape(M_w_sphere_f(1,:),n_line,61);
%             Yf = reshape(M_w_sphere_f(2,:),n_line,61);
%             Zf = reshape(M_w_sphere_f(3,:),n_line,61);
%             surf(Xr, Yr, Zr,'FaceColor','blue','FaceAlpha','0.5','LineStyle',':')
%             surf(Xf, Yf, Zf,'FaceColor','blue','FaceAlpha','0.5','LineStyle',':')
            
              % �m�F�p�̃v���b�g
%              xp = [Pr2(1);Pf2(1)];
%              yp = [Pr2(2);Pf2(2)];
%              zp = [Pr2(3);Pf2(3)];
%              plot3(xp,yp,zp)
         
         
         
            
            
            
            
            
            
             pause(0.0001)
             if t == 1
                 w = waitforbuttonpress;
                 w = waitforbuttonpress;
                 w = waitforbuttonpress;
                 w = waitforbuttonpress;
                 w = waitforbuttonpress;
             end
    end
%%         w = waitforbuttonpress;
end








%% �֐�

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

%%
%Ln2��^���āCLn1��Ԃ��֐�
function [Ln1,OrCr,OfCf,OrOf,OrOw1r,OrOw2r,OfOw1f,OfOw2f] = node_cal1(the_xr,the_yr,the_zr,the_xf,the_yf,the_zf,phi_nr1,the_nr,phi_nf1,the_nf,Ln2,Lc,Rw,rw,d,mode)
    if mode == 1 %�E���򃂁[�h
        %���
        OrCr =   R("x",the_xr)*R("y",the_yr)*[Rw*sin(the_zr);d/2+rw-Rw*cos(the_zr);0];
        OrOw1r = R("x",the_xr)*R("y",the_yr)*[0;d/2+rw;0];
        OrOw2r = R("x",the_xr)*R("y",the_yr)*[2*Rw*sin(the_zr);d/2+rw-2*Rw*cos(the_zr);0];
        %�O��
        OfCf =   R("x",the_xf)*R("y",the_yf)*[Rw*sin(the_zf);d/2+rw-Rw*cos(the_zf);0];
        OfOw1f = R("x",the_xf)*R("y",the_yf)*[0;d/2+rw;0];
        OfOw2f = R("x",the_xf)*R("y",the_yf)*[2*Rw*sin(the_zf);d/2+rw-2*Rw*cos(the_zf);0];
    elseif mode == 2 %�����򃂁[�h
        %���
        OrCr =   R("x",the_xr)*R("y",the_yr)*[-Rw*sin(the_zr);-d/2-rw+Rw*cos(the_zr);0];
        OrOw1r =  R("x",the_xr)*R("y",the_yr)*[-2*Rw*sin(the_zr);-d/2-rw+2*Rw*cos(the_zr);0];
        OrOw2r =  R("x",the_xr)*R("y",the_yr)*[0;-d/2-rw;0];
        %�O��
        OfCf =    R("x",the_xf)*R("y",the_yf)*[-Rw*sin(the_zf);-d/2-rw+Rw*cos(the_zf);0];
        OfOw1f =  R("x",the_xf)*R("y",the_yf)*[-2*Rw*sin(the_zf);-d/2-rw+2*Rw*cos(the_zf);0];
        OfOw2f =  R("x",the_xf)*R("y",the_yf)*[0;-d/2-rw;0];   
    else         
    end
    
      
    R1 = R("x",phi_nr1)*R("z",the_nr);
    R2 = R("x",phi_nf1)*R("z",the_nf);
    
    
    x_ = OfCf(1)-OrCr(1); y_ = OfCf(2)-OrCr(2); z_ = OfCf(3)-OrCr(3);
    
    a = [1 0 0]*R1*[1;0;0]; b = [0 1 0]*R1*[1;0;0]; c = [0 0 1]*R1*[1;0;0];
    d = [1 0 0]*R2*[1;0;0]; e = [0 1 0]*R2*[1;0;0]; f = [0 0 1]*R2*[1;0;0];
    
    A = a^2+b^2+c^2; B = d^2+e^2+f^2; C = x_^2+y_^2+z_^2;
    D = a*x_+b*y_+c*z_; E = d*x_+e*y_+f*z_; F = a*d+b*e+c*f;
    
    Ln1 = (-(D+F*Ln2)+sqrt((D+F*Ln2)^2-A*(B*Ln2*Ln2+2*E*Ln2+C-Lc^2)))/A;
    OrOf = R1*[Ln1;0;0]+R2*[Ln2;0;0];  
    
end

%% RCM���S���W�̌v�Z1(�E���򃂁[�h�C�����򃂁[�h)kakikaezumi
function [Ln2,OrCr,OfCf,OrOf,OrOw1r,OrOw2r,OfOw1f,OfOw2f] = node_cal2(the_xr,the_yr,the_zr,the_xf,the_yf,the_zf,phi_nr1,the_nr,phi_nf1,the_nf,Ln1,Lc,Rw,rw,d,mode)

    %gap  = 2*Rw*cos(the_zr)-d-2*rw;
    if mode == 1 %�E���򃂁[�h
        %���
        OrCr =   R("x",the_xr)*R("y",the_yr)*[Rw*sin(the_zr);d/2+rw-Rw*cos(the_zr);0];
        OrOw1r = R("x",the_xr)*R("y",the_yr)*[0;d/2+rw;0];
        OrOw2r = R("x",the_xr)*R("y",the_yr)*[2*Rw*sin(the_zr);d/2+rw-2*Rw*cos(the_zr);0];
        %�O��
        OfCf =   R("x",the_xf)*R("y",the_yf)*[Rw*sin(the_zf);d/2+rw-Rw*cos(the_zf);0];
        OfOw1f = R("x",the_xf)*R("y",the_yf)*[0;d/2+rw;0];
        OfOw2f = R("x",the_xf)*R("y",the_yf)*[2*Rw*sin(the_zf);d/2+rw-2*Rw*cos(the_zf);0];
    elseif mode == 2 %�����򃂁[�h
        %���
        OrCr =   R("x",the_xr)*R("y",the_yr)*[-Rw*sin(the_zr);-d/2-rw+Rw*cos(the_zr);0];
        OrOw1r = R("x",the_xr)*R("y",the_yr)*[-2*Rw*sin(the_zr);-d/2-rw+2*Rw*cos(the_zr);0];
        OrOw2r = R("x",the_xr)*R("y",the_yr)*[0;-d/2-rw;0];
        %�O��
        OfCf =   R("x",the_xf)*R("y",the_yf)*[-Rw*sin(the_zf);-d/2-rw+Rw*cos(the_zf);0];
        OfOw1f = R("x",the_xf)*R("y",the_yf)*[-2*Rw*sin(the_zf);-d/2-rw+2*Rw*cos(the_zf);0];
        OfOw2f = R("x",the_xf)*R("y",the_yf)*[0;-d/2-rw;0];   
    else         
    end
    
    %OrOf�Ԃ̌v�Z
    R1 = R("x",phi_nr1)*R("z",the_nr);
    R2 = R("x",phi_nf1)*R("z",the_nf);
    
    x_ = OfCf(1)-OrCr(1); y_ = OfCf(2)-OrCr(2); z_ = OfCf(3)-OrCr(3);
    
    a = [1 0 0]*R1*[1;0;0]; b = [0 1 0]*R1*[1;0;0]; c = [0 0 1]*R1*[1;0;0];
    d = [1 0 0]*R2*[1;0;0]; e = [0 1 0]*R2*[1;0;0]; f = [0 0 1]*R2*[1;0;0];
    
    A = a^2+b^2+c^2; B = d^2+e^2+f^2; C = x_^2+y_^2+z_^2;
    D = a*x_+b*y_+c*z_; E = d*x_+e*y_+f*z_; F = a*d+b*e+c*f;
      
    Ln2 = (-(E+F*Ln1)+sqrt((E+F*Ln1)^2-B*(A*Ln1*Ln1+2*D*Ln1+C-Lc^2)))/B;
    OrOf = R1*[Ln1;0;0]+R2*[Ln2;0;0];
end

%% ���ʐ��`��ԁ{�I�C���[�p�\��
%��ԃN�H�[�^�j�I�������߂���C�I�C���[�p�i�p���p�\������j
function [X,Y,Z] = slerp_xyz_euler(q1,q2,t)
        
    the = dist(q1,q2);  %q1,q2�̓���
    X = zeros(t,1);
    Y = zeros(t,1);
    Z = zeros(t,1);
       
    for i=1:t
        %t�͕������Ci��i�Ԗڂ̕����_
        k = 6*(i/t)^5-15*(i/t)^4+10*(i/t)^3;
        qt = sin((1-k)*the)/sin(the)*q1+sin(k*the)/sin(the)*q2;
        temp = euler(qt,'XYZ','frame');
        X(i,1) = temp(1);
        Y(i,1) = temp(2);
        Z(i,1) = temp(3);
    end
end

%% �ڐG����
function [f1,f2] = collide(gamma1,phi_n,the_nR,xp,yp,zp,rj,L)
     %�ڐG���蕔��p2-90
     %0522�@����_�𒴂��������ł̐ڐG�����m���Ȃ��悤�ɉ���
     f1 = 0;
     f2 = 0;
     
     a = cos(the_nR)*cos(gamma1)+sin(phi_n)*sin(the_nR)*sin(gamma1);
     b = cos(phi_n)*sin(the_nR);
     c = -cos(the_nR)*sin(gamma1)+sin(phi_n)*sin(the_nR)*cos(gamma1);
     A = a^2 + b^2 + c^2;
     B = a*xp+b*yp+c*zp-a*L*cos(gamma1)+c*L*sin(gamma1); 
     C = (L*cos(gamma1)-xp)^2+yp^2+(-L*sin(gamma1)-zp)^2-rj^2;
     
     %���s�P�[�u��
     judge1 = (cos(gamma1)*xp-sin(gamma1)*zp)^2-(xp^2+yp^2+zp^2-rj^2);
     
     %����P�[�u��
     judge2 = B^2-A*C;
     
     if (judge1 >= 0) && (cos(gamma1)*xp-sin(gamma1)*zp-sqrt(judge1) <= L)   %�ڐG���C���̉���L�����������Ȃ��
         f1 = 1;
     end
%      if judge2 >= 0 %�ڐG������
%          f1 = 1;
%      end
     
     if B <= 0
         if (judge2 >= 0) && (C<0)
             f2 =1;
         end
     else
         if judge2 >= 0
             f2 =1;
         end
     end
end
%% ����ȉ~���쐬����֐��i�G���h�G�t�F�N�^�̍��W�n�j
function [e,M_ellipsoid] =  manipulability_ellipsoid(a1,a2,the2,the3,phi,the,psi,phi_n1,the_n1,phi_n2,gamma1,L1,Ln1,OC,phi_n3,the_n2,phi_n4)

    J = [sin(a1)*(cos(the3)*sin(the2) + cos(a2)*cos(the2)*sin(the3)) + cos(a1)*sin(a2)*sin(the3) sin(a2)*sin(the3) 0; ...
         cos(a1)*sin(a2)*cos(the3) - sin(a1)*(sin(the2)*sin(the3) - cos(a2)*cos(the2)*cos(the3)) sin(a2)*cos(the3) 0; ...
         cos(a1)*cos(a2) - sin(a1)*sin(a2)*cos(the2) cos(a2) 1];

     [e,g] = eig(J*J');    %�ŗL�x�N�g���ƁC�ŗL�l�̑Ίp���s��*
     
     
     %g�̃T�C�Y����
     G = 75;
    [x, y, z] = ellipsoid(0,0,0,G*sqrt(g(1,1)),G*sqrt(g(2,2)),G*sqrt(g(3,3)));
     

     x = reshape(x,1,[]);
     y = reshape(y,1,[]);
     z = reshape(z,1,[]);

     M_ellipsoid = [x;y;z]; 
end 
%% ���[�N�X�y�[�X�̉���
function [n_line,M_w_sphere] =  Work_space_sphere(a1,a2,l2)

    A1 = a1+a2-pi/2;
    A2 = a1-a2-pi/2;
    [X, Y, Z]=sphere(60);

    z_max = sin(A1);
    z_min = sin(A2);

    t_min = 1;
    t_max = 1;
    while(Z(t_min)<z_min)
        t_min = t_min+1;
    end    
    while(Z(t_max)<z_max)
        t_max = t_max+1;
    end
    X = X(t_min:t_max,:)*l2;
    Y = Y(t_min:t_max,:)*l2;
    Z = Z(t_min:t_max,:)*l2;
    
    X = reshape(X,1,[]);
    Y = reshape(Y,1,[]);
    Z = reshape(Z,1,[]);
    
   M_w_sphere=[X;Y;Z];
   n_line = t_max-t_min+1;
end
%% �`��֘A�̊֐�

function clinder_plot(Cl_bottom,Cl_top)
    %Cl_top�͉~���̏�ʂ̍��W
    %Cl_bottom�͉~���̉��ʂ̍��W
    ax = gca;
    ax.YDir = 'reverse';
    ax.ZDir = 'reverse';
    X(1,:)= Cl_bottom(1,:);
    X(2,:)= Cl_top(1,:);
    Y(1,:)= Cl_bottom(2,:);
    Y(2,:)= Cl_top(2,:);
    Z(1,:)= Cl_bottom(3,:);
    Z(2,:)= Cl_top(3,:);


    surf(X,Y,Z,'FaceColor','r','EdgeColor','none')    %���ʃv���b�g
    hold on
    fill3(X(1,:),Y(1,:),Z(1,:),"w")                     %��ʃv���b�g
    fill3(X(2,:),Y(2,:),Z(2,:),"w")                     %��ʃv���b�g
    xlim([-300 500]) 
    ylim([-300 500]) 
    zlim([-300 500])
    % view(-135, 10);
end

function gourd_plot(W)
    %�Ђ傤����^����ʏo�͂���֐�
    %���͍s��̑傫����ǂݎ��΁C�ėp�I�Ȃ��̂����邯�Ǎ���̓X���[
    ax = gca;
    ax.YDir = 'reverse';
    ax.ZDir = 'reverse';

    X = zeros(11,21);
    Y = zeros(11,21);
    Z = zeros(11,21);

    for i = 1:11
           X(i,:)=W(1,i+20*(i-1):i+20*i); 
           Y(i,:)=W(2,i+20*(i-1):i+20*i);
           Z(i,:)=W(3,i+20*(i-1):i+20*i);
    end

    surf(X,Y,Z,'FaceColor','green','EdgeColor','k')    %���ʃv���b�g

    hold on

    fill3(X(1,:),Y(1,:),Z(1,:),"g",'EdgeColor','k')                     %��ʃv���b�g
    fill3(X(11,:),Y(11,:),Z(11,:),"g",'EdgeColor','k')                     %��ʃv���b�g
    xlim([-300 500]) 
    ylim([-300 500]) 
    zlim([-300 500])
    % view(-135, 10);
end

function cable_plot(Cl_bottom,Cl_top)
%Cl_top�͉~���̏�ʂ̍��W
%Cl_bottom�͉~���̉��ʂ̍��W
ax = gca;
ax.YDir = 'reverse';
ax.ZDir = 'reverse';
X(1,:)= Cl_bottom(1,:);
X(2,:)= Cl_top(1,:);
Y(1,:)= Cl_bottom(2,:);
Y(2,:)= Cl_top(2,:);
Z(1,:)= Cl_bottom(3,:);
Z(2,:)= Cl_top(3,:);


surf(X,Y,Z,'FaceColor','blue','EdgeColor','none')    %���ʃv���b�g
hold on
fill3(X(1,:),Y(1,:),Z(1,:),"b")                     %��ʃv���b�g
fill3(X(2,:),Y(2,:),Z(2,:),"b")                     %��ʃv���b�g
%xlim([-250 350]) 
xlim([-50 700]) 
ylim([-400 350]) 
zlim([-250 550])
%�ʐ^�p
xlim([-300 500]) 
ylim([-400 400]) 
zlim([-400 400])

% % %�A�b�v�p
%  xlim([200 600]) 
%  ylim([-100 300]) 
%  zlim([-250 150])
view(-30,30);
end