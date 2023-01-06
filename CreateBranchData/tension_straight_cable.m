%% �ϐ���`
global save           %�f�[�^�̕ۑ�,CSV�����o��on,off
global animation       %�A�j���[�V����on,off
global robot_plot
global switching 
global time_step
global t_step

idx=0;
count=0;
start_count=0;
t_step = 1.6/33; %�L���̂��������ɂ��邽��(���������菭���͂��)

%% �����ݒ�
save = 1;           %�f�[�^�̕ۑ�,CSV�����o��on,off
animation = 0;      %�A�j���[�V����on,off
robot_plot = 1;
testmode=3;

if testmode==0%�m�[�}�����[�h�œ����Ƃ�
    switching = 11; 
    max_time=1000;    
elseif testmode==1%�P�[�u���̕`����e�X�g����Ƃ�
    switching = 11; 
    max_time=11;
elseif testmode==2 %���~���[�h���e�X�g����Ƃ�
    switching = 13; 
    max_time=1000;  
elseif testmode==3
    switching=14;
    max_time=2000;  
end
time_step=10;

%% �v���O�����J�n
if animation==1
    figure(1)
    Main(-10,15,20,60,1,max_time)
    return
end

tic
for loop=1:2
    for mode=1:2
        for a=-30:5:90
            for b=-30:5:30
                for left=0:5:90
                    for right=0:5:90
                        if loop==1
                            idx=idx+1;
                            continue
                        else
                            count=count+1;
                        end
                        if(count<start_count)
                            continue
                        end
                        
                        if exist("O:\�}�C�h���C�u\Research\���~���򓮍�\����f�[�^\")>0
                            text = "O:\�}�C�h���C�u\Research\���~���򓮍�\����f�[�^\";
                        else
                            text = "C:\Users\MSD\Documents\GitHub\Data\";
                        end

                        name= text+a+ "_" + b+ "_" + right+ "_" + left+"_"+mode+".csv";
                        if(exist(name, 'file')~=0)
                            continue
                        end
            
                        disp(["clac",count,"/",idx,"elapsed time:",toc,"speed[s/1000data]:",1000*toc/(count-start_count),"remind count:",(idx-count),"remind time:", toc/(count-start_count)*(idx-count)/60+"[min]"]);
                        disp(["data:",a,b,right,left,mode]);
                        if left+right>100 || left+right<80
                            continue
                        end
                        try
                            Main(a,b,right,left,mode,max_time)
                        catch exception
                            disp(exception)
                        end
                    end
                end
            end
        end
    end
end
disp("fin")

%% ���C���֐�
function Main(a,b,right,left,mode,max_time)
    %% �v�ݒ�ϐ�
    L_cable=800;
  
    %% �ϐ���`
    global save           %�f�[�^�̕ۑ�,CSV�����o��on,off
    global animation       %�A�j���[�V����on,off
    global robot_plot
    global switching 
    global gamma1
    global phi_n
    global L1
    global L2
    global time_step
    global t_step
    
    L1=L_cable;
    L2=L_cable;

    gamma1 = a*pi/180;      %�P�[�u���Γx
    phi_n = b*pi/180;      %�P�[�u�����ʂ̉�]�p
    the_nR = right*pi/180;     %�E����p
    the_nL = left*pi/180;    %������p

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

       % a1 = 95/180*pi;         %�˂���p
        %a2 = 65/180*pi;
       % a3 = 30/180*pi;         %1���ڂ̎��t���p�i�_���ł�a0�j 

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

        %�d�S�ʒu
        g0 = [0;0;0];
        %g1 = [0;-19.248;91.207];
        g2 = [0;0;l2];
        g3 = [Lc/2;0;La];
        g4 = [0;0;l2];
        %g5 = [0;-83.118;42.198];
        g6 = [0;0;0];

    % data�֘A
        data_r = zeros(max_time,4);
        data_f = zeros(max_time,4);
        data = zeros(max_time,9);
        debug_data = zeros(max_time,10); 

        %���X�̏�����
        t1 = 0; t2 = 0; t3 = 0;t4 = 0;t5 = 0;

        if mode == 1
            the_x_init = -pi/2;
            the_n = the_nR;
        else
            the_x_init = pi/2;
            the_n = -the_nL;
        end
    
        
    %% �P�[�u�����̌v�Z
        a_w=6;
        b_w=30;
        
        x_cable=Plane2World([1;0;0]);
        y_cable=Plane2World([0;1;0]);
        z_cable=Plane2World([0;0;1]);
        Sigma_cable=Plane2World([0;0;0]);

        O_n=Plane2World([L_cable;0;0]);
        P_e=Plane2World(World2Plane(O_n)+L_cable*[cos(the_n);sin(the_n);0]);
        e1=Plane2World([-1;0;0]);
        e2=(P_e-O_n)/norm(P_e-O_n);

        e2_=axang2rotm([z_cable.' -pi/2])*e2;
    
        Sigma_cable_=Sigma_cable-b_w*y_cable;
        P_e_=P_e+b_w*e2_;

        %On_�̓��o
        fun = @(x)norm((Sigma_cable_+x(1)*e1)-(P_e_+x(2)*e2));
        [x,fval] = fminunc(fun,[0,0], optimoptions('fminunc','display',"none"));%���ʂɌv�Z���ĂƂ��Ă��������ǖʓ|�Ȃ̂Ő��l�v�Z�ɓ�����
        t=x(1);
        s=x(2);
        O_n_=Sigma_cable_+t*e1; %P2_+s*e2�ł��悢

        theta_rute=(pi-the_n)/2;

        R_a=norm(O_n_-O_n)-a_w;
        R_rute=(R_a*sin(theta_rute))/(1-sin(theta_rute));
        O_R=O_n_+(e1+e2)/norm(e1+e2)*(R_a+R_rute);
        O_R_=O_R+O_n-O_n_;
        the_arc=pi/2-acos(dot(e1,-e2_));
        
        Q1=O_R_-R_rute*y_cable;
        Q2=O_R_+R_rute*e2_;
        
        L_arc=R_rute*the_arc;

        if Q1(1)<0
            disp("L_cable is too small");
        end

    %% �p���x�Ȃǂ̐ݒ�
        %�^�������
    %   t_step = 0.0485; %50[ms] �L���������Ƌ��������������Ȃ�D48ms�ł���
        r_v = 13.5; %�L�����a[mm]
        vel = 50; %���򎞂̌�ւ̈ړ����x[mm/s]
        L_gap = 150;%����_���O�Ŏ~�܂鋗��(��^�C�����a�̔���)

        t_start=0;t_branch_s=0;t_branch_c=0;t_branch_e=0;t_end=0;

        Ln=0;
        flag = 0;
        for t=1:max_time
            Ln=Ln+omega(t)*pi/180*r_v*t_step;;
            if flag == 0    %���{�b�g�̏����l�v�Z
                if Ln>Lc
                    t_start=t;
                    flag=flag+1;
                end
            elseif flag==1
                if Ln>=norm(Q1-Sigma_cable)
                    t_branch_s=t;
                    flag=flag+1;
                end
            elseif flag==2
                if Ln>=norm(Q1-Sigma_cable)+L_arc/2
                    t_branch_c=t;
                    flag=flag+1;
                end
            elseif flag==3
                if Ln>=norm(Q1-Sigma_cable)+L_arc
                    t_branch_e=t;
                    flag=flag+1;
                end
            elseif flag==4
                if Ln>=norm(Q1-Sigma_cable)+L_arc+norm(P_e-Q2)
                    t_end=t;
                    break;
                end
            end
        end
        max_time=t_end;
        
        %�]���I�Ɍ��܂���̂���
        if switching==14
            t_bra=t_branch_e-t_branch_s;
        else
            t_bra = 2*L_gap/vel;                    %�쓮�ւ�����ɂ����鎞��(80/50=1.6[s])
        end
        
        n_bra = round(t_bra/t_step);            %����ɂ�����J�E���g
        omega_c =vel/r_v*180/pi ;                 %[deg/s] �ʏ푖�s���̑��x    


    %% XYX�I�C���[�p�ƃN�H�[�^�j�I���ɂ�镪�򎞂̎p���\��

        %���򓮍�ɂ�����p
        E0_R = [-pi/2,0,0];               %�I�C���[�p�̏����l
        E1_R = [-pi/2-phi_n,-pi/2+the_nL,phi_nw];  %�T�u�P�[�u���𔲂��p�� 
        E2_R = [-pi/2-phi_n,-the_nR,atan((-sin(gamma1)*sin(the_nR) + cos(gamma1)*cos(the_nR)*sin(phi_n))/cos(phi_n)*cos(gamma1))]; 

        E0_L = [pi/2,0,0];               %�I�C���[�p�̏����l
        E1_L = [pi/2-phi_n,-pi/2+the_nR,-phi_nw];  %�T�u�P�[�u���𔲂��p�� 
        E2_L = [pi/2-phi_n,-the_nL,atan((sin(gamma1)*sin(the_nL) + cos(gamma1)*cos(the_nL)*sin(phi_n))/cos(phi_n)*cos(gamma1))]; 

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
            tension_angle = pi/2*1.2;
            %tension_angle = acos((d+2*rw)/(2*Rw));
        else
            quat0 = quaternion(E0_L,'euler','XYX','frame');   %�N�H�[�^�j�I���̏����l
            quat1 = quaternion(E1_L,'euler','XYX','frame');   %�T�u�P�[�u���𔲂��p��
            quat2 = quaternion(E2_L,'euler','XYX','frame');   %�����̎p��
            E2 = E2_L;
            tension_angle = -pi/2*1.2;
            %tension_angle = -acos((d+2*rw)/(2*Rw));
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


    
    %% �ʒu�Ǝp���̃��X�g�𐶐�
        List_time=[];
        List_Ln=[];
        List_P=[];
        List_C=[];
        List_the_x=[];
        List_the_y=[];
        List_the_z=[];
    
        Ln=0;
        for t=1:t_end
            Ln=Ln+omega(t)*pi/180*r_v*t_step;
            if t<=t_branch_s%�X�^�[�g�`�J�[�u�J�n
                t1=t1+1;
                the_xf = X(1); the_yf = Y(1); the_zf = Z(1);
            elseif t<=t_branch_c%�J�[�u�J�n�`��s����P�[�u��
                t2=t2+1;
                the_xf = X(round(t2/t_step)); the_yf = Y(round(t2/t_step)); the_zf = Z(round(t2/t_step));
            elseif t<=t_branch_e%��s����P�[�u���`�J�[�u�I��
                t3=t3+1;
                the_xf = X (round((t2+t3)/t_step)); the_yf = Y( round((t2+t3)/t_step)); the_zf = Z(round((t2+t3)/t_step));
            elseif t<=t_end%�J�[�u�I���`�I���
                t4=t4+1;
                the_xf = X(n_bra); the_yf = Y(n_bra); the_zf = Z(n_bra);
            else
                disp("error:out of range");
                break;
            end

            if 0<=Ln && Ln<norm(Q1-Sigma_cable)
                P=Ln*x_cable+Sigma_cable;
            elseif norm(Q1-Sigma_cable)<=Ln && Ln<norm(Q1-Sigma_cable)+L_arc
                t_arc=(Ln-norm(Q1-Sigma_cable))/(R_rute);
                P=O_R_-y_cable*R_rute*cos(t_arc)+x_cable*R_rute*sin(t_arc);
            elseif norm(Q1-Sigma_cable)+L_arc<=Ln && Ln<norm(Q1-Sigma_cable)+L_arc+norm(P_e-Q2)
                P=Q2+(Ln-L_arc-norm(Q1-Sigma_cable))*e2;
            else
                P=P_e;
            end
            P_f=World2Plane(P);
            C_f=Plane2World(P_f)+ R("x",the_xf)*R("y",the_yf)*[Rw*sin(the_zf);d/2+rw-Rw*cos(the_zf);0];
            P_f=Plane2World(P_f);
            List_time=[List_time t];
            List_Ln=[List_Ln Ln];
            List_P=[List_P P_f];%List_Pf(:,2)�̂悤�ɂ��Ď��o��
            List_C=[List_C C_f];
    
            List_the_x=[List_the_x the_xf];
            List_the_y=[List_the_y the_yf];
            List_the_z=[List_the_z the_zf];
        end
    
    %% ���{�b�g�ɂ�鑖�s
        Lnf=0;
    for t = t_start:max_time%�Œ���i�񂾂Ƃ���������l�ɂ�����
        Lnf=Lnf+omega(t)*pi/180*r_v*t_step;
        P_f=List_P(:,t);
        C_f=List_C(:,t);
        best_idx=0;
        best_diff=1000000;
        for i=1:t
            diff=norm(List_C(:,i)-C_f);
            if abs(diff-Lc)<best_diff
                best_diff=abs(diff-Lc);
                best_idx=i;
            end
        end
        P_r=List_P(:,best_idx);
        C_r=List_C(:,best_idx);
        Lnr=List_Ln(:,best_idx);

        if switching==14
            %% switching==14
                the_xf=List_the_x(:,t);
                the_yf=List_the_y(:,t);
                the_zf=List_the_z(:,t);
                the_xr=List_the_x(:,best_idx);
                the_yr=List_the_y(:,best_idx);
                the_zr=List_the_z(:,best_idx);
                omega_f=omega(t);
        end
        
        %% �e�֐ߊp�x�̎����v�Z
            OrCr=C_r-P_r;
            OfCf=C_f-P_f;
            OrOf=R("y",-gamma1)*(P_f-P_r);
            OrCf =OrOf+OfCf;
            
            fun = @(x)Joint_angle_calc8(x,the_xr,the_yr,the_zr,the_xf,the_yf,the_zf,...
                                               gamma1,a1,a2,a3,Lc,OrCr,OfCf,OrOf,g0,g2,g3,g4,g6,w0,w2,w3,w4,w6,mode);
            [x,fval,exitflag,output]= fzero(fun,0);
            output;
            %�����������ʂ�p���āC�֐ߊp�x���Z�o
            [F,the_2r,the_3r,the_1r,the_2f,the_3f,the_1f] ...
                    = Joint_angle_calc8(x,the_xr,the_yr,the_zr,the_xf,the_yf,the_zf,...
                                        gamma1,a1,a2,a3,Lc,OrCr,OfCf,OrOf,g0,g2,g3,g4,g6,w0,w2,w3,w4,w6,mode);


              
        %% �֐ߊp�x�f�[�^�̕ۑ�
             data_r(t,:) = [t*t_step the_1r the_2r the_3r];
             data_f(t,:) = [t*t_step the_1f the_2f the_3f];
             data(t,:) = [the_1f the_2f the_3f the_1r the_2r the_3r omega_f -1 t*t_step];%-1�̂Ƃ���ɂ͂��Ƃ���omega_r������
             debug_data(t,:) = [the_xr,the_yr,the_zr,the_xf,the_yf,the_zf,Lnf,Lnr,-1,t*t_step];%-1�̂Ƃ���ɂ͌ォ��^��Lnr������

        %% �`��
            if animation == 1
                if mod(t,time_step)~=0
                    continue
                end
                %% �f�[�^�̕`��
    
                    lw = 45;  %��~�������i�^�C�������j
    
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
                    Pr_Place=R("y",-gamma1)*(P_r);
                    m1_coord = R("y",gamma1)*(Pr_Place+OrCr + R("x",the_xr)*R("y",the_yr)*R("z",the_zr)*R("z",-the_3r+pi/2)*horzcat(m1_top,m1_bottom));
                    m2_coord = R("y",gamma1)*(Pr_Place+OrCr + R("x",the_xr)*R("y",the_yr)*R("z",the_zr)*R("z",-the_3r+pi/2)*R("x",a2)*R("z",-the_2r)*horzcat(m2_top,m2_bottom));
                    m3_coord = R("y",gamma1)*(Pr_Place+OrCr + R("x",the_xr)*R("y",the_yr)*R("z",the_zr)*R("z",-the_3r+pi/2)*R("x",a2)*R("z",-the_2r)*R("x",-a1)*R("z",-the_1r)*R("z",-pi/2)*horzcat(m3_top,m3_bottom));
                    m4_coord = R("y",gamma1)*(Pr_Place+OrCf + R("x",the_xr)*R("y",the_yr)*R("z",the_zr)*R("z",-the_3r+pi/2)*R("x",a2)*R("z",-the_2r)*R("x",-a1)*R("z",-the_1r)*R("z",-pi/2)*R("y",2*a3)*R("z",-pi/2)*R("z",the_1f)*horzcat(m4_top,m4_bottom));
                    m5_coord = R("y",gamma1)*(Pr_Place+OrCf + R("x",the_xr)*R("y",the_yr)*R("z",the_zr)*R("z",-the_3r+pi/2)*R("x",a2)*R("z",-the_2r)*R("x",-a1)*R("z",-the_1r)*R("z",-pi/2)*R("y",2*a3)*R("z",-pi/2)*R("z",the_1f)*R("x",a1)*R("z",the_2f)*horzcat(m5_top,m5_bottom));
                    m6_coord = R("y",gamma1)*(Pr_Place+OrCf + R("x",the_xr)*R("y",the_yr)*R("z",the_zr)*R("z",-the_3r+pi/2)*R("x",a2)*R("z",-the_2r)*R("x",-a1)*R("z",-the_1r)*R("z",-pi/2)*R("y",2*a3)*R("z",-pi/2)*R("z",the_1f)*R("x",a1)*R("z",the_2f)*R("x",-a2)*R("z",the_3f+pi/2)*horzcat(m6_top,m6_bottom));
    
    
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
                    w1r_coord = R("y",gamma1)*(Pr_Place+OrCr + R("x",the_xr)*R("y",the_yr)*R("z",the_zr)*W1);
                    w2r_coord = R("y",gamma1)*(Pr_Place+OrCr + R("x",the_xr)*R("y",the_yr)*R("z",the_zr)*W2);
                    w1f_coord = R("y",gamma1)*(Pr_Place+OrCf + R("x",the_xr)*R("y",the_yr)*R("z",the_zr)*R("z",-the_3r+pi/2)*R("x",a2)*R("z",-the_2r)*R("x",-a1)*R("z",-the_1r)*R("z",-pi/2)*R("y",2*a3)*R("z",-pi/2)*R("z",the_1f)*R("x",a1)*R("z",the_2f)*R("x",-a2)*R("z",the_3f+pi/2)*W2);
                    w2f_coord = R("y",gamma1)*(Pr_Place+OrCf + R("x",the_xr)*R("y",the_yr)*R("z",the_zr)*R("z",-the_3r+pi/2)*R("x",a2)*R("z",-the_2r)*R("x",-a1)*R("z",-the_1r)*R("z",-pi/2)*R("y",2*a3)*R("z",-pi/2)*R("z",the_1f)*R("x",a1)*R("z",the_2f)*R("x",-a2)*R("z",the_3f+pi/2)*W1);

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
                if(robot_plot == 1)
                    
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
                end
                     %�P�[�u���̕`��
                    [Xc,Yc,Zc] = cylinder(3);
                    Zc = (Zc - 0.5).*2*L1;                        %�~���̍����␳
    
                    c_top     = [Xc(1,:);Yc(1,:);Zc(1,:)];      %��~�����W�i�㉺�e�����j
                    c_bottom  = [Xc(2,:);Yc(2,:);Zc(2,:)];
                    c_top = R("y",gamma1)*R("y",pi/2)*c_top;
                    c_bottom = R("y",gamma1)*R("y",pi/2)*c_bottom;
                    cable_plot(c_top,c_bottom)
    
                    %����P�[�u��(���C���E�T�u)
                    
                    L_cable=1000;
    
                    [Xc2,Yc2,Zc2] = cylinder(3);
                    Zc2 = Zc2*L_cable;
                    c_top2     = [Xc2(1,:);Yc2(1,:);Zc2(1,:)];      %��~�����W�i�㉺�e�����j
                    c_bottom2  = [Xc2(2,:);Yc2(2,:);Zc2(2,:)];
    
                    %�����͕`�悾���Ȃ̂œK���ɂ��Ă���
                    c_top2h =    R("y",gamma1)*([L1;0;0]+R("x",-phi_n)*R("z",the_nR)*R("y",pi/2)*c_top2);
                    c_bottom2h = R("y",gamma1)*([L1;0;0]+R("x",-phi_n)*R("z",the_nR)*R("y",pi/2)*c_bottom2);
                    c_top2s =    R("y",gamma1)*([L1;0;0]+R("x",-phi_n)*R("z",-the_nL)*R("y",pi/2)*c_top2);
                    c_bottom2s = R("y",gamma1)*([L1;0;0]+R("x",-phi_n)*R("z",-the_nL)*R("y",pi/2)*c_bottom2);
    
                    cable_plot(c_top2h,c_bottom2h)
                    cable_plot(c_top2s,c_bottom2s)
    
                    %�Ȑ���̃P�[�u�����v���b�g����
                    curve_cable_plot(O_R_,e2_,-e2,R_rute,the_arc)
    
                    %�K�v�ȓ_���v���b�g����                    
                    if false
                        plotP(Sigma_cable_,"��cable");
                        plotP(Sigma_cable_,"��cable'");
        
                        plotP(O_n,"On");
                        plotP(O_n_,"On'");
                        
                        plotP(O_R,"OR");
                        plotP(O_R_,"OR'");
        
                        plotP(P_e,"Pe");
                        plotP(P_e_,"Pe'");
                    end
    
                    plotP(P_f,"P_f",100,20);
                    plotP(C_f,"C_f",100,20);
    
                    plotP(P_r,"P_r",100,20);
                    plotP(C_r,"C_r",100,20);
    
                    xlabel('x[mm]')
                    ylabel('y[mm]')
                    zlabel('z[mm]')
                    pause(0.0001)
            end
    end

    %% ��ւ̊p���x���v�Z
    chect_num=10;
        for t = t_start:t_end
            if t<t_start+chect_num+1
                data(t,8)=(debug_data(t+chect_num,8)-debug_data(t,8))/(chect_num*r_v*t_step)*180/pi;
            elseif t>t_end-chect_num
                data(t,8)=(debug_data(t,8)-debug_data(t-chect_num,8))/(chect_num*r_v*t_step)*180/pi;
            else
                data(t,8)=(debug_data(t+chect_num,8)-debug_data(t-chect_num,8))/(2*chect_num*r_v*t_step)*180/pi;
            end
        end
        for t = 2:t_end
            debug_data(t,9)=debug_data(t-1,9)+data(t,8)*r_v*t_step*pi/180;%Lnr_real=�O���Lnr_real+����i�񂾗�
        end
        
    %% ���ʂ̏o��
        
        data=data(t_start:t,:);
        debug_data=debug_data(t_start:t,:);
        T = size(data,1);
        if save ==1
        %�f�[�^�̏����o�� 
            if exist("O:\�}�C�h���C�u\Research\���~���򓮍�\����f�[�^\")>0
                text = "O:\�}�C�h���C�u\Research\���~���򓮍�\����f�[�^\"+gamma1*180/pi+ "_" + phi_n*180/pi+ "_" + the_nR*180/pi+ "_" + the_nL*180/pi+"_"+mode;
            else
                text = "C:\Users\MSD\Documents\GitHub\Data\"+gamma1*180/pi+ "_" + phi_n*180/pi+ "_" + the_nR*180/pi+ "_" + the_nL*180/pi+"_"+mode;
            end
            writematrix(data,text+'.csv')
        end

    %% �O���t�`��i�p���j
    if true && animation==1
           %�����o���f�[�^�̕`��
           figure(2)
           plot(data_r(1:T,1),debug_data(1:T,1)*180/pi,'LineWidth',2)
           hold on
           plot(data_r(1:T,1),debug_data(1:T,2)*180/pi,'LineWidth',2)
           plot(data_r(1:T,1),debug_data(1:T,3)*180/pi,'LineWidth',2)
           plot(data_r(1:T,1),debug_data(1:T,4)*180/pi,'LineWidth',2)
           plot(data_r(1:T,1),debug_data(1:T,5)*180/pi,'LineWidth',2)
           plot(data_r(1:T,1),debug_data(1:T,6)*180/pi,'LineWidth',2)
           legend('��xr','��yr','��zr','��xf','��yf','��zf','Location','NorthEast')
           ylim([-180 180]) 
           yticks(-180:30:180)
           xlabel('���ԁC�X�e�b�v')
           ylabel('�p�x[deg]')
    end

    %% �O���t�`��i�ʒu�j
    if true && animation==1
           %�����o���f�[�^�̕`��
           figure(3)
           plot(data_r(1:T,1),debug_data(1:T,7),'LineWidth',2)%Lnf
           hold on
           plot(data_r(1:T,1),debug_data(1:T,8),'LineWidth',2)%Lnr
           plot(data_r(1:T,1),debug_data(1:T,9),'LineWidth',2)%Lnr_real
           legend('Lnf','Lnr','Lnr_real','Location','NorthEast')
           xlim([0 max_time*t_step]) 
           ylim([-100 Lnf]) 
           yticks(-100:100:Lnf)
           xlabel('���ԁC�X�e�b�v')
           ylabel('�ʒu[mm]')
    end

    %% �O���t�`��i�p���x�j
    if true && animation==1
           %�����o���f�[�^�̕`��
           figure(4)
           plot(data_r(1:T,1),data(1:T,7),'LineWidth',2)%�p���x
           hold on
           plot(data_r(1:T,1),data(1:T,8),'LineWidth',2)%�p���x
           legend('��f','��r','Location','NorthEast')
           xlim([0 max_time*t_step]) 
           ylim([-1000 1000]) 
           yticks(-1000:100:1000)
           xlabel('���ԁC�X�e�b�v')
           ylabel('�p���x[deg/s]')
    end

    %% �O���t�`��i���j
    if false && animation==1
           %�����o���f�[�^�̕`��
           figure(2)
           plot(data_r(1:T,1),data(1:T,1)*180/pi,'LineWidth',2)
           hold on
           plot(data_r(1:T,1),data(1:T,2)*180/pi,'LineWidth',2)
           plot(data_r(1:T,1),data(1:T,3)*180/pi,'LineWidth',2)
           plot(data_r(1:T,1),data(1:T,4)*180/pi,'LineWidth',2)
           plot(data_r(1:T,1),data(1:T,5)*180/pi,'LineWidth',2)
           plot(data_r(1:T,1),data(1:T,6)*180/pi,'LineWidth',2)

           legend('1����(�O)','2����(�O)','3����(�O)','1����(��)','2����(��)','3����(��)','Location','NorthEast')
           ylim([-180 180]) 
           yticks(-180:30:180)
           xlabel('���ԁC�X�e�b�v')
           ylabel('�p�x[deg]')

           %figure(3)
           plot(data_r(1:T,1),data(1:T,7),'LineWidth',2)
           hold on
           plot(data_r(1:T,1),data(1:T,8),'LineWidth',2)
           ylim([0 350])
           xlabel('���ԁC�X�e�b�v')
           ylabel('�p���x[deg/s]')
    end
end

%% ��]�s��̌v�Z
function X = R(i,j)
    if i == "x"
        X = [1 0 0 ; 0 cos(j) -sin(j) ; 0 sin(j) cos(j)];
    elseif i == "y"
        X = [cos(j) 0 sin(j) ; 0 1 0 ; -sin(j) 0 cos(j)];
    elseif i == "z"
        X = [cos(j) -sin(j) 0 ; sin(j) cos(j) 0 ; 0 0 1];
    end    
end

%% Ln2��^���āCLn1��Ԃ��֐�
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


%% ���򃂁[�h�X�C�b�`�̂��߂̓���Ȋ֐��i�E���獶�j
function [Ln2,OrCr,OfCf,OrOf,OrOw1r,OrOw2r,OfOw1f,OfOw2f] = node_cal_tension(the_xr,the_yr,the_zr,the_xf,the_yf,the_zf,phi_nr1,the_nr,phi_nf1,the_nf,Ln1,Lc,Rw,rw,d,mode)

    %gap  = 2*Rw*cos(the_zr)-d-2*rw;

        %���
        OrCr =   R("x",the_xr)*R("y",the_yr)*[Rw*sin(the_zr);d/2+rw-Rw*cos(the_zr);0];
        OrOw1r = R("x",the_xr)*R("y",the_yr)*[0;d/2+rw;0];
        OrOw2r = R("x",the_xr)*R("y",the_yr)*[2*Rw*sin(the_zr);d/2+rw-2*Rw*cos(the_zr);0];
        %�O��
        OfCf =   R("x",the_xf)*R("y",the_yf)*[Rw*sin(the_zf);d/2+rw-Rw*cos(the_zf);0];
        OfOw1f = R("x",the_xf)*R("y",the_yf)*[0;d/2+rw;0];
        OfOw2f = R("x",the_xf)*R("y",the_yf)*[2*Rw*sin(the_zf);d/2+rw-2*Rw*cos(the_zf);0];

    
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
    xlim([-300 1500]) 
    ylim([-300 1500]) 
    zlim([-300 1500])
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
    xlim([-300 1500]) 
    ylim([-300 1500]) 
    zlim([-300 1500])
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
    
    
    % % %�A�b�v�p
    %  xlim([200 600]) 
    %  ylim([-100 300]) 
    %  zlim([-250 150])

end

%�_O�𒆐S��e1,e2�x�N�g���ɂ���č\�������~�ʂ�`�悷��
function curve_cable_plot(O,e1,e2,R,theta_max)
    %Cl_top�͉~���̏�ʂ̍��W
    %Cl_bottom�͉~���̉��ʂ̍��W
    ax = gca;
    ax.YDir = 'reverse';
    ax.ZDir = 'reverse';
    
    theta = linspace(0, theta_max, 100); % x coordinates
    points=O+e1*R*cos(theta)+e2*R*sin(theta);
    x=points(1,:);
    y=points(2,:);
    z=points(3,:);
    
    scatter3(x, y, z, 20, 'filled')
    
    %surf(X,Y,Z,'FaceColor','blue','EdgeColor','none')    %���ʃv���b�g
    hold on
    %fill3(X(1,:),Y(1,:),Z(1,:),"b")                     %��ʃv���b�g
    %fill3(X(2,:),Y(2,:),Z(2,:),"b")                     %��ʃv���b�g
    %xlim([-250 350]) 
    xlim([-50 1500]) 
    ylim([-400 350]) 
    zlim([-250 550])
    %�ʐ^�p
    xlim([-300 1500]) 
    ylim([-450 750]) 
    zlim([-450 450])
    
    
    % % %�A�b�v�p
    %  xlim([200 600]) 
    %  ylim([-100 300]) 
    %  zlim([-250 150])
    %view(-95,10);
    view(-65,25);
end

function plotP(P,label,plotsize,fontsize)
    x=P(1,:);
    y=P(2,:);
    z=P(3,:);
    
    if nargin<2
        plotsize=7;
    end
    
    if nargin<3
        fontsize=20;
    end

    scatter3(x,y,z,plotsize,"fill");
    
    if nargin>1
        text(x,y,z,label,"FontSize",fontsize)
    end
end

%% RCM���S���W�̌v�Z1(�E���򃂁[�h�C�����򃂁[�h)kakikaezumi
function [Ln1,OrCr,OfCf,R1,R2] = CalcOrOf(the_xr,the_yr,the_zr,the_xf,the_yf,the_zf,phi_nr1,the_nr,phi_nf1,the_nf,Ln2,Lc,Rw,rw,d,mode)

    if mode == 1 %�E���򃂁[�h
        %���
        OrCr =   R("x",the_xr)*R("y",the_yr)*[Rw*sin(the_zr);d/2+rw-Rw*cos(the_zr);0];

        %�O��
        OfCf =   R("x",the_xf)*R("y",the_yf)*[Rw*sin(the_zf);d/2+rw-Rw*cos(the_zf);0];

    elseif mode == 2 %�����򃂁[�h
        %���
        OrCr =   R("x",the_xr)*R("y",the_yr)*[-Rw*sin(the_zr);-d/2-rw+Rw*cos(the_zr);0];

        %�O��
        OfCf =    R("x",the_xf)*R("y",the_yf)*[-Rw*sin(the_zf);-d/2-rw+Rw*cos(the_zf);0];

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

    
end

%% ���W�ϊ��n�̊֐�
function [P]=Plane2World(P)
    global gamma1
    global phi_n
    P =  R("y",gamma1)*R("x",-phi_n)*P;
end

function [P]=World2Plane(P)
    global gamma1
    global phi_n
    P =  R("x",phi_n)*R("y",-gamma1)*P;
end

%% ���{�b�g�̃p�����[�^�n�̊֐�
function omega=omega(t)
    omega=212;%deg/s
end

