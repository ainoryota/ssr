%% 右分岐のみに特化したプログラム
%11/28にリンクフレームの張り方を修正

% CSVに出力する

%% 初期設定
%分岐モード
    mode = 1;   %右分岐
%   mode = 2;   %左分岐
%   mode = 3;   %分岐モードの入れ替え

%どの解を用いるかの選択
    check_r = 1;
    check_f = 2;
    
%定数
    Rw = 83/2;      %タイヤの直径，内径
    rw = 22.185/2;
    d = 6;          %ケーブル直径
    phi_nw = atan(12/23.5);   %p88タイヤの形状によってケーブルを抜く角度

    rm = 45/2;  %基準円柱半径（モータ部分）
    lm = 50;  %基準円柱長さ（モータ部分）
    rj = 35; %接触判定用の球の半径
    
%data用の入れ物
    count1 = zeros(500,3);
    count2 = zeros(500,3);
    data_r = zeros(500,4);
    data_f = zeros(500,4);
    data = zeros(500,8);
    data_r_full = zeros(180,4);
    data_f_full = zeros(180,4);
    moment = zeros(180,2);
    data_collide_r = zeros(1000,2);
    data_collide_f = zeros(1000,2);
    
    


%初期化
    c  = 0;
    flag_fin = 0;
%謎の値
    phi_min_evaluate = -2.4582;      %93,63,30のパラメータ
    phi_max_evaluate = -0.1561;
    phi_evaluate = 0.1561;

%% リンクのねじれ角の洗い出し
%alpha3も適当な範囲
    alpha2 = 45:3:70;
    alpha3 = 3:3:45;

%考えられるすべての組み合わせ
%             (alpha1,alpha2,alpha3)
    comb = combvec(alpha2,alpha3);
    comb2 = [comb(1,:)+comb(2,:);comb(1,:);comb(2,:)];

    [m,n] = size(comb2);      %行列サイズの取得
    angle_V = zeros(m,n);
%条件に適さないものの削除

    t_ = 1;
    for i = 1:n
    
        if (comb2(1,i)+comb2(2,i)) >= 120 
            angle_V(:,t_) = comb2(:,i);
            t_ = t_+1;
        end
    end

    angle_V(:,t_:n) = [];
    [m_,n_] = size(angle_V);


%% 分岐点の条件だし
%これの根拠についてはノート，資料に記述
    angle1 = 20:10:80;
    angle2 = -80:10:-20;
    angle3 = -30:5:30;

%考えられるすべての組み合わせ
%             (gamma1,phi_n ,the_nh,the_ns)
    comb = combvec(angle3,angle3,angle1,angle2);
    [m,n] = size(comb);      %行列サイズの取得
    node_V = zeros(m,n);

%条件に適さないものの削除
    k = 1;
    
    for i = 1:n
        A = R("y",comb(1,i)/180*pi)*R("x",comb(2,i)/180*pi)*R("z",comb(3,i)/180*pi)*[1;0;0];
        C = R("y",comb(1,i)/180*pi)*R("x",comb(2,i)/180*pi)*R("z",comb(4,i)/180*pi)*[1;0;0];
        B = [0;0;A(3)];
        D = [0;0;C(3)];

        x = asin(norm(B)/norm(A))*180/pi;
        y = asin(norm(D)/norm(C))*180/pi;

        if (comb(3,i)-comb(4,i))>=100 && -30<=x && -30<=y && x<=30 && y<=30
            node_V(:,k) = comb(:,i);
            k = k+1;
        end
    end

    node_V(:,k:n) = [];
    [m,n] = size(node_V); 
    Data_sim = zeros(n_,20);
    p = 1;


%分割数
    n_d = 150;
    n_b = 2;
    
%初期化
    phi_min = -pi/2;
    phi_max = -pi/2;
    flag_fin = 0;
%% メインループ
    
    %確認用
    
    a1 = 79/180*pi;         %ねじれ角
    a2 = 64/180*pi;
    a3 = 15/180*pi;         %1軸目の取り付け角（論文ではa0）
    
    gamma1 = 0*pi/180;      %ケーブル斜度
    phi_n =  20*pi/180;      %ケーブル平面の回転角
    the_n =  60*pi/180;     %右旋回角
    the_ns = 0*pi/180;    %左旋回角
    
    %各リンクの重さ，重心位置，長さ
    Lc = 300;
    l2_min = (Lc-80)/2;
    l1 = 90;                %各モーターの中心点からの距離
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
    
    Data_sim(p,1) = a1/pi*180;
    Data_sim(p,2) = a2/pi*180;
    Data_sim(p,3) = a3/pi*180;
    Data_sim(p,4) = gamma1/pi*180;
    Data_sim(p,5) = phi_n/pi*180;
    Data_sim(p,6) = the_n/pi*180;
    Data_sim(p,7) = the_ns/pi*180;

    Data_sim(p,10) = l1;
    Data_sim(p,11) = l2;
    Data_sim(p,12) = l3;
    Data_sim(p,13) = Lc;
    Data_sim(p,14) = La;

%分岐動作における姿勢（オイラー角，クォータニオン,すべてXZXオイラーが基準）
    E0 = [0,0,0];               %オイラー角の初期値
    E1 = [phi_n,pi/2+the_ns,phi_nw];  %サブケーブルを抜く姿勢

%分岐後の姿勢 p84  
    E2 = [phi_n,the_n,atan2(-(sin(gamma1)*sin(the_n)+cos(gamma1)*cos(the_n)*sin(phi_n)),cos(phi_n)*cos(gamma1))]; 

    quat0 = quaternion(E0,'euler','XZX','frame');   %クォータニオンの初期値
    quat1 = quaternion(E1,'euler','XZX','frame');   %サブケーブルを抜く姿勢
    quat2 = quaternion(E2,'euler','XZX','frame');   %分岐後の姿勢

    %刻み幅(ものすごく頭いい気がする．)
    n_count01 = round(dist(quat0,quat1)/n_b/pi*180);
    n_count12 = round(dist(quat1,quat2)/n_b/pi*180);   %1度刻みだったのをn_b度刻みにする
    n_cnt = n_count01+n_count12;

    %分岐時の目標角度(補間後)
    [X_01,Z_01,Y_01] = slerp_xzy_euler(quat0,quat1,n_count01);
    [X_12,Z_12,Y_12] = slerp_xzy_euler(quat1,quat2,n_count12);
    X = [X_01;X_12];   %連結
    Z = [Z_01;Z_12];
    Y = [Y_01;Y_12];

    %諸々の初期化
    buf1 = 0; buf2 = 0; buf3 = 0;
    t1 = 0; t2 = 0; t3 = 0;
    flag = 1;
    cnt = 0;
    L1 = Lc; %ケーブルの長さ
    L2 = Lc;
    
    omega = 210; %[deg/s] 通常走行時の速度


    %1ステップずつfsolveの計算するところ
        %for t = 1:500
            
%                 if flag == 1 %分岐直前    
%                     t1 = t1+1;
%                         phi_nr1 = 0/180*pi; the_nr = 0/180*pi ;phi_nr2 = 0/180*pi; phi_nf1 = 0/180*pi; the_nf = 0/180*pi; phi_nf2 = 0/180*pi;
%                         psi_r = 0/180*pi; phi_r = 0/180*pi; the_r = 0/180*pi;
% 
%                         phi_f = X(t1);
%                         the_f = Z(t1);
%                         psi_f = Y(t1);
%                         
%                         Ln1 = Len(phi_r,the_r,psi_r,phi_f,the_f,psi_f,Lc,Rw,r,d);
%                         [Ln2,OrCr,OfCf,OrOf] = node_cal2(psi_r,phi_r,the_r,psi_f,phi_f,the_f,0,0,Ln1,Lc,Rw,r,d);
%                         %[Ln2,OrCr,OfCf,OrOf,flag_] = node_cal(psi_r,phi_r,the_r,psi_f,phi_f,the_f,phi_nr1,phi_nr2,the_nr,phi_nf1,phi_nf2,the_nf,Ln1,Lc,Rw,r,d);
%                         omega_r = 2*omega*(t1/n_cnt)^3 - 3*omega*(t1/n_cnt)^2 + omega;
%                         omega_f = omega_r; % 後で変更
%                         
%                         %分岐の修了判定
%                         if t1 == n_count01 + n_count12
%                             L_def = Ln1;
%                             flag = 2;
%                         end
% 
%                 elseif flag == 2 %分岐中
%                          
%                         phi_nr1 = 0/180*pi; the_nr = 0/180*pi ;phi_nr2 = 0/180*pi; phi_nf1 = E2(1); the_nf = E2(2); phi_nf2 = E2(3);
%                         phi_nf1 = 0; the_nf = 0; phi_nf2 = 0;
%                         
%                         
%                         psi_r = 0/180*pi; phi_r = 0/180*pi; the_r = 0/180*pi;
%                         psi_f = 0/180*pi; phi_f = 0/180*pi; the_f = 0/180*pi; 
%                         
%                          phi_f = X(n_count01 + n_count12);
%                          the_f = Z(n_count01 + n_count12);
%                          psi_f = Y(n_count01 + n_count12);
%                          
%                         
%                         Ln1 = L_def - (6*(t2/n_d)^5 - 15*(t2/n_d)^4 + 10*(t2/n_d)^3)*L_def;
%                         %Ln1 = L_def - (t2/n_d)*L_def;
%                         [Ln2,OrCr,OfCf,OrOf] = node_cal2(psi_r,phi_r,the_r,psi_f,phi_f,the_f,phi_n,the_n,Ln1,Lc,Rw,r,d);
%                         omega_r = L_def*(30*(t2/n_d)^4-60*(t2/n_d)^3+30*(t2/n_d)^2)*(2/(2*r+d))*(180/pi)/7.5;
%                         omega_f = omega_r;
%                         
%                         t2 = t2+1;
%                         %終了判定
%                         if t2 == n_d+1
%                             flag = 3;
%                         end
%                         if t2 == n_d/2
%                             
%                         end
%  
%                 elseif flag == 3 %分岐直前
%                     Ln1 = 0;
%                     t3 = t3+1;
%                         phi_nr1 = 0/180*pi; the_nr = 0/180*pi ;phi_nr2 = 0/180*pi; phi_nf1 = E2(1); the_nf = E2(2); phi_nf2 = E2(3);
%                         phi_nf1 = 0; the_nf = 0; phi_nf2 = 0;
%                         %psi_f = 0/180*pi; phi_f = 0/180*pi; the_f = 0/180*pi;  
%                         
%                         phi_f = X(n_count01 + n_count12);
%                         the_f = Z(n_count01 + n_count12);
%                         psi_f = Y(n_count01 + n_count12);
%                         phi_r = X(t3);
%                         the_r = Z(t3);
%                         psi_r = Y(t3);
%                         [Ln2,OrCr,OfCf,OrOf] = node_cal2(psi_r,phi_r,the_r,psi_f,phi_f,the_f,phi_n,the_n,Ln1,Lc,Rw,r,d);
%                         %[Ln2,OrCr,OfCf,OrOf,flag_] = node_cal(psi_r,phi_r,the_r,psi_f,phi_f,the_f,phi_nr1,phi_nr2,the_nr,phi_nf1,phi_nf2,the_nf,Ln1,Lc,Rw,r,d);           
%                        omega_r = -2*omega*(t3/n_cnt)^3+3*omega*(t3/n_cnt)^2;
%                        omega_f = omega_r;
%                         
%                         %分岐の修了判定
%                         if t3 == n_count01 + n_count12
%                             text = gamma1*180/pi+ "_" + phi_n*180/pi+ "_" + the_n*180/pi+ "_" + the_ns*180/pi;
%                             %writematrix(data,text+'.csv')
%                             T = t1+t2+t3-1;
% %%
%                             %書き出しデータの描画
%                             figure(2)
%                             %ダミーデータの作成
%                             x0 = [1 1];
%                             x1 = [n_count01 n_count01];
%                             x2 = [t1 t1];                            
%                             x3 = [t1+t2 t1+t2];
%                             x4 = [t1+t2+n_count01 t1+t2+n_count01];                            
%                             x5 = [t1+t2+t3 t1+t2+t3];
%                             y = [-180 350];
%                             
%                             
%                             plot(data_r(1:T,1),data(1:T,1)*180/pi,'LineWidth',2)
%                             hold on
%                             plot(data_r(1:T,1),data(1:T,2)*180/pi,'LineWidth',2)
%                             plot(data_r(1:T,1),data(1:T,3)*180/pi,'LineWidth',2)
%                             plot(data_r(1:T,1),data(1:T,4)*180/pi,'LineWidth',2)
%                             plot(data_r(1:T,1),data(1:T,5)*180/pi,'LineWidth',2)
%                             plot(data_r(1:T,1),data(1:T,6)*180/pi,'LineWidth',2)
%                             plot(x1,y,'--k','LineWidth',0.5)
%                             plot(x2,y,'--k','LineWidth',0.5)
%                             plot(x3,y,'--k','LineWidth',0.5)
%                             plot(x4,y,'--k','LineWidth',0.5)
%                             plot(x0,y,'--k','LineWidth',0.5)
%                             plot(x5,y,'--k','LineWidth',0.5)
%                             
%                             legend('1軸目(前)','2軸目(前)','3軸目(前)','1軸目(後)','2軸目(後)','3軸目(後)','Location','East')
%                             ylim([-180 180]) 
%                             yticks(-180:30:180)
%                             xlabel('時間，ステップ')
%                             ylabel('角度[deg]')
%                             
%                             figure(3)
%                             plot(data_r(1:T,1),data(1:T,7),'LineWidth',2)
%                             hold on
%                             plot(data_r(1:T,1),data(1:T,8),'LineWidth',2)
%                             plot(x1,y,'--k','LineWidth',0.5)
%                             plot(x2,y,'--k','LineWidth',0.5)
%                             plot(x3,y,'--k','LineWidth',0.5)
%                             plot(x4,y,'--k','LineWidth',0.5)
%                             plot(x0,y,'--k','LineWidth',0.5)
%                             plot(x5,y,'--k','LineWidth',0.5)
%                             ylim([0 350])
%                             xlabel('時間，ステップ')
%                             ylabel('角速度[deg/s]')
%                             break;
%                         end
%                   
%                 end       
           
%% 
 t_step = 0.0485;
 the_switch = acos((d+2*rw)/(2*Rw));
 t1 = 0; t2 = 0; t3 = 0;
 T1 = 20;
 T2 = 40;
 T3 = 20;
 
 %分岐点までの距離
 Ln1 = 500;
 
 %モードの設定
 switching = 1;
 mode =1;

for t = 1:500
%右から左へのスイッチング

%メインケーブルに対して，どのケーブルに乗っているのかの情報，
 phi_nr1 = 0/180*pi; the_nr = 0/180*pi ;phi_nr2 = 0/180*pi; 
 phi_nf1 = 0/180*pi; the_nf = 0/180*pi; phi_nf2 = 0/180*pi;

 if flag == 0
    the_yr = 0/180*pi; the_xr = -90/180*pi; the_zr = 0/180*pi;
    the_yf = 0/180*pi; the_xf = -90/180*pi; the_zf = 0/180*pi; 
    flag = 1;
    
 elseif flag == 1
    t1 = t1+1; 
    the_yr = 0/180*pi; the_xr = -90/180*pi; the_zr = -the_switch*(6*(t1/T1)^5-15*(t1/T1)^4+10*(t1/T1)^3);
    the_yf = 0/180*pi; the_xf = -90/180*pi; the_zf = the_switch*(6*(t1/T1)^5-15*(t1/T1)^4+10*(t1/T1)^3); 
    if t1 == T1
        flag = 2;
    end
 elseif flag == 2
     t2 = t2+1;
     the_yr = 0/180*pi; the_xr = (-90+180*(6*(t2/T2)^5-15*(t2/T2)^4+10*(t2/T2)^3))/180*pi; the_zr = -the_switch;
     the_yf = 0/180*pi; the_xf = (-90+180*(6*(t2/T2)^5-15*(t2/T2)^4+10*(t2/T2)^3))/180*pi; the_zf = the_switch; 
     if t2 == T2
        flag = 3;
     end
 elseif flag == 3
     t3 = t3+1;
     the_yr = 0/180*pi; the_xr = 90/180*pi; the_zr = -the_switch + the_switch*(6*(t3/T3)^5-15*(t3/T3)^4+10*(t3/T3)^3);
     the_yf = 0/180*pi; the_xf = 90/180*pi; the_zf = the_switch - the_switch*(6*(t3/T3)^5-15*(t3/T3)^4+10*(t3/T3)^3); 
     if t3 == T3
        flag = 4;
     end
 elseif flag == 4
     text = "switching";
     writematrix(data,text+'.csv')
     T = t1+t2+t3;
     figure(2)
     plot(data_r(1:T,1),data(1:T,1)*180/pi,'LineWidth',2)
    hold on
    plot(data_r(1:T,1),data(1:T,2)*180/pi,'LineWidth',2)
    plot(data_r(1:T,1),data(1:T,3)*180/pi,'LineWidth',2)
    plot(data_r(1:T,1),data(1:T,4)*180/pi,'LineWidth',2)
    plot(data_r(1:T,1),data(1:T,5)*180/pi,'LineWidth',2)
    plot(data_r(1:T,1),data(1:T,6)*180/pi,'LineWidth',2)
     legend('1軸目(前)','2軸目(前)','3軸目(前)','1軸目(後)','2軸目(後)','3軸目(後)','Location','East')
     break;
 end

 
 [Ln2,OrCr,OfCf,OrOf,OrOw1r,OrOw2r,OfOw1f,OfOw2f] = node_cal2(the_yr,the_xr,the_zr,the_yf,the_xf,the_zf,0,0,Ln1,Lc,Rw,rw,d,mode);

 



        %% 各関節角度の計算
        
            %収束計算
            fun = @(x)Joint_angle_calc3(x,the_yr,the_xr,the_zr,the_yf,the_xf,the_zf,phi_nr1,phi_nr2,the_nr,phi_nf1,phi_nf2,the_nf,...
                                       gamma1,a1,a2,a3,Lc,OrCr,OfCf,OrOf,g0,g2,g3,g4,g6,w0,w2,w3,w4,w6,mode);
            [x,fval,exitflag,output]= fzero(fun,0);
            
            %収束した結果を用いて，関節角度を算出
            [F,the_2r,the_3r,the_1r,the_2f,the_3f,the_1f,the_03x] ...
            = Joint_angle_calc3(x,the_yr,the_xr,the_zr,the_yf,the_xf,the_zf,phi_nr1,phi_nr2,the_nr,phi_nf1,phi_nf2,the_nf,...
                                gamma1,a1,a2,a3,Lc,OrCr,OfCf,OrOf,g0,g2,g3,g4,g6,w0,w2,w3,w4,w6,mode);
   
               %% 左右分岐のスイッチング
            
                if (switching*mode == 1) && (the_03x > 0)
                    %(switching == 1 && mode == 1 の略）
                    mode = 2;
                   
                    [Ln2,OrCr,OfCf,OrOf,OrOw1r,OrOw2r,OfOw1f,OfOw2f] = node_cal2(the_yr,the_xr,the_zr,the_yf,the_xf,the_zf,0,0,Ln1,Lc,Rw,rw,d,mode);
                    
                    %収束計算
                    fun = @(x)Joint_angle_calc3(x,the_yr,the_xr,the_zr,the_yf,the_xf,the_zf,phi_nr1,phi_nr2,the_nr,phi_nf1,phi_nf2,the_nf,...
                                       gamma1,a1,a2,a3,Lc,OrCr,OfCf,OrOf,g0,g2,g3,g4,g6,w0,w2,w3,w4,w6,mode);
                    [x,fval,exitflag,output]= fzero(fun,0);
            
                    %収束した結果を用いて，関節角度を算出
                    [F,the_2r,the_3r,the_1r,the_2f,the_3f,the_1f,the_03x] ...
                    = Joint_angle_calc3(x,the_yr,the_xr,the_zr,the_yf,the_xf,the_zf,phi_nr1,phi_nr2,the_nr,phi_nf1,phi_nf2,the_nf,...
                                gamma1,a1,a2,a3,Lc,OrCr,OfCf,OrOf,g0,g2,g3,g4,g6,w0,w2,w3,w4,w6,mode);

                end
                if (switching*mode == 2) && (the_03x <= 0)
                    %（switching == 1 && mode == 2 の略）
                    mode = 1;
                    [Ln2,OrCr,OfCf,OrOf,OrOw1r,OrOw2r,OfOw1f,OfOw2f] = node_cal2(the_yr,the_xr,the_zr,the_yf,the_xf,the_zf,0,0,Ln1,Lc,Rw,rw,d,mode);
                    
                    %収束計算
                    fun = @(x)Joint_angle_calc3(x,the_yr,the_xr,the_zr,the_yf,the_xf,the_zf,phi_nr1,phi_nr2,the_nr,phi_nf1,phi_nf2,the_nf,...
                                       gamma1,a1,a2,a3,Lc,OrCr,OfCf,OrOf,g0,g2,g3,g4,g6,w0,w2,w3,w4,w6,mode);
                    [x,fval,exitflag,output]= fzero(fun,0);
            
                    %収束した結果を用いて，関節角度を算出
                    [F,the_2r,the_3r,the_1r,the_2f,the_3f,the_1f,the_03x] ...
                    = Joint_angle_calc3(x,the_yr,the_xr,the_zr,the_yf,the_xf,the_zf,phi_nr1,phi_nr2,the_nr,phi_nf1,phi_nf2,the_nf,...
                                gamma1,a1,a2,a3,Lc,OrCr,OfCf,OrOf,g0,g2,g3,g4,g6,w0,w2,w3,w4,w6,mode);
   
                end
               
                            
%%                            
             %関節角度データの保存
             data_r(t,:) = [t*t_step the_1r the_2r the_3r];
             data_f(t,:) = [t*t_step the_1f the_2f the_3f];
             data(t,:) = [the_1f the_2f the_3f the_1r the_2r the_3r 0 0];




        %% データの描画
           
            lw = 45;  %基準円柱長さ（体あy部分）
    
            %モータの位置だし
            %準備
            [Xm,Ym,Zm] = cylinder(rm);                  %基準円柱座標（モータ部分）
            Zm = (Zm - 0.5).*lm;                        %円柱の高さ補正
            %Xm = Xm + 0.01*t; 
            m_top     = [Xm(1,:);Ym(1,:);Zm(1,:)];      %基準円柱座標（上下各成分）
            m_bottom  = [Xm(2,:);Ym(2,:);Zm(2,:)];
            m1_top    = [Xm(1,:);Ym(1,:);Zm(1,:)+l1];   %最初のモータの位置
            m1_bottom = [Xm(2,:);Ym(2,:);Zm(2,:)+l1];
            m2_top    = [Xm(1,:);Ym(1,:);Zm(1,:)+l2];   %2番目のモータの基準位置
            m2_bottom = [Xm(2,:);Ym(2,:);Zm(2,:)+l2];
            m3_top    = [Xm(1,:);Ym(1,:);Zm(1,:)+l3];   %3番目のモータの基準位置
            m3_bottom = [Xm(2,:);Ym(2,:);Zm(2,:)+l3];
            m4_top    = [Xm(1,:);Ym(1,:);Zm(1,:)+l4];   %4番目のモータの基準位置
            m4_bottom = [Xm(2,:);Ym(2,:);Zm(2,:)+l4];
            m5_top    = [Xm(1,:);Ym(1,:);Zm(1,:)+l5];   %5番目のモータの基準位置
            m5_bottom = [Xm(2,:);Ym(2,:);Zm(2,:)+l5];
            m6_top    = [Xm(1,:);Ym(1,:);Zm(1,:)+l6];   %6番目のモータの基準位置
            m6_bottom = [Xm(2,:);Ym(2,:);Zm(2,:)+l6];
            
            %モータの座標だし
            OrCf = OrOf+OfCf;
            m1_coord = R("y",gamma1)*([L1;0;0]+R("x",phi_nr1)*R("z",the_nr)*R("x",phi_nr2)*([-Ln1;0;0]+OrCr + R("y",the_yr)*R("x",the_xr)*R("z",the_zr)*R("z",-the_3r-pi/2)*horzcat(m1_top,m1_bottom)));
            m2_coord = R("y",gamma1)*([L1;0;0]+R("x",phi_nr1)*R("z",the_nr)*R("x",phi_nr2)*([-Ln1;0;0]+OrCr + R("y",the_yr)*R("x",the_xr)*R("z",the_zr)*R("z",-the_3r-pi/2)*R("x",a2)*R("z",-the_2r)*horzcat(m2_top,m2_bottom)));
            m3_coord = R("y",gamma1)*([L1;0;0]+R("x",phi_nr1)*R("z",the_nr)*R("x",phi_nr2)*([-Ln1;0;0]+OrCr + R("y",the_yr)*R("x",the_xr)*R("z",the_zr)*R("z",-the_3r-pi/2)*R("x",a2)*R("z",-the_2r)*R("x",-a1)*R("z",-the_1r)*R("z",-pi)*horzcat(m3_top,m3_bottom)));
            m4_coord = R("y",gamma1)*([L1;0;0]+R("x",phi_nr1)*R("z",the_nr)*R("x",phi_nr2)*([-Ln1;0;0]+OrCf + R("y",the_yr)*R("x",the_xr)*R("z",the_zr)*R("z",-the_3r-pi/2)*R("x",a2)*R("z",-the_2r)*R("x",-a1)*R("z",-the_1r)*R("z",-pi)*R("y",2*a3)*R("z",the_1f)*horzcat(m4_top,m4_bottom)));
            m5_coord = R("y",gamma1)*([L1;0;0]+R("x",phi_nr1)*R("z",the_nr)*R("x",phi_nr2)*([-Ln1;0;0]+OrCf + R("y",the_yr)*R("x",the_xr)*R("z",the_zr)*R("z",-the_3r-pi/2)*R("x",a2)*R("z",-the_2r)*R("x",-a1)*R("z",-the_1r)*R("z",-pi)*R("y",2*a3)*R("z",the_1f)*R("x",a1)*R("z",the_2f)*horzcat(m5_top,m5_bottom)));
            m6_coord = R("y",gamma1)*([L1;0;0]+R("x",phi_nr1)*R("z",the_nr)*R("x",phi_nr2)*([-Ln1;0;0]+OrCf + R("y",the_yr)*R("x",the_xr)*R("z",the_zr)*R("z",-the_3r-pi/2)*R("x",a2)*R("z",-the_2r)*R("x",-a1)*R("z",-the_1r)*R("z",-pi)*R("y",2*a3)*R("z",the_1f)*R("x",a1)*R("z",the_2f)*R("x",-a2)*R("z",the_3f+pi/2)*horzcat(m6_top,m6_bottom)));
            
            
            %タイヤ部分の位置だし（タイヤはひょうたん型）
            i = 1:11;
            [Xw,Yw,Zw] = cylinder((Rw+rw)/2+(Rw-rw)/2*cos(2*pi*(i-1)/10));
            Zw = (Zw - 0.5).*lw;
            
            for i = 1:11
                W(1:3,i+20*(i-1):i+20*i) = [Xw(i,:);Yw(i,:);Zw(i,:)];
            end
            W1 = W - [0;Rw;0];
            W2 = W + [0;Rw;0];
            
            %phi_nr1とかは常に0なので実質的に何もしていない（分岐後を描画したいときのみ使用）
            w1r_coord = R("y",gamma1)*([L1;0;0]+R("x",phi_nr1)*R("z",the_nr)*R("x",phi_nr2)*([-Ln1;0;0]+OrCr + R("y",the_yr)*R("x",the_xr)*R("z",the_zr)*W1));
            w2r_coord = R("y",gamma1)*([L1;0;0]+R("x",phi_nr1)*R("z",the_nr)*R("x",phi_nr2)*([-Ln1;0;0]+OrCr + R("y",the_yr)*R("x",the_xr)*R("z",the_zr)*W2));
            w1f_coord = R("y",gamma1)*([L1;0;0]+R("x",phi_nr1)*R("z",the_nr)*R("x",phi_nr2)*([-Ln1;0;0]+OrCf + R("y",the_yr)*R("x",the_xr)*R("z",the_zr)*R("z",-the_3r-pi/2)*R("x",a2)*R("z",-the_2r)*R("x",-a1)*R("z",-the_1r)*R("z",-pi)*R("y",2*a3)*R("z",the_1f)*R("x",a1)*R("z",the_2f)*R("x",-a2)*R("z",the_3f+pi/2)*W2));
            w2f_coord = R("y",gamma1)*([L1;0;0]+R("x",phi_nr1)*R("z",the_nr)*R("x",phi_nr2)*([-Ln1;0;0]+OrCf + R("y",the_yr)*R("x",the_xr)*R("z",the_zr)*R("z",-the_3r-pi/2)*R("x",a2)*R("z",-the_2r)*R("x",-a1)*R("z",-the_1r)*R("z",-pi)*R("y",2*a3)*R("z",the_1f)*R("x",a1)*R("z",the_2f)*R("x",-a2)*R("z",the_3f+pi/2)*W1));
           
        %     w1r_coord = R("y",gamma1)*([L1;0;0]+R("x",phi_nr1)*R("z",the_nr)*R("x",phi_nr2)*[-Ln1;0;0]) +R("y",gamma1)*R("x",phi_nr1)*R("z",the_nr)*R("x",phi_nr2)*(OrCr + R("y",the_yr)*R("x",the_xr)*R("z",the_zr)*W1);
        %     w2r_coord = R("y",gamma1)*([L1;0;0]+R("x",phi_nr1)*R("z",the_nr)*R("x",phi_nr2)*[-Ln1;0;0]) +R("y",gamma1)*R("x",phi_nr1)*R("z",the_nr)*R("x",phi_nr2)*(OrCr + R("y",the_yr)*R("x",the_xr)*R("z",the_zr)*W2);
        %     w1f_coord = R("y",gamma1)*([L1;0;0]+R("x",phi_nr1)*R("z",the_nr)*R("x",phi_nr2)*[-Ln1;0;0]) +R("y",gamma1)*R("x",phi_nr1)*R("z",the_nr)*R("x",phi_nr2)*(OrCf + R("y",the_yr)*R("x",the_xr)*R("z",the_zr)*R("z",the_3r)*R("x",a2)*R("z",the_2r)*R("x",a1)*R("z",the_1r)*R("y",2*a3)*R("z",the_1f)*R("x",a1)*R("z",the_2f)*R("x",a2)*R("z",the_3f)*W2);
        %     w2f_coord = R("y",gamma1)*([L1;0;0]+R("x",phi_nr1)*R("z",the_nr)*R("x",phi_nr2)*[-Ln1;0;0]) +R("y",gamma1)*R("x",phi_nr1)*R("z",the_nr)*R("x",phi_nr2)*(OrCf + R("y",the_yr)*R("x",the_xr)*R("z",the_zr)*R("z",the_3r)*R("x",a2)*R("z",the_2r)*R("x",a1)*R("z",the_1r)*R("y",2*a3)*R("z",the_1f)*R("x",a1)*R("z",the_2f)*R("x",a2)*R("z",the_3f)*W1);
        %     
            w1r_middle_bottom = (sum(w1r_coord(:,11*21-20:11*21),2)-w1r_coord(:,231))/20;
            w2r_middle_bottom = (sum(w2r_coord(:,11*21-20:11*21),2)-w2r_coord(:,231))/20;
            w1f_middle_bottom = (sum(w2f_coord(:,11*21-20:11*21),2)-w2f_coord(:,231))/20;
            w2f_middle_bottom = (sum(w1f_coord(:,11*21-20:11*21),2)-w1f_coord(:,231))/20;
            
            
            %直線の描画データ生成(リンク部分)
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
            
            %デバック用
            %Y = zeros(3,2);
            %Y(:,1)=O2;
            %Y(:,2)=Of2;
            %line(Y(1,:)',Y(2,:)',Y(3,:)','LineWidth',3,'Color','g')
         
            
             %ケーブルの描画
            [Xc,Yc,Zc] = cylinder(3);
            Zc = (Zc - 0.5).*2*Lc;                        %円柱の高さ補正
            
            c_top     = [Xc(1,:);Yc(1,:);Zc(1,:)];      %基準円柱座標（上下各成分）
            c_bottom  = [Xc(2,:);Yc(2,:);Zc(2,:)];
            c_top = R("y",gamma1)*R("y",pi/2)*c_top;
            c_bottom = R("y",gamma1)*R("y",pi/2)*c_bottom;
            cable_plot(c_top,c_bottom)
            
            %分岐ケーブル(メイン・サブ)
            [Xc2,Yc2,Zc2] = cylinder(3);
            Zc2 = Zc2*400;
            c_top2     = [Xc2(1,:);Yc2(1,:);Zc2(1,:)];      %基準円柱座標（上下各成分）
            c_bottom2  = [Xc2(2,:);Yc2(2,:);Zc2(2,:)];
            
            %ここは描画だけなので適当にしてある
            c_top2h = R("y",gamma1)*([L1;0;0]+R("x",phi_n)*R("z",the_n)*R("y",pi/2)*c_top2);
            c_bottom2h = R("y",gamma1)*([L1;0;0]+R("x",phi_n)*R("z",the_n)*R("y",pi/2)*c_bottom2);
            c_top2s = R("y",gamma1)*([L1;0;0]+R("x",phi_n)*R("z",the_ns)*R("y",pi/2)*c_top2);
            c_bottom2s = R("y",gamma1)*([L1;0;0]+R("x",phi_n)*R("z",the_ns)*R("y",pi/2)*c_bottom2);
            
            cable_plot(c_top2h,c_bottom2h)
            cable_plot(c_top2s,c_bottom2s)
            %line([400;-400],[0;0],[0;0],'Color','k','LineWidth',6) 
        %     line([0;cable1(1)],[0;cable1(2)],[0;cable1(3)],'LineWidth',1)
        %     line([0;cable2(1)],[0;cable2(2)],[0;cable2(3)],'LineWidth',1)
            xlabel('x[mm]')
            ylabel('y[mm]')
            zlabel('z[mm]')
          
    %
            %座標のトレースと接触判定
            %モータの中心座標
%            Pr2 = R("y",gamma1)*([L1;0;0]+R("x",phi_nr1)*R("z",the_nr)*R("x",phi_nr2)*([-Ln1;0;0]+OrCr + R("y",the_yr)*R("x",the_xr)*R("z",the_zr)*R("z",-the_3r-pi/2)*R("x",a2)*[0;0;l2]));
%            Pf2 = R("y",gamma1)*([L1;0;0]+R("x",phi_nr1)*R("z",the_nr)*R("x",phi_nr2)*([-Ln1;0;0]+OrCf + R("y",the_yr)*R("x",the_xr)*R("z",the_zr)*R("z",-the_3r-pi/2)*R("x",a2)*R("z",-the_2r)*R("x",-a1)*R("z",-the_1r)*R("z",-pi)*R("y",2*a3)*R("z",the_1f)*R("x",a1)*[0;0;l2]));

%             [data_collide_r(t,1),data_collide_r(t,2)] = collide(gamma1,phi_n,the_n,Pr2(1),Pr2(2),Pr2(3),rj,L1);
%             [data_collide_f(t,1),data_collide_f(t,2)] = collide(gamma1,phi_n,the_n,Pf2(1),Pf2(2),Pf2(3),rj,L1);
% 
%             if (data_collide_r(t,1)+data_collide_r(t,2)+data_collide_f(t,1)+data_collide_f(t,2)) ~= 0
%                 flag_fin =1;     %接触を検知したらフラグを立てる
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
            
            %可操作楕円の描画
%             [e_r,M_ellipsoid_r] = manipulability_ellipsoid(a1,a2,the_2r,the_3r,phi_r,the_r,psi_r,phi_nr1,the_nr,phi_nr2,gamma1,L1,Ln1,OrCr,phi_nr1,the_nr,phi_nr2);
%             [e_f,M_ellipsoid_f] = manipulability_ellipsoid(a1,a2,the_2f,the_3f,phi_f,the_f,psi_f,phi_nr1,the_nr,phi_nr2,gamma1,L1,Ln1,OrCf,phi_nf1,the_nf,phi_nf2);
            
            %ガンマが値を持つときはｚ軸周りにpi回すだけじゃだめっぽい
 %           M_R_ellipsoid_r = R("y",gamma1)*([L1;0;0]+R("x",phi_nr1)*R("z",the_nr)*R("x",phi_nr2)*([-Ln1;0;0]+OrCr)) + R("x",phi_nr1)*R("z",the_nr)*R("x",phi_nr2)*R("x",phi_r)*R("z",the_r)*R("y",psi_r)*R("z",pi)*R("x",-pi/2)*e_r*M_ellipsoid_r;
 %           M_R_ellipsoid_f = R("y",gamma1)*([L1;0;0]+R("x",phi_nr1)*R("z",the_nr)*R("x",phi_nr2)*([-Ln1;0;0]+OrCf)) + R("x",phi_nf1)*R("z",the_nf)*R("x",phi_nf2)*R("x",phi_f)*R("z",the_f)*R("y",psi_f)*R("x",-pi/2)*e_f*M_ellipsoid_f;
     
            %ガンマが値を持つときはｚ軸周りにpi回すだけじゃだめっぽい
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
            
            %ワークスペースの可視化
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
            
              % 確認用のプロット
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
%          w = waitforbuttonpress;
end








%% 関数

%% 回転行列
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
%Ln2=0のときにLn1を返す関数
%基本的には，cos(phi)で見分けることができるが，分岐点の条件によっては見分けられない場合もあるので，
%左分岐モード，→分岐モードは分けて使う．
%この関数はいらないかも

function X = Len(phi_r,the_r,psi_r,phi_f,the_f,psi_f,Lc,Rw,r,d)
    Or1Cr = R("x",phi_r)*R("z",the_r)*[Rw*sin(psi_r);0;Rw*cos(psi_r)-(r+d/2)];
    Or2Cr = R("x",phi_r)*R("z",the_r)*[-Rw*sin(psi_r);0;-Rw*cos(psi_r)+(r+d/2)];
    Of1Cf = R("x",phi_f)*R("z",the_f)*[Rw*sin(psi_f);0;Rw*cos(psi_f)-(r+d/2)]; 
    Of2Cf = R("x",phi_f)*R("z",the_f)*[-Rw*sin(psi_f);0;-Rw*cos(psi_f)+(r+d/2)]; 

    
    %OrOf間の計算
    xr  = OrCr(1);
    yr  = OrCr(2);
    zr  = OrCr(3);
    xf_ = OfCf(1);  %xf'の意味
    yf_ = OfCf(2);
    zf_ = OfCf(3);
    
    X = -(xf_-xr)+sqrt(-(yf_-yr)^2-(zf_-zr)^2+Lc^2);
    
end

%% RCM中心座標の計算1(右分岐モード，左分岐モード)kakikaezumi
function [Ln2,OrCr,OfCf,OrOf,OrOw1r,OrOw2r,OfOw1f,OfOw2f] = node_cal2(the_yr,the_xr,the_zr,the_yf,the_xf,the_zf,phi_n,the_n,Ln1,Lc,Rw,rw,d,mode)

    %gap  = 2*Rw*cos(the_zr)-d-2*rw;
    
    if mode == 2 %右分岐モード
        %後輪
        OrCr =   R("y",the_yr)*R("x",the_xr)*[Rw*sin(the_zr);d/2+rw-Rw*cos(the_zr);0];
        OrOw1r = R("y",the_yr)*R("x",the_xr)*[0;d/2+rw;0];
        OrOw2r = R("y",the_yr)*R("x",the_xr)*[2*Rw*sin(the_zr);d/2+rw-2*Rw*cos(the_zr);0];
        %前輪
        OfCf =   R("y",the_yf)*R("x",the_xf)*[Rw*sin(the_zf);d/2+rw-Rw*cos(the_zf);0];
        OfOw1f = R("y",the_yf)*R("x",the_xf)*[0;d/2+rw;0];
        OfOw2f = R("y",the_yf)*R("x",the_xf)*[2*Rw*sin(the_zf);d/2+rw-2*Rw*cos(the_zf);0];
    elseif mode == 1 %左分岐モード
        %後輪
        OrCr =   [2*Rw*sin(the_zr);0;0] + R("y",the_yr)*R("x",the_xr)*[-Rw*sin(the_zr);-d/2-rw+Rw*cos(the_zr);0];
        OrOw1r = [2*Rw*sin(the_zr);0;0] + R("y",the_yr)*R("x",the_xr)*[-2*Rw*sin(the_zr);-d/2-rw+2*Rw*cos(the_zr);0];
        OrOw2r = [2*Rw*sin(the_zr);0;0] + R("y",the_yr)*R("x",the_xr)*[0;-d/2-rw;0];
        %前輪
        OfCf =   [2*Rw*sin(the_zf);0;0] + R("y",the_yf)*R("x",the_xf)*[-Rw*sin(the_zf);-d/2-rw+Rw*cos(the_zf);0];
        OfOw1f = [2*Rw*sin(the_zf);0;0] + R("y",the_yf)*R("x",the_xf)*[-2*Rw*sin(the_zf);-d/2-rw+2*Rw*cos(the_zf);0];
        OfOw2f = [2*Rw*sin(the_zf);0;0] + R("y",the_yf)*R("x",the_xf)*[0;-d/2-rw;0];
    else
    end

    %ここからは未変更
    R1 = [1;0;0];
    R2 = R("x",phi_n)*R("z",the_n)*[1;0;0];
    xr = OrCr(1); yr = OrCr(2); zr = OrCr(3);
    xf = OfCf(1); yf = OfCf(2); zf = OfCf(3);
    an = R1(1); bn = R1(2); cn = R1(3);
    dn = R2(1); en = R2(2); fn = R2(3);
    C = xf-xr+an*Ln1; D = yf-yr+bn*Ln1; E = zf-zr+cn*Ln1;
    Ln2 = (-(C*dn+D*en+E*fn)+sqrt((C*dn+D*en+E*fn)^2-(dn^2+en^2+fn^2)*(C^2+D^2+E^2-Lc^2)))/(dn^2+en^2+fn^2);
    
    OrOf = Ln1*R1+Ln2*R2;
end

%% 球面線形補間＋オイラー角表示
%7/8等分割ではなく，5次両停留曲線で分割してみる
function [X,Z,Y] = slerp_xzy_euler(q1,q2,t)
        
        the = dist(q1,q2);
        X = zeros(t,1);
        Z = zeros(t,1);
        Y = zeros(t,1);
       
    for i=1:t
        %tは分割数，iはi番目の分割点
        k = 6*(i/t)^5-15*(i/t)^4+10*(i/t)^3;
       % k = i/t;
%         qt = sin((1-i/t)*the)/sin(the)*q1+sin(i/t*the)/sin(the)*q2;
        qt = sin((1-k)*the)/sin(the)*q1+sin(k*the)/sin(the)*q2;
        temp = euler(qt,'XZY','frame');
        X(i,1) = temp(1);
        Z(i,1) = temp(2);
        Y(i,1) = temp(3);
    end
end

%% 接触判定
function [f1,f2] = collide(gamma1,phi_n,the_n,xp,yp,zp,rj,L)
     %接触判定部分p2-90
     %0522　分岐点を超えた部分での接触を検知しないように改良
     f1 = 0;
     f2 = 0;
     
     a = cos(the_n)*cos(gamma1)+sin(phi_n)*sin(the_n)*sin(gamma1);
     b = cos(phi_n)*sin(the_n);
     c = -cos(the_n)*sin(gamma1)+sin(phi_n)*sin(the_n)*cos(gamma1);
     A = a^2 + b^2 + c^2;
     B = a*xp+b*yp+c*zp-a*L*cos(gamma1)+c*L*sin(gamma1); 
     C = (L*cos(gamma1)-xp)^2+yp^2+(-L*sin(gamma1)-zp)^2-rj^2;
     
     %走行ケーブル
     judge1 = (cos(gamma1)*xp-sin(gamma1)*zp)^2-(xp^2+yp^2+zp^2-rj^2);
     
     %分岐ケーブル
     judge2 = B^2-A*C;
     
     if (judge1 >= 0) && (cos(gamma1)*xp-sin(gamma1)*zp-sqrt(judge1) <= L)   %接触し，その解がLよりも小さいならば
         f1 = 1;
     end
%      if judge2 >= 0 %接触したら
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
%% 可操作楕円を作成する関数（エンドエフェクタの座標系）
function [e,M_ellipsoid] =  manipulability_ellipsoid(a1,a2,the2,the3,phi,the,psi,phi_n1,the_n1,phi_n2,gamma1,L1,Ln1,OC,phi_n3,the_n2,phi_n4)

    J = [sin(a1)*(cos(the3)*sin(the2) + cos(a2)*cos(the2)*sin(the3)) + cos(a1)*sin(a2)*sin(the3) sin(a2)*sin(the3) 0; ...
         cos(a1)*sin(a2)*cos(the3) - sin(a1)*(sin(the2)*sin(the3) - cos(a2)*cos(the2)*cos(the3)) sin(a2)*cos(the3) 0; ...
         cos(a1)*cos(a2) - sin(a1)*sin(a2)*cos(the2) cos(a2) 1];

     [e,g] = eig(J*J');    %固有ベクトルと，固有値の対角化行列*
     
     
     %gのサイズ調整
     G = 75;
    [x, y, z] = ellipsoid(0,0,0,G*sqrt(g(1,1)),G*sqrt(g(2,2)),G*sqrt(g(3,3)));
     

     x = reshape(x,1,[]);
     y = reshape(y,1,[]);
     z = reshape(z,1,[]);

     M_ellipsoid = [x;y;z]; 
end 
%% ワークスペースの可視化
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
%% 描画関連の関数

function clinder_plot(Cl_bottom,Cl_top)
    %Cl_topは円柱の上面の座標
    %Cl_bottomは円柱の下面の座標
    ax = gca;
    ax.YDir = 'reverse';
    ax.ZDir = 'reverse';
    X(1,:)= Cl_bottom(1,:);
    X(2,:)= Cl_top(1,:);
    Y(1,:)= Cl_bottom(2,:);
    Y(2,:)= Cl_top(2,:);
    Z(1,:)= Cl_bottom(3,:);
    Z(2,:)= Cl_top(3,:);


    surf(X,Y,Z,'FaceColor','r','EdgeColor','none')    %側面プロット
    hold on
    fill3(X(1,:),Y(1,:),Z(1,:),"w")                     %底面プロット
    fill3(X(2,:),Y(2,:),Z(2,:),"w")                     %上面プロット
    xlim([-300 500]) 
    ylim([-300 500]) 
    zlim([-300 500])
    % view(-135, 10);
end

function gourd_plot(W)
    %ひょうたん型を画面出力する関数
    %入力行列の大きさを読み取れば，汎用的なものが作れるけど今回はスルー
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

    surf(X,Y,Z,'FaceColor','green','EdgeColor','k')    %側面プロット

    hold on

    fill3(X(1,:),Y(1,:),Z(1,:),"g",'EdgeColor','k')                     %底面プロット
    fill3(X(11,:),Y(11,:),Z(11,:),"g",'EdgeColor','k')                     %上面プロット
    xlim([-300 500]) 
    ylim([-300 500]) 
    zlim([-300 500])
    % view(-135, 10);
end

function cable_plot(Cl_bottom,Cl_top)
%Cl_topは円柱の上面の座標
%Cl_bottomは円柱の下面の座標
ax = gca;
ax.YDir = 'reverse';
ax.ZDir = 'reverse';
X(1,:)= Cl_bottom(1,:);
X(2,:)= Cl_top(1,:);
Y(1,:)= Cl_bottom(2,:);
Y(2,:)= Cl_top(2,:);
Z(1,:)= Cl_bottom(3,:);
Z(2,:)= Cl_top(3,:);


surf(X,Y,Z,'FaceColor','blue','EdgeColor','none')    %側面プロット
hold on
fill3(X(1,:),Y(1,:),Z(1,:),"b")                     %底面プロット
fill3(X(2,:),Y(2,:),Z(2,:),"b")                     %上面プロット
%xlim([-250 350]) 
xlim([-50 700]) 
ylim([-400 350]) 
zlim([-250 550])
%写真用
xlim([-300 600]) 
ylim([-450 450]) 
zlim([-450 450])

% % %アップ用
%  xlim([200 600]) 
%  ylim([-100 300]) 
%  zlim([-250 150])
view(-33,33 );
end