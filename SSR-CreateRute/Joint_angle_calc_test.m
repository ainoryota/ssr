%% 索道上における駆動部の位置，姿勢から関節角度を求める関数
%所望の位置姿勢において，モーメントの釣り合いが成り立つ関節角度の算出
%kineto static inverse analysis


function [F,the_2r,the_3r,the_1r,the_2f,the_3f,the_1f,check] ...
   = Joint_angle_calc(x,the_yr,the_xr,the_zr,the_yf,the_xf,the_zf,phi_nr1,phi_nr2,the_nr,phi_nf1,phi_nf2,the_nf,...
                     gamma1,a1,a2,a3,Lc,OrCr,OfCf,OrOf,g0,g2,g3,g4,g6,w0,w2,w3,w4,w6,mode)
                 
%% 不要部分










    %% 初期化 
    check = 0;

    %% リンク3から見た駆動部の姿勢角の導出(p23~)
    OrCf = OrOf+OfCf;            %後ろ駆動輪の接地点から前駆動輪の原点に向けたベクトル
    CrCf = (OrCf - OrCr);        %後ろ駆動輪の原点から前駆動輪の原点に向けたベクトル

    xb0 = CrCf/Lc;
    yb0 = cross([-sin(gamma1);0;cos(gamma1)],xb0);
    zb0 = cross(xb0,yb0);
    Co_base0 = [xb0 yb0 zb0];    %base座標系への座標変換行列
    
    Co_base = Rod_R(xb0(1),xb0(2),xb0(3),x)*Co_base0; %xb0軸周りにｘだけ回転させ，base座標系への座標変換行列を求める．
    xb = Co_base(:,1);
    yb = Co_base(:,2);
    zb = Co_base(:,3);
    
    Co_0r = Rod_R(yb(1),yb(2),yb(3),-a3)*[xb yb zb];
    Co_0f = Rod_R(yb(1),yb(2),yb(3),a3)*[xb yb zb];
    
    %リンク0の座標系から駆動部（リンク3の座標系）への座標変換行列
    M_0r = Co_0r\R("y",the_yr)*R("x",the_xr)*R("z",the_zr);              
    M_0f = Co_0f\R("x",phi_nf1-phi_nr1)*R("z",the_nf-the_nr)*R("x",phi_nf2-phi_nr2)*R("y",the_yf)*R("x",the_xf)*R("z",the_zf);
   
    %逆運動学解（後輪）
    if mode == 1
        the_2r =  acos((cos(a1)*cos(a2)-M_0r(3,3))/(sin(a1)*sin(a2)));
    else
        the_2r = -acos((cos(a1)*cos(a2)-M_0r(3,3))/(sin(a1)*sin(a2)));
    end
    
    Ar = sin(a2)*sin(the_2r); Br = -cos(a1)*sin(a2)*cos(the_2r) - sin(a1)*cos(a2); Cr = M_0r(1,3); Dr = M_0r(2,3);
    Er = cos(a1)*sin(a2) + sin(a1)*cos(a2)*cos(the_2r); Fr = sin(a1)*sin(the_2r) ; Gr = M_0r(3,2); Hr = M_0r(3,1);
    
    the_1r = atan2(Ar*Dr-Br*Cr,Ar*Cr+Br*Dr); 
    the_3r = atan2(Er*Hr-Fr*Gr,Er*Gr+Fr*Hr);
    
    %逆運動学解（前輪）
    if mode == 1
        the_2f =  acos((cos(a1)*cos(a2)-M_0f(3,3))/(sin(a1)*sin(a2)));
    else
        the_2f = -acos((cos(a1)*cos(a2)-M_0f(3,3))/(sin(a1)*sin(a2)));
    end
    
    Af = sin(a2)*sin(the_2f); Bf = -cos(a1)*sin(a2)*cos(the_2f) - sin(a1)*cos(a2); Cf = M_0f(1,3); Df = M_0f(2,3);
    Ef = cos(a1)*sin(a2) + sin(a1)*cos(a2)*cos(the_2f); Ff = sin(a1)*sin(the_2f) ; Gf = M_0f(3,2); Hf = M_0f(3,1);
    
    the_1f = atan2(Af*Df-Bf*Cf,Af*Cf+Bf*Df); 
    the_3f = atan2(Ef*Hf-Ff*Gf,Ef*Gf+Ff*Hf);

    %% 重心計算（the_xの）決定

    %Or基準,ケーブル座標系(後ろ駆動輪とケーブルの接触点)
    %少しトリッキー%なことしている(最終的にはちゃんとした座標系で計算を行うのが望ましいが，傾向は把握のため，各リンクの重心は，対偶位置に一致すると仮定)
    %(リンク１の重量はリンク0に，リンク5の重量は,リンク6に含まれると仮定する)
    %(すなわちリンク3以外は，重さの成分はｚ成分しかない)

    m0 = cross((OrCr + R("y",the_yr)*R("x",the_xr)*R("z",the_zr)*g0),[-w0*sin(gamma1);0;w0*cos(gamma1)]);
    m2 = cross((OrCr + R("y",the_yr)*R("x",the_xr)*R("z",the_zr)*R("z",-the_3r)*R("x",-a2)*R("z",-the_2r)*g2),[-w2*sin(gamma1);0;w2*cos(gamma1)]);
    m3 = cross((OrCr + R("y",the_yr)*R("x",the_xr)*R("z",the_zr)*R("z",-the_3r)*R("x",-a2)*R("z",-the_2r)*R("x",-a1)*R("z",-the_1r)*R("y",a3)*g3),[-w3*sin(gamma1);0;w3*cos(gamma1)]);
    m4 = cross((OrCf + R("y",the_yr)*R("x",the_xr)*R("z",the_zr)*R("z",-the_3r)*R("x",-a2)*R("z",-the_2r)*R("x",-a1)*R("z",-the_1r)*R("y",2*a3)*R("z",the_1f)*R("x",a1)*g4),[-w4*sin(gamma1);0;w4*cos(gamma1)]);
    m6 = cross((OrCf + R("y",the_yr)*R("x",the_xr)*R("z",the_zr)*R("z",-the_3r)*R("x",-a2)*R("z",-the_2r)*R("x",-a1)*R("z",-the_1r)*R("y",2*a3)*R("z",the_1f)*R("x",a1)*R("z",the_2f)*R("x",a2)*R("z",the_3f)*g6),[-w6*sin(gamma1);0;w6*cos(gamma1)]);
    
    %評価関数（モーメント）
    F = dot(m0,OrOf/norm(OrOf))+dot(m2,OrOf/norm(OrOf))+dot(m3,OrOf/norm(OrOf))+dot(m4,OrOf/norm(OrOf))+dot(m6,OrOf/norm(OrOf));

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
    %% ロドリゲスの回転行列
    function X = Rod_R(n1,n2,n3,the)
    
        X = [cos(the)+(1-cos(the))*n1^2      n1*n2*(1-cos(the))-n3*sin(the)   n1*n3*(1-cos(the))+n2*sin(the); ...
             sin(the)*n3+n2*n1*(1-cos(the))  cos(the)+(1-cos(the))*n2^2       n2*n3*(1-cos(the))-n1*sin(the); ...
            -sin(the)*n2+n3*n1*(1-cos(the))  n3*n2*(1-cos(the))+n1*sin(the)   cos(the)+(1-cos(the))*n3^2   ];
    
    end

end