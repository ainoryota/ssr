wc = 200;
t1 = 20;
t2 = 80;

for i = 1:100
    if i <= 20
       
        
end











function X = R(i,j)
    if i == "x"
        X = [1 0 0 ; 0 cos(j) -sin(j) ; 0 sin(j) cos(j)];
    elseif i == "y"
        X = [cos(j) 0 sin(j) ; 0 1 0 ; -sin(j) 0 cos(j)];
    elseif i == "z"
        X = [cos(j) -sin(j) 0 ; sin(j) cos(j) 0 ; 0 0 1];
    end    
end

function X = Rod_R(n1,n2,n3,the)
    
    X = [cos(the)+(1-cos(the))*n1^2      n1*n2*(1-cos(the))-n3*sin(the)   n1*n3*(1-cos(the))+n2*sin(the); ...
         sin(the)*n3+n2*n1*(1-cos(the))  cos(the)+(1-cos(the))*n2^2       n2*n3*(1-cos(the))-n1*sin(the); ...
        -sin(the)*n2+n3*n1*(1-cos(the))  n3*n2*(1-cos(the))+n1*sin(the)   cos(the)+(1-cos(the))*n3^2   ];
    
end