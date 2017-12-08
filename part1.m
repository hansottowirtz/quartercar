m1=100;m2=400;
k1=3e5;k2=1e5;
c1=1e3;c2=200;
M = [m1,0;0,m2];
K = [k1+k2,-k2;-k2,k2];
C = [c1+c1,-c2;-c2,c2];
 
M1 = [eye(2) zeros(2);zeros(2) M];
K1 = [zeros(2) -eye(2);K C];
[V,D] = eig(K,M);
[V1,D1] = eig(K1,M1);
 
pvec = diag(D1);
omega_n = abs(pvec([1 3]))
xi = real(pvec([1 3]))./abs(pvec([1 3]))
