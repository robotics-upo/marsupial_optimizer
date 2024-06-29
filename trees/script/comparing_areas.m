clear;
clc;
v1=[8.50000 1.80000 0.00000 8.50000 1.50000 1.00000 -0.238 0.269 0.272;
8.50000 1.80000 0.00000 8.15000 1.40000 1.20000 -0.445 0.207 0.597;
8.50000 1.80000 0.00100 8.45000 0.70000 1.10000 -0.596 0.285 1.889;
8.50000 1.80000 0.00100 8.40000 0.17500 1.20000 -0.610 0.318 2.977;
8.46250 1.76250 0.00075 8.35000 -0.35000 1.30000 -0.587 0.337 4.057;
8.42500 1.72500 0.00050 7.95000 -0.50000 1.30000 -0.510 0.351 4.504;
8.38750 1.68750 0.00025 7.60000 -0.70000 1.40000 -0.555 0.350 5.076;
8.35000 1.65000 0.00000 7.20000 -0.80000 1.35000 -0.459 0.360 5.217;
8.27500 1.45000 0.00000 6.80000 -0.90000 1.35000 -0.344 0.370 5.805;
8.20000 1.25000 0.00000 6.45000 -1.05000 1.40000 -0.462 0.361 5.571;
8.30000 1.07500 0.00000 6.10000 -1.25000 1.35000 -0.225 0.376 6.410;
8.40000 0.90000 0.00000 5.70000 -1.15000 1.30000 -0.082 0.379 6.696;
8.00000 0.70000 0.00000 5.30000 -1.30000 1.35000 -0.179 0.378 6.596;
7.85000 0.52500 0.00000 4.90000 -1.15000 1.40000 -0.157 0.378 7.109;
7.70000 0.35000 0.00000 4.50000 -1.00000 1.40000 -0.218 0.377 6.818;
7.62000 0.31000 0.00000 4.10000 -1.05000 1.40000 -0.040 0.380 7.630;
7.54000 0.27000 0.00000 3.75000 -1.20000 1.30000 -290.019 -30.710 1357.834;
7.46000 0.23000 0.00000 3.35000 -1.30000 1.30000 -290.189 -27.991 1488.774;
7.38000 0.19000 0.00000 3.20000 -0.90000 1.30000 -290.192 -27.882 1494.523;
7.30000 0.15000 0.00000 2.80000 -0.95000 1.25000 -274.474 -25.103 1482.398;
7.25556 0.15556 0.00000 2.40000 -1.05000 1.30000 -290.268 -25.786 1614.336;
7.21111 0.16111 0.00000 2.10000 -0.75000 1.30000 -290.290 -24.632 1688.739;
7.16667 0.16667 0.00000 1.75000 -0.65000 1.50000 -353.255 -34.505 1794.356;
7.12222 0.17222 0.00000 1.40000 -0.45000 1.65000 -400.389 -42.014 1897.738;
7.07778 0.17778 0.00000 1.00000 -0.60000 1.70000 -416.239 -42.438 2030.235;
7.03333 0.18333 0.00000 0.70000 -0.80000 1.90000 -478.946 -53.598 2133.767;
6.98889 0.18889 0.00000 0.70000 -0.75000 2.25000 -0.267 0.377 13.117;
6.94444 0.19444 0.00000 0.40000 -0.75000 2.50000 -0.566 0.368 13.619;
6.90000 0.20000 0.00000 0.10000 -0.40000 3.30000 0.119 0.379 8.147;
6.65000 0.02500 0.00000 -0.05000 -0.15000 3.65000 0.453 0.365 6.929;
6.40000 -0.15000 0.00000 0.40000 -0.45000 4.30000 1.066 0.217 3.516;
6.05000 0.05000 0.00000 0.55000 -0.25000 4.60000 1.078 0.165 2.739;
5.65000 -0.05000 0.00000 0.60000 -0.25000 4.95000 0.921 0.195 2.323;
5.25000 0.10000 0.00000 0.75000 0.00000 5.10000 0.792 0.213 1.910;
4.45000 0.15000 0.00000 0.90000 -0.80000 5.30000 0.382 0.333 1.561;
4.10000 -0.05000 0.00000 1.10000 -0.20000 6.00000 0.366 0.314 1.029;
4.00000 -0.08750 0.00000 1.10000 -0.95000 6.10000 0.412 0.300 1.067;
3.90000 -0.12500 0.00000 1.05000 -0.75000 6.45000 0.388 0.307 1.049;
3.80000 -0.16250 0.00000 1.00000 -0.55000 6.80000 0.401 0.300 1.019;
3.70000 -0.20000 0.00000 1.00000 -0.50000 7.00000 0.058 0.378 0.967];

v2=[8.50000 1.80000 0.00000 8.50000 1.50000 1.00000 4.185 0.811 0.380;
8.50000 1.80000 0.00000 8.15000 1.40000 1.20000 1.536 0.726 0.380;
8.50000 1.80000 0.00100 8.45000 0.70000 1.10000 0.320 0.301 0.381;
8.50000 1.80000 0.00100 8.40000 0.17500 1.20000 0.189 0.195 0.381;
8.46250 1.76250 0.00075 8.35000 -0.35000 1.30000 0.154 0.108 0.381;
8.42500 1.72500 0.00050 7.95000 -0.50000 1.30000 0.154 0.053 0.381;
8.38750 1.68750 0.00025 7.60000 -0.70000 1.40000 0.150 0.028 0.380;
8.35000 1.65000 0.00000 7.20000 -0.80000 1.35000 0.102 0.081 0.380;
8.27500 1.45000 0.00000 6.80000 -0.90000 1.35000 0.138 -0.034 0.380;
8.20000 1.25000 0.00000 6.45000 -1.05000 1.40000 0.096 0.076 0.380;
8.30000 1.07500 0.00000 6.10000 -1.25000 1.35000 0.092 0.008 0.380;
8.40000 0.90000 0.00000 5.70000 -1.15000 1.30000 0.078 0.008 0.380;
8.00000 0.70000 0.00000 5.30000 -1.30000 1.35000 0.079 0.022 0.380;
7.85000 0.52500 0.00000 4.90000 -1.15000 1.40000 0.104 -0.051 0.380;
7.70000 0.35000 0.00000 4.50000 -1.00000 1.40000 0.077 0.027 0.380;
7.62000 0.31000 0.00000 4.10000 -1.05000 1.40000 0.078 -0.024 0.380;
7.54000 0.27000 0.00000 3.75000 -1.20000 1.30000 0.007 0.196 0.380;
7.46000 0.23000 0.00000 3.35000 -1.30000 1.30000 0.009 0.172 0.380;
7.38000 0.19000 0.00000 3.20000 -0.90000 1.30000 0.012 0.163 0.380;
7.30000 0.15000 0.00000 2.80000 -0.95000 1.25000 0.000 0.186 0.380;
7.25556 0.15556 0.00000 2.40000 -1.05000 1.30000 0.001 0.178 0.380;
7.21111 0.16111 0.00000 2.10000 -0.75000 1.30000 0.002 0.167 0.380;
7.16667 0.16667 0.00000 1.75000 -0.65000 1.50000 0.003 0.189 0.380;
7.12222 0.17222 0.00000 1.40000 -0.45000 1.65000 0.004 0.200 0.380;
7.07778 0.17778 0.00000 1.00000 -0.60000 1.70000 0.004 0.192 0.380;
7.03333 0.18333 0.00000 0.70000 -0.80000 1.90000 0.005 0.208 0.380;
6.98889 0.18889 0.00000 0.70000 -0.75000 2.25000 0.052 -0.035 0.380;
6.94444 0.19444 0.00000 0.40000 -0.75000 2.50000 0.052 -0.022 0.380;
6.90000 0.20000 0.00000 0.10000 -0.40000 3.30000 0.068 -0.034 0.380;
6.65000 0.02500 0.00000 -0.05000 -0.15000 3.65000 0.099 -0.172 0.380;
6.40000 -0.15000 0.00000 0.40000 -0.45000 4.30000 0.177 -0.409 0.380;
6.05000 0.05000 0.00000 0.55000 -0.25000 4.60000 0.241 -0.559 0.380;
5.65000 -0.05000 0.00000 0.60000 -0.25000 4.95000 0.302 -0.622 0.380;
5.25000 0.10000 0.00000 0.75000 0.00000 5.10000 0.392 -0.715 0.380;
4.45000 0.15000 0.00000 0.90000 -0.80000 5.30000 0.538 -0.639 0.380;
4.10000 -0.05000 0.00000 1.10000 -0.20000 6.00000 1.000 -1.132 0.380;
4.00000 -0.08750 0.00000 1.10000 -0.95000 6.10000 1.111 -1.472 0.380;
3.90000 -0.12500 0.00000 1.05000 -0.75000 6.45000 1.359 -1.886 0.380;
3.80000 -0.16250 0.00000 1.00000 -0.55000 6.80000 1.647 -2.384 0.380;
3.70000 -0.20000 0.00000 1.00000 -0.50000 7.00000 1.284 -1.050 0.380]

[numRows, numCols] = size(v2);

v=[]; R=[];D=[];
syms x;
for i = 1:1:numRows
    xmin1 = 0.0;
    xmax1 = sqrt( (v1(i,1)-v1(i,4)).^2 +  (v1(i,2)-v1(i,5)).^2);
    xmin2 = 0.0;
    xmax2 = sqrt( (v2(i,1)-v2(i,4)).^2 +  (v2(i,2)-v2(i,5)).^2);
    diff_ugv_x = (v1(i,1)-v2(i,1)); diff_ugv_y = (v1(i,2)-v2(i,2)); diff_ugv_z = (v1(i,3)-v2(i,3));
    diff_uav_x = (v1(i,4)-v2(i,4)); diff_uav_y = (v1(i,5)-v2(i,5)); diff_uav_z = (v1(i,6)-v2(i,6));

    x0 = v1(i,7); y0= v1(i,8); c = v1(i,9);
    p  = v2(i,7); q = v2(i,8); r = v2(i,9);
    fun1 = @(x) c * cosh((x - x0)/c)+ (y0 - c);
    fun2 = @(x) p* x.^2 + q*x + r;

    A1 = integral(fun1,xmin1,xmax1);
    A2 = integral(fun2,xmin1,xmax1);
    diff_A = A1-A2;
    condition = 0;
    if (abs(diff_A) > 0.01)
        condition = 1;
        R =[R;i];
    end
    v=[v;A1,A2,diff_A,condition];
    D=[D;diff_ugv_x,diff_ugv_y,diff_ugv_z, diff_uav_x, diff_uav_y, diff_uav_z];
end

fprintf("Difference between Areas grather than 0.1 (10 cm) :")
fprintf('\n')
for j=1:1:length(R)
    fprintf('%d', R(j))
    if j < length(R)
        fprintf(', ')
    else
        fprintf('\n')
    end

end
%In collision 29 30 31 32 33 36 39 40 