v=[];
sb = 0.1;
min_val = 0.01;
for x = 0.01:0.01:1
    if (x > sb)
        y = (1/min_val)*10 * x;
    else
        y = 1/x;
    end
    v=[v;x y];
end

plot(v(:,1), v(:,2))