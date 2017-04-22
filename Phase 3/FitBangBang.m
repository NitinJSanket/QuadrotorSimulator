function Coeff = FitBangBang(t0, tf, Conditions)
Coeff = [1 t0 t0^2 0 0 0;
         0  1 2*t0 0 0 0;
         0  0  2   0 0 0;
         0 0 0 1 tf tf^2;
         0 0 0 0  1 2*tf;
         0 0 0 0  0  2   ]\Conditions;
end
