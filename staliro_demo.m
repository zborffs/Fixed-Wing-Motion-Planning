% model = @(t,out) dubins3d(t,out,p); % p = stuff;
model = @(t,y) [y(2); (1-y(1)^2) * y(2) - y(1)];
time = 24;
input_range = [];
cp_array = [];
X0 = [1.5 2.5; 0 0.1];
phi = '[]a';
Pred(1).str = 'a';
Pred(1).A = eye(2);
Pred(1).b = [100; 100];
opt = staliro_options();
opt.runs = 1;

results = staliro(model, X0, input_range, cp_array, phi, Pred, time, opt);

figure(1)
clf