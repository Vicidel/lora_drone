% define antenna and frequency
antenna = dipole('Length', 0.1, 'Width', 0.01);
freq = 868e6;

% view antenna
figure();
show(antenna);

% vue en 3d
figure();
pattern(antenna, freq);

% vue en coupe
figure();
pattern(antenna, freq, 0, 0:1:360);