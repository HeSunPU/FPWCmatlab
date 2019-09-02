%plot mask

mask.rangeR = [5, 11];
mask.rangeAngle = 40;%42.5;
lw=1;
n=2;
% cross
line(linspace(RX,RX,2), linspace(0,2*RY,2), 'color', 'c', 'linewidth', lw)
line(linspace(0,2*RX,2), linspace(RY,RY,2), 'color', 'c', 'linewidth', lw)
% dark holes
rs = linspace(mask.rangeR(1),mask.rangeR(2),n)*pixperlamD;
line(RX + cosd(mask.rangeAngle)*rs, RY + sind(mask.rangeAngle)*rs, 'color', 'c', 'linewidth', lw)
line(RX - cosd(mask.rangeAngle)*rs, RY + sind(mask.rangeAngle)*rs, 'color', 'c', 'linewidth', lw)
line(RX - cosd(mask.rangeAngle)*rs, RY - sind(mask.rangeAngle)*rs, 'color', 'c', 'linewidth', lw)
line(RX + cosd(mask.rangeAngle)*rs, RY - sind(mask.rangeAngle)*rs, 'color', 'c', 'linewidth', lw)
n=100;
circd = @(radius,deg_ang)  [radius*cosd(deg_ang);  radius*sind(deg_ang)];
arc1 = circd(mask.rangeR(1),linspace(mask.rangeAngle, 180-mask.rangeAngle, n))*pixperlamD; 
arc2 = circd(mask.rangeR(2),linspace(mask.rangeAngle, 180-mask.rangeAngle, n))*pixperlamD;
line(RX + arc1(1,:), RY + arc1(2,:), 'color', 'c', 'linewidth', lw)
line(RX + arc2(1,:), RY + arc2(2,:), 'color', 'c', 'linewidth', lw)
line(RX + arc1(1,:), RY - arc1(2,:), 'color', 'c', 'linewidth', lw)
line(RX + arc2(1,:), RY - arc2(2,:), 'color', 'c', 'linewidth', lw)
axis([0  2*RX    0  2*RY])%axis equal
