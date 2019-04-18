
clear all;
close;

N_cont = 14;
N_comp = 1;
hold = 4; 
% a stability constraint provided by the aircraft manifacture 
%eps = 
Ma = 180000;    % zero fuel weight of the aircraft
Mmax = 275000; %Maximum load aircraft can carry
Mpax = 22125;  %Max passenger weight, max passengers= 295 and ~ 75kg of one passsenger weight
Mfuel= 95000; %3.23 kg per 4.72L when the fuel used is aviation gas (total fuel cap = 140640L) 
Mmail = 5000;   %Max mail weight
Mbulk = 35000;    %bulk cargo
Mlandweight = 192000;

Mgoods = Ma - Mpax - Mmail - Mbulk-120000;

 
Mc = Mmax-Mpax-Mfuel-Mmail-Mbulk;
Mi = zeros(N_cont,1);

X_a = 15.1;            %position of the CG before loading
X_j = zeros(N_comp,1);  % position of CG of the each compartment

X_target = 18.0;       % target position of the CG
X_stable = 17.8;       % stable position of the CG
eps = 3.0;

nx = N_cont * N_comp;

xij = zeros(N_cont,N_comp);



%% Data load

containerData = 'dataload_sc.txt';
holdData = 'holdData.txt';
M = dlmread(containerData);
X = dlmread(holdData); 
Mi = M(:,2);
% X_j(1:4) = X(1:4);
% X_j(5:8) = X(1:4);
% X_j(9:12) = X(5:8);
% X_j(13:16) = X(5:8);

X_j(1) = 0.5*(X(1) + X(2));
% X_j(2) = 0.5*(X(3) + X(4));
% X_j(3) = 0.5*(X(5) + X(6));
% X_j(4) = 0.5*(X(7) + X(8));

cont_cat = M(:,3);
cont_type = M(:,4);
 cont_1 = length(cont_cat(cont_cat==1));
 cont_2 = length(cont_cat(cont_cat==2));
% cont_3 = length(cont_cat(cont_cat==3));
% cont_4 = length(cont_cat(cont_cat==4));
% cont_5 = length(cont_cat(cont_cat==5));

map_cont_1 = cont_cat==1;
map_cont_2 = cont_cat==2; 
map_cont_3 = cont_cat==3;
map_cont_4 = cont_cat==4;
map_cont_5 = cont_cat==5;

 map_cont_type_1 = cont_type==1;
 map_cont_type_2 = cont_type==2; 
% map_cont_type_3 = cont_type==3;
% map_cont_type_4 = cont_type==4;
% map_cont_type_5 = cont_type==5;
% 
 map_cont_type_dec_1 =bin2dec(int2str(map_cont_type_1));
 map_cont_type_dec_2 = bin2dec(int2str(map_cont_type_2)); 
% map_cont_type_dec_3 = bin2dec(int2str(map_cont_type_3));
% map_cont_type_dec_4 = bin2dec(int2str(map_cont_type_4));
% map_cont_type_dec_5 = bin2dec(int2str(map_cont_type_5));


map_cont_dec_1 = bin2dec(int2str(map_cont_1));
map_cont_dec_2 = bin2dec(int2str(map_cont_2));
map_cont_dec_3 = bin2dec(int2str(map_cont_3));
map_cont_dec_4 = bin2dec(int2str(map_cont_4));
map_cont_dec_5 = bin2dec(int2str(map_cont_5));

tempControl_Comp = [2,3,4];


weight_map_1 = map_cont_1.*Mi;
weight_map_2 = map_cont_2.*Mi;
weight_map_3 = map_cont_3.*Mi;
weight_map_4 = map_cont_4.*Mi;
weight_map_5 = map_cont_5.*Mi;

MM= Mi;%,Mi,Mi,Mi];

%% initiliase the optimisation problem in matlab

 airload = optimproblem('ObjectiveSense','maximize');
 
 x = optimvar('binMatrix',N_cont, N_comp, 'Type','integer','LowerBound',0, 'UpperBound',1);
 %Mi = optimvar('massMatrix',N_comp,1,'LowerBound',0);
 Mx = sum(sum(MM.*x)); % Main Objective
 Mxx = (x.*MM)*X_j;
 airload.Objective = Mx;
%  noDuplicates = sum(x,1)<=1;
  noDuplicatesPos = sum(x,2)<=1;
%  airload.Constraints.noDuplicates = noDuplicates; 
  airload.Constraints.noDuplicatesPos = noDuplicatesPos; 

 maxWeight = Mx<=Mc;     % Stress conditions
 
 airload.Constraints.maxWeight = maxWeight;
  %CGx = Ma*X_a + (Mxx'*X_j)./(Ma + Mmax); %CGCons = CGx <= X_stable; %airload.Constraints.CGCons = CGCons; %stability constaints 
 stability1 = (Ma + Mx)*(X_target - eps) <= Ma*X_a +sum(sum(Mxx));
 stability2 =  Ma*X_a + sum(sum(Mxx)) <= (Ma + Mx)*(X_target + eps); 
  
 airload.Constraints.stability1 = stability1; 
 airload.Constraints.stability2 = stability2;
 
 perishibaleItem = find(map_cont_type_1);
 
 for i=1:length(perishibaleItem)
   
 peri(i) = x(perishibaleItem(i)) == 1;
% peri(i) = sum(x(perishibaleItem(i),:)) == 1;
 %airload.Constraints.peri(i) = peri(i);
 
 end
 
 airload.Constraints.peri = peri(:);
 
%  dangerousItem = int16(find(map_cont_type_2));
%  k=0;
%  for i=1:length(dangerousItem)
%  
%       for j=1:length(tempControl_Comp)
%         tempControl(k) = (x(dangerousItem(i),int16(tempControl_Comp(j)))) == 0;
%         k= k+1;
%       end
%  %airload.Constraints.peri(i) = peri(i);
%  
%  end
%  
%   airload.Constraints.tempControl = tempControl(:);
% 
%   mindanger = sum((x(dangerousItem,1)))<= ceil(length(dangerousItem)*0.2);
%   
%   airload.Constraints.mindanger = mindanger;

 
 %% compartment 1

s1 = sum(x(:,1).*map_cont_dec_1) + 2*((sum(x(:,1).*map_cont_dec_2)) + (sum(x(:,1).*map_cont_dec_3))); 
l1 = sum(x(:,1).*map_cont_dec_4) + sum(x(:,1).*map_cont_dec_5);
B11 = 8; B12 = 1 ; B13 = 6; B14 = 2; B15 =4;
y11 = optimvar('y11',1, 'Type','integer','LowerBound',0, 'UpperBound',1);
y12 = optimvar('y12',1, 'Type','integer','LowerBound',0, 'UpperBound',1);

comp1_constraint1 = s1+ 4*l1 - B11*y11 <= 6;
comp1_constraint2 = l1 - B12*y11 <= 1;
comp1_constraint3 = s1 - B13*(1-y11) <= 0;
comp1_constraint4 = -l1 - B14*(y12) <= -2;
comp1_constraint5 = l1 - B15*(y12) <= 2;
comp1_constraint6 = y11+y12 == 1;

airload.Constraints.comp1_constraint1 = comp1_constraint1;
airload.Constraints.comp1_constraint2 = comp1_constraint2;
airload.Constraints.comp1_constraint3 = comp1_constraint3;
airload.Constraints.comp1_constraint4 = comp1_constraint4;
airload.Constraints.comp1_constraint5 = comp1_constraint5;
airload.Constraints.comp1_constraint6 = comp1_constraint6;

%% constrainst for special cargo types

ps = sum(x(:,1).*map_cont_type_dec_1);
ds = sum(x(:,1).*map_cont_type_dec_2);

z11 = optimvar('z11',1, 'Type','integer','LowerBound',0, 'UpperBound',1);
z12 = optimvar('z12',1, 'Type','integer','LowerBound',0, 'UpperBound',1);
z13 = optimvar('z13',1, 'Type','integer','LowerBound',0, 'UpperBound',1);

BS11 =8;
BS12 =3;
BS13 =3;
BS14 =2;
BS15 =4;
BS16 =6;
BS17 =3;
BS18 =5;

comp1_sc_constraint1 = z11 + z12 + z13 == 2;

comp1_sc_constraint2 = ds+ 3*ps - BS11*z11 <= 6;
comp1_sc_constraint3 = ps - BS12*z11 <= 1;

comp1_sc_constraint4 = ds+ ps - BS13*z12 <= 4;
comp1_sc_constraint5 = -ps - BS14*z12 <= -2;
comp1_sc_constraint6 = ps - BS15*z12 <= 3;

comp1_sc_constraint7 = ds - BS16*z13 <= 0;
comp1_sc_constraint8 = -ps - BS17*z13 <= -4;
%comp1_sc_constraint9 = -ds - BS18*z13 <= 0;

airload.Constraints.comp1_sc_constraint1 = comp1_sc_constraint1;
airload.Constraints.comp1_sc_constraint2 = comp1_sc_constraint2;
airload.Constraints.comp1_sc_constraint3 = comp1_sc_constraint3;
airload.Constraints.comp1_sc_constraint4 = comp1_sc_constraint4;
airload.Constraints.comp1_sc_constraint5 = comp1_sc_constraint5;
airload.Constraints.comp1_sc_constraint6 = comp1_sc_constraint6;
airload.Constraints.comp1_sc_constraint7 = comp1_sc_constraint7;
airload.Constraints.comp1_sc_constraint8 = comp1_sc_constraint8;
%airload.Constraints.comp1_sc_constraint9 = comp1_sc_constraint9;



%%

%% compartment 2

% s2 = sum(x(:,2).*map_cont_dec_1) + 2*(sum(x(:,2).*map_cont_dec_2) + sum(x(:,2).*map_cont_dec_3)); 
% l2 = sum(x(:,2).*map_cont_dec_4) + sum(x(:,2).*map_cont_dec_5);
% B21 = 16; B22 = 3 ; B23 = 14; B24 = 2; B25 =1; B26 = 4 ; B27 = 4; B28 = 0;
% 
% y21 = optimvar('y21',1, 'Type','integer','LowerBound',0, 'UpperBound',1);
% y22 = optimvar('y22',1, 'Type','integer','LowerBound',0, 'UpperBound',1);
% y23 = optimvar('y23',1, 'Type','integer','LowerBound',0, 'UpperBound',1);
% 
% %y23 = 2 - y21 - y22; 
% 
% comp2_constraint9 = y23 + y21 + y22 == 2 ;
% comp2_constraint1 = s2+ 4*l2 - B21*y21 <= 12;
% comp2_constraint2 = l2 - B22*y21 <= 1;
% comp2_constraint3 = s2+ 4*l2 - B23*y22 <= 14;
% comp2_constraint4 = -l2 - B24*y22 <= -2;
% comp2_constraint5 = l2 - B25*y22 <= 3;
% comp2_constraint6 = s2 - B26*y23 <= 0;
% comp2_constraint7 = -l2 - B27*y23 <= -4;
% comp2_constraint8 = l2 - B28*y23 <= 4;
%  
% airload.Constraints.comp2_constraint9 = comp2_constraint9;
% 
% airload.Constraints.comp2_constraint1 = comp2_constraint1;
% airload.Constraints.comp2_constraint2 = comp2_constraint2;
% airload.Constraints.comp2_constraint3 = comp2_constraint3;
% airload.Constraints.comp2_constraint4 = comp2_constraint4;
% airload.Constraints.comp2_constraint5 = comp2_constraint5;
% airload.Constraints.comp2_constraint6 = comp2_constraint6;
% airload.Constraints.comp2_constraint7 = comp2_constraint7;
% airload.Constraints.comp2_constraint8 = comp2_constraint8;
% 
% 
% %%
% 
% 
% %% compartment 3
% 
% s3 = sum(x(:,3).*map_cont_dec_1) + 2*((sum(x(:,3).*map_cont_dec_2)) + (sum(x(:,3).*map_cont_dec_3))); 
% l3 = sum(x(:,3).*map_cont_dec_4) + sum(x(:,3).*map_cont_dec_5);
% B31 = 2; B32 = 8 ; B33 = 12; B34 = 1; B35 =6; B36 = 2 ; B37 = 1; B38 = 8; B39 = 2; 
% 
% y31 = optimvar('y31',1, 'Type','integer','LowerBound',0, 'UpperBound',1);
% y32 = optimvar('y32',1, 'Type','integer','LowerBound',0, 'UpperBound',1);
% y33 = optimvar('y33',1, 'Type','integer','LowerBound',0, 'UpperBound',1);
% 
% 
% 
% comp3_constraint1 = -sum(x(:,4).*map_cont_dec_5) - B31*y31 <=-2;
% comp3_constraint2 = s3 + 2*l3 - B32*y31 <= 6 ; 
% comp3_constraint3 = s3 + 4*l3 - B33*y31 <= 8; 
% comp3_constraint4 = sum(x(:,4).*map_cont_dec_5) - B34*y32 <=1;
% comp3_constraint5 = s3 +2*l3 - B35*y32 <=8;
% comp3_constraint6 = l3 - B36*y32 <= 1;
% comp3_constraint7 =  sum(x(:,4).*map_cont_dec_5) - B37*y33 <=1;
% comp3_constraint8 =  s3 + 2*l3 - B38*y33 <= 6;
% comp3_constraint9 =  -l3 - B39*y33 <= -2;
% comp3_constraint10 = l3 - y33 <= 2; 
% comp3_constraint11 = y31+y32+y33 == 2; 
% 
% 
% airload.Constraints.comp3_constraint1 = comp3_constraint1;
% airload.Constraints.comp3_constraint2 = comp3_constraint2;
% airload.Constraints.comp3_constraint3 = comp3_constraint3;
% airload.Constraints.comp3_constraint4 = comp3_constraint4;
% airload.Constraints.comp3_constraint5 = comp3_constraint5;
% airload.Constraints.comp3_constraint6 = comp3_constraint6;
% airload.Constraints.comp3_constraint7 = comp3_constraint7;
% airload.Constraints.comp3_constraint8 = comp3_constraint8;
% airload.Constraints.comp3_constraint9 = comp3_constraint9;
% airload.Constraints.comp3_constraint10 = comp3_constraint10;
% airload.Constraints.comp3_constraint11 = comp3_constraint11;
% 
% 
% %%
% 
% 
% %% compartment 4
% 
% s4 = sum(x(:,4).*map_cont_dec_1) + 2*((sum(x(:,4).*map_cont_dec_2)) + (sum(x(:,4).*map_cont_dec_3))); 
% l4 = sum(x(:,4).*map_cont_dec_4) + sum(x(:,4).*map_cont_dec_5);
% B41 = 8; B42 = 1 ; B43 = 6; B44 = 2; B45 =4;
% y41 = optimvar('y41',1, 'Type','integer','LowerBound',0, 'UpperBound',1);
% 
% comp4_constraint1 = s4 + 4*l4 - B41*y41 <= 6;
% comp4_constraint2 = l4 - B42*y41 <= 1;
% comp4_constraint3 = s4 - B43*(1-y41) <= 0;
% comp4_constraint4 = -l4 - B44*(1-y41) <= -2;
% comp4_constraint5 = l4 - B45*(1-y41) <= 2;
% 
% airload.Constraints.comp4_constraint1 = comp4_constraint1;
% airload.Constraints.comp4_constraint2 = comp4_constraint2;
% airload.Constraints.comp4_constraint3 = comp4_constraint3;
% airload.Constraints.comp4_constraint4 = comp4_constraint4;
% airload.Constraints.comp4_constraint5 = comp4_constraint5;


%%
 options = optimoptions(airload);
 options.MaxNodes= 500000;
 [sol,fval] = solve(airload, 'Options', options);
%%

 


