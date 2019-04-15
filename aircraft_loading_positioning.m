
clear all;
close;

N_cont = 25;
N_comp = 4;
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

containerData = 'dataload.txt';
holdData = 'holdData.txt';
M = dlmread(containerData);
X = dlmread(holdData); 
Mi = M(:,2);
% X_j(1:4) = X(1:4);
% X_j(5:8) = X(1:4);
% X_j(9:12) = X(5:8);
% X_j(13:16) = X(5:8);

X_j(1) = 0.5*(X(1) + X(2));
X_j(2) = 0.5*(X(3) + X(4));
X_j(3) = 0.5*(X(5) + X(6));
X_j(4) = 0.5*(X(7) + X(8));

cont_cat = M(:,3);

% cont_1 = length(cont_cat(cont_cat==1));
% cont_2 = length(cont_cat(cont_cat==2));
% cont_3 = length(cont_cat(cont_cat==3));
% cont_4 = length(cont_cat(cont_cat==4));
% cont_5 = length(cont_cat(cont_cat==5));

map_cont_1 = cont_cat==1;
map_cont_2 = cont_cat==2; 
map_cont_3 = cont_cat==3;
map_cont_4 = cont_cat==4;
map_cont_5 = cont_cat==5;

map_cont_dec_1 = bin2dec(int2str(map_cont_1));
map_cont_dec_2 = bin2dec(int2str(map_cont_2));
map_cont_dec_3 = bin2dec(int2str(map_cont_3));
map_cont_dec_4 = bin2dec(int2str(map_cont_4));
map_cont_dec_5 = bin2dec(int2str(map_cont_5));



weight_map_1 = map_cont_1.*Mi;
weight_map_2 = map_cont_2.*Mi;
weight_map_3 = map_cont_3.*Mi;
weight_map_4 = map_cont_4.*Mi;
weight_map_5 = map_cont_5.*Mi;

MM= [Mi,Mi,Mi,Mi];

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
 
 %% compartment 1

s1 = sum(x(:,1).*map_cont_dec_1) + 2*((sum(x(:,1).*map_cont_dec_2)) + (sum(x(:,1).*map_cont_dec_3))); 
l1 = sum(x(:,1).*map_cont_dec_4) + sum(x(:,1).*map_cont_dec_5);
B11 = 8; B12 = 1 ; B13 = 6; B14 = 2; B15 =4;
y11 = optimvar('y11',1, 'Type','integer','LowerBound',0, 'UpperBound',1);

comp1_constraint1 = s1+ 4*l1 - B11*y11 <= 8;
comp1_constraint2 = l1 - B12*y11 <= 1;
comp1_constraint3 = s1 - B13*(1-y11) <= 0;
comp1_constraint4 = -l1 - B14*(1-y11) <= -2;
comp1_constraint5 = l1 - B15*(1-y11) <= 2;

airload.Constraints.comp1_constraint1 = comp1_constraint1;
airload.Constraints.comp1_constraint2 = comp1_constraint2;
airload.Constraints.comp1_constraint3 = comp1_constraint3;
airload.Constraints.comp1_constraint4 = comp1_constraint4;
airload.Constraints.comp1_constraint5 = comp1_constraint5;


%%

%% compartment 2

s2 = sum(x(:,2).*map_cont_dec_1) + 2*((sum(x(:,2).*map_cont_dec_2)) + (sum(x(:,2).*map_cont_dec_3))); 
l2 = sum(x(:,2).*map_cont_dec_4) + sum(x(:,2).*map_cont_dec_5);
B21 = 16; B22 = 3 ; B23 = 14; B24 = 2; B25 =1; B26 = 4 ; B27 = 4; B28 = 0;

y21 = optimvar('y21',1, 'Type','integer','LowerBound',0, 'UpperBound',1);
y22 = optimvar('y22',1, 'Type','integer','LowerBound',0, 'UpperBound',1);
y23 = optimvar('y23',1, 'Type','integer','LowerBound',0, 'UpperBound',1);

%y23 = 2 - y21 - y22; 

comp2_constraint1 = s2+ 4*l1 - B21*y21 <= 12;
comp2_constraint2 = l1 - B22*y21 <= 1;
comp2_constraint3 = s2+ 4*l1 - B23*y22 <= 14;
comp2_constraint4 = -l2 - B24*y22 <= -2;
comp2_constraint5 = l2 - B25*y22 <= 3;
comp2_constraint6 = s2 - l2 - B26*y23 <= 0;
comp2_constraint7 = -l2 - B27*y23 <= -4;
comp2_constraint8 = l2 - B28*y23 <= 4;
comp2_constraint9 = y23 + y21 + y22 == 2 ; 
airload.Constraints.comp2_constraint1 = comp2_constraint1;
airload.Constraints.comp2_constraint2 = comp2_constraint2;
airload.Constraints.comp2_constraint3 = comp2_constraint3;
airload.Constraints.comp2_constraint4 = comp2_constraint4;
airload.Constraints.comp2_constraint5 = comp2_constraint5;
airload.Constraints.comp2_constraint6 = comp2_constraint6;
airload.Constraints.comp2_constraint7 = comp2_constraint7;
airload.Constraints.comp2_constraint8 = comp2_constraint8;
airload.Constraints.comp2_constraint9 = comp2_constraint9;


%%


%% compartment 3

s3 = sum(x(:,3).*map_cont_dec_1) + 2*((sum(x(:,3).*map_cont_dec_2)) + (sum(x(:,3).*map_cont_dec_3))); 
l3 = sum(x(:,3).*map_cont_dec_4) + sum(x(:,3).*map_cont_dec_5);
B31 = 2; B32 = 8 ; B33 = 12; B34 = 1; B35 =6; B36 = 2 ; B37 = 1; B38 = 8; B39 = 2; 

y31 = optimvar('y31',1, 'Type','integer','LowerBound',0, 'UpperBound',1);
y32 = optimvar('y32',1, 'Type','integer','LowerBound',0, 'UpperBound',1);
y33 = 2 - y31 - y32; 


comp3_constraint1 = -sum(x(:,4).*map_cont_dec_5) - B31*y31 <=-2;
comp3_constraint2 = s3 + 2*l3 - B32*y31 <= 6 ; 
comp3_constraint3 = s3 + 4*l3 - B33*y31 <= 8; 
comp3_constraint4 = sum(x(:,4).*map_cont_dec_5) - B34*y32 <=1;
comp3_constraint5 = s3 +2*l3 - B35*y32 <=8;
comp3_constraint6 = l3 - B36*y32 <= 1;
comp3_constraint7 =  sum(x(:,4).*map_cont_dec_5) - B37*y33 <=6;
comp3_constraint8 =  s3 + 2*l3 - B38*y33 <= 6;
comp3_constraint9 =  -l3 - B39*y33 <= -2;
comp3_constraint10 = l3 - B39*y33 <= 2; 

airload.Constraints.comp2_constraint1 = comp3_constraint1;
airload.Constraints.comp2_constraint2 = comp3_constraint2;
airload.Constraints.comp2_constraint3 = comp3_constraint3;
airload.Constraints.comp2_constraint4 = comp3_constraint4;
airload.Constraints.comp2_constraint5 = comp3_constraint5;
airload.Constraints.comp2_constraint6 = comp3_constraint6;
airload.Constraints.comp2_constraint7 = comp3_constraint7;
airload.Constraints.comp2_constraint8 = comp3_constraint8;
airload.Constraints.comp2_constraint9 = comp3_constraint9;
airload.Constraints.comp2_constraint10 = comp3_constraint10;


%%


%% compartment 4

s4 = sum(x(:,4).*map_cont_dec_1) + 2*((sum(x(:,4).*map_cont_dec_2)) + (sum(x(:,4).*map_cont_dec_3))); 
l4 = sum(x(:,4).*map_cont_dec_4) + sum(x(:,4).*map_cont_dec_5);
B41 = 8; B42 = 1 ; B43 = 6; B44 = 2; B45 =4;
y41 = optimvar('y41',1, 'Type','integer','LowerBound',0, 'UpperBound',1);

comp4_constraint1 = s4 + 4*l4 - B41*y41 <= 8;
comp4_constraint2 = l4 - B42*y41 <= 1;
comp4_constraint3 = s4 - B43*(1-y41) <= 0;
comp4_constraint4 = -l4 - B44*(1-y41) <= -2;
comp4_constraint5 = l4 - B45*(1-y41) <= 2;

airload.Constraints.comp4_constraint1 = comp4_constraint1;
airload.Constraints.comp4_constraint2 = comp4_constraint2;
airload.Constraints.comp4_constraint3 = comp4_constraint3;
airload.Constraints.comp4_constraint4 = comp4_constraint4;
airload.Constraints.comp4_constraint5 = comp4_constraint5;


%%

 options = optimoptions(airload);
 options.MaxNodes= 500000;
 [sol,fval] = solve(airload, 'Options', options);
%%

 


