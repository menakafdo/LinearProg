
clear all;
close;

N_cont = 25;
N_comp = 16;
hold = 4; 
% a stability constraint provided by the aircraft manifacture 
%eps = 
Ma = 180000;    % mass of the aircraft
Mmax = 275000;
Mpax = 30000;
Mmail= 5000;
Mc = Mmax-Ma-Mpax-Mmail;
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
X_j(1:4) = X(1:4);
X_j(5:8) = X(1:4);
X_j(9:12) = X(5:8);
X_j(13:16) = X(5:8);

MM= [Mi,Mi,Mi,Mi,Mi,Mi,Mi,Mi,Mi,Mi,Mi,Mi,Mi,Mi,Mi,Mi];

%% initiliase the optimisation problem in matlab

 airload = optimproblem('ObjectiveSense','maximize');
 
 x = optimvar('binMatrix',N_cont, N_comp, 'Type','integer','LowerBound',0, 'UpperBound',1);
 %Mi = optimvar('massMatrix',N_comp,1,'LowerBound',0);
 Mx = sum(sum(MM.*x)); % Main Objective
 Mxx = (x.*MM)*X_j;
 airload.Objective = Mx;


 noDuplicates = sum(x,1)<=1;
 noDuplicatesPos = sum(x,2)<=1;
 airload.Constraints.noDuplicates = noDuplicates; 
 airload.Constraints.noDuplicatesPos = noDuplicatesPos; 

 maxWeight = Mx<=Mc;     % Stress conditions
 
 airload.Constraints.maxWeight = maxWeight;
 
 %CGx = Ma*X_a + (Mxx'*X_j)./(Ma + Mmax);

 %CGCons = CGx <= X_stable;
 %airload.Constraints.CGCons = CGCons;

 
 %stability constaints 
 
  stability1 = (Ma + Mx)*(X_target - eps) <= Ma*X_a +sum(sum(Mxx));
  stability2 =  Ma*X_a + sum(sum(Mxx)) <= (Ma + Mx)*(X_target + eps); 
  
 airload.Constraints.stability1 = stability1; 
 airload.Constraints.stability2 = stability2;
 
 options = optimoptions(airload);
 options.MaxNodes= 500000;
 


 [sol,fval] = solve(airload, 'Options', options);
%%

 


