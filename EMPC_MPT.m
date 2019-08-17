clear all
close all
clc

%% EMPC par l'extension MPT

%% XY-Plotter EMPC

% Variables
Ts = 0.5; %s %sampling time
N = 5; %Horizon de prédiction
q = 10; %poids sur les outputs
r = 25; %poids sur les inputs

% Système
n_x = 2; %nbre d'états
n_u = 2; %nbre d'entrées
n_y = 2; %nbre de sorties
A = eye(n_x);
B = Ts*eye(n_u);
C = eye(n_y,n_x);
D = [0];
model = LTISystem('A',A,'B',B,'C',C);

% Valeurs contraintes
vit_x_max = 11.42; %mm/s
vit_x_min = -vit_x_max;
vit_y_max = 11.42; %mm/s
vit_y_min = -vit_y_max;
pos_x_max = 300; %mm
pos_x_min = 0; %mm
pos_y_max = 380; %mm
pos_y_min = 0; %mm

model.u.min=[vit_x_min; vit_y_min];
model.u.max=[vit_x_max; vit_y_max];
model.y.min=[pos_x_min; pos_y_min];
model.y.max=[pos_x_max; pos_y_max];

% Ref track
model.y.with('reference');
model.y.reference = 'free';

% Poids
Q = q*eye(n_y); %poids sur les outputs
R = r*eye(n_u); %poids sur les inputs
UseDare = 1; %1 pour calculer P par Ricatti
if UseDare == 0
  P = Q;
elseif UseDare == 1
  [P,L,K] = dare(A,B,Q,R); %Poids sur l'output final 
end

model.u.penalty = QuadFunction(R);
model.y.penalty = QuadFunction(Q);

% Optimal controller based on on-line optimization (implicit)
ctrl = MPCController(model,N);
% Explicit
ectrl = ctrl.toExplicit();

% Plot
% ectrl.partition.plot() % => impossible car dimension > ou = 4D

% Get Gains =>  u = F_i x + g_i if H_ix <= k_i
F = {};
H = {};
g = {};
k = {};

for i = 1:ectrl.optimizer.Num
  F{i} = ectrl.optimizer.Set(i).Functions('primal').F;
  g{i} = ectrl.optimizer.Set(i).Functions('primal').g;
  H{i} = ectrl.optimizer.Set(i).A;
  k{i} = ectrl.optimizer.Set(i).b;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% EMPC un seul axe du XY-Plotter

% Variables
Ts = 0.5; %s %sampling time
N = 5; %Horizon de prédiction
q = 10; %poids sur les outputs
r = 25; %poids sur les inputs

% Système
n_x = 1; %nbre d'états
n_u = 1; %nbre d'entrées
n_y = 1; %nbre de sorties
A = eye(n_x);
B = Ts*eye(n_u);
C = eye(n_y,n_x);
D = [0];
model = LTISystem('A',A,'B',B,'C',C);

% Valeurs contraintes
vit_x_max = 11.42; %mm/s
vit_x_min = -vit_x_max;
vit_y_max = 11.42; %mm/s
vit_y_min = -vit_y_max;
pos_x_max = 300; %mm
pos_x_min = 0; %mm
pos_y_max = 380; %mm
pos_y_min = 0; %mm

model.u.min=[vit_x_min]; %; vit_y_min];
model.u.max=[vit_x_max]; %; vit_y_max];
model.y.min=[pos_x_min]; %; pos_y_min];
model.y.max=[pos_x_max]; %; pos_y_max];

% Ref track
model.y.with('reference');
model.y.reference = 'free';

% Poids
Q = q*eye(n_y); %poids sur les outputs
R = r*eye(n_u); %poids sur les inputs
UseDare = 1;
if UseDare == 0
  P = Q;
elseif UseDare == 1
  [P,L,K] = dare(A,B,Q,R); %Poids sur l'output final 
end
model.u.penalty = QuadFunction(R);
model.y.penalty = QuadFunction(Q);

% Optimal controller based on on-line optimization (implicit)
ctrl_axeX = MPCController(model,N);
% Explicit
ectrl_axeX = ctrl_axeX.toExplicit();

% Plot
ectrl_axeX.partition.plot()
hold on
xlabel('x [mm]')
ylabel('x_{ref} [mm]')
set(gca,'FontSize',30)
pbaspect([1 1 1])

% Get Gains -> u = F_i x + g_i if H_ix <= k_i
F = {};
H = {};
g = {};
k = {};

for i = 1:ectrl_axeX.optimizer.Num
  F{i} = ectrl_axeX.optimizer.Set(i).Functions('primal').F;
  g{i} = ectrl_axeX.optimizer.Set(i).Functions('primal').g;
  H{i} = ectrl_axeX.optimizer.Set(i).A;
  k{i} = ectrl_axeX.optimizer.Set(i).b;
end

% Vérification i^ème région (par exemple i = 1)
i = 1;
x = 200;
x_ref = 200;

% Il faut que la commande u > 0 
F_mat = cell2mat(F(i));
H_mat = cell2mat(H(i));
g_mat = cell2mat(g(i));
k_mat = cell2mat(k(i));

% Bien dans la région ? H_i X <= k_i avec X donné par :
X = [x;x_ref];
H_matFoisX = H_mat*X;

if(k_mat >= H_matFoisX)
  disp('Bien dans la région considérée')
else
  disp("Le point considéré n'est pas dans la région visée")
end

% Calcul des commandes U par u = F_i x + g_i
U = F_mat*X+g_mat
