clc 
close all
clear all

%% INPUT

% dossier contenant le G-Code du dessin
filepath = 'Batman_0012.ngc';

% Communication série Arduino
comPort = 'COM4'; % Port USB correspondant à l'Arduino
baudrate = 115200; % Vitesses de communication %bits/s

%% CONFIG

% Calcul MPC
Calcul_MPC = 1; %Mettre 1 pour calculer MPC
% Lecture fichier G-Code
Lecture_GCode = 1; %Mettre 1 pour lire le G-Code
% Adapter feedrate max en fonction du rayon de l'arc
Adapt_Feedrate_Arc = 1; %1 pour limiter feedrate max
% Display pour LectureLigneGCode
verbose = 0; % 1 pour afficher le type de segment dans la lecture du GCode
% MPC poids final
UseDare = 1; %Pour que P calculé selon équation de Ricatti si 1. Si 0, P=Q
% Affichage des valeurs des positions, cibles et commandes en fonctionnement
Affichage_Debug = 0;

%% Constantes

STEPS_PER_MM = 87.49; % Nombre de pas par mm
MM_PER_STEPS = 1/STEPS_PER_MM; % Nombre de mm par pas

%% Variables MPC

% Système
n_x = 2; %nbre d'états
n_u = 2; %nbre d'entrées
n_y = 2; %nbre de sorties
% Poids, horizon et sampling time
Ts = 0.5; %s %sampling time
N = 5; %Horizon de prédiction
q = 10; %poids sur les outputs
r = 25; %poids sur les inputs

% Valeurs contraintes
vit_x_max = 11.42; %mm/s
vit_x_min = -vit_x_max;
vit_y_max = 11.42; %mm/s
vit_y_min = -vit_y_max;
pos_x_max = 300; %mm
pos_x_min = 0; %mm
pos_y_max = 380; %mm
pos_y_min = 0; %mm

if Calcul_MPC == 1
    clearvars u x y ref constraints objective
    yalmip('clear')
    % Yalmip variables 
    u = sdpvar(repmat(n_u,1,N),repmat(1,1,N)); 
    x = sdpvar(repmat(n_x,1,N+1),repmat(1,1,N+1));
    y = sdpvar(repmat(n_y,1,N+1),repmat(1,1,N+1));
    ref = sdpvar(repmat(n_y,1,N+1),repmat(1,1,N+1));
    % Yalmip cost and constraints
    constraints = [];
    objective = 0;
end

%% Variables Lecture G-Code

% Modes pour GCode
Rapid_positioning = 0;
Linear_interpolation = 1;
CW_interpolation = 2;
CCW_interpolation = 3;
No_mode = NaN;
current_mode = NaN;

% Pour lecture G-Code
end_target = [0,0,0];
previous_end_target = [0,0,0];
current_end_target = [0,0,0];
target_on_horizon = [];
toolPath = [];
arc_offsets = [0,0,0];
Center = zeros(2,1);

% Feedrate
Feedrate = 0.0;
Default_Feedrate = 1; %mm/s = 60 mm/min
Old_Feedrate = Default_Feedrate; %Initialise à valeur par défaut
Feedrate_G0 = 10; %mm/s mouvement rapide
Feedrate_Cercle_R25 = 100/60;%mm/s %100 mm/min pour R = 25mm
R_25 = 25; %mm

% Variables de mesure
x_0 = [0;0]; %état initial
Actual_pos = zeros(2,1);
Old_Line = ''; %Eviter ligne double G-Code

%Structure pour récupérer les données du GCode
Arc = struct('R',{},'Theta_i',{},'Theta_f',{},'Centre',{});%Arc de cercle
Droite = struct('L',{},'Theta',{}); %Segment de droite
DataGCode = struct('Mode',{}, ... %G00, G01, G02 ou G03
    'Line',{}, ... %Ligne de GCode correspondante
    'pos_init',{},'pos_fin',{}, ... %points de départ et d'arrivée
    's_dot',{},... %vitesse de s avec s allant de 0 a 1 sur une courbe
    'Arc',{Arc},'Droite',{Droite});
EcartSkip = 0.05;%mm %Skip si écart entre positions init et finale du ...
                     % ... segment est inférieure 

%% % Initialisation valeurs

% Pour positions et cibles
Pos = [0;0]; %Position de départ en (0;0)
x_target = []; %Coordonnée X des cibles
y_target = []; %Coordonnée Y des cibles

% Pour Plot
X_plot = [Pos(1) ]; %Reprend l'ensemble des positions (coordonnées X)
Y_plot = [Pos(2) ]; %Reprend l'ensemble des positions (coordonnées Y)
X_target = []; %Reprend l'ensemble des cibles (coordonnées X)
Y_target = []; %Reprend l'ensemble des cibles (coordonnées Y)

%% MPC

% Système
A = eye(n_x);
B = Ts*eye(n_u);
C = eye(n_y,n_x);
% Poids pour le coût du MPC
Q = q*eye(n_y); %poids sur les outputs
R = r*eye(n_u); %poids sur les inputs
if UseDare == 0
  P = Q;
elseif UseDare == 1
  [P,L,K] = dare(A,B,Q,R); %Poids sur l'output final 
end
% Contraintes inputs
u_min = [vit_x_min; vit_y_min];
u_max = [vit_x_max; vit_y_max];
% Contraintes outputs
y_min = [pos_x_min; pos_y_min];
y_max = [pos_x_max; pos_y_max];

if Calcul_MPC == 1
  % Implémentation
    % Va de 0 à N mais comme Matlab ne prend pas l'indice zéro, 
    % il faut prendre de 1 à N+1
  for k = 1:N % de 0 à N-1 pour les outputs et inputs
    objective = objective+(y{k}-ref{k})'*Q*(y{k}-ref{k})+u{k}'*R*u{k};
    constraints = [constraints, x{k+1} == A*x{k}+B*u{k}]; 
    constraints = [constraints, y{k} == C*x{k}];
    constraints = [constraints, u_min <= u{k}<= u_max];
    constraints = [constraints, y_min <= y{k+1}<= y_max];
    %on prend k+1 pour les y car on part d'un y_k fixé au départ
  end
    % N pour les outputs seulement
  objective = objective + (y{N+1}-ref{N+1})'*P*(y{N+1}-ref{N+1});
    % Définitions parametres et solutions
  parameters_in = {x{1},[ref{:}]}; 
  solutions_out = {[u{:}], [x{:}]};
    % => MPC 
  MPC = optimizer(constraints, objective,sdpsettings('solver','gurobi'),...
      parameters_in,solutions_out);
end

%% Lecture G-Code

if Lecture_GCode == 1
  
  clear DataGCode
  
  % Open GCode file
  raw_gcode_file = fopen(filepath);
  
  %Passer l'entête
  while ~feof(raw_gcode_file) %jusqu'à ce que fin du fichier est atteint
    tline = fgetl(raw_gcode_file);
    if (strcmp(tline,'(Change tool to Default tool)')==1)
      break %quitte le while quand en-tête est passée
    end
  end

  row = 1; %compteur ligne pour DataGCode
  while ~feof(raw_gcode_file) 
    %Lecture ligne
    tline = fgetl(raw_gcode_file);
    %if (strcmp(tline,'')==0)
    if(isempty(tline))  %Si ligne vide
      continue
    elseif strcmp(Old_Line,tline)
      %Peut arriver que deux fois la meme ligne
      disp('Line 2 times in a row. Check the G-Code file on line :');
      disp(tline);
      continue
    else %ligne non vide et pas une double
      if(tline(1)=='G') %Si ligne commence par G => à traiter
        [current_mode,current_end_target,arc_offsets,Feedrate] = ...
          LectureLigneGCode(tline,verbose);
          if(current_mode == Rapid_positioning)
            Feedrate = Feedrate_G0;
          end
      else
        continue 
      end
    end
    Old_Line = tline;

    %Si valeur en Z non stipulée, prendre celui de l'ancienne
    if isnan(current_end_target(3))
      current_end_target(3) = previous_end_target(3); 
    end

    %Feedrate n'est pas donné à chaque ligne
    if isnan(Feedrate)
      Feedrate = Old_Feedrate;
    end

    %Extraction des paramètres 
    if(current_mode == Rapid_positioning || ...
              current_mode == Linear_interpolation) 
      %G00 et G01 traités de la même manière mais G00 utilisera pas MPC.
      %et G00 utilise déplacement linéaire à vitesse par défaut
      %Positionnement mais aussi servomoteur par déplacement en Z only.
      
      DataGCode(row).Mode = current_mode;
      DataGCode(row).Line = tline;      

      %ServoMoteur
      if isnan(current_end_target(1)) && isnan(current_end_target(2))
        %Si pas de valeur pour positions X et Y, alors seulement Z
        DataGCode(row).Droite = NaN;
        DataGCode(row).Arc = NaN;
        DataGCode(row).pos_init = previous_end_target;
        %report des valeurs en X et Y du cas précédent
        previous_end_target(3) = current_end_target(3); 
        current_end_target = previous_end_target;
        DataGCode(row).pos_fin = current_end_target;            
        %report de la valeur cible précédente
        current_end_target = previous_end_target;

      %Positionnement
      else 
        %déplacement trop petit pour être pris en compte
        if (norm(current_end_target - previous_end_target)) < EcartSkip
          disp('Line without mouvement. Check the G-Code file on line :');
          disp(tline);
          continue %On passe cette ligne
        end
        DataGCode(row).pos_init = previous_end_target;
        DataGCode(row).pos_fin = current_end_target;
        DataGCode(row).Arc = NaN;
        [DataGCode(row).Droite.L,DataGCode(row).Droite.Theta,...
          DataGCode(row).s_dot] = GetLineSegment(previous_end_target,...
          current_end_target,Feedrate);
      end
      Old_Feedrate = Feedrate;
      row = row+1;     

    elseif(current_mode == CW_interpolation || ...
      current_mode == CCW_interpolation)
      %Retrouver l'équation de l'arc de cercle
      DataGCode(row).Mode = current_mode;
      DataGCode(row).Line = tline;
      DataGCode(row).pos_init = previous_end_target;
      DataGCode(row).pos_fin = current_end_target;
      DataGCode(row).Droite = NaN;
      if(Adapt_Feedrate_Arc == 1)
        % Sécurité Feedrate max selon le rayon 
        R = sqrt(arc_offsets(1)^2 + arc_offsets(2)^2);
        Feedrate_max = Feedrate_Cercle_R25*sqrt(R/R_25);
        if(Feedrate > Feedrate_max)
          Feedrate = Feedrate_max;
        end
      end
      % Obtention des paramètres
      [DataGCode(row).Arc.Centre,DataGCode(row).Arc.R,...
        DataGCode(row).Arc.Theta_i,DataGCode(row).Arc.Theta_f,...
        DataGCode(row).s_dot] = GetArc(previous_end_target,...
        current_end_target,arc_offsets,Feedrate,current_mode);
      
      Old_Feedrate = Feedrate;
      row = row+1;
    end

    %Actualisation valeurs pour ligne suivante
    previous_end_target = current_end_target;  
    current_mode = NaN; %Réinitialise pour ligne suivante
  
  end 
  % Fermeture fichier
  fclose(raw_gcode_file);
end

%% Setup Communication

% Connexion serial avec Arduino sur port comPort avec vitesse baudrate
if ~isempty(instrfind) %ferme connexion serial si déjà existante
  fclose(instrfind);
  delete(instrfind);
end
[Arduino,flag] = setupSerial(comPort,baudrate);
% Format des données provenant de l'arduino
formatSpec = '%f,%f'; %Format position %deux floats séparés par une virgule
formatVerif = '%c'; %Format Handshake %seulement 1 caractere

%% Parcours des segments du dessin

for row = 1:length(DataGCode)
  disp(row);
  
  %Lecture des paramètres du segment considéré
  target_begin = DataGCode(row).pos_init;
  target_end = DataGCode(row).pos_fin;
  s_dot = DataGCode(row).s_dot;
  %-> Arc de cercle
  if isstruct(DataGCode(row).Arc)
    R = DataGCode(row).Arc.R;
    C_x = DataGCode(row).Arc.Centre(1);
    C_y = DataGCode(row).Arc.Centre(2);
    theta_i = DataGCode(row).Arc.Theta_i;
    theta_f = DataGCode(row).Arc.Theta_f;
  %-> Segment de droite
  elseif isstruct(DataGCode(row).Droite)
    L = DataGCode(row).Droite.L;
    Theta = DataGCode(row).Droite.Theta;
  end
  
  % Initialisation paramètre de courbe, s allant de 0 à 1
  s = 0; %initialisation valeur : départ du segment en s = 0
  s_nexttime = 0; %sert pour sécurité et empêcher de bloquer
  
  %Tant que fin du segment non atteinte, on reste sur même segment
  
  
  while(s<1)
 
    % Acquisition position :
      % Attente de données provenant de l'Arduino
    IsThereData = Arduino.BytesAvailable; %Bytes provenant de l'Arduino ?
    while IsThereData==0 % Attente
      IsThereData = Arduino.BytesAvailable;
    end
    
    Old_Pos = Pos; %Sauvegarde ancienne position (sert à éviter répétition)
    Pos = fscanf(Arduino,formatSpec); %Lecture de la position envoyée

    %Affichage debug
    if(Affichage_Debug == 1)
      disp('Position'); %affichage position
      disp(Pos);
    end
    
    %- CAS 1)------------------------ ARC DE CERCLE ----------------------%
    
    if isstruct(DataGCode(row).Arc)
      % paramètre de courbe s le plus proche de la position réelle
      s = PosOnArcCircle([C_x; C_y],Pos,theta_i,theta_f);
  
      %Sécurité parcours de courbe :
      % - Empêcher de passer un segment complètement par s_nexttime
      % initialisé à zéro
      % - Revenir en arrière si le chariot s'est déplacé trop loin
      if s>s_nexttime
        s=s_nexttime;
      end
    
      %Sauvegarde de l'ancienne valeur 
      s_nexttime_old = s_nexttime; %Empêche bloquage fin de courbe
      %Calcul du s correspondant à la target k+1|k
      s_nexttime = s+s_dot*Ts;
      
      %Génération des targets
      for i=0:N %N+1 targets à générer
        ds = s+s_dot*Ts*i;
        %Ne pas dépasser fin du segment s = 1
        if ds >1
          ds = 1;
        end
        x_target = [x_target R*cos(ds*(theta_f-theta_i)+theta_i)+C_x];
        y_target = [y_target R*sin(ds*(theta_f-theta_i)+theta_i)+C_y];
      end
        
      %MPC
      Target = [x_target; y_target];
      inputs = {Pos,Target};
      [solutions,diagnostics] = MPC{inputs};    
      U = solutions{1};
      X = solutions{2};
      if diagnostics == 1
          error('The problem is infeasible');
      end
      
      % Transfert
      fprintf(Arduino,'a');
      %Handshake
      a=fscanf(Arduino,formatVerif);
      while (a~='a') 
        %Arduino n'a pas fini de vider son buffer
        a=fscanf(Arduino,formatVerif);
      end
        %conversion resultat en un string
      command_u = float2str(U(1,1));
      command_v = float2str(U(2,1));
      fprintf(Arduino,[command_u ',' command_v]); %Envoi
        % Accumulation data pour Plot
      X_plot = [X_plot Pos(1)];
      Y_plot = [Y_plot Pos(2)];
      X_target = [X_target x_target(1)];
      Y_target = [Y_target y_target(1)];
        
      % Forcer passage prochaine ligne pour éviter de rester bloqué
      if (s_nexttime_old >= s_nexttime) && s_nexttime > 1 ...
          && norm(Pos-Old_Pos)<0.5*MM_PER_STEPS
        disp('Passage forcé à la prochaine ligne');
        s=1; %Force passage fin while
      end 
      
    %- CAS 2)----------------- SEGMENT DE DROITE -------------------------%
    
    elseif isstruct(DataGCode(row).Droite)
      %Avoir le s correspondant au point le plus proche de la Pos
      s = PosOnLineSegment(L,Theta,target_begin,Pos);
     
      %Sécurité parcours de courbe :
      % - Empêcher de passer un segment complètement par s_nexttime
      % initialisé à zéro
      % - Revenir en arrière si le chariot s'est déplacé trop loin
      if s>s_nexttime
        s=s_nexttime;
      end
      
      %Sauvegarde de l'ancienne valeur 
      s_nexttime_old = s_nexttime; %Empêche bloquage fin de courbe
      %Calcul du s correspondant à la target k+1|k
      s_nexttime = s+s_dot*Ts;
      
      %Génération des targets
      for i=0:N %N+1 targets à générer
        ds = s+s_dot*Ts*i;
        %Ne pas dépasser fin du segment s = 1
        if ds >1
          ds = 1;
        end
        x_target = [x_target L*ds*cos(Theta)+target_begin(1)];
        y_target = [y_target L*ds*sin(Theta)+target_begin(2)];
      end
        
      %MPC
      Target = [x_target; y_target];
      inputs = {Pos,Target};
      [solutions,diagnostics] = MPC{inputs};    
      U = solutions{1};
      X = solutions{2};
      if diagnostics == 1
          error('The problem is infeasible');
      end      
      
      % Transfert
      fprintf(Arduino,'a');
        %attend que confirmation du buffer vide
      %Handshake
      a=fscanf(Arduino,formatVerif);
      while (a~='a') 
        %Arduino n'a pas fini de vider le buffer
        a=fscanf(Arduino,formatVerif);
      end
        %conversion resultat en un string
      command_u = float2str(U(1,1));
      command_v = float2str(U(2,1));
      fprintf(Arduino,[command_u ',' command_v]); %Envoi
        % Accumulation data pour Plot
      X_plot = [X_plot Pos(1)];
      Y_plot = [Y_plot Pos(2)];
      X_target = [X_target x_target(1)];
      Y_target = [Y_target y_target(1)];
      
      %Forcer passage prochaine ligne pour éviter de rester bloqué
      if (s_nexttime_old >= s_nexttime) && s_nexttime > 1 ...
          && norm(Pos-Old_Pos)<0.5*MM_PER_STEPS
        disp('Passage forcé à la prochaine ligne');
        s=1; %Force passage fin while
      end  
   
    %- CAS 3)---------------------- SERVOMOTEUR --------------------------%
    
    else
      disp("Pas arc ni segment de droite");
      %Handshake
      fprintf(Arduino,'a');
      a=fscanf(Arduino,formatVerif);
      while (a~='a') 
        %Arduino n'a pas fini de vider son buffer
        a=fscanf(Arduino,formatVerif);
      end
      %Simplifié : servomoteur levé/baissé si Z > 0 / Z < 0 respectivement
      if DataGCode(row).pos_fin(3) > 0
        %Servo position haute
          fprintf(Arduino,"Z00000000000001");
      else %if DataGCode(row).pos_fin(3)<= 0
        %Servo position basse
          fprintf(Arduino,"Z00000000000000");
      end
      s = 1; %Force passage fin while
    end
    
    %----------------------------- FIN -----------------------------------%
    
    %Plot en temps réel 
    if(isstruct(DataGCode(row).Arc) || isstruct(DataGCode(row).Droite))
      plot(X_plot, Y_plot,'r*');
      label_1 = 'position reelle';
      hold on
      %plot target 
      plot(X_target,Y_target,'k');
      label_2 = 'cibles';
      %plot future target
      plot(x_target,y_target,'g+-');
      label_3 = 'futures cibles';
      %plot estimation future position
      plot(X(1,2:end),X(2,2:end),'bx');
      label_4 = 'futures positions';
      %Affichage dans la zone du dernier point
      axis([X_plot(end)-5 X_plot(end)+5 Y_plot(end)-5 Y_plot(end)+5]) 
      legend(label_1,label_2,label_3,label_4);
      drawnow;
      hold off
      
      %AFFICHAGE DEBUG
      if(Affichage_Debug == 1)
        disp("Target");
        disp(Target(:,2));
        disp("Commande");
        disp(U(:,1));
        disp("---------------------------------------");
      end
    end
    
    %Réinitialisation targets
    x_target = [];
    y_target = [];   
    
  end
  disp("Prochaine ligne");
  %Réinitialise valeur pour prochaine ligne
  s_dot = [];
  target_begin = [];
  target_end = []; 
end

%% Fermer port

disp('fin de transmission, fermeture des ports');
if ~isempty(instrfind)
  fclose(instrfind);
  delete(instrfind);
end
% Vérifier que tout est bien fermé
if(~isempty(instrfind))
  disp('Problème de fermeture des ports');
end

%% Plot graphe final

labels = cellstr(num2str([1:length(X_plot)]'));
figure()
hold on
plot(X_target, Y_target,'b');
plot(X_plot, Y_plot,'r*');
text(X_plot, Y_plot,labels,'VerticalAlignment','bottom',...
  'HorizontalAlignment','right');