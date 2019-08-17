function [Center,Radius,theta_i,theta_f,s_dot]= ...
GetArc(previous_end_target,current_end_target,arc_offsets,Feedrate,mode)
% Give the parameters needed for the equation of an arc of a circle
% x = Center_x + R*cos(s*(theta_f-theta_i)+theta_i)
% y = Center_y + R*sin(s*(theta_f-theta_i)+theta_i)
% with s between 0 and 1 
% Give also s_dot corresponding to the given feedrate
% Mode is for rotation : CW = 2 or CCW = 3 

    %Retrouver l'équation de l'arc de cercle
        %Coordonnées du centre
    Center(1) = arc_offsets(1) + previous_end_target(1);
    Center(2) = arc_offsets(2) + previous_end_target(2);
        %Valeur du rayon
    dx = -arc_offsets(1);
    dy = -arc_offsets(2);
    Radius = sqrt(dx^2 + dy^2);
        %Angles
    tmp_theta_i = dx/Radius;
    tmp_theta_f = (current_end_target(1)-Center(1))/Radius;
        % erreur numérique peut faire que les acos donne imaginaire
           %Pour theta_i
    if (tmp_theta_i > 1) ||(tmp_theta_i < -1) %not in [-1;1]
      if abs(round(tmp_theta_i,4)) == 1 %erreur de 10^-5 acceptable
        tmp_theta_i = sign(tmp_theta_i)*1;
      else 
        disp("ERREUR: calcul des angles de l'arc de cercle impossible");
      end
    end
          %Pour theta_f
    if (tmp_theta_f > 1) ||(tmp_theta_f < -1) %not in [-1;1]
      if abs(round(tmp_theta_f,4)) == 1 
        tmp_theta_f = sign(tmp_theta_f)*1;
      else 
        disp("ERREUR: calcul des angles de l'arc de cercle impossible");
      end
    end
    theta_i = acos(tmp_theta_i); %ou asin(arc_offsets(2)/R)
    theta_f = acos(tmp_theta_f);
            %Si dans quadrant 3 ou 4, arccos ne suffit pas
    if dy < 0
        theta_i = 2*pi - theta_i;
    end
    if current_end_target(2)-Center(2) < 0
        theta_f = 2*pi - theta_f;
    end
    
            %Prendre les angles entre 0 et 2pi
    theta_i = wrapTo2Pi(theta_i);
    theta_f = wrapTo2Pi(theta_f);
    
            %Si rotation CW, faut theta_i > theta_f
            %Exemple : quand theta_i = 2*pi mais a été mis à 0
    CW = 2; %G02 pour CW
    if((mode == CW)&&(theta_i<=theta_f))
      theta_i = theta_i + 2*pi;
    end
            %Si rotation CCW, faut theta_i < theta_f
            %Exemple : quand theta_f = 2*pi mais a été mis à 0
    CCW = 3; %G03 pour CCW
    if((mode == CCW)&&(theta_i>=theta_f))
      theta_f = theta_f + 2*pi;
    end
        
    %Calcul s_dot en fonction du feedrate donné
        %V = omega*R %si V en mm/s et R en mm/s alors ok 
        % s_dot = ((s_i - s_f)/(theta_i - theta_f))*omega
    omega = Feedrate/Radius;
    s_dot = abs((0 - 1)/(theta_i - theta_f))*omega;
end