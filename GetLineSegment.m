function [L,Theta,s_dot]= ...
GetLineSegment(previous_end_target,current_end_target,Feedrate)
% Give the parameters needed for the equation of an line segment
% x = x_ini + s*L*cos(theta)
% y = y_ini + s*L*sin(theta)
% with s between 0 and 1 
% Give also s_dot corresponding to the given feedrate

    %Retrouver l'équation du segment de droite
    d_x = current_end_target(1)-previous_end_target(1);
    d_y = current_end_target(2)-previous_end_target(2);
        %Longueur du segment
    L = sqrt(d_x^2 + d_y^2);
        %Theta
    
    Theta = acos(d_x/L); 
            %Si dans quadrant 3 ou 4, arccos ne suffit pas
    if d_y < 0
        Theta = 2*pi - Theta;
    end
    
    %Calcul s_dot en fonction du feedrate donné
    s_dot = Feedrate/L;
end