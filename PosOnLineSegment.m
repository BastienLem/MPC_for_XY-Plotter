function [s]= ...
PosOnLineSegment(Length,Theta,pos_ini,Pos)
    % Give the nearest position to the coordinates Pos given on the line
    % segment described by the coordinates of its starting point pos_ini, 
    % its orientation Theta and its length
    
    % inspiré par la source suivante :
    % https://diego.assencio.com/?index=ec3d5dfdfc0b6a0d147a656f0af332bd
    % Consulté le 08/07/2019
    
    d_x = Length*cos(Theta);
    d_y = Length*sin(Theta);
    
    s = ((Pos(1)-pos_ini(1))*d_x+(Pos(2)-pos_ini(2))*d_y)/Length^2;
    if s >1
        s=1;
    elseif s<0
        s=0;
    end
end