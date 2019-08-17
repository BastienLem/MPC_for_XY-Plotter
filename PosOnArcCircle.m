function [s]= ...
PosOnArcCircle(Center,Pos,theta_i,theta_f)%,mode)
    %Give the nearest position to the coordinates Pos given on the circle
    %described by the coordinates of its Center and its radius
    
    d_x = Pos(1) - Center(1);
    d_y = Pos(2) - Center(2);
    
    %Angle correspondant
    Angle = acos((d_x)/sqrt(d_x^2 + d_y^2));
      %Si l'angle se trouve quand les quadrants III ou IV
    if d_y < 0
        Angle = 2*pi - Angle;
    end
      
    %Retrouver position s correspondante
      %Correction de l'angle si une partie de l'arc où > 2pi
    if (theta_f > theta_i) && (theta_f >= 2*pi)
      %Sens anti-horlogique et passe par 2pi
      if (Angle<theta_f)
        %Alors angle doit etre a + que 2pi
        Angle = Angle +2*pi;
      end
    elseif (theta_i > theta_f) && (theta_i > 2*pi) 
      %Sens horlogique et passe par 2pi
      if (Angle<theta_f)
        %Alors angle doit etre a + que 2pi
        Angle = Angle +2*pi;
      end
    end
    
      %Calcul de s
    s = (Angle - theta_i) /(theta_f - theta_i); 
    if s >1
        s=1;
    elseif s<0
        s=0;
    elseif isnan(s)
        %peut donner un s=NaN si le point est au centre
        % => on choisit de partir du début de la courbe
        s=0;
    end
end