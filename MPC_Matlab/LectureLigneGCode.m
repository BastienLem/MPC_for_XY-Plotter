function [current_mode,current_end_target,arc_offsets,Feedrate]= ...
LectureLigneGCode(tline,verbose)
    
    %Variables locales
        %Modes de fonctionnement
    Rapid_positioning = 0;
    Linear_interpolation = 1;
    CW_interpolation = 2;
    CCW_interpolation = 3;
    No_mode = NaN;
        %Initialisation variables locales
    X = NaN;
    Y = NaN;
    Z = NaN;
    I = NaN;
    J = NaN;
    Feedrate = NaN;
    
    splitLine = strsplit(tline,' '); %Séparation ligne en éléments
    
    %Type de commande
    if strcmp(splitLine{1}, 'G00')
        current_mode = Rapid_positioning;
        if verbose == 1
            disp('Rapid positioning')
            %ou positionnement servomoteur si commence par Z
        end
     
    elseif strcmp(splitLine{1}, 'G01')
        current_mode = Linear_interpolation;
        if verbose == 1
        	disp('Linear interpolation')
        end
            
    elseif strcmp(splitLine{1}, 'G02')
        current_mode = CW_interpolation;
        if verbose == 1
          	disp('Circular interpolation, clockwise')
        end
    elseif strcmp(splitLine{1}, 'G03')
    	if verbose == 1
            disp('Circular interpolation, counterclockwise')
        end
        current_mode = CCW_interpolation;
    else
        current_mode = No_mode;
        if verbose == 1
            disp('No Mode')
            return
        end
    end
   
    %Extraction des valeurs
    for i = 2:length(splitLine) %parcours des éléments de la ligne      
        if splitLine{i}(1) == 'X'
            X = str2num(splitLine{i}(2:end));
        elseif splitLine{i}(1) == 'Y'
            Y = str2num(splitLine{i}(2:end));
        elseif splitLine{i}(1) == 'Z'
            Z = str2num(splitLine{i}(2:end));
        elseif splitLine{i}(1) == 'I'
            I = str2num(splitLine{i}(2:end));
        elseif splitLine{i}(1) == 'J'
            J = str2num(splitLine{i}(2:end));
        elseif splitLine{i}(1) == 'F'
            %peut avoir commentaire entre parenthèse
            tmp = regexp(splitLine{i}(2:end),'\d*','Match');
            if ~isempty(tmp{1})
                Feedrate=str2num(tmp{1})/60; %en mm/s
            else
                Feedrate=NaN;
            end
        end
    end  
    current_end_target = [X,Y,Z];
    arc_offsets = [I,J,0]; %K = 0 rotation seulement dans plan XY
                
end