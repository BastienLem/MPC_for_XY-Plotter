function[str] = float2str(number)
% float to string conversion with 2 digits before comma and 3 digits after
% comma. Add sign at the beginning
signe = sign(number); %sav du signe
number = abs(number);
str = sprintf('%2.3f',number);
if(number<10) %Ajout d'un "0" => 2 chiffres avant la virgule
  str = ['0' str]; 
end
if signe == 1 %positif
   str = ['+' str];
elseif signe == -1 %negatif
   str = ['-' str];
else 
   str = ['0' str];
end