function[obj,flag] = setupSerial(comPort,baudrate)
% It accept as the entry value, the index of the serial port
% Arduino is connected to, and as output values it returns the serial 
% element obj and a flag value used to check if when the script is compiled
% the serial element exists yet.
flag = 1;
% Initialize Serial object
obj = serial(comPort);
set(obj,'DataBits',8);
set(obj,'StopBits',1);
set(obj,'BaudRate',baudrate);
set(obj,'Parity','none');
set(obj,'Timeout',70); %d'abord aller position départ, besoin de temps
fopen(obj);

a = 'b';
while (a~='a') 
    %Arduino n'a pas fini son setup
    a=fread(obj,1,'uchar')
end
if (a=='a')
    disp('Serial Ready');
end
flushinput(obj); %Flush input buffer 
fprintf(obj,'%c','a');
mbox = msgbox('Serial Communication setup'); 
uiwait(mbox); % comme un waitfor => attend que utilisateur clique sur ok
% fscanf(obj,'%u'); %read integer unsigned 
set(obj,'Timeout',0.05); %Retour à temps d'attente moins long

end
