function [] = Transmit_All_Directions(Directions_Array)
pause on
[rows,columns] = size(Directions_Array);
for (i = 1: rows)
    Direction = Directions_Array(i,1);
    Duration = Directions_Array(i,2);
    if (Direction == 1)
        disp('f')
        fprintf(getappdata(0,'SerialPort'),'f');
        pause(.1)
        Transmit_Number( Duration )
    elseif (Direction == 2)
        disp('l')
        fprintf(getappdata(0,'SerialPort'),'l');
        pause(.1)
        Transmit_Number( Duration )
    else
        disp('r')
        fprintf(getappdata(0,'SerialPort'),'r');
        pause(.1)
        Transmit_Number( Duration )
    end
    
end

end

