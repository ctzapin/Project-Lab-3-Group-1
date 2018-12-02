function [] = Transmit_Number( Duration )
pause on
Duration_String = num2str(Duration);
Length = length(Duration_String);

for i = 1:Length
    disp(Duration_String(i));
    fwrite(getappdata(0,'SerialPort'),Duration_String(i));
    pause(.1)
end

