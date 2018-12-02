function [ Milliseconds  ] = Degrees_To_Milliseconds( Degrees )
Degrees = abs(Degrees);
Milliseconds = Degrees * 30;                              %rough estimation
end

