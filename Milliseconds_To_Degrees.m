function [ Degrees ] = Milliseconds_To_Degrees( Milliseconds )
Milliseconds = abs(Milliseconds);
Degrees = Milliseconds / 30;  
end