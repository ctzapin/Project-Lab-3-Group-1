function [ Centimeters ] = Milliseconds_To_Centimeters( Milliseconds )
Milliseconds = abs(Milliseconds);
Centimeters = Milliseconds / 33;
end
