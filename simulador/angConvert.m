function [angs2] = angConvert(angs)
%angConvert faz com que o �ngulo fique de -pi � pi
%   Detailed explanation goes here
   for i = 1:length(angs)
        if angs(i) > pi
            angs2(i) = angs(i) - 2*pi; 
        elseif angs(i) < -pi
            angs2(i) = angs(i) + 2*pi; 
        else
            angs2(i) = angs(i);
        end
   end
   
end
