function [KP,KD,epsilon] = Controller_Gains
%#codegen
% This function specifies the PD gain for the I/O linearizing controller.

%KP = 10^2;
%KD = 2*10;

KP = 50;%100;
KD = 10;%20;

epsilon = 0.1;

end

