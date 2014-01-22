function y = cd_get_signal(SignalID, SignalIndex, len)
%#eml
y = zeros(len,1);
eml.ceval('cd_read_signal',  eml.rref(SignalID), SignalIndex, eml.wref(y), length(y));
