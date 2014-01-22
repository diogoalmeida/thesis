function cd_set_signal(SignalID, SignalIndex, x)
%#eml
eml.ceval('cd_write_signal', eml.rref(SignalID), SignalIndex, eml.rref(x), length(x));
