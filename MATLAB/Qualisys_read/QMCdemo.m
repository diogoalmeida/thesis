%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%   QMC DEMO
%
%   Initializes QMC, and creates objecthandle 'qtm'.
%
%   getting 3D, analog and 6DOF data from QMC(qtm, 1), one sample at a time.
%   Setup in QMC_conf.txt in order to send the wanted data. 
%
%   Terminate qtm objecthandle, by issuing command 'QMC(qtm, 'disconnect')'.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function QMCdemo(n)		%n is the amount of frames the demo should retrieve before disconnecting.

qtm = QMC('QMC_conf.txt'); % Creates the objecthandle 'qtm' which keeps the connection alive.

for i = 1:n
    [the3D analog the6DOF] = QMC(qtm) % Gather data from QTM. This configuration gets three different types of data from QTM,
                                      % though the number of data types must be the same as in the config-file.
    frameinfo = QMC(qtm, 'frameinfo')
end

QMC(qtm, 'disconnect');		      % Terminates the QMC-object.
clear mex