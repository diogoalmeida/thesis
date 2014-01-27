function makeInfo=rtwmakecfg
% RTWMAKECFG adds include and source directories to rtw make files. 
%    makeInfo=RTWMAKECFG returns a structured array containing build info. 
%    Please refer to the rtwmakecfg API section in the Real-Time workshop 
%    Documentation for details on the format of this structure. 

makeInfo.includePath =  { pwd };
makeInfo.sourcePath  =  { pwd };
% makeInfo.sources     =  {'cd_signals.c'};
if ispc
    makeInfo.linkLibsObjs = {};
    % makeInfo.linkLibsObjs = { fullfile(pwd, 'PvAPI.lib') };
else
    makeInfo.linkLibsObjs = {};
    % makeInfo.linkLibsObjs = { fullfile(pwd, 'libPvAPI.a'  ) };
end

% [EOF] rtwmakecfg.m