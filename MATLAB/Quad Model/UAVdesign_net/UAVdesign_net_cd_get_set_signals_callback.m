function UAVdesign_net_cd_get_set_signals_callback(gcb)
model = get_param(gcb, 'Parent');
index = strfind(model, '/');
if ~isempty(index)
    model = model(1:(index(1)-1));
end
cr = char(13);

headers = get_param(model, 'SimCustomHeaderCode');
sources = get_param(model, 'SimUserSources');

if isempty(strfind(lower(headers), 'cd_signals.h'))
    headers = strtrim([strtrim(headers) cr '#include "cd_signals.h"']);
    set_param(model, 'SimCustomHeaderCode', headers);
end

if isempty(strfind(lower(sources), 'cd_signals.c'))
    sources = strtrim([strtrim(sources) ' cd_signals.c']);
    set_param(model, 'SimUserSources', sources);
end

headers = get_param(model, 'CustomHeaderCode');
sources = get_param(model, 'CustomSource');

if isempty(strfind(lower(headers), 'cd_signals.h'))
    headers = strtrim([strtrim(headers) cr '#include "cd_signals.h"']);
    set_param(model, 'CustomHeaderCode', headers);
end

if isempty(strfind(lower(sources), 'cd_signals.c'))
    sources = strtrim([strtrim(sources) ' cd_signals.c']);
    set_param(model, 'CustomSource', sources);
end
