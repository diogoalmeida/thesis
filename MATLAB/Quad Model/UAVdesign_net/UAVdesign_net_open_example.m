function UAVdesign_net_open_example(folder, system)
p=get_param(get_param(gcb,'Parent'), 'FileName');
p=p(1:length(p)-length(get_param(gcb,'Parent'))-4);
p=[p 'Examples' filesep folder];
cd(p);
open_system(system, 'force');