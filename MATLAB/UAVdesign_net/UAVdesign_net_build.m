function UAVdesign_net_build

p = mfilename('fullpath');
p = p(1:(length(p)-length(mfilename)-1));
cd(p);

disp 'cd_integrator.c'
mex 'cd_integrator.c'

disp 'cd_get_set_signals.c'
mex 'cd_get_set_signals.c'

disp 'datatypeconv_sfun.c'
mex 'datatypeconv_sfun.c'

disp 'display_time_sfun.c'
mex 'display_time_sfun.c'

disp 'jacobian_sfun.c'
mex 'jacobian_sfun.c'

disp 'kalman_filter_sfun.c'
mex 'kalman_filter_sfun.c'

disp 'quaternionintegrator.c'
mex 'quaternionintegrator.c'

disp 'trim_sfun.c'
mex 'trim_sfun.c' 'matrixlib.c'

disp 'serialcomm.c'
mex 'serialcomm.c'

disp 'tomultimediafile.c'
mex 'tomultimediafile.c'

if ispc
    disp 'simulationrate.c'
    mex 'simulationrate.c'

    disp 'udpcomm.c'
    mex 'udpcomm.c' wsock32.lib
    
    disp 'gigecomm.c'
    if exist('PvApi.h', 'file') == 2
        s = fileread('PvApi.h');
        if ~isempty(regexp(s, '//([^\r\n]*)', 'once'))
            cd_c_comments('PvApi.h', 'PvApi.h');
        end
        mex 'gigecomm.c' wsock32.lib
        % eval(['mex ''gigecomm.c'' wsock32.lib ' p '\PvAPI.lib']);
    else
        disp 'The PvApi.h file is not present in the UAVdesign_net folder. No support for the GigE camera will be provided.'
        disp 'You can obtain the file from the AVT PvAPI SDK for GigE Vision cameras'
        disp 'http://www.alliedvisiontec.com/us/products/software/avt-pvapi-sdk.html'
    end

else
    disp 'simulationrate.c'
    mex 'simulationrate.c' -lrt

    disp 'udpcomm.c'
    mex 'udpcomm.c'
    
    disp 'threadpriority.c'
    mex 'threadpriority.c' -lrt
    
    disp 'gigecomm.c'
    if exist('PvApi.h', 'file') == 2
        s = fileread('PvApi.h');
        if ~isempty(regexp(s, '//([^\r\n]*)', 'once'))
            cd_c_comments('PvApi.h', 'PvApi.h');
        end
        mex 'gigecomm.c' -lrt -ldl
        % eval(['mex ''gigecomm.c'' ' p '/libPvAPI.a' ' -lrt']);
    else
        disp 'The PvApi.h file is not present in the UAVdesign_net folder. No support for the GigE camera will be provided.'
        disp 'You can obtain the file from the AVT PvAPI SDK for GigE Vision cameras'
        disp 'http://www.alliedvisiontec.com/us/products/software/avt-pvapi-sdk.html'
    end
end
