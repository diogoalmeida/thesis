function cd_integrator_callback(gcb, param)
    switch param
        % Integration limits
        case 1,
            maskval = get_param(gcb, 'MaskValues');
            visibility = get_param(gcb, 'MaskVisibilities');
            if strcmp(maskval{2}, 'Internal'),
                visibility{3} = 'on';
                visibility{4} = 'on';
            else
                visibility{3} = 'off';
                visibility{4} = 'off';
            end
            set_param(gcb, 'MaskVisibilities', visibility);
        
        % Initial Condition
        case 2,
            maskval = get_param(gcb, 'MaskValues');
            visibility = get_param(gcb, 'MaskVisibilities');
            if strcmp(maskval{6}, 'Internal'),
                visibility{7} = 'on';
            else
                visibility{7} = 'off';
            end
            set_param(gcb, 'MaskVisibilities', visibility);
        
        % Limit output
        case 3,
            maskval = get_param(gcb, 'MaskValues');
            visibility = get_param(gcb, 'MaskVisibilities');
            if strcmp(maskval{9}, 'on'),
                visibility{10} = 'on';
                visibility{11} = 'on';
            else
                visibility{10} = 'off';
                visibility{11} = 'off';
            end
            set_param(gcb, 'MaskVisibilities', visibility);
        
        % Draw mask
        case 10,
            maskval = get_param(gcb, 'MaskValues');
            maskdisp = ['disp(''\fontsize{16}\int'', ''texmode'', ''on''); port_label(''input'', 1, ''x^{\bullet}'', ''texmode'', ''on''); '];
            index = 2;
            if strcmp(maskval{6}, 'External')
                maskdisp = [maskdisp 'port_label(''input'', ' num2str(index) ', ''x_0'', ''texmode'', ''on''); '];
                index = index+1;
            end
            if strcmp(maskval{2}, 'External')
                maskdisp = [maskdisp 'port_label(''input'', ' num2str(index) ', ''Par'', ''texmode'', ''on''); '];
                index = index+1;
            end
            if strcmp(maskval{5}, 'External')
                maskdisp = [maskdisp 'port_label(''input'', ' num2str(index) ', ''R'', ''texmode'', ''on''); '];
                index = index+1;
            end
            maskdisp = [maskdisp 'port_label(''output'', 1, ''fn'', ''texmode'', ''on''); '];
            maskdisp = [maskdisp 'port_label(''output'', 2, ''x''); '];
            if strcmp(maskval{8}, 'on')
                maskdisp = [maskdisp 'port_label(''output'', 3, ''t''); '];
            end
            set_param(gcb, 'MaskDisplay', maskdisp);
    end
end