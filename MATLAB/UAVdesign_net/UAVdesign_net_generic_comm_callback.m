function UAVdesign_net_generic_comm_callback(gcb, param, start, varargin)
    if param <= 1
        maskval = get_param(gcb, 'MaskValues');
        visibility = get_param(gcb, 'MaskVisibilities');
        if strcmp(maskval{start+1}, 'COM port')
            visibility{start+2} = 'on';
            visibility{start+3} = 'on';
            visibility{start+4} = 'off';
            visibility{start+5} = 'off';
            visibility{start+6} = 'off';
        elseif strcmp(maskval{start+1}, 'UDP port')
            visibility{start+2} = 'off';
            visibility{start+3} = 'off';
            visibility{start+4} = 'on';
            visibility{start+5} = 'on';
            visibility{start+6} = 'on';
        end
        set_param(gcb, 'MaskVisibilities', visibility);
    end
    
    if param == 0
        % Draw mask and replace the communication block if needed
        maskval = get_param(gcb, 'MaskValues');
        loaded = ~isempty(find_system('UAVdesign_net', 'SearchDepth', 0));
        if ~loaded
            load_system('UAVdesign_net');
        end

        if strcmp(maskval{start+1}, 'COM port')
            if ischar(varargin{1})
                maskdisp = ['disp(''' varargin{1} ''');'];
            else
                maskdisp = ['disp(''err'');'];
            end
            if isempty(find_system(gcb, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'Name', 'Serial Port'))
                try
                    pos = get_param([gcb '/UDP'], 'Position');
                    delete_line(gcb, 'TX/1', 'UDP/1');
                    delete_line(gcb, 'UDP/1', 'RX/1');
                    delete_line(gcb, 'UDP/2', 'num_rx_packets/1');
                    delete_block([gcb '/UDP']);
                catch
                    pos = [90 42];
                end
                add_block('UAVdesign_net/Hardware/Serial Port', [gcb '/Serial Port'], 'Position', pos, 'num_packets_received', 'on', 'port', 'com_port', 'baud_rate', 'baud', 'timeout', 'timeout', 'receive', 'rx_size', 'st', 'sample_time');
                add_line(gcb, 'TX/1', 'Serial Port/1', 'autorouting','on');
                add_line(gcb, 'Serial Port/1', 'RX/1', 'autorouting','on');
                add_line(gcb, 'Serial Port/2', 'num_rx_packets/1', 'autorouting','on');
            end
        elseif strcmp(maskval{start+1}, 'UDP port')
            if ischar(varargin{2})
                p = varargin{2};
            else
                p = 'err';
            end
            try
                pL = num2str(varargin{3});
            catch
                pL = 'err';
            end
            try
                pR = num2str(varargin{4});
            catch
                pR = 'err';
            end
            maskdisp = ['text(0.1, 0.7, ''' p '''); text(0.1, 0.3, ''L:' pL '  R:' pR ''')'];
            if isempty(find_system(gcb, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'Name', 'UDP'))
                try
                    pos = get_param([gcb '/Serial Port'], 'Position');
                    delete_line(gcb, 'TX/1', 'Serial Port/1');
                    delete_line(gcb, 'Serial Port/1', 'RX/1');
                    delete_line(gcb, 'Serial Port/2', 'num_rx_packets/1');
                    delete_block([gcb '/Serial Port']);
                catch
                    pos = [90 42];
                end
                add_block('UAVdesign_net/Hardware/UDP', [gcb '/UDP'], 'Position', pos, 'num_packets_received', 'on', 'host', 'host_name', 'remote_port', 'remote_port', 'local_port', 'local_port', 'timeout', 'timeout', 'receive', 'rx_size', 'handshake', maskval{9}, 'st', 'sample_time');
                add_line(gcb, 'TX/1', 'UDP/1', 'autorouting','on');
                add_line(gcb, 'UDP/1', 'RX/1', 'autorouting','on');
                add_line(gcb, 'UDP/2', 'num_rx_packets/1', 'autorouting','on');
            end
            set_param([gcb '/UDP'], 'handshake', maskval{start+9});
        end
        set_param(gcb, 'MaskDisplay', maskdisp);
        if ~loaded
            close_system('UAVdesign_net');
        end
    end
end