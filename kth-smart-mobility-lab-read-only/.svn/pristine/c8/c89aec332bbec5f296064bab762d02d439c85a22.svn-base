function receivefile(port,ip)
% Bra fil som sparar filer

tcpipClient = tcpip(ip,str2double(port),'NetworkRole','Client');
set(tcpipClient,'InputBufferSize',1000000);
set(tcpipClient,'Timeout',15);

fprintf('Created Connection\n');

fopen(tcpipClient);

sizefilename = fread(tcpipClient,1,'int32');

name = char(fread(tcpipClient,sizefilename,'char')');

sizefile = fread(tcpipClient,1,'int32');
 if sizefile~=0
    file = char(fread(tcpipClient,sizefile,'char')');
    
    fid=fopen(name,'w' );
    
    fprintf(fid,file);

    fclose(fid);

 end
flushinput(tcpipClient);
fclose(tcpipClient);
