classdef radarWrapper < handle
    % This is a Matlab wrapper function which allows for Flat Earth Salsa Radar
    % modules to be easily accessed from within the matlab enviroment.
    %
    % This module can be called without parameters to connect to the radar
    % module over the USB interface with default username and password,
    % over an ethernet connection (recomended for better perfomance) by
    % also passing in an IP address.
    %
    % Efforts have been made to make this nearly compatible with Novelda's
    % .NET RadarLib library functions to ease porting of functions that
    % were developed using earlier development boards, but 1:1
    % functionality is not guaranteed.
    %
    % The socket port can be changed in the ip address variable using the :
    % delimiter.  For example, to connect over socket to 9000 on the usb 
    % interface, the command would be radarWrapper('192.168.7.2:9000').
    %
    % Useage:
    % obj = radarWrapper()
    % obj = radarWrapper(ipAddress)
    % obj = radarWrapper(ipAddress, updateConnector)
    % obj = radarWrapper(ipAddress, username, password)
    % obj = radarWrapper(ipAddress, username, password, updateConnector)
    %
    % Copyright Flat Earth Inc. 2015-2017
    
    properties
        isOpen=0;               % Is there a radar connection open at the moment
               
        % Unused but for compadibility
        objectCounter = 1;      % Number of radars objects, only support single at the moment
        
        %Global Radar Parameters
        numSamplers = 0;
        
        Actions = '';
        Registers = '';
    end
    
    properties(Hidden)
        %Network Options
        hostname = '192.168.7.2';
        port = 8192;
        username = 'root';
        password = '';
        t;
        x4DownConverter = 0;
        
        
        selectedChipType = 1;           % 1='Novelda X1/X2', 2='Novelda X4'
        
        %System options
        dirpath = fileparts(which('radarWrapper'));
       
        %Global Radar Parameters
%         numSamplers = 0;

        %Socket server options
        DEV_binaryDataTransfer = 1;     % Transfer radar frames as binary data instead of ASCII CSV data

        %Experimental Options (Alpha features)
        DEV_compressDataTransfer = 0;   % GZip the frame data over sockets (should be useful for x4 frames)
        DEV_v2_packet_type = 0;
        
        timeout = 5;
        
        %CDF vars
        CDF_Rows = 0;       %Number of rows in the returned CDF
        CDF_Cols = 0;       %Number of Cols in the returned CDF
        
        lastError = '';
        
        connectorVer = 0.00;                %Connector version on the BBB
        
        packagedConnectorVersion = 0.00;    %Connector version packaged in /server
        
        installerName = '';
        pwArg = '';
    end
    
    methods
        function obj = radarWrapper(ipAddress, username, password, updateConnector)          
            orig_state = warning;
            warning('off','all')
            
            %Redeclare the default setting
            updateConnector_i=0;
            obj.hostname = '192.168.7.2';
            obj.username = 'root';
            obj.password = '';
            
            %Get the last file in the server directory and get its version number field
            x = obj.dirpath
            y = [obj.dirpath,'/server']
            serverDir = dir([obj.dirpath,'/server']);
            try
                parts = textscan(serverDir(end).name,'%s','delimiter','_');
            catch
                %Special case for packaged apps in Windows only to resolve
                %serverdir getting a bad path.
                %
                %This is non-ideal, non portable, etc, but should work in the short term....
                obj.dirpath = fullfile(getenv('USERPROFILE'),'/Documents/MATLAB/Toolboxes/SalsaLab');
                serverDir = dir([obj.dirpath,'\server']);
                parts = textscan(serverDir(end).name,'%s','delimiter','_');
            end
            if strcmp('salsasockets', parts{1}(1))                          %Verify its the correct filename
                obj.packagedConnectorVersion = str2double(parts{1}(2));     %Get its version number
                obj.installerName = serverDir(end).name;
            end
            
            if (nargin>0)
                tempstr = strsplit(ipAddress,':');
                obj.hostname = tempstr{1};
                if length(tempstr) == 2
                    obj.port = round(str2double(tempstr{2}));
                end
            end
            if (nargin==2)
                updateConnector_i = username;
            end
            if (nargin>2)
                obj.username = username;
                obj.password = password;
                if isempty(password)==0
                    obj.pwArg = [' -pw ', password];
                end
            end
            if (nargin>3)
                updateConnector_i = updateConnector;
            end
                        
            obj.StartServer(updateConnector_i);
                                  
%             obj.NVA_PowerOnRadar();
            
            % Turn warnings back to original state
            warning(orig_state);
        end
        
        %% Delete the radar handle/close the program
        function delete(obj)
            %Deletes a radar handle
            %
            %See also Open, Delete
            
            write(obj.t,uint8('Exit()'))          %Send command
            obj.isOpen = 0;
        end               
                      
        %% Create a radar handle
        function status = NVA_CreateHandle(obj)
            %CreateHandle Creates a handle to a radar object prior to
            %opening the radar
            %
            %See also Open
            
            write(obj.t,uint8('InitHandle()'))          %Send command
            status = obj.getData();                     %Wait for an ACK
        end

        %% Open the connection to the radar
        function status = Open(obj,connectString) 
            %Open Opens a connection to a radar module such that it can be
            %accessed.
            %
            %See also ConnectedModules
            
            write(obj.t,uint8(['OpenRadar(' connectString ')']));   %Send command
            status = obj.getData();                                 %Wait for ACK            
            obj.isOpen = 1;                                         %Set open flag
            
            %Update the number of samplers
            obj.updateNumberOfSamplers();                           %Get the number of samplers in the default frame
            
            if (obj.selectedChipType == 1)
                obj.initializeRegisterMap();
                obj.initializeActionsMap();
            end;
        end;
        
        %% Close the program/radar handle
        function Close(obj)          
            % Close Closes an open radar handle object, but the connection
            % to the BeagleBone platform is maintained
            %
            % See also Open, Delete
            
            write(obj.t,uint8('Close()'))          %Send command
            obj.getData();                         %Wait for ACK            
            obj.isOpen = 0;                        %Set open flag
        end

        %% Get the temp from the temp sensor on the module
        function status = GetTemperature(obj)
            % GetTemperature Get the temperature from a sensor next to the radar chip in
            % degC
            %
            %Example
            %   radar = radarWrapper;
            %   modules = radar.ConnectedModules;
            %   radar.Open(modules{1});
            %   temperature = radar.getTemperature;
            %   disp([temperature,' degC']);
            %   radar.Close;  
            
            write(obj.t,uint8('GetTemperature()'))      %Send command
            a = obj.getData();
            
            status = str2double(a);
        end;

        %% Get the accel from the IMU sensor on the module
        function status = GetAccel(obj)
            % todo - Update
            % GetTemperature Get the temperature from a sensor next to the radar chip in
            % degC
            %
            %Example
            %   radar = radarWrapper;
            %   modules = radar.ConnectedModules;
            %   radar.Open(modules{1});
            %   temperature = radar.getTemperature;
            %   disp([temperature,' degC']);
            %   radar.Close;  
            
            write(obj.t,uint8('GetAccel()'))      %Send command
            a = obj.getData();
            
            status = str2num(a);
        end;

        %% Get the accel from the IMU sensor on the module
        function status = GetGyro(obj)
            % todo - Update
            % GetTemperature Get the temperature from a sensor next to the radar chip in
            % degC
            %
            %Example
            %   radar = radarWrapper;
            %   modules = radar.ConnectedModules;
            %   radar.Open(modules{1});
            %   temperature = radar.getTemperature;
            %   disp([temperature,' degC']);
            %   radar.Close;  
            
            write(obj.t,uint8('GetGyro()'))      %Send command
            a = obj.getData();
            
            status = str2num(a);
        end;        

        %% Get the accel from the IMU sensor on the module
        function status = GetMag(obj)
            % todo - Update
            % GetTemperature Get the temperature from a sensor next to the radar chip in
            % degC
            %
            %Example
            %   radar = radarWrapper;
            %   modules = radar.ConnectedModules;
            %   radar.Open(modules{1});
            %   temperature = radar.getTemperature;
            %   disp([temperature,' degC']);
            %   radar.Close;  
            
            write(obj.t,uint8('GetMag()'))      %Send command
            a = obj.getData();
            
            status = str2num(a);
            
        end;    
        
        %% Get the accel from the IMU sensor on the module
        function [] = SetMagFullScale(obj, scaleOption)
            % todo - Update
            % GetTemperature Get the temperature from a sensor next to the radar chip in
            % degC
            %
            %Example
            %   radar = radarWrapper;
            %   modules = radar.ConnectedModules;
            %   radar.Open(modules{1});
            %   temperature = radar.getTemperature;
            %   disp([temperature,' degC']);
            %   radar.Close;  
            
            write(obj.t,uint8(['SetMagRange(',num2str(scaleOption),')']));      %Send command
            obj.getData();
            
        end;    

        %% Get the accel from the IMU sensor on the module
        function [] = SetAccelFullScale(obj, scaleOption)
            % todo - Update
            % GetTemperature Get the temperature from a sensor next to the radar chip in
            % degC
            %
            %Example
            %   radar = radarWrapper;
            %   modules = radar.ConnectedModules;
            %   radar.Open(modules{1});
            %   temperature = radar.getTemperature;
            %   disp([temperature,' degC']);
            %   radar.Close;  
            
            write(obj.t,uint8(['SetAccelRange(',num2str(scaleOption),')']));      %Send command
            obj.getData();
            
        end;    
        
        
        %% Get the accel from the IMU sensor on the module
        function [] = SetGyroFullScale(obj, scaleOption)
            % todo - Update
            % GetTemperature Get the temperature from a sensor next to the radar chip in
            % degC
            %
            %Example
            %   radar = radarWrapper;
            %   modules = radar.ConnectedModules;
            %   radar.Open(modules{1});
            %   temperature = radar.getTemperature;
            %   disp([temperature,' degC']);
            %   radar.Close;  
            
            write(obj.t,uint8(['SetAccelRange(',num2str(scaleOption),')']));      %Send command
            obj.getData();
            
        end;    
        
        %% Set the TX voltage on the Ancho to pull the center frequency
        function status = SetVoltage(obj, voltage)
            % SetVoltage Set the voltage of into the transmit pin on the
            % ANCHO cape.  This will pull the voltage to allow for a wider
            % range of center frequencies to be chosen then the chip
            % natively supports. 
            %
            %This variable is limited to the range .9-1.28V to prevent damage to the chip.
            
            write(obj.t,uint8(['SetVoltage(',num2str(voltage),')']))          %Send command
            status = obj.getData();
        end
        
        %% Get the TX voltage on the Ancho to pull the center frequency
        function voltage = GetVoltage(obj)
            % SetVoltage Set the voltage of into the transmit pin on the
            % ANCHO cape.  This will pull the voltage to allow for a wider
            % range of center frequencies to be chosen then the chip
            % natively supports. 
            %
            %This variable is limited to the range .9-1.28V to prevent damage to the chip.
            
            write(obj.t,uint8('GetVoltage()'))          %Send command
            voltage = str2double(obj.getData());
        end
        
        %% Select a cape based on the ID
        function status = SelectCape(obj, capeId)
            % SelectCape This function will select the ANCHO cape based on
            % the CapeID DIP switches on the board when there are multiple
            % capes on the same BeagleBone Black.
            
            write(obj.t,uint8(['SelectCape(',num2str(capeId),')']))          %Send command
            status = obj.getData();                                          %Wait for an ACK           
        end
        
        %% Run an action on the BBB (For RadarLib3.NET Compadibilty)
        function status = ExcecuteAction(obj, actionName)
            %ExcecuteAction Request a RadarLib "action" be run on the radar
            %chip.  This function is a passthrough function to
            %ExecuteAction for backwards compatibility.
            %
            %See also ExecuteAction
            
            status = obj.ExecuteAction(actionName);     %Send command
        end
        
        %% Run an action on the BBB
        function status = ExecuteAction(obj, actionName)
            %ExecuteAction Request a RadarLib "action" be run on the radar
            %chip.
            %
            %Example
            %   radar = radarWrapper;
            %   modules = radar.ConnectedModules;
            %   radar.Open(modules{1});
            %   radar.ExecuteAction('MeasureAll')
            %   radar.Close;  
            %
            
            write(obj.t,uint8(['actionExecute(' actionName ')']))          %Send command
            status = obj.getData();                                        %Wait for an ACK         
        end
        
        %% Update a register on the BBB
        function status = TryUpdateChip(obj, registerName, value)
            % TryUpdateChip Updates a register value on the chip
            % based on the register name.
            %
            %Example
            %   radar = radarWrapper;
            %   modules = radar.ConnectedModules;
            %   radar.Open(modules{1});
            %   radar.TryUpdateChip('Iterations', 10);
            %   radar.Close;            
            %
            % See also Item
            
            write(obj.t,uint8(['VarSetValue_ByName(' registerName ',' num2str(value) ')']))          %Send command
            status = obj.getData();
            
            %Update the number of samplers in case it changed
            obj.updateNumberOfSamplers();
        end
        
        %% Update a register on the BBB
        function status = EnableX4HWDDC(obj,value)
            status = obj.TryUpdateChip('DownConvert', value);
            obj.x4DownConverter = value;
        end
        
        %% Get a register value from the BBB
        function register = Item(obj, registerName)
            % Item Get a register value from the chip from its name
            % 
            %Example
            %   radar = radarWrapper;
            %   modules = radar.ConnectedModules;
            %   radar.Open(modules{1});
            %   iterations = radar.Item('Iterations');
            %   radar.Close;
            %
            % See also RegisterRead, TryUpdateChip
            
            write(obj.t,uint8(['VarGetValue_ByName(' registerName ')']))     %Send command
            a = obj.getData();                                               %Wait for ACK
            if isnan(str2double(a))
                register = a;
            else
                register = str2num(a); %#ok                                      %Parse the returned data
            end
        end
        
        %% Get a radar frame from the BBB
        function frame = GetFrameRawDouble(obj)
            
            frame = double(GetFrameRaw(obj));
        
            if obj.x4DownConverter == 1
                frame = frame(1:2:end)+ 1i*frame(2:2:end);
            end
        
        end
        
        %% Get a radar frame from the BBB
        function frame = GetFrameRaw(obj)
            %GetFrameRaw Get a complete radar frame in raw count values.
            %These counts will change based on the radar settings
            %
            %Example
            %   radar = radarWrapper;
            %   modules = radar.ConnectedModules;
            %   radar.Open(modules{1});
            %   frame = radar.GetFrameRaw;  
            %   radar.Close;
            %
            % See also GetFrameRawDouble GetNFramesRaw
            
            if(~obj.DEV_binaryDataTransfer)
               % Get the data as a ASCI CSV transfer
                write(obj.t,uint8('GetFrameRaw()'))          %Send command
                frame = obj.getData();
                frame = str2num(char(frame));   %#ok
            elseif (obj.DEV_compressDataTransfer == 1)
                % Compressed binary data tranfer
                write(obj.t,uint8('GetFrameRaw()'))                 %Send command

                while (obj.t.BytesAvailable<4)
                end
                frameSize = typecast(read(obj.t,4), 'uint32');
                
                while (obj.t.BytesAvailable<frameSize)
                end
                frame = read(obj.t,frameSize);
                
                while (obj.t.BytesAvailable<5)
                end
                read(obj.t,5);
                
                frame = dunzip(frame)';

            else
                %Binary data tranfer (faster)
                write(obj.t,uint8('GetFrameRaw()'))                 %Send command
                if obj.DEV_v2_packet_type == 1
%                     frame = obj.getFrameData();
                    while (obj.t.BytesAvailable<4)
                    end
%                     packetlength = read(obj.t, 1,'int32');
                    packetlength = typecast(read(obj.t, 4),'int32');
                    frame = read(obj.t, packetlength);
                    obj.parseErrReturn(frame);
                    frame=frame(1:end-5);
                    
                    
                    frame = typecast(frame,'int32');
                    return
                else                
                    frame = [];                                         %Temp to hold value
                    ind = 1;                                            %Iteration value for length of string
                    frameSize = obj.numSamplers*4+5;                    %Calculate the expected size of the frame in bytes
                    while(1==1)                                         %Loop until ack received
                        %Collect the radar frame 
    %                     frame = read(obj.t,frameSize);
    %                     frame = frame(1:(end-5));
    %                     break;    

                        if (obj.t.BytesAvailable==frameSize)            %If char in buffer
                             frame = read(obj.t,frameSize);             %read it and put it on "stack"
                             if (obj.parseIsDone(char(frame)))          %if ack is received
                                 frame = frame(1:(end-5));
                                 break;                                 %break the loop and continue program
                             end;
                        end;
                        %Timeout
                        if (ind == 50000)
                            disp('Timeout');
                            frame = read(obj.t,obj.t.BytesAvailable);   %read it and put it on "stack"
                            error(char(frame(6:end-5)));
                            frame = zeros(1,obj.numSamplers);
                        end
                        ind = ind+1;                                    %else, get ready for the next char
                    end;
                    frame = typecast(frame,'int32');
                end
            end
        end
        
        %% Get a a burst collection of radar frames from the BBB
        function frames = GetNFramesRawDouble(obj,numberOfFrames)
            % GetNFramesRawDouble Collect and get a burst collection
            % of frames from the BBB.  This reduces overhead and can allow
            % for a small time window to be covered with significanly
            % higher framerates.
            %
            %Example
            %   radar = radarWrapper;
            %   modules = radar.ConnectedModules;
            %   radar.Open(modules{1});
            %   frame = radar.GetNFramesRawDouble(64);  
            %   radar.Close;
            %
            % Note: This currently only works when DEV_compressDataTransfer
            % is disabled.
            %
            % See also GetNFramesRaw GetFrameRawDouble GetFrameNormalizedDouble
            
            frames = double(GetNFramesRaw(obj,numberOfFrames));
            
            if obj.x4DownConverter == 1
                frames = frames(1:2:end,:)+ 1i*frames(2:2:end,:);
            end
            
        end
        
        %% Get N radar frame from the BBB
        function frame = GetNFramesRaw(obj, numberOfFrames)
            %GetNFramesRaw Get N complete radar frames in raw count values.
            %
            %Example
            %   radar = radarWrapper;
            %   modules = radar.ConnectedModules;
            %   radar.Open(modules{1});
            %   frame = radar.GetNFramesRaw(64);  
            %   radar.Close;
            %
            % See also GetNFramesRawDouble GetFrameRaw GetFrameNormalized
            
            if(~obj.DEV_binaryDataTransfer)
               % Get the data as a ASCI CSV transfer
                write(obj.t,uint8('GetFrameRaw()'))          %Send command
                frame = obj.getData();
                frame = str2num(char(frame));   %#ok
            else
%                 obj.numSamplers = 1536/4;
                %Get the data as a binary much faster
                write(obj.t,uint8(['GetNFramesRaw(',num2str(numberOfFrames),')']))                 %Send command
                ind = 1;                                            %Iteration value for length of string
                inBuffer = 0;
                frame = zeros(1,numberOfFrames*obj.numSamplers*4+5,'uint8');
                while(1==1)                                         %Loop until ack recieved
                    %Collect the radar frame                    
                    if (obj.t.BytesAvailable>4000)
                        frame(inBuffer+1:inBuffer+4000) = read(obj.t,4000);
                        inBuffer = inBuffer+4000;
                    end
                    
                    if (obj.t.BytesAvailable==numberOfFrames*obj.numSamplers*4+5-inBuffer) %If char in buffer
                          frame(inBuffer+1:end) = read(obj.t,obj.t.BytesAvailable);                         
                          if (obj.parseIsDone(char(frame)))          %if ack is recieved
                             frame = frame(1:(end-5));
                             break;                                 %break the loop and continue program
                         end;
                    end;
                    %Timeout
                    if (ind == 200000)
                        frame = read(obj.t,obj.t.BytesAvailable);   %read it and put it on "stack"
                        error(char(frame(6:end-5)));
                        frame = zeros(1,obj.numSamplers);
                    end
                    ind = ind+1;                               %else, get ready for the next char
                end;
                frame = typecast(frame,'int32');
                frame = reshape(frame,[obj.numSamplers,numberOfFrames]);
            end
        end
       
        
        %% Get a radar frame from the BBB
        function frame = GetFrameNormalizedDouble(obj)
            % GetFrameNormalizedDouble Get a radar frame that has been
            % normalized such that the bins will not change based on the
            % radar parameters.
            %
            %Example
            %   radar = radarWrapper;
            %   modules = radar.ConnectedModules;
            %   radar.Open(modules{1});
            %   frame = radar.GetFrameNormalizedDouble;  
            %   radar.Close;
            %
            % See also GetFrameNormalized GetFrameRawDouble GetNFramesRawDouble
            
          
            if (~obj.DEV_binaryDataTransfer)
                write(obj.t,uint8('GetFrameNormalized()'));         %Send command
                frame = obj.getData();
                frame = str2num(char(frame));   %#ok
            elseif (obj.DEV_compressDataTransfer == 1)
                % Compressed binary data tranfer
                write(obj.t,uint8('GetFrameNormalized()'))          %Send command

                while (obj.t.BytesAvailable<4)
                end
                frameSize = typecast(read(obj.t,4), 'single');
                
                while (obj.t.BytesAvailable<frameSize)
                end
                frame = read(obj.t,frameSize);
                
                while (obj.t.BytesAvailable<5)
                end
                read(obj.t,5);
                keyboard
                frame = dunzip(frame)';
            else
                %Get the data as a binary (should be faster)
                
                %Request a frame
                write(obj.t,uint8('GetFrameNormalized()'));         %Send command
                frame = [];                                         %Temp to hold value
                ind = 1;                                            %Iteration value for length of string
                
                %X1/X2 send doubles, X4 sends floats
                 if (obj.selectedChipType == 1)
                    sizeofType = 8;
                 else
                     sizeofType = 4;
                 end
                
                while(1==1)                                         %Loop until ack recieved
                    %Collect the radar frame 
                    if (obj.t.BytesAvailable == obj.numSamplers*sizeofType+5) %If char in buffer
                         frame = read(obj.t, obj.numSamplers*sizeofType+5);   %read it and put it on "stack"
                         if (obj.parseIsDone(char(frame)))          %if ack is recieved
                             frame = frame(1:(end-5));
                             break;                                 %break the loop and continue program
                         end;
                    end;
                    %Timeout
                    if (ind == 50000)
                        disp('Timeout');
                        frame = read(obj.t,obj.t.BytesAvailable);   %read it and put it on "stack"
                        error(char(frame(6:end-5)));
                        frame = zeros(1,obj.numSamplers);
                    end
                    ind = ind+1;                               %else, get ready for the next char
                end;
                
                if (sizeofType == 8)
                    frame = typecast(frame,'double');
                else
                    frame = typecast(frame,'single');
                end
            end

            if obj.x4DownConverter == 1
                frame = frame(1:2:end)+ 1i*frame(2:2:end);
            end

        end
        
        %% Get a list of the connected radar modules
        function list = ConnectedModules(obj)
            % ConnectedModules List the radar modules connection strings
            % that are currently avalbile and connected to the BeagleBone
            %
            %
            %Example
            %   radar = radarWrapper;
            %   modules = radar.ConnectedModules;   % Get a list of modules
            %   radar.Open(modules{1});
            %   %
            %   % Do Something
            %   %
            %   radar.Close;
            %
            % See also Open
            
            write(obj.t,uint8('ListRadarModules()'))          %Send command
            list = obj.getData();
            list = strsplit(list,',');
        end
        
        %% Get a list of the variables on the radar
        function list = ListVariables(obj)
            %ListVariables Get a list of all the variables supported on the
            %radar chip as a matrix of strings
            %
            %See also TryUpdateChip, Item
            
            write(obj.t,uint8('ListVariables()'));
            list = obj.getData();
            list = strsplit(list,',');
        end
        
        %%
        function list=getEnumItems(obj,registerName)
            %getEnumItems Get a list of all the enumerations for an
            %enumeration register.
            %
            %See also TryUpdateChip, Item
            
            write(obj.t,uint8(['getEnumItems(',registerName,')']));
            list = obj.getData();
            list = strsplit(list,',');
        end
        
        %% Get a list of the variables on the radar
        function register = RegisterRead(obj, address, length)
            %RegisterRead Read a register based on its address (decimal)
            %and length in bytes rather then its name
            %
            %Example
            %   radar = radarWrapper;
            %   modules = radar.ConnectedModules;
            %   radar.Open(modules{1});
            %   zeros = radar.RegisterRead(0,4);    % 32 bits of the "zeros" register
            %   ones = radar.RegisterRead(1,4);     % 32 bits of the "ones" register
            %   radar.Close;
            %
            %See also Item
            
            write(obj.t,uint8(['RegisterRead(', num2str(address),',',num2str(length),')']))          %Send command
            a = obj.getData();
            register = str2double(a(1:end-5));
        end
        
        %% Get the offset distance of the radar
        function offset = OffsetDistance(obj)
            % OffsetDistance Get the set OffsetDistanceFromReference
            % parameter from the radar
            
            offset = double(obj.Item('OffsetDistanceFromReference'));
        end
        
        %% Get the actual sampler resolution
        function resolution = SamplerResolution(obj)
            %SamplerResolution Get the measured sampler resolution on the
            %radar chip.  This is dependent both on settings and
            %temperature.
            
            c = 2.99e8;
            resolution = 1/double(obj.Item('SamplesPerSecond'))*c/2;
        end
        
        %% Get a list of all the actions from radarlib and the chip
        function actions = ListActions(obj)
            %ListVariables Get a list of all the actions supported on the
            %radar chip as a matrix of strings
            %
            %See also ExecuteAction()
            
            write(obj.t,uint8('ListActions()'));        %Send command
            list = obj.getData();
            actions = strsplit(list,',');
        end
        
        %% Number of samplers with the current radar settings
        function samplers = SamplersPerFrame(obj)
            %SamplersPerFrame Get the number of samplers for the radar with
            %its current settings
            
            samplers = double(obj.Item('SamplersPerFrame'));
        end
        
        %% Get the message from the last error that occured
        function err = LibraryError(obj)
            %%LibraryError Get the message from the last error that occured
            
            err = obj.lastError;
        end
    
    
        %% Set all chip registers to their default values
        function []=ResetAllVariablesToDefault(obj)
            write(obj.t,uint8('VarsResetAllToDefault()'))          %Send command
            obj.getData();
            obj.updateNumberOfSamplers();
        end
    
        %% get the cdf from the radar chip
        function cdf = getCDF(obj)
            %%getCDF get the cdf from the radar chip
            
            %Get the dimensions of the CDF
            write(obj.t,uint8('NVA_CalculateCDF_Size()'))          %Send command
            cdfdata_internal = str2num(obj.getData());  %#ok
            obj.CDF_Cols = cdfdata_internal(1);
            obj.CDF_Rows = cdfdata_internal(2);
            
            %Get the CDF
            write(obj.t,uint8(['NVA_GetCDF(',num2str(obj.CDF_Cols*obj.CDF_Rows),')']));          %Send command
%             cdf = obj.getData(); %obj.getFrameData(4*obj.CDF_Cols*obj.CDF_Rows+5);
            cdf = obj.getFrameData(4*obj.CDF_Cols*obj.CDF_Rows+5);
            cdf = typecast(uint8(cdf),'uint32');
            cdf = reshape(cdf,obj.CDF_Cols,obj.CDF_Rows);
        end
              
%         function enableFrameBuffer()
%         end
%         
%         function setFrameBufferRate()
%             
%         end
%         
%         function getBufferedFrame()
%             
%         end
        
        function getBufferStatus(obj)
            write(obj.t,uint8('getBufferStatus()'))          %Send command
            obj.getData()
        end
        
        function enableGovernor(obj)
            write(obj.t,uint8('enableGovernor()'))          %Send command
            obj.getData()
        end
        
        %% Function to set pin direction
        function status = SetIOPinDirection(obj,bank,pin,direction)      
            %SetIOPinDirection Set the direction of a pin on the BBB
            %This must be done prior to using a custom pin to set it up
            %correctly.
            %
            %Bank is the connector to use (8 or 9)
            %Pin is the pin on that bank, 1 indexed
            %Direction 'input' or 'output'.  0 and 1 are also valid directions where 1 is output
            %
            % 
            %Example
            %   radar = radarWrapper;
            %   modules = radar.ConnectedModules;
            %   radar.Open(modules{1});
            %   radar.SetIOPinDirection(8,45,'output');
            %   radar.SetIOPinDirection(8,46,'input');
            %   radar.WriteIOPin(8,45,1);
            %   radar.ReadIOPin(8,46);
            %   radar.Close;
            %
            %See also WriteIOPin, ReadIOPin
            
            if (ischar(direction)==1)
                if strcmp(direction,'output')
                    direction = 1;
                end
                if strcmp(direction,'input')
                    direction = 0;
                end
                if strcmp(direction,'0')
                    direction = 0;
                end
                if strcmp(direction,'1')
                    direction = 1;
                end
            end
            
            dirVal = direction;
            if (dirVal == 0) || (dirVal == 1)
                write(obj.t,uint8(['SetIOPinDirection(' num2str(bank) ',' num2str(pin) ',' num2str(dirVal) ')']));
                status = str2double(obj.getData());
            end
        end
        
        
        %% Function to set pin value
        function status = WriteIOPin(obj,bank,pin,value)
            %WriteIOPin Set the value of a pin on the BBB
            %
            %SetIOPinDirection must be run prior to using a custom pin.
            %
            %Bank is the connector to use (8 or 9)
            %Pin is the pin on that bank, 1 indexed
            %Value 0 or 1.  The inputs 'low' and 'high' are also valid
            %
            % 
            %Example
            %   radar = radarWrapper;
            %   modules = radar.ConnectedModules;
            %   radar.Open(modules{1});
            %   radar.SetIOPinDirection(8,45,'output');
            %   radar.SetIOPinDirection(8,46,'input');
            %   radar.WriteIOPin(8,45,1);
            %   radar.ReadIOPin(8,46);
            %   radar.WriteIOPin(8,45,'low');
            %   radar.ReadIOPin(8,46);
            %   radar.Close;
            %
            %See also SetIOPinDirection, WriteIOPin
            
            if (ischar(value)==1)
                if strcmp(value,'high')
                    value = 1;
                end
                if strcmp(value,'low')
                    value = 0;
                end
                if strcmp(value,'0')
                    value = 0;
                end
                if strcmp(value,'1')
                    value = 1;
                end
            end
            valueVal = value;
            if (valueVal == 0) || (valueVal == 1)
                write(obj.t,uint8(['WriteIOPin(' num2str(bank) ',' num2str(pin) ',' num2str(valueVal) ')']));
                status = str2double(obj.getData());
            end
        end
        
        %% Function to read pin value
        function Value = ReadIOPin(obj,bank,pin)
            %ReadIOPin Read the current value of being received on the pin
            %
            %SetIOPinDirection must be run prior to using a custom pin.
            %
            %Bank is the connector to use (8 or 9)
            %Pin is the pin on that bank, 1 indexed
            %
            %Value returns  0 or 1 with the digitial value of the pin
            % 
            %Example
            %   radar = radarWrapper;
            %   modules = radar.ConnectedModules;
            %   radar.Open(modules{1});
            %   radar.SetIOPinDirection(8,45,'output');
            %   radar.SetIOPinDirection(8,46,'input');
            %   radar.WriteIOPin(8,45,1);
            %   radar.ReadIOPin(8,46);
            %   radar.WriteIOPin(8,45,'low');
            %   radar.ReadIOPin(8,46);
            %   radar.Close;
            %
            %See also SetIOPinDirection, WriteIOPin
            
            write(obj.t,uint8(['ReadIOPin(' num2str(bank) ',' num2str(pin) ')']));
            Value = str2double(obj.getData());
        end
        
        %% Turn of the beaglebone unit
        function []=ShutdownBeaglebone(obj)
            %ShutdownBeaglebone Turn of the beaglebone
            obj.pwArg = [];
            if isempty(obj.password)==0
                obj.pwArg = [' -pw ', obj.password];
            end
            systemType = computer;
            if (strcmp(systemType,'PCWIN64') || strcmp(systemType,'PCWIN'))
                system([obj.dirpath,'\putty\plink ', obj.hostname, ' -l ', obj.username, obj.pwArg, ' poweroff']);
            else
                system([obj.dirpath,'ssh ', obj.hostname, ' -l ', obj.username, obj.pwArg, ' poweroff']);
            end
        end
        
        %% Restart the beaglebone unit
        function []=RestartBeaglebone(obj)
            %RestartBeaglebone Restart beaglebone
            %
            %Note, the connection will not be maintained and a new
            %radarwrapper handle will need to be created to continue
            %communicating with the radar module
            
            obj.pwArg = [];
            if isempty(obj.password)==0
                obj.pwArg = [' -pw ', obj.password];
            end
            systemType = computer;
            if (strcmp(systemType,'PCWIN64') || strcmp(systemType,'PCWIN'))
                system([obj.dirpath,'\putty\plink ', obj.hostname, ' -l ', obj.username, obj.pwArg, ' reboot']);
            else
                system([obj.dirpath,'ssh ', obj.hostname, ' -l ', obj.username, obj.pwArg, ' reboot']);
            end
        end
        
        %% PowerOff the radar chip itself on the module
        function [] = NVA_PowerOffRadar (obj)
            %DisableRadarChip Disable the radar output
            %
            %See also NVA_PowerOnRadar
            
            WriteIOPin(obj,8,26,0)
        end
     
        %% Power on the radar chip itself on the module
        function [] = NVA_PowerOnRadar (obj)
            %DisableRadarChip Disable the radar output
            %
            %Note: The radar is initially powered on
            %
            %See also NVA_PowerOffRadar
            
            WriteIOPin(obj,8,26,1)
        end
   
        %% Power on the radar chip itself on the module
        function [] = Cilantro_Init(obj)
            obj.SetIOPinDirection(9,25,'output');
            obj.SetIOPinDirection(9,27,'output');
        end
        
        %% Power on the radar chip itself on the module
        function [] = Cilantro_SelectOutputChannel(obj, channel)
            % THIS FUNCTION IS BETA AND SUBJECT TO CHANGE....
%             obj.SetIOPinDirection(9,25,'output');
%             obj.SetIOPinDirection(9,27,'output');
            
%             obj.WriteIOPin(9,25,1);
            
            if (channel == 1)
                obj.WriteIOPin(9,27,1);
            elseif (channel == 2)
                obj.WriteIOPin(9,27,0);
            else
                warning('Invalid Channel Number');
            end
        end        
        
        %% Set what type of chip is on the board to connect to
        function [] = SetChipType(obj, ChipType)
            % THIS FUNCTION IS BETA AND SUBJECT TO CHANGE....
            
            if strcmp(ChipType, 'X1')
                obj.selectedChipType = 1;
            elseif strcmp(ChipType, 'X2')
                obj.selectedChipType = 1;
            elseif strcmp(ChipType, 'X4')
                obj.selectedChipType = 2;
            end
                
            obj.StartServer(0);
%             obj.startConnector(sshPath)
        end
        
        
        
        %% Read information about a variable
        function VarInfo = GetVariableInfo(obj, name)
            % GetVariableInfo Read information about a variable such as its
            % minimum value, maximum value, if its writable, and if it has
            % been set to default or as a side effect of another variable
            %
            % This returns a structure with the fields:
            % .name - Name of the variable
            % .minimum - Min value the variable can be set to
            % .maximum - Max value the variable can be set to
            % .writable - 0 for Read Only, 1 for Read/Write
            % .isAuto - Is it currently set to a default or as a side-effect
            
            write(obj.t,uint8(['GetVariableInfo(',name,')']))          %Send command
            unparsedData = obj.getData();
            
            %Parse the return into a structure to return
            try
                VarInfo.name = unparsedData{1};
                VarInfo.minimum = unparsedData{2};
                VarInfo.maximum = unparsedData{3};
                VarInfo.writable = unparsedData{4};
                VarInfo.isAuto = unparsedData{5};
            catch
                VarInfo.name = name;
                VarInfo.minimum = NaN;
                VarInfo.maximum = NaN;
                VarInfo.writable = NaN;
                VarInfo.isAuto = NaN;
                error('Unable to read variable information');
            end
        end
        
        
        
    end
        
    methods(Hidden)
        %% Function to get a full frame of data from the radar and check it for errors
        function a = getData(obj)
            if obj.DEV_v2_packet_type == 0
                ind = 1;                                    %Iteration value for length of string
                a = [];
                while(1==1)                                 %Loop until ack recieved
                     bytes = obj.t.BytesAvailable;
                     if (bytes>0)                           %If char in buffer
                         temp = char(read(obj.t,bytes));    %read it and put it on "stack"
                         a = [a, temp]; %#ok                %read it and put it on "stack"                     
                         if (obj.parseIsDone(a))
                             a=a(1:end-5);                  %Pull the ack out of the signal
                             obj.parseErrReturn(a);
                             break;                         %break the loop and continue program
                         end
                         ind = ind+1;                       %else, get ready for the next char
                     end;
                end;     %Wait for ack
            else
                packetlength = typecast(read(obj.t, 4),'int32');
                a = char(read(obj.t, packetlength));
                obj.parseErrReturn(a);
                a=a(1:end-5);
            end
        end
        
        %% Function to get a full frame of data from the radar and check it for errors
        function a = getFrameData(obj,size)
            if obj.DEV_v2_packet_type == 0
                ind = 1;                                    %Iteration value for length of string
                a = [];
                tic
                while(1==1)                                 %Loop until ack recieved
                     bytes = obj.t.BytesAvailable;
                     if ((bytes==size) || (toc>1))          %If char in buffer
                         temp = char(read(obj.t,bytes));    %read it and put it on "stack"
                         a = [a, temp]; %#ok                %read it and put it on "stack"                     
                         if (obj.parseIsDone(a))
                             a=a(1:end-5);                  %Pull the ack out of the signal
                             obj.parseErrReturn(a);
                             break;                         %break the loop and continue program
                         end
                         ind = ind+1;                       %else, get ready for the next char
                     end;
                end;     %Wait for ack
            else
                while (obj.t.BytesAvailable<4)
                end
                packetlength = typecast(read(obj.t, 4),'int32');
                a = read(obj.t, packetlength);
                obj.parseErrReturn(a);
                a=a(1:end-5);
            end
        end
        
        %% Test the return to test and pull out error messages
        function parseErrReturn(obj,returnString)
            if (length(returnString)>6)
                if(strcmp(returnString(1:5),'<ERR>'))
                    obj.lastError = returnString(6:(end-5));
                    error(returnString(6:(end)));
                end
                if(strcmp(returnString(1:5),'<WRN>'))
                    obj.lastError = returnString(6:(end-5));
                    warning(returnString(6:(end)));
                end
            end
        end
        
        %% Test to see if the ACK string has been received
        function [isDone]=parseIsDone(~, returnString)
            isDone = 0;
            if (length(returnString)>4)
                if(strcmp(char(returnString((end-4):end)),'<ACK>'))
                    isDone=1;
                end
            end
        end
        
        %% Update "Number of Samplers" interval var
        function []=updateNumberOfSamplers(obj)
            obj.numSamplers = obj.Item('SamplersPerFrame');
        end
        
        % --------------------------------------------------
        %%Future API's that will eventually be made public
        %---------------------------------------------------
        
%         %% Set the registerName variable to its default value
%          function []=SetDefaultValue(obj,registerName)    
%             TryUpdateChip(registerName, value, 1)
%              
%             write(obj.t,uint8(['VarSetValue_ByName(' registerName ',' num2str(value) ')']))          %Send command
%             obj.getData();
%             
%             %Update the number of samplers in case it changed
%             obj.updateNumberOfSamplers();
%          end
%         


        
        %% Get information about the cape and BBB
%         function []=ReadBoardInfo(obj)
%         end
            
        %% Get detailed timing information from the radar
%         function timingData = TimingMeasurementData(obj)
        

        
        %% function [] = SamplerFirst(obj)
        
        %% Create a container.Map containing all the possible registers on the chip
        function [] = initializeRegisterMap(obj)
            varList = obj.ListVariables();
            obj.Registers = '';
            for i = 1:length(varList)
                obj.Registers.(varList{i}) = varList{i};
            end
        end

        %% Create a continer.Map containing all the possible actions on the chip
        function [] = initializeActionsMap(obj)
            actList = obj.ListActions();
            obj.Actions = '';
            for i = 1:length(actList)
                obj.Actions.(actList{i}) = actList{i};
            end
        end
        
        %% Function to write the default turn on TX voltage on the Ancho
        function status = writeDefaultVoltage(obj,passcode,voltage)
            % writeDefaultVoltage Set a tx voltage that the device will
            % turn on to every time the device is started.  The device will
            % only do this 50 times.
            %
            %This variable is limited to the range .9-1.28V to prevent damage to the chip.
            
            write(obj.t,uint8(['Write50TPVoltage(',passcode,',',num2str(voltage),')']))
            status = obj.getData();
        end 
        
        %% Function to read the NonVolititle voltage setting address on the Ancho
        function address = Read50TPAddress(obj)
            write(obj.t,uint8('Read50TPAddress()'))
            address = obj.getData();
        end 
        
        %% Function to read the NonVolititle voltage setting on the Ancho
        function voltage = Read50TPVoltage(obj,address)
            
            write(obj.t,uint8(['Read50TPVoltage(',num2str(address),')']))
            voltage = str2double(obj.getData());
        end 
        
        %% SSH into the beaglebone and start the connector software
        function startConnector(obj,sshPath)
            disp('Starting Connector Program on the BBB:')
            disp sshPath
            system([sshPath,' /root/FlatEarth/MatlabConnector2/startConnector ', num2str(obj.port),' ' num2str(obj.selectedChipType)]);
        end
        
        %% SCP into the BBB and update the connector software on the unit
        function updateBBBWrapper(obj,sshPath, scpPath, installerName)
            %Display a message
            disp('Updating Matlab Connector')
            %Make sure the sever is on the BBB and give it run permissions
            system([sshPath, ' mkdir -p /root/Installers']);
            system(scpPath);
            system([sshPath, ' dpkg -i /root/Installers/', installerName]);
            
            %Update the variable with the new version number
            obj.getBBBWrapperVersion(sshPath);
        end
        
        %% Poll the BBB for its currently installed connector version
        function getBBBWrapperVersion(obj,sshPath)
            [~,sshReturn] = system([sshPath, ' dpkg -s salsasockets ']);
            sshReturn = textscan(sshReturn,'%s','delimiter','\r\n');
            try
                %Find the line with the version number
                for loopiter = 1:size(sshReturn{1},1)
                    if (findstr('Version:',char(sshReturn{1}(loopiter))) ~= 0 )
                        break;
                    end
                end
                %extract the version number
                versionField = sshReturn{1}(loopiter);
                obj.connectorVer = str2double(versionField{1}(10:end));
            catch
                obj.connectorVer = 0.00;
            end
        end
        
        %% Start an instance of the connector
        function StartServer(obj, updateConnector_i)
            % Load the path to get to the ssh and scp programs
            systemType = computer;
            if (strcmp(systemType,'PCWIN64') || strcmp(systemType,'PCWIN'))
                
                if exist([obj.dirpath,'\putty\plink.exe'],'file') ~= 2
                    disp('Cannot Find Plink')
                end
                
                if exist([obj.dirpath,'\putty\pscp.exe'],'file') ~= 2
                    disp('Cannot Find PSCP')
                end
                
                %Paths to plink and pscp
%                 sshPath = ['"',obj.dirpath,'\putty\plink.exe','" ', obj.hostname, ' -l ', obj.username, obj.pwArg,' '];
                sshPath = ['"', obj.dirpath,'\putty\plink.exe', '" ', obj.hostname, ' -l ', obj.username, obj.pwArg,' '];
                scpPath = ['"', obj.dirpath,'\putty\pscp.exe', '" ', obj.pwArg, ' -q ','"',obj.dirpath,'\server\* ','" ', obj.username, '@',obj.hostname,':/root/Installers/'];
                
                %Kill any previously running radar instances and test if the connector is alread installed
                system(['echo Y | ','"',obj.dirpath,'\putty\plink','" ', obj.hostname, ' -l ', obj.username, obj.pwArg, ' pkill matlabConnector; pkill radarTest;']);           %THIS LINE MUST NOT BE COMBINED WITH THE FOLLOWING LINE or it will load load properly on the first run on a different computer
            else %This is for GLNXA64 (64 bit linux), and MACI64 (64 bit OSX).  This will work on windows if ssh.exe and scp.exe are present as well....
                %Paths to ssh and scp
                sshPath = ['ssh ', obj.hostname, ' -l ', obj.username, obj.pwArg,' '];
                scpPath = ['scp ', obj.pwArg, ' -q ',obj.dirpath,'\server\* ', obj.username, '@',obj.hostname,':/root/Installers/'];

                %Kill any previously running radar instances and test if the connector is alread installed
                system(['ssh ', obj.hostname, ' -l ', obj.username, obj.pwArg, ' pkill matlabConnector; pkill radarTest;']);           %THIS LINE MUST NOT BE COMBINED WITH THE FOLLOWING LINE or it will load load properly on the first run on a different computer
            end
            
            %Determine the version of the connector on the BBB
            obj.getBBBWrapperVersion(sshPath)
            if (obj.connectorVer<obj.packagedConnectorVersion)
                updateConnector_i=1;
            end
            
            %If its not installed or an update is requested....
            if (updateConnector_i == 1)
                obj.updateBBBWrapper(sshPath,scpPath, obj.installerName)
            end

            %Establish a connection to the BBB and start the connector
            obj.startConnector(sshPath);
            
            %Establish a socket connection to the BBB
            for i=1:20
                try
%                     obj.startConnector(sshPath);
                    obj.t = tcpclient(obj.hostname,obj.port,'Timeout',.5);
                    break
                catch
                    disp('trying to connect....');
                end
            end
            if (i==20)
                error('Could not connect to beaglebone')
            end

            obj.t.Timeout = 0.25;

            %NVA_CreateHandle()
            write(obj.t,uint8('NVA_CreateHandle()'))
            obj.getData();

            %Use binary data transfer mode
            if (obj.DEV_binaryDataTransfer)
                write(obj.t,uint8('BinaryMode_IntsEnabled()'))
                obj.getData();
            end
        end
    
        %% Instruct the connector to gzip the frame prior to sending it out over the socket - Not Recomended
        function CompressionMode(obj, value)
            % GZIPs the frame data prior to sending it over the socket
            % connection to roughly half the amount of data needing to be
            % sent.  Unfortunitly over ethernet or USB, the time it takes
            % to compress the data is longer then the transmission time of
            % the larger packet.  This may be useful though in Threaded
            % mode or with REALLLLLY slow connections in the future so I'm
            % leaving it in for now....
            %
            % Currently only works with getFrameRaw() for the X4
            write(obj.t,uint8(['BinaryMode_CompressionEnable(',num2str(value),')']))
            obj.getData();
            
            if (value > 0)
                obj.DEV_compressDataTransfer = 1;
            else 
                obj.DEV_compressDataTransfer = 0;
            end
        end
        
        %% Instruct the connector to gzip the frame prior to sending it out over the socket
        function ThreadedMode(obj, value)
            % This instructs the beaglebone to send a previous frame while
            % starting collection of a new frame.  This results in faster
            % framerates.
            %
            % Note: The frame returned from this function was a PREVIOUS
            % frame collection time.
            %
            % IN ACTIVE DEVELOPMENT
            %
            % Currently only for getFrameRaw() with the X4
            write(obj.t,uint8(['BinaryMode_ThreadedEnable(',num2str(value),')']))
            obj.getData();
        end
        
        %% Instruct the connector to send the length of the packet as the first word of the packet
        function SendPacketLengths(obj, value)
            % This instructs the beaglebone to send a previous frame while
            % starting collection of a new frame.  This results in faster
            % framerates.
            %
            % Note: The frame returned from this function was a PREVIOUS
            % frame collection time.
            %
            % IN ACTIVE DEVELOPMENT
            %
            % Currently only for getFrameRaw() with the X4
            
            obj.DEV_v2_packet_type = value;
            write(obj.t,uint8(['SendPacketLengths(',num2str(value),')']))

            obj.getData();
        end
        
        
        
        
    end
end
