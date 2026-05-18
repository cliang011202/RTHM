classdef RTHM_Agent < handle
    % RTHM_Agent  Independent UDP bridge between MATLAB/Simulink and STM32H743
    %
    % Responsibilities:
    %   - Pack 6-DOF loads into 28-byte protocol (seq + 6×float32 LE)
    %   - Send at 25 Hz fixed rate via UDP to STM32 :8080
    %   - Maintain monotonic sequence counter for drop detection
    %   - Optional feedback receive from STM32
    %
    % Usage (B1 test — single packet):
    %   agent = RTHM_Agent();
    %   agent.start();
    %   agent.send([11.2; 0; -0.69; 1.56; 0.042; 0]);  % model-scale loads
    %   agent.stop();
    %
    % Usage (B1 test — replay logged data):
    %   agent = RTHM_Agent();
    %   agent.start();
    %   agent.replay(loadsNx6, 0.04);  % 25 Hz
    %   agent.stop();
    %
    % Usage (silent, for batch):
    %   agent = RTHM_Agent(Verbose=false);
    %   agent.start();
    %   agent.send(pkt28, Raw=true);
    %   agent.stop();

    properties
        StmIP     (1,1) string = "192.168.1.100"
        StmPort   (1,1) uint16 = 8080
        LocalPort (1,1) uint16 = 12345
        Verbose   (1,1) logical = true
    end

    properties (SetAccess = private)
        Seq       (1,1) uint32 = 0     % Current packet sequence number
        SentCount (1,1) uint32 = 0     % Total packets sent since start()
        IsRunning (1,1) logical = false
    end

    properties (Access = private)
        UdpOut                          % udpport object for sending
        UdpIn                           % udpport object for feedback (optional)
        RateTimer                       % tic handle for replay timing
    end

    methods
        function obj = RTHM_Agent(opts)
            arguments
                opts.StmIP     (1,1) string = "192.168.1.100"
                opts.StmPort   (1,1) uint16 = 8080
                opts.LocalPort (1,1) uint16 = 12345
                opts.Verbose   (1,1) logical = true
            end
            obj.StmIP = opts.StmIP;
            obj.StmPort = opts.StmPort;
            obj.LocalPort = opts.LocalPort;
            obj.Verbose = opts.Verbose;
        end

        function delete(obj)
            stop(obj);
        end

        function start(obj)
            % start  Open UDP socket. Must call before send()/replay().
            if obj.IsRunning
                warning('RTHM_Agent:alreadyStarted', 'Agent is already running.');
                return;
            end
            obj.UdpOut = udpport("byte", "LocalPort", obj.LocalPort);
            obj.Seq = 0;
            obj.SentCount = 0;
            obj.IsRunning = true;
            obj.RateTimer = tic;
            if obj.Verbose
                fprintf('[RTHM_Agent] UDP ready: *:%d → %s:%d\n', ...
                    obj.LocalPort, obj.StmIP, obj.StmPort);
            end
        end

        function stop(obj)
            % stop  Close UDP socket and clean up.
            if ~obj.IsRunning
                return;
            end
            obj.IsRunning = false;
            if ~isempty(obj.UdpOut)
                delete(obj.UdpOut);
                obj.UdpOut = [];
            end
            if ~isempty(obj.UdpIn)
                delete(obj.UdpIn);
                obj.UdpIn = [];
            end
            if obj.Verbose
                fprintf('[RTHM_Agent] Stopped. %d packets sent.\n', obj.SentCount);
            end
        end

        function send(obj, load, opts)
            % send  Pack and transmit one 6-DOF load packet.
            %
            %   agent.send(load6)        load6 = [fx;fy;fz;mx;my;mz] model-scale
            %   agent.send(pkt28, Raw=true)  pre-packed 28-byte uint8 vector
            arguments
                obj
                load  (:,1) double
                opts.Raw (1,1) logical = false
            end
            assert(obj.IsRunning, 'RTHM_Agent:notStarted', ...
                'Agent not started. Call start() first.');

            if opts.Raw
                assert(numel(load) == 28, 'Raw packet must be 28 bytes.');
                pkt = uint8(load(:));
            else
                assert(numel(load) == 6, 'Load must be 6 elements [fx fy fz mx my mz].');
                obj.Seq = obj.Seq + 1;
                pkt = obj.packBytes(double(load(:)), obj.Seq);
            end

            write(obj.UdpOut, pkt, "uint8", obj.StmIP, obj.StmPort);
            obj.SentCount = obj.SentCount + 1;

            if obj.Verbose
                obj.printSend(load, opts.Raw);
            end
        end

        function replay(obj, loads, period)
            % replay  Replay a time series of 6-DOF loads at fixed rate.
            %
            %   agent.replay(loadsNx6, 0.04)   replay at 25 Hz (40 ms period)
            %   agent.replay(loadsNx6, 0)       send as fast as possible
            %
            %   loads: [N×6] matrix, each row = [fx fy fz mx my mz] model-scale.
            arguments
                obj
                loads (:,:) double
                period (1,1) double = 0.04
            end
            assert(obj.IsRunning, 'RTHM_Agent:notStarted', ...
                'Agent not started. Call start() first.');
            assert(size(loads,2) == 6, 'Loads must be N×6 matrix.');

            N = size(loads, 1);
            if obj.Verbose
                if period > 0
                    fprintf('[RTHM_Agent] Replaying %d packets at %.0f Hz (T=%.0f ms)...\n', ...
                        N, 1/period, period*1000);
                else
                    fprintf('[RTHM_Agent] Replaying %d packets at max rate...\n', N);
                end
            end

            t0 = tic;
            for k = 1:N
                if ~obj.IsRunning, break; end
                if period > 0
                    t_target = (k-1) * period;
                    t_elapsed = toc(t0);
                    if t_elapsed < t_target
                        pause(t_target - t_elapsed);
                    end
                end
                obj.send(loads(k, :)');
            end

            if obj.Verbose
                wall_time = toc(t0);
                sim_time = N * period;
                if sim_time > 0
                    fprintf('[RTHM_Agent] Done. Wall: %.1f s, Sim: %.1f s (%.0f× realtime)\n', ...
                        wall_time, sim_time, sim_time / wall_time);
                else
                    fprintf('[RTHM_Agent] Done. Wall: %.1f s.\n', wall_time);
                end
            end
        end

        function startFeedback(obj, localPort)
            % startFeedback  Open a second UDP port to receive STM32 status.
            %   STM32 is expected to send 7×PWM values + timestamp to this port.
            arguments
                obj
                localPort (1,1) uint16 = 12346
            end
            obj.UdpIn = udpport("byte", "LocalPort", localPort, "Timeout", 0.05);
            if obj.Verbose
                fprintf('[RTHM_Agent] Feedback listener on port %d\n', localPort);
            end
        end

        function [data, ip, port] = recvFeedback(obj, maxBytes)
            % recvFeedback  Non-blocking read from STM32 status channel.
            %   Returns [] if no data available.
            arguments
                obj
                maxBytes (1,1) uint16 = 256
            end
            data = []; ip = ""; port = uint16(0);
            if isempty(obj.UdpIn) || ~obj.IsRunning
                return;
            end
            try
                [data, info] = read(obj.UdpIn, maxBytes, "uint8");
                data = data(:);
                ip = info.Address;
                port = info.Port;
            catch
                % Timeout — no data available, this is expected.
            end
        end
    end

    methods (Access = private)
        function pkt = packBytes(~, load, seq)
            % packBytes  seq(uint32) + 6×float32 → 28 bytes little-endian
            pkt = zeros(28, 1, 'uint8');
            s = uint32(seq);
            pkt(1) = bitand(s, 255);
            pkt(2) = bitand(bitshift(s, -8), 255);
            pkt(3) = bitand(bitshift(s, -16), 255);
            pkt(4) = bitand(bitshift(s, -24), 255);
            for i = 1:6
                b = typecast(single(load(i)), 'uint8');
                pkt((i*4+1):(i*4+4)) = b;
            end
        end

        function printSend(obj, load, raw)
            if raw
                fprintf('[RTHM_Agent] Sent raw 28 B packet #%d → %s:%d\n', ...
                    obj.SentCount, obj.StmIP, obj.StmPort);
                return;
            end
            fprintf('[RTHM_Agent] #%d → %s:%d  F=[%+7.2f %+7.2f %+7.2f] N  M=[%+7.4f %+7.4f %+7.4f] N·m\n', ...
                obj.Seq, obj.StmIP, obj.StmPort, load(1:3), load(4:6));
        end
    end
end
