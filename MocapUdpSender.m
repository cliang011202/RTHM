classdef MocapUdpSender < handle
    % MocapUdpSender  Async UDP sender for mocap data, driven by MATLAB timer
    %
    % Usage:
    %   s = MocapUdpSender(packets, '127.0.0.1', 10001);
    %   s.start();          % Start async timer-based sending @ 100 Hz
    %   ... run simulation ...
    %   s.stop();           % Stop and report stats

    properties
        SentCount  = 0
        TotalCount = 0
    end

    properties (Access = private)
        Sender
        Packets
        Timer
    end

    methods
        function obj = MocapUdpSender(packets, host, port, sendHz)
            % sendHz: sending rate in Hz (e.g. 100 for real-time, 120 to match ×1.2 sim)
            obj.Packets = packets;
            obj.TotalCount = length(packets);
            obj.Sender = dsp.UDPSender('RemoteIPAddress', host, ...
                                       'RemoteIPPort', port);
            obj.Timer = timer('ExecutionMode', 'fixedRate', ...
                              'Period', 1/sendHz, ...
                              'TimerFcn', @(~,~) obj.sendOne(), ...
                              'TasksToExecute', obj.TotalCount, ...
                              'BusyMode', 'queue', ...
                              'Name', 'MocapSender');
        end

        function start(obj)
            obj.SentCount = 0;
            start(obj.Timer);
        end

        function stop(obj)
            stop(obj.Timer);
            delete(obj.Timer);
        end

        function n = sent(obj)
            n = obj.SentCount;
        end
    end

    methods (Access = private)
        function sendOne(obj)
            obj.SentCount = obj.SentCount + 1;
            if obj.SentCount <= obj.TotalCount
                obj.Sender(obj.Packets{obj.SentCount});
            end
        end
    end
end
