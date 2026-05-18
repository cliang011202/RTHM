classdef MocapPacketUnpacker < matlab.System
    % MocapPacketUnpacker  Unpack 76-byte UDP motion-capture datagram
    %
    %  Packet format (from MockMocap / real mocap system):
    %    Bytes 1-4:   uint32 LE  sequence number
    %    Bytes 5-28:  6×single LE  position  [px,py,pz,qx,qy,qz]
    %    Bytes 29-52: 6×single LE  velocity  [vx,vy,vz,wx,wy,wz]
    %    Bytes 53-76: 6×single LE  acceleration [ax,ay,az,αx,αy,αz]
    %
    %  Output 1: pos [6×1 double] — platform 6-DOF position
    %  Output 2: vel [6×1 double] — platform 6-DOF velocity
    %  Output 3: acc [6×1 double] — platform 6-DOF acceleration
    %  Output 4: seq [1×1 uint32] — sequence number
    %  Output 5: valid [1×1 logical] — packet parsed OK

    properties (Nontunable)
        ExpectedPacketSize = 76
    end

    methods (Access = protected)
        function [pos, vel, acc, seq, valid] = stepImpl(obj, pkt)
            pkt = pkt(:);
            if numel(pkt) ~= obj.ExpectedPacketSize
                pos = zeros(6,1); vel = zeros(6,1); acc = zeros(6,1);
                seq = uint32(0); valid = false;
                return;
            end

            seq = uint32(pkt(1)) + bitshift(uint32(pkt(2)), 8) + ...
                  bitshift(uint32(pkt(3)), 16) + bitshift(uint32(pkt(4)), 24);

            pos = zeros(6,1);
            for i = 1:6
                b0 = 4 + (i-1)*4 + 1;
                pos(i) = double(typecast(pkt(b0:b0+3), 'single'));
            end

            vel = zeros(6,1);
            for i = 1:6
                b0 = 28 + (i-1)*4 + 1;
                vel(i) = double(typecast(pkt(b0:b0+3), 'single'));
            end

            acc = zeros(6,1);
            for i = 1:6
                b0 = 52 + (i-1)*4 + 1;
                acc(i) = double(typecast(pkt(b0:b0+3), 'single'));
            end

            valid = true;
        end

        function num = getNumInputsImpl(~)
            num = 1;
        end
        function num = getNumOutputsImpl(~)
            num = 5;
        end
        function varargout = getOutputSizeImpl(~)
            varargout = {[6 1], [6 1], [6 1], [1 1], [1 1]};
        end
        function varargout = getOutputDataTypeImpl(~)
            varargout = {'double','double','double','uint32','logical'};
        end
        function varargout = isOutputFixedSizeImpl(~)
            varargout = {true, true, true, true, true};
        end
        function varargout = isOutputComplexImpl(~)
            varargout = {false, false, false, false, false};
        end
        function sz = getInputSizeImpl(~)
            sz = [76 1];
        end
        function dt = getInputDataTypeImpl(~)
            dt = 'uint8';
        end
        function flag = isInputFixedSizeImpl(~)
            flag = true;
        end
        function flag = isInputComplexImpl(~)
            flag = false;
        end
    end
end
