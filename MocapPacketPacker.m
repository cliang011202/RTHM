classdef MocapPacketPacker < matlab.System
    % MocapPacketPacker  Pack 18-DOF motion data into 76-byte UDP datagram
    %
    %  Packet format:
    %    Bytes 1-4:   uint32 LE  sequence number
    %    Bytes 5-28:  6×single LE  position  [px,py,pz,qx,qy,qz]
    %    Bytes 29-52: 6×single LE  velocity  [vx,vy,vz,wx,wy,wz]
    %    Bytes 53-76: 6×single LE  acceleration [ax,ay,az,αx,αy,αz]
    %
    %  Input 1: pos [6×1] — platform position
    %  Input 2: vel [6×1] — platform velocity
    %  Input 3: acc [6×1] — platform acceleration
    %  Input 4: seq [1×1] — sequence number
    %  Output:  pkt [76×1 uint8] — UDP datagram

    methods (Access = protected)
        function pkt = stepImpl(~, pos, vel, acc, seq)
            pkt = zeros(76, 1, 'uint8');
            s = uint32(seq);

            pkt(1) = uint8(bitand(s, 255));
            pkt(2) = uint8(bitand(bitshift(s, -8), 255));
            pkt(3) = uint8(bitand(bitshift(s, -16), 255));
            pkt(4) = uint8(bitand(bitshift(s, -24), 255));

            for i = 1:6
                b = typecast(single(pos(i)), 'uint8');
                pkt(4+(i-1)*4+(1:4)) = b;
            end
            for i = 1:6
                b = typecast(single(vel(i)), 'uint8');
                pkt(28+(i-1)*4+(1:4)) = b;
            end
            for i = 1:6
                b = typecast(single(acc(i)), 'uint8');
                pkt(52+(i-1)*4+(1:4)) = b;
            end
        end

        function num = getNumInputsImpl(~)
            num = 4;
        end
        function num = getNumOutputsImpl(~)
            num = 1;
        end
        function varargout = getInputSizeImpl(~)
            varargout = {[6 1], [6 1], [6 1], [1 1]};
        end
        function sz = getOutputSizeImpl(~)
            sz = [76 1];
        end
        function varargout = getInputDataTypeImpl(~)
            varargout = {'double','double','double','double'};
        end
        function dt = getOutputDataTypeImpl(~)
            dt = 'uint8';
        end
        function varargout = isInputFixedSizeImpl(~)
            varargout = {true, true, true, true};
        end
        function f = isOutputFixedSizeImpl(~)
            f = true;
        end
        function varargout = isInputComplexImpl(~)
            varargout = {false, false, false, false};
        end
        function c = isOutputComplexImpl(~)
            c = false;
        end
    end
end
