classdef UDPPacketPacker < matlab.System
    % UDPPacketPacker  Pack 6-DOF load + seq into 28-byte UDP packet
    %
    % Input 1: ttLoad [6x1 double] — tower-top 6-DOF load
    % Input 2: seq    [1x1 double] — packet sequence number
    % Output:  pkt    [28x1 uint8] — UDP datagram bytes

    methods (Access = protected)
        function pkt = stepImpl(~, ttLoad, seq)
            pkt = zeros(28, 1, 'uint8');
            s = uint32(seq);
            pkt(1) = uint8(bitand(s, 255));
            pkt(2) = uint8(bitand(bitshift(s, -8), 255));
            pkt(3) = uint8(bitand(bitshift(s, -16), 255));
            pkt(4) = uint8(bitand(bitshift(s, -24), 255));

            b = typecast(single(ttLoad(1)), 'uint8'); pkt(5) = b(1); pkt(6) = b(2); pkt(7) = b(3); pkt(8) = b(4);
            b = typecast(single(ttLoad(2)), 'uint8'); pkt(9) = b(1); pkt(10) = b(2); pkt(11) = b(3); pkt(12) = b(4);
            b = typecast(single(ttLoad(3)), 'uint8'); pkt(13) = b(1); pkt(14) = b(2); pkt(15) = b(3); pkt(16) = b(4);
            b = typecast(single(ttLoad(4)), 'uint8'); pkt(17) = b(1); pkt(18) = b(2); pkt(19) = b(3); pkt(20) = b(4);
            b = typecast(single(ttLoad(5)), 'uint8'); pkt(21) = b(1); pkt(22) = b(2); pkt(23) = b(3); pkt(24) = b(4);
            b = typecast(single(ttLoad(6)), 'uint8'); pkt(25) = b(1); pkt(26) = b(2); pkt(27) = b(3); pkt(28) = b(4);
        end

        function num = getNumInputsImpl(~)
            num = 2;
        end

        function num = getNumOutputsImpl(~)
            num = 1;
        end

        function [sz1, sz2] = getInputSizeImpl(~)
            sz1 = [6 1];
            sz2 = [1 1];
        end

        function sz = getOutputSizeImpl(~)
            sz = [28 1];
        end

        function [dt1, dt2] = getInputDataTypeImpl(~)
            dt1 = 'double';
            dt2 = 'double';
        end

        function dt = getOutputDataTypeImpl(~)
            dt = 'uint8';
        end

        function [f1, f2] = isInputFixedSizeImpl(~)
            f1 = true;
            f2 = true;
        end

        function f = isOutputFixedSizeImpl(~)
            f = true;
        end

        function [c1, c2] = isInputComplexImpl(~)
            c1 = false;
            c2 = false;
        end

        function c = isOutputComplexImpl(~)
            c = false;
        end
    end
end
