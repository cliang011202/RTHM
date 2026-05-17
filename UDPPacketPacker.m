classdef UDPPacketPacker < matlab.System
    % UDPPacketPacker  Compensate TTLoad → hub-equivalent aero, pack into UDP bytes
    %
    %  Han et al. (2025) Eq.2-5 compensation:
    %    F_aero = F_ttLoad - m_RNA·a_nac - m_RNA·g_towerTop
    %    M_aero = M_ttLoad - r_cg × (m_RNA·a_nac) - r_cg × (m_RNA·g_towerTop)
    %
    %  Input 1: ttLoad [6×1] — raw tower-top 6-DOF load  [Fx,Fy,Fz,Mx,My,Mz]
    %  Input 2: seq    [1×1] — packet sequence number
    %  Input 3: nacAcc [3×1] — nacelle linear acceleration [ax,ay,az]
    %  Output:  pkt    [28×1 uint8] — UDP datagram bytes (compensated loads)

    properties (Nontunable)
        RNA_mass  = 921778           % [kg] nacelle(646895) + hub(69360) + 3×blade(68508)
        RNA_cog   = [-6.90; 0; 10.83] % [m] RNA CG relative to tower top (yaw bearing)
                                      %   weighted avg of component CGs from Properties_IEA15MW
        Gravity   = [0; 0; -9.80665]  % [m/s²] gravity in inertial frame
        EnableCompensation = true     % toggle compensation on/off
    end

    methods (Access = protected)
        function pkt = stepImpl(obj, ttLoad, seq, nacAcc)
            if obj.EnableCompensation
                load = obj.compensate(ttLoad, nacAcc);
            else
                load = ttLoad;
            end
            pkt = obj.packBytes(load, seq);
        end

        function load = compensate(obj, ttLoad, nacAcc)
            % Split force and moment
            F_tt = ttLoad(1:3);   % tower-top force
            M_tt = ttLoad(4:6);   % tower-top moment

            % Inertial force of RNA
            F_inertial = obj.RNA_mass * nacAcc(:);

            % Gravity force (approx: assume tower nearly vertical)
            % TODO: use platform attitude to rotate gravity vector properly
            F_grav = obj.RNA_mass * obj.Gravity(:);

            % Translational compensation
            F_aero = F_tt - F_inertial - F_grav;

            % Moment compensation: subtract moments from inertial+gravity
            % forces acting at RNA CG
            M_inertial = cross(obj.RNA_cog(:), F_inertial);
            M_grav     = cross(obj.RNA_cog(:), F_grav);
            M_aero = M_tt - M_inertial - M_grav;

            load = [F_aero(:); M_aero(:)];
        end

        function pkt = packBytes(~, load, seq)
            pkt = zeros(28, 1, 'uint8');
            s = uint32(seq);
            pkt(1) = uint8(bitand(s, 255));
            pkt(2) = uint8(bitand(bitshift(s, -8), 255));
            pkt(3) = uint8(bitand(bitshift(s, -16), 255));
            pkt(4) = uint8(bitand(bitshift(s, -24), 255));

            b = typecast(single(load(1)), 'uint8'); pkt(5) = b(1); pkt(6) = b(2); pkt(7) = b(3); pkt(8) = b(4);
            b = typecast(single(load(2)), 'uint8'); pkt(9) = b(1); pkt(10) = b(2); pkt(11) = b(3); pkt(12) = b(4);
            b = typecast(single(load(3)), 'uint8'); pkt(13) = b(1); pkt(14) = b(2); pkt(15) = b(3); pkt(16) = b(4);
            b = typecast(single(load(4)), 'uint8'); pkt(17) = b(1); pkt(18) = b(2); pkt(19) = b(3); pkt(20) = b(4);
            b = typecast(single(load(5)), 'uint8'); pkt(21) = b(1); pkt(22) = b(2); pkt(23) = b(3); pkt(24) = b(4);
            b = typecast(single(load(6)), 'uint8'); pkt(25) = b(1); pkt(26) = b(2); pkt(27) = b(3); pkt(28) = b(4);
        end

        % --- System interface ---
        function num = getNumInputsImpl(~)
            num = 3;
        end

        function num = getNumOutputsImpl(~)
            num = 1;
        end

        function [sz1, sz2, sz3] = getInputSizeImpl(~)
            sz1 = [6 1];
            sz2 = [1 1];
            sz3 = [3 1];
        end

        function sz = getOutputSizeImpl(~)
            sz = [28 1];
        end

        function [dt1, dt2, dt3] = getInputDataTypeImpl(~)
            dt1 = 'double';
            dt2 = 'double';
            dt3 = 'double';
        end

        function dt = getOutputDataTypeImpl(~)
            dt = 'uint8';
        end

        function [f1, f2, f3] = isInputFixedSizeImpl(~)
            f1 = true;
            f2 = true;
            f3 = true;
        end

        function f = isOutputFixedSizeImpl(~)
            f = true;
        end

        function [c1, c2, c3] = isInputComplexImpl(~)
            c1 = false;
            c2 = false;
            c3 = false;
        end

        function c = isOutputComplexImpl(~)
            c = false;
        end
    end
end
