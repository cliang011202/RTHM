classdef UDPPacketPacker < matlab.System
    % UDPPacketPacker  Han Eq.2-5 compensation + Froude scaling + byte packing for UDP output
    %
    %  Compensation (prototype scale):
    %    F_aero = F_tt - m·a - m·g_body
    %    M_aero = M_tt - r_cg×(m·a) - r_cg×(m·g_body) - I·α - ω×(I·ω)
    %
    %  Froude scaling: prototype → model scale (λ=50, Froude similarity)
    %    F_model = F_proto / (ρ_ratio × λ³)
    %    M_model = M_proto / (ρ_ratio × λ⁴)
    %
    %  Input 1: ttLoad [6×1] — tower-top 6-DOF load (prototype scale)
    %  Input 2: seq    [1×1] — packet sequence number
    %  Input 3: nacAcc [3×1] — nacelle linear acceleration (prototype scale)
    %  Input 4: eulAng [3×1] — platform Euler angles [qx,qy,qz] [rad]
    %  Input 5: angAcc [3×1] — platform angular acceleration [rad/s²] (prototype scale)
    %  Input 6: angVel [3×1] — platform angular velocity [rad/s] (prototype scale)
    %  Output:  pkt    [28×1 uint8] — UDP datagram (model-scale loads)

    properties (Nontunable)
        RNA_mass     = 950058           % [kg] nacelle(646895)+yawBearing(28280)+hub(69360)+3×blade(205523)
        RNA_cog      = [-6.70; 0; 10.51] % [m] RNA CG relative to tower top
        RNA_inertia  = [                  % [kg·m²] about RNA CG (with yaw bearing)
            2.8538e+08, 0.0000e+00, 3.3045e+07;
            0.0000e+00, 2.9418e+08, 0.0000e+00;
            3.3045e+07, 0.0000e+00, 3.5860e+07];
        Gravity      = [0; 0; -9.80665]  % [m/s²] in inertial frame
        EnableCompensation = true
        Lambda       = 50                % Froude geometric scale factor λ (prototype/model)
        RhoRatio     = 1000/1025         % ρ_fresh(model) / ρ_sea(proto) for force scaling
        EnableScaling = true
    end

    methods (Access = protected)
        function pkt = stepImpl(obj, ttLoad, seq, nacAcc, eulAng, angAcc, angVel)
            if obj.EnableCompensation
                load = obj.compensate(ttLoad, nacAcc, eulAng, angAcc, angVel);
            else
                load = ttLoad;
            end
            if obj.EnableScaling
                load = obj.scaleToModel(load);
            end
            pkt = obj.packBytes(load, seq);
        end

        function load = compensate(obj, ttLoad, nacAcc, eulAng, angAcc, angVel)
            F_tt = ttLoad(1:3);
            M_tt = ttLoad(4:6);

            g_body = obj.rotateGravity(eulAng);

            % Translational
            F_inertial = obj.RNA_mass * nacAcc(:);
            F_grav     = obj.RNA_mass * g_body;
            F_aero = F_tt - F_inertial - F_grav;

            % Moments from translational forces at CG offset
            M_inertial = cross(obj.RNA_cog(:), F_inertial);
            M_grav     = cross(obj.RNA_cog(:), F_grav);

            % Rotational: I·α + ω×(I·ω)
            M_rot  = obj.RNA_inertia * angAcc(:);
            M_gyro = cross(angVel(:), obj.RNA_inertia * angVel(:));

            M_aero = M_tt - M_inertial - M_grav - M_rot - M_gyro;

            load = [F_aero(:); M_aero(:)];
        end

        function load = scaleToModel(obj, load)
            % Froude scaling: prototype → model scale
            % F_model = F_proto / (ρ_ratio × λ³)
            % M_model = M_proto / (ρ_ratio × λ⁴)
            fScale = obj.RhoRatio * obj.Lambda^3;   % ≈ 128,125
            mScale = obj.RhoRatio * obj.Lambda^4;   % ≈ 6,406,250
            load(1:3) = load(1:3) / fScale;
            load(4:6) = load(4:6) / mScale;
        end

        function g_body = rotateGravity(obj, eulAng)
            qx = eulAng(1); qy = eulAng(2);
            cx = cos(qx); sx = sin(qx);
            cy = cos(qy); sy = sin(qy);

            g = obj.Gravity(3);  % -9.80665
            g_body = [-g * sy;       % g_mag * sin(pitch)
                       g * cy * sx;   % -g_mag * cos(pitch) * sin(roll)
                       g * cy * cx];  % -g_mag * cos(pitch) * cos(roll)
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

        function num = getNumInputsImpl(~)
            num = 6;
        end
        function num = getNumOutputsImpl(~)
            num = 1;
        end
        function [sz1,sz2,sz3,sz4,sz5,sz6] = getInputSizeImpl(~)
            sz1=[6 1]; sz2=[1 1]; sz3=[3 1]; sz4=[3 1]; sz5=[3 1]; sz6=[3 1];
        end
        function sz = getOutputSizeImpl(~)
            sz = [28 1];
        end
        function [dt1,dt2,dt3,dt4,dt5,dt6] = getInputDataTypeImpl(~)
            dt1='double';dt2='double';dt3='double';dt4='double';dt5='double';dt6='double';
        end
        function dt = getOutputDataTypeImpl(~)
            dt = 'uint8';
        end
        function [f1,f2,f3,f4,f5,f6] = isInputFixedSizeImpl(~)
            f1=true;f2=true;f3=true;f4=true;f5=true;f6=true;
        end
        function f = isOutputFixedSizeImpl(~)
            f = true;
        end
        function [c1,c2,c3,c4,c5,c6] = isInputComplexImpl(~)
            c1=false;c2=false;c3=false;c4=false;c5=false;c6=false;
        end
        function c = isOutputComplexImpl(~)
            c = false;
        end
    end
end
