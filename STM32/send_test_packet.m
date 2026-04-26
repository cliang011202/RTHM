% send_test_packet.m
  STM32_IP   = "192.168.1.100";
  STM32_PORT = 8080;

  u = udpport("byte", "LocalPort", 12345);

  for k = 1:5
      seq = typecast(uint32(k),    'uint8');  % 4 B
      fx  = typecast(single( 10.5),'uint8');  % 4 B
      fy  = typecast(single( 20.5),'uint8');
      fz  = typecast(single( 30.5),'uint8');
      mx  = typecast(single(  1.5),'uint8');
      my  = typecast(single(  2.5),'uint8');
      mz  = typecast(single(  3.5),'uint8');  % 共 28 B

      pkt = [seq, fx, fy, fz, mx, my, mz];
      write(u, pkt, "uint8", STM32_IP, STM32_PORT);
      pause(0.1);
  end

  clear u;   % 关闭 socket