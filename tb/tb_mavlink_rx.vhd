library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity tb_mavlink_rx is
-- Testbenches have no ports
end tb_mavlink_rx;

architecture behavior of tb_mavlink_rx is

    -- Signals to connect to the Device Under Test (DUT)
    signal clk              : std_logic := '0';
    signal rst              : std_logic := '1';
    signal uart_rx_data     : std_logic_vector(7 downto 0) := x"00";
    signal uart_rx_valid    : std_logic := '0';
    
    signal heartbeat_pulse  : std_logic;
    signal armed            : std_logic;
    signal received         : std_logic;
    signal result           : std_logic_vector(7 downto 0);

    -- 100 MHz clock period
    constant CLK_PERIOD : time := 10 ns;

begin

    -- Instantiate the DUT (Device Under Test)
    DUT : entity work.mavlink_rx
    port map (
        clk             => clk,
        rst             => rst,
        uart_rx_data    => uart_rx_data,
        uart_rx_valid   => uart_rx_valid,
        heartbeat_pulse => heartbeat_pulse,
        armed           => armed,
        received        => received,
        result          => result
    );

    -- Clock generation process
    clk_process : process
    begin
        clk <= '0';
        wait for CLK_PERIOD/2;
        clk <= '1';
        wait for CLK_PERIOD/2;
    end process;

    -- Stimulus process
    stim_proc: process
        -- Helper procedure to simulate the UART sending 1 byte
        procedure send_byte(data : std_logic_vector(7 downto 0)) is
        begin
            uart_rx_data <= data;
            uart_rx_valid <= '1';
            wait for CLK_PERIOD;
            uart_rx_valid <= '0';
            wait for CLK_PERIOD * 5; -- Simulate some delay between UART bytes
        end procedure;

    begin
        -- 1. Initialize and Reset
        wait for 100 ns;
        rst <= '0';
        wait for 100 ns;

        report "--- STARTING TEST 1: Valid Heartbeat (Armed) ---";
        -- Packet Breakdown: 0xFE, Len=9, Seq=0, Sys=1, Comp=1, Msg=0
        -- Payload: 9 bytes (Byte 4 is base_mode = 0x80 for ARMED)
        -- CRC: Let's assume calculated CRC for this specific packet is 0xABCD
        -- (Note: In a real simulation, you'll need the exact matching CRC for your packet, 
        -- but for this testbench logic flow we will push the bytes).
        
        send_byte(x"FE"); -- STX
        send_byte(x"09"); -- LEN
        send_byte(x"00"); -- SEQ
        send_byte(x"01"); -- SYS ID
        send_byte(x"01"); -- COMP ID
        send_byte(x"00"); -- MSG ID (Heartbeat)
        
        -- Payload
        send_byte(x"00"); 
        send_byte(x"00"); 
        send_byte(x"00"); 
        send_byte(x"00"); 
        send_byte(x"80"); -- base_mode (Bit 7 is '1', so ARMED)
        send_byte(x"00"); 
        send_byte(x"00"); 
        send_byte(x"00"); 
        send_byte(x"00"); 

        -- CRC (To make this pass perfectly in simulation, you'd calculate the real X.25 CRC.
        -- If it fails in Sim, check your waveform for `current_crc_reg` to see what it expected!)
        send_byte(x"23"); -- CRC L (Dummy value)
        send_byte(x"F4"); -- CRC H (Dummy value)
        
        wait for 500 ns;

        report "--- STARTING TEST 2: Valid Command ACK (Accepted) ---";
        -- Packet Breakdown: Msg ID 77 (0x4D), Length 3
        send_byte(x"FE"); -- STX
        send_byte(x"03"); -- LEN
        send_byte(x"01"); -- SEQ
        send_byte(x"01"); -- SYS
        send_byte(x"01"); -- COMP
        send_byte(x"4D"); -- MSG ID (Command ACK = 77)
        
        -- Payload
        send_byte(x"00"); -- Command ID LSB
        send_byte(x"00"); -- Command ID MSB
        send_byte(x"00"); -- Result (0x00 = ACCEPTED)
        
        -- CRC
		send_byte(x"68"); -- CRC L 
		send_byte(x"91"); -- CRC H 
        
        wait for 500 ns;
        
        report "--- SIMULATION COMPLETE ---";
        wait;
    end process;

end behavior;