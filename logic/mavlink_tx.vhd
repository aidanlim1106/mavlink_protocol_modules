library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity mavlink_tx is
    port(
        clk : in std_logic;
        rst : in std_logic;
        out_heartbeat : in std_logic;
        out_arm : in std_logic;
        out_disarm : in std_logic;
        pot_mode : in std_logic;
        rc_override : in std_logic;
        steering_pwm : in std_logic_vector(15 downto 0);
        throttle_pwm : in std_logic_vector(15 downto 0);
        uart_tx_accept : in std_logic;
        uart_tx_empty : in std_logic;
        uart_tx_data : out std_logic_vector (7 downto 0);
        uart_tx_rdy : out std_logic;
        busy : out std_logic
    );
end mavlink_tx;

architecture behavioral of mavlink_tx is

    type tx_state_type is (IDLE, CRC_RUN, CRC_EX, CRC_STORE, SEND, SWAIT);
    signal state : tx_state_type := IDLE;

    signal crc_din : std_logic_vector(7 downto 0);
    signal crc_in : std_logic_vector(15 downto 0);
    signal crc_out : std_logic_vector(15 downto 0);

    signal crc_reg : std_logic_vector(15 downto 0) := x"FFFF";
    signal crc_extra : std_logic_vector(7 downto 0) := x"00";

    -- frame buffer big enough for COMMAND_LONG (8 + 33 = 41 bytes)
    type frame_array is array (0 to 43) of std_logic_vector(7 downto 0);
    signal frame : frame_array := (others => x"00");

    signal pay_len : integer range 0 to 40 := 0;
    signal total_len : integer range 0 to 44 := 0;
    signal fold_idx : integer range 0 to 43 := 0;
    signal k : integer range 0 to 43 := 0;

    signal seq_num : integer range 0 to 255 := 0;

    constant SYS_ID : std_logic_vector(7 downto 0) := x"FA";
    constant COMP_ID : std_logic_vector(7 downto 0) := x"00";

begin

    crc_BLOCK : entity work.mavlink_crc
        port map(
            din => crc_din, 
            crc_in => crc_in, 
            crc_out => crc_out
        );

    crc_in  <= crc_reg;
    crc_din <= crc_extra when state = CRC_EX else frame(fold_idx);

    process(clk, rst)
    begin
        if (rst = '1') then
            state <= IDLE;
            busy <= '0';
            uart_tx_rdy <= '0';
            seq_num <= 0;
            crc_reg <= x"FFFF";
        elsif rising_edge(clk) then
            uart_tx_rdy <= '0';
            case state is

                when IDLE =>
                    busy <= '0';
                    crc_reg <= x"FFFF";
                    if (out_arm = '1' or out_disarm = '1') then
                        busy <= '1';
                        crc_extra <= x"98"; -- 152
                        pay_len   <= 33;
                        total_len <= 41;
                        frame(0) <= x"FE"; frame(1) <= x"21"; -- len 33
                        frame(2) <= std_logic_vector(to_unsigned(seq_num,8));
                        frame(3) <= SYS_ID; frame(4) <= COMP_ID; frame(5) <= x"4C"; -- msg 76
                        -- param1 (float): 1.0 = arm, 0.0 = disarm  (LE)
                        if (out_disarm = '1') then
                            frame(6) <= x"00"; frame(7) <= x"00"; frame(8) <= x"00"; frame(9) <= x"00";
                        else
                            frame(6) <= x"00"; frame(7) <= x"00"; frame(8) <= x"80"; frame(9) <= x"3F";
                        end if;
                        -- param2..param7 = 0.0  (set param2=21196.0 -> 00 4C A6 46 to FORCE-arm past checks)
                        for i in 10 to 33 loop frame(i) <= x"00"; end loop;
                        -- command = 400 (0x0190) LE
                        frame(34) <= x"90"; frame(35) <= x"01";
                        frame(36) <= x"01";           -- target_system  (rover SYSID_THISMAV, default 1)
                        frame(37) <= x"01";           -- target_component (autopilot = 1)
                        frame(38) <= x"00";           -- confirmation
                        if (seq_num = 255) then seq_num <= 0; else seq_num <= seq_num + 1; end if;
                        fold_idx <= 1;
                        state <= CRC_RUN;

                    elsif (out_heartbeat = '1') then
                        busy <= '1';
                        crc_extra <= x"32";
                        pay_len   <= 9;
                        total_len <= 17;
                        frame(0) <= x"FE"; frame(1) <= x"09";
                        frame(2) <= std_logic_vector(to_unsigned(seq_num,8));
                        frame(3) <= SYS_ID; frame(4) <= COMP_ID; frame(5) <= x"00";
                        frame(6)  <= x"00"; frame(7) <= x"00"; frame(8) <= x"00";
                        frame(9)  <= x"00"; frame(10) <= x"0A"; frame(11) <= x"03";
                        frame(12) <= x"00"; frame(13) <= x"00"; frame(14) <= x"00";
                        if (seq_num = 255) then seq_num <= 0; else seq_num <= seq_num + 1; end if;
                        fold_idx <= 1;
                        state <= CRC_RUN;

                    elsif (rc_override = '1') then
                        busy <= '1';
                        crc_extra <= x"7C";
                        pay_len   <= 18;
                        total_len <= 26;
                        frame(0) <= x"FE"; frame(1) <= x"12";
                        frame(2) <= std_logic_vector(to_unsigned(seq_num,8));
                        frame(3) <= SYS_ID; frame(4) <= COMP_ID; frame(5) <= x"46";
                        -- chan1 (frame 6-7) = steering, chan2 (frame 8-9) = throttle
                        -- servo 1 turn, servo 2 throttle
                        frame(6)  <= steering_pwm(7 downto 0);
                        frame(7)  <= steering_pwm(15 downto 8);
                        frame(8)  <= throttle_pwm(7 downto 0);
                        frame(9)  <= throttle_pwm(15 downto 8);
                        frame(10) <= x"FF"; frame(11) <= x"FF"; 
                        for i in 12 to 21 loop frame(i) <= x"FF"; end loop; 
                        frame(22) <= x"01"; -- target_system  = rover
                        frame(23) <= x"01"; -- target_component = autopilot
                        if (seq_num = 255) then seq_num <= 0; else seq_num <= seq_num + 1; end if;
                        fold_idx <= 1;
                        state <= CRC_RUN;
                    end if;

                when CRC_RUN =>
                    crc_reg <= crc_out;
                    if (fold_idx = 5 + pay_len) then
                        state <= CRC_EX;
                    else
                        fold_idx <= fold_idx + 1;
                    end if;

                when CRC_EX =>
                    crc_reg <= crc_out;
                    state <= CRC_STORE;

                when CRC_STORE =>
                    frame(6 + pay_len) <= crc_reg(7 downto 0);
                    frame(7 + pay_len) <= crc_reg(15 downto 8);
                    k <= 0;
                    state <= SEND;

                when SEND =>
                    uart_tx_data <= frame(k);
                    uart_tx_rdy  <= '1';
                    if (uart_tx_accept = '1') then
                        uart_tx_rdy <= '0';
                        state <= SWAIT;
                    end if;

                when SWAIT =>
                    if (uart_tx_empty = '1') then
                        if (k = total_len - 1) then
                            busy <= '0';
                            state <= IDLE;
                        else
                            k <= k + 1;
                            state <= SEND;
                        end if;
                    end if;

            end case;
        end if;
    end process;

end behavioral;