library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity mavlink_rx is
    port (
        clk : in std_logic;
        rst : in std_logic;
        uart_rx_data : in std_logic_vector(7 downto 0);
        uart_rx_valid : in std_logic;
        heartbeat_pulse : out std_logic;
        armed : out std_logic;
        received : out std_logic;
        result : out std_logic_vector(7 downto 0)
    );
end mavlink_rx;

architecture behavioral of mavlink_rx is

    type rx_state_type is (
        IDLE, 
        READ_START, 
        READ_LEN, 
        READ_SEQ, 
        READ_SYSTEM_ID,
        READ_COMPONENT_ID, 
        READ_MESSAGE_ID, 
        READ_PAYLOAD, 
        APPLY_CRC_EXTRA, 
        READ_CRC
    );
    signal state : rx_state_type := IDLE;

    signal crc_din : std_logic_vector(7 downto 0);
    signal crc_in : std_logic_vector(15 downto 0);
    signal crc_out : std_logic_vector(15 downto 0);
    signal current_crc_reg : std_logic_vector(15 downto 0) := x"FFFF";

    signal length_id : integer range 0 to 255 := 0;
    signal byte_count : integer range 0 to 255 := 0;
    signal message_id : std_logic_vector(7 downto 0) := x"00";
    signal crc_extra : std_logic_vector(7 downto 0) := x"00";
    signal temp_base_mode : std_logic_vector(7 downto 0) := x"00";
    signal temp_ack_res : std_logic_vector(7 downto 0) := x"00";
    signal rx_crc_low : std_logic_vector(7 downto 0) := x"00";
    signal rx_sysid : std_logic_vector(7 downto 0) := x"00";
    constant OWN_SYS_ID : std_logic_vector(7 downto 0) := x"FA";
    signal rx_compid : std_logic_vector(7 downto 0) := x"00";
    constant OWN_COMP_ID : std_logic_vector(7 downto 0) := x"00";

begin

    crc_BLOCK : entity work.mavlink_crc
        port map(din => crc_din, crc_in => crc_in, crc_out => crc_out);

    crc_in  <= current_crc_reg;
    crc_din <= crc_extra when state = APPLY_CRC_EXTRA else uart_rx_data;

    process(clk, rst)
    begin
        if (rst = '1') then
            state <= IDLE;
            heartbeat_pulse <= '0';
            received <= '0';
            armed <= '0';
            result <= x"00";
            current_crc_reg <= x"FFFF";
        elsif rising_edge(clk) then
            heartbeat_pulse <= '0';
            received <= '0';
            if (uart_rx_valid = '1') then
                case state is
                    when IDLE =>
                        state <= READ_START;

                    when READ_START =>
                        if (uart_rx_data = x"FE") then
                            state <= READ_LEN;
                            current_crc_reg <= x"FFFF";
                        end if;

                    when READ_LEN =>
                        length_id <= to_integer(unsigned(uart_rx_data));
                        current_crc_reg <= crc_out; 
                        state <= READ_SEQ;

                    when READ_SEQ =>
                        current_crc_reg <= crc_out; 
                        state <= READ_SYSTEM_ID;

                    when READ_SYSTEM_ID =>
                        rx_sysid <= uart_rx_data;
                        current_crc_reg <= crc_out;
                        state <= READ_COMPONENT_ID;

                    when READ_COMPONENT_ID =>
                        rx_compid <= uart_rx_data;
                        current_crc_reg <= crc_out;
                        state <= READ_MESSAGE_ID;

                    when READ_MESSAGE_ID =>
                        message_id <= uart_rx_data;
                        byte_count <= 0;
                        current_crc_reg <= crc_out; 
                        case uart_rx_data is
                            when x"00" => crc_extra <= x"32"; -- Heartbeat
                            when x"4D" => crc_extra <= x"8F"; -- Command Ack (77)
                            when others => crc_extra <= x"00";
                        end case;
                        if (length_id = 0) then
                            state <= APPLY_CRC_EXTRA;
                        else
                            state <= READ_PAYLOAD;
                        end if;

                    when READ_PAYLOAD =>
                        current_crc_reg <= crc_out;  
                        if (message_id = x"00") then
                            if (byte_count = 6) then temp_base_mode <= uart_rx_data; end if;
                        elsif (message_id = x"4D") then
                            if (byte_count = 2) then temp_ack_res <= uart_rx_data; end if;
                        end if;
                        if (byte_count = length_id - 1) then
                            state <= APPLY_CRC_EXTRA;
                        else
                            byte_count <= byte_count + 1;
                        end if;

                    when APPLY_CRC_EXTRA =>
                        current_crc_reg <= crc_out;
                        rx_crc_low <= uart_rx_data; -- this byte is CRC low
                        state <= READ_CRC;

                    when READ_CRC =>
                        if (rx_crc_low = current_crc_reg(7 downto 0)
                            and uart_rx_data = current_crc_reg(15 downto 8)) then
                            if (message_id = x"00" and rx_compid = x"01") then

                                heartbeat_pulse <= '1';
                                armed <= temp_base_mode(7);
                            elsif (message_id = x"4D") then
                                received <= '1';
                                result <= temp_ack_res;
                            end if;
                        end if;
                        state <= READ_START;
                end case;
            end if;
        end if;
    end process;

end behavioral;