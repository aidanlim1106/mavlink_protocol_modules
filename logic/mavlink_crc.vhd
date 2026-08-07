library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity mavlink_crc is 
	port(
		din : in std_logic_vector(7 downto 0);
		crc_in : in std_logic_vector(15 downto 0);
		crc_out : out std_logic_vector(15 downto 0)
	);
end mavlink_crc;

architecture behavioral of mavlink_crc is
begin
	
	
	crc_acc_proc : process(din, crc_in) 
		variable temp : std_logic_vector(7 downto 0);
	begin
		temp := din XOR crc_in(7 downto 0);
		temp := temp XOR (temp(3 downto 0) & "0000");
		
        -- *crcAccum = (*crcAccum >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4);
        crc_out <= 
            ("00000000" & crc_in(15 downto 8)) 
            XOR 
            (temp & "00000000") 
            XOR 
            ("00000" & temp & "000") 
            XOR 
            ("000000000000" & temp(7 downto 4));
            
    end process;
	
end behavioral;