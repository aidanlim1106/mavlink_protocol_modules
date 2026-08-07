library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity tb_mavlink_tx is
-- Testbenches have no ports
end tb_mavlink_tx;

architecture behavior of tb_mavlink_tx is

    -- Signals to connect to the DUT
    signal clk               : std_logic := '0';
    signal rst               : std_logic := '1';
    
    -- Command Triggers
    signal out_heartbeat     : std_logic := '0';
    signal out_arm           : std_logic := '0';
    signal out_disarm        : std_logic := '0';
    signal pot_mode          : std_logic := '0';
    signal rc_override       : std_logic := '0';
    signal steering_pwm      : std_logic_vector(15 downto 0) := x"05DC"; -- 1500
    signal throttle_pwm      : std_logic_vector(15 downto 0) := x"0640"; -- 1600
    
    -- Mock UART Signals
    signal uart_tx_accept    : std_logic := '0';
    signal uart_tx_empty     : std_logic := '1';
    
    -- Outputs from DUT
    signal uart_tx_data      : std_logic_vector(7 downto 0);
    signal uart_tx_rdy       : std_logic;
    signal busy              : std_logic;

    constant CLK_PERIOD : time := 10 ns;

begin

    -- Instantiate the Device Under Test (DUT)
    DUT : entity work.mavlink_tx
    port map (
        clk             => clk,
        rst             => rst,
        out_heartbeat   => out_heartbeat,
        out_arm         => out_arm,
        out_disarm      => out_disarm,
        pot_mode        => pot_mode,
        rc_override     => rc_override,
        steering_pwm    => steering_pwm,
        throttle_pwm    => throttle_pwm,
        uart_tx_accept  => uart_tx_accept,
        uart_tx_empty   => uart_tx_empty,
        uart_tx_data    => uart_tx_data,
        uart_tx_rdy     => uart_tx_rdy,
        busy            => busy
    );

    -- Clock Generation
    clk_process : process
    begin
        clk <= '0';
        wait for CLK_PERIOD/2;
        clk <= '1';
        wait for CLK_PERIOD/2;
    end process;

    -- MOCK UART RECEIVER PROCESS
    -- This simulates the hardware UART accepting bytes
    mock_uart_proc : process
    begin
        wait until rising_edge(clk);
        uart_tx_accept <= '0';
        
        -- If the TX module says it has a byte ready to send
        if uart_tx_rdy = '1' then
            -- Wait a couple clock cycles to simulate UART latency
            wait for CLK_PERIOD * 3;
            uart_tx_accept <= '1'; -- Tell the FSM we accepted it
            wait for CLK_PERIOD;
            uart_tx_accept <= '0';
        end if;
    end process;

    -- Main Stimulus Process
    stim_proc: process
    begin
        -- 1. Initialize and Reset
        wait for 100 ns;
        rst <= '0';
        wait for 100 ns;

        -- --------------------------------------------------------
        -- TEST 1: SEND A HEARTBEAT
        -- --------------------------------------------------------
        report "--- Triggering Heartbeat ---";
        out_heartbeat <= '1';
        wait for CLK_PERIOD;
        out_heartbeat <= '0';
        
        -- Wait until the FSM finishes sending the whole packet
        wait until busy = '0';
        wait for 500 ns;

        -- --------------------------------------------------------
        -- TEST 2: SEND RC OVERRIDE (Drive Commands)
        -- --------------------------------------------------------
        report "--- Triggering RC Override ---";
        -- Change steering to 1900 (0x076C)
        steering_pwm <= x"076C";
        
        rc_override <= '1';
        wait for CLK_PERIOD;
        rc_override <= '0';
        
        wait until busy = '0';
        wait for 500 ns;
        
        report "--- SIMULATION COMPLETE ---";
        wait;
    end process;

end behavior;