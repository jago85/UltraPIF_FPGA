library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity SerialInterface is
port (
    CLK_I : in std_logic;
    RST_I : in std_logic;
    
    PIF_CLK_I : in std_logic;
    
    -- request next input data / ready for next data
    DATA_REQ_O : out std_logic;
    
    -- tx data input
    DATA_WR_I : in std_logic;
    DATA_I : in std_logic_vector(7 downto 0);
    
    -- rx data output
    DATA_VALID_O : out std_logic;
    DATA_O : out std_logic_vector(7 downto 0);
    
    -- tx / rx state machine busy
    BUSY_O : out std_logic;
    
    SERIAL_CLK2X_I : in std_logic;
    SERIAL_CLK_I : in std_logic;
    SERIAL_DAT_I : in std_logic;
    SERIAL_DAT_O : out std_logic
);
end entity SerialInterface;

architecture Behavioral of SerialInterface is
    
    type InterfaceState_t is (
        S_IDLE,
        S_SEND,
        S_RECEIVE_SYNC,
        S_RECEIVE_LOW,
        S_RECEIVE_HIGH
    );
    signal ifState : InterfaceState_t;
    
    attribute syn_encoding : string;
    attribute syn_encoding of ifState : signal is "onehot";
    
    signal hasNextData, hasNextData_ff, hasNextData_ff2 : std_logic;
    signal serialDat_in_ff, serialDat_in_ff2 : std_logic;
    signal ackNextData, ackNextData_ff, ackNextData_ff2, ackNextData_ff3 : std_logic;
    signal nextData : std_logic_vector(7 downto 0);
    
    signal shiftOut : std_logic_vector(7 downto 0);
    signal shiftIn : std_logic_vector(7 downto 0);
    signal shiftCnt : unsigned(3 downto 0);
    
    signal bitCnt : unsigned(11 downto 0);
    signal lowCnt : unsigned(7 downto 0);
    signal pulseWidth : unsigned(2 downto 0);
    
    signal serialDat_out : std_logic;
    
    signal dat_valid, dat_valid_ff, dat_valid_ff2, dat_valid_ff3 : std_logic;
    signal data_valid_out : std_logic;
    signal busy, busy_ff, busy_ff2 : std_logic;
    
    signal received_data : std_logic_vector(7 downto 0);
    
    signal serial_out_tick : std_logic;
    
begin
    
    -- send data with the falling edge of the serial clock
    serial_out_tick <= '1' when (SERIAL_CLK2X_I = '1') and (SERIAL_CLK_I = '1') else '0';
    
    serialDat_out <= '0' when (ifState = S_SEND) and (bitCnt(pulseWidth'range) < pulseWidth) else '1';
    
    SERIAL_DAT_O <= serialDat_out;
    DATA_REQ_O <= (not hasNextData) and (not DATA_WR_I);
    BUSY_O <= busy_ff2;
    
    DATA_VALID_O <= data_valid_out;
    DATA_O <= received_data;
    
    with shiftOut(shiftOut'left) select pulseWidth <=
        to_unsigned(6, pulseWidth'length) when '0',
        to_unsigned(2, pulseWidth'length) when others;
        
    process (CLK_I)
    begin
        if rising_edge(CLK_I) then
            if RST_I = '1' then
                hasNextData <= '0';
                ackNextData_ff <= '0';
                ackNextData_ff2 <= '0';
                ackNextData_ff3 <= '0';
                dat_valid_ff <= '0';
                dat_valid_ff2 <= '0';
                dat_valid_ff3 <= '0';
                busy_ff <= '0';
                busy_ff2 <= '0';
                data_valid_out <= '0';
            else
                -- synchronize ackNextData
                ackNextData_ff <= ackNextData;
                ackNextData_ff2 <= ackNextData_ff;
                ackNextData_ff3 <= ackNextData_ff2;
                
                -- synchronize dat_valid
                dat_valid_ff <= dat_valid;
                dat_valid_ff2 <= dat_valid_ff;
                dat_valid_ff3 <= dat_valid_ff2;
                
                busy_ff <= busy;
                busy_ff2 <= busy_ff;
                
                data_valid_out <= '0';
                
                if (hasNextData = '0') and (DATA_WR_I = '1') then
                    nextData <= DATA_I;
                    hasNextData <= '1';
                elsif (ackNextData_ff3 = '1') and (ackNextData_ff2 = '0') then
                    hasNextData <= '0';
                end if;
                
                if (dat_valid_ff3 = '0') and (dat_valid_ff2 = '1') then
                    received_data <= shiftIn(7 downto 0);
                    data_valid_out <= '1';
                end if;
                
            end if;
        end if;
    end process;
    
    busy <= '1' when (ifState /= S_IDLE) else '0';
    
    -- process in PIF clock domain
    -- RST_I is used asynchronous
    -- FIXME: the design should not use synchronous and asynchronous resets at the same time
    process (PIF_CLK_I, RST_I)
    begin
        if RST_I = '1' then
            ifState <= S_IDLE;
            ackNextData <= '0';
            bitCnt <= (others => '0');
            dat_valid <= '0';
            hasNextData_ff <= '0';
            hasNextData_ff2 <= '0';
            serialDat_in_ff <= '0';
            serialDat_in_ff2 <= '0';
        elsif rising_edge(PIF_CLK_I) then
            hasNextData_ff <= hasNextData;
            hasNextData_ff2 <= hasNextData_ff;
            
            serialDat_in_ff <= SERIAL_DAT_I;
            serialDat_in_ff2 <= serialDat_in_ff;
            
            ackNextData <= '0';
            dat_valid <= '0';
            
            bitCnt <= bitCnt + 1;
            
            case ifState is
            when S_IDLE =>
                -- sending data is synchronous to the falling edge of the serial clock
                -- data is received by counting the clocks while the signal is low and high
                -- the value of the received bit is determined by comparing the low and high
                -- times on the falling edge of the next data bit or stop bit
                if serial_out_tick = '1' then 
                    if hasNextData_ff2 = '1' then
                        shiftOut <= nextData;
                        ifState <= S_SEND;
                        shiftCnt <= to_unsigned(8, shiftCnt'length);
                        bitCnt <= (others => '0');
                        ackNextData <= '1';
                    end if;
                end if;
                
            when S_SEND =>
                if serial_out_tick = '1' then
                    if bitCnt = 7 then
                        bitCnt <= (others => '0');
                        shiftOut <= shiftOut(shiftOut'length - 2 downto 0) & "1";
                        shiftCnt <= shiftCnt - 1;
                        if shiftCnt = 1 then
                            if hasNextData_ff2 = '1' then
                                shiftOut <= nextData;
                                ackNextData <= '1';
                                shiftCnt <= to_unsigned(8, shiftCnt'length);
                            end if;
                        end if;
                    end if;
                    
                    -- the controller starts to send without awaiting the end of the stop bit (high phase)
                    -- so we start listening for the response right after the low phase of the stop bit
                    if (shiftCnt = 0) and (bitCnt = 2) then
                        ifState <= S_RECEIVE_SYNC;
                        bitCnt <= (others => '0');
                        shiftCnt <= to_unsigned(7, shiftCnt'length);
                    end if;
                else
                    -- don't count
                    bitCnt <= bitCnt;
                end if;
            
            -- sync to the falling edge of the data line
            -- if timeout occurs go to idle
            when S_RECEIVE_SYNC =>
                if serialDat_in_ff2 = '0' then
                    ifState <= S_RECEIVE_LOW;
                    bitCnt <= (others => '0');
                elsif bitCnt = 1560 then -- initial timeout 100 us
                    -- timeout
                    ifState <= S_IDLE;
                end if;
            
            -- count the low time of the signal
            -- no timeout - the signal is expected to come high again
            when S_RECEIVE_LOW =>
                if serialDat_in_ff2 = '1' then
                    lowCnt <= bitCnt(lowCnt'length - 1 downto 0);
                    bitCnt <= (others => '0');
                    ifState <= S_RECEIVE_HIGH;
                end if;
            
            -- count the high time of the signal
            -- if timeout occurs go to idle
            -- shift a bit into the input register if there is a new bit
            -- pulse dat_valid if an input byte is complete
            when S_RECEIVE_HIGH =>
                if bitCnt = 171 then -- timeout for next bit 11 us
                    -- timeout
                    ifState <= S_IDLE;
                elsif serialDat_in_ff2 = '0' then
                    if lowCnt > bitCnt then
                        shiftIn <= shiftIn(shiftIn'length - 2 downto 0) & '0';
                    else
                        shiftIn <= shiftIn(shiftIn'length - 2 downto 0) & '1';
                    end if;
                    shiftCnt <= shiftCnt - 1;
                    if shiftCnt = 0 then -- RX byte complete
                        shiftCnt <= to_unsigned(7, shiftCnt'length);
                        dat_valid <= '1';
                    end if;
                    
                    ifState <= S_RECEIVE_LOW;
                    bitCnt <= (others => '0');
                end if;
                
            when others =>  ifState <= S_IDLE;
            end case;

        end if;
    end process;
    
end Behavioral;
