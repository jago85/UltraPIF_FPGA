library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity CicInterface is
port (
    CLK_I : in std_logic;
    RST_I : in std_logic;
    START_I : in std_logic;
    DATA_I : in std_logic;
    DATA_O : out std_logic;
    BUSY_O : out std_logic;
    
    CIC_DCLK_O : out std_logic;
    CIC_DATA_I : in std_logic;
    CIC_DATA_O : out std_logic
);
end entity CicInterface;

architecture Behavioral of CicInterface is

    constant LOW_TIME : unsigned(11 downto 0) := to_unsigned(1064, 12);
    constant HIGH_TIME : unsigned(11 downto 0) := to_unsigned(1330, 12);
    
    signal bitTime : unsigned(11 downto 0);
    
    signal ifState : std_logic;
    
    signal data_in : std_logic;
    
    signal cic_dclk : std_logic;

begin

DATA_O <= data_in;
CIC_DCLK_O <= cic_dclk;

BUSY_O <= '1' when (ifState = '1') or (START_I = '1') else '0';

process (CLK_I)
begin
    if rising_edge(CLK_I) then
    
        if RST_I = '1' then
            ifState <= '0';
            CIC_DATA_O <= '1';
            cic_dclk <= '1';
        else
            
            if ifState = '0' then
                CIC_DATA_O <= '1';
                cic_dclk <= '1';
                if START_I = '1' then 
                    CIC_DATA_O <= DATA_I;
                    bitTime <= LOW_TIME + HIGH_TIME - 1;
                    ifState <= '1';
                    cic_dclk <= '0';
                end if;
            else
                bitTime <= bitTime - 1;
                if bitTime = HIGH_TIME then
                    cic_dclk <= '1';
                    CIC_DATA_O <= '1';
                    data_in <= CIC_DATA_I;
                end if;
                if bitTime = 0 then
                    ifState <= '0';
                end if;
            end if;
        end if;
        
    end if;
end process;


end Behavioral;
