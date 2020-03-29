library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity bram8_to_32 is
port (
        Clock_I : in  std_logic; 
        Reset_I : in std_logic;
        
        DataIn_I : in  std_logic_vector(31 downto 0);
        En_I : in  std_logic;
        Wr_I : in  std_logic;
        Q_O : out  std_logic_vector(31 downto 0);
        Ready_O : out std_logic;
        
        DataIn_O : out  std_logic_vector(7 downto 0);
        Address_O : out  std_logic_vector(1 downto 0);
        Wr_O : out  std_logic;
        Q_I : in  std_logic_vector(7 downto 0)
        );
end bram8_to_32;

architecture Behavioral of bram8_to_32 is

    type RamState_t is (
        stIdle,
        stReading,
        stWriting
    );
    
    signal ramState : RamState_t;
    signal bram_addr : std_logic_vector(1 downto 0);
    signal bram_wr_dat : std_logic_vector(31 downto 0);
    signal bram_rd_dat : std_logic_vector(31 downto 0);
    signal read_ctrl : std_logic_vector(4 downto 0);
    
    attribute syn_encoding : string;
    attribute syn_encoding of ramState : signal is "onehot";
    
begin
    
    Ready_O <= '1' when (Reset_I = '0') and (ramState = stIdle) else '0';
    
    Address_O <= bram_addr;
    with bram_addr(1 downto 0) select DataIn_O <= 
        bram_wr_dat(31 downto 24) when "00",
        bram_wr_dat(23 downto 16) when "01",
        bram_wr_dat(15 downto 8) when "10",
        bram_wr_dat(7 downto 0) when others;
    Q_O <= bram_rd_dat;
    
    process (Clock_I)
    begin
        if rising_edge(Clock_I) then
            
        
            if Reset_I = '1' then
                ramState <= stIdle;
                bram_addr <= (others => '0');
                Wr_O <= '0';
                bram_wr_dat <= (others => '0');
                bram_rd_dat <= (others => '0');
            else
                Wr_O <= '0';
                read_ctrl <= "0" & read_ctrl(read_ctrl'length - 1 downto 1);
                case ramState is
                when stIdle =>
                    if En_I = '1' then
                        bram_addr <= "00";
                        if Wr_I = '1' then
                            bram_wr_dat <= DataIn_I;
                            ramState <= stWriting;
                            Wr_O <= '1';
                        else
                            read_ctrl(read_ctrl'length - 1) <= '1';
                            ramState <= stReading;
                        end if;
                    end if;
                    
                when stReading =>
                    bram_addr <= std_logic_vector(unsigned(bram_addr) + 1);
                    case read_ctrl(3 downto 0) is
                        when "1000" => bram_rd_dat(31 downto 24) <= Q_I;
                        when "0100" => bram_rd_dat(23 downto 16) <= Q_I;
                        when "0010" => bram_rd_dat(15 downto 8) <= Q_I;
                        when "0001" => bram_rd_dat(7 downto 0) <= Q_I;
                        when others => null;
                    end case;
                    --if read_ctrl = (read_ctrl'range => '0') then
                    if read_ctrl(0) = '1' then
                        ramState <= stIdle;
                    end if;
                    
                when stWriting =>
                    Wr_O <= '1';
                    bram_addr <= std_logic_vector(unsigned(bram_addr) + 1);
                    if bram_addr = "11" then
                        ramState <= stIdle;
                        bram_wr_dat <= (others => '0');
                        Wr_O <= '0';
                    end if;
                    
                when others => ramState <= stIdle;
                end case;
            end if;
        end if;
    end process;

end Behavioral;
