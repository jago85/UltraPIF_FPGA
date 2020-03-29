-- Module ResetGen
-- 
-- Generate a reset pulse.
-- 
library ieee;
use ieee.std_logic_1164.all;

library lattice;
use lattice.all;

entity ResetGen is
port (
    CLK_I : in STD_LOGIC;
    HOLD_NRST_I : in STD_LOGIC;
    NRST_O : out STD_LOGIC
);
end ResetGen;

architecture Behavioral of ResetGen is

    component FD1S1A
    generic (GSR : STRING := "ENABLED");
    port (
        CK : in STD_LOGIC;
        D : in STD_LOGIC;
        Q : out STD_LOGIC
    );
    end component;
    
    -- signal rst : std_logic := '0';
    -- signal rst_done : std_logic := '0';
    
    signal nhold : std_logic := '0';

begin
    
    nhold <= not HOLD_NRST_I;
    
    FD1P3AX_Inst : FD1S1A
    port map (
        CK => CLK_I,
        D => nhold,
        Q => NRST_O
    );
    
    
    -- NRST_O <= rst;
    
    -- rst_proc: process (CLK_I)
    -- begin
        -- if rising_edge(CLK_I) then
            -- if HOLD_NRST_I = '1' then
                -- rst <= '0';
                -- rst_done <= '0';
            -- else
                -- if (rst = '0') and (rst_done = '0') then
                    -- rst <= '0';
                    -- rst_done <= '1';
                -- else
                    -- rst <= '1';
                    -- rst_done <= '0';
                -- end if;
            -- end if;
        -- end if;
    -- end process rst_proc;

end Behavioral;
