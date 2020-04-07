library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

library lattice;
use lattice.all;
-- library machxo2;
-- use machxo2.all;

entity top is
    Port ( -- PIF
           PIF_CLK_I : in STD_LOGIC;
           PIF_ADR_I : in STD_LOGIC;
           PIF_DAT_O : out STD_LOGIC;
           PIF_EN_I : in STD_LOGIC;
           
		   COLD_RESET_O : out STD_LOGIC;
		   NMI_CPU_O : out STD_LOGIC;
		   INT_CPU_O : out STD_LOGIC;
		   
		   RESET_BTN_I : in STD_LOGIC;
           RESET_BTN_O : out STD_LOGIC;
		   
		   CTRL_O : out STD_LOGIC_VECTOR(3 downto 0);
		   CTRL_I : in STD_LOGIC_VECTOR(3 downto 0);
		   EEP_DAT_IO : inout STD_LOGIC;
           EEP_CIC_CLK_O : out STD_LOGIC;
           CIC_DATA_CLK_O : out STD_LOGIC;
           CIC_DATA_IO : inout STD_LOGIC;
		   RC_POR_I : in STD_LOGIC;
		   
           -- IO
           SPI_NSS_I : in STD_LOGIC;
           SPI_SCK_I : inout STD_LOGIC;
           SPI_MOSI_I : inout STD_LOGIC;
           SPI_MISO_O : inout STD_LOGIC;
           NIRQ_O : out STD_LOGIC;
           
           SCL_IO : inout STD_LOGIC;
           SDA_IO : inout STD_LOGIC;
           
           JUMPER_I : in STD_LOGIC_VECTOR(3 downto 0);
           
           CTRL0_AUX_O : out std_logic;
           
           -- LED
		   RED_O : out STD_LOGIC;
		   GREEN_O : out STD_LOGIC;
		   BLUE_O : out STD_LOGIC;
           LED_O : out STD_LOGIC_VECTOR (1 downto 0));
end top;

architecture Behavioral of top is

component ResetGen
port (
    CLK_I : in STD_LOGIC;
    HOLD_NRST_I : in STD_LOGIC;
    NRST_O : out STD_LOGIC
);
end component;

component pif_ram
    port (
        DataInA: in  std_logic_vector(7 downto 0); 
        DataInB: in  std_logic_vector(7 downto 0); 
        AddressA: in  std_logic_vector(10 downto 0); 
        AddressB: in  std_logic_vector(10 downto 0); 
        ClockA: in  std_logic; 
        ClockB: in  std_logic; 
        ClockEnA: in  std_logic; 
        ClockEnB: in  std_logic; 
        WrA: in  std_logic; 
        WrB: in  std_logic; 
        ResetA: in  std_logic; 
        ResetB: in  std_logic; 
        QA: out  std_logic_vector(7 downto 0); 
        QB: out  std_logic_vector(7 downto 0));
end component;

component bram8_to_32
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
end component;

component dma_mem
    port (DataInA: in  std_logic_vector(7 downto 0); 
        DataInB: in  std_logic_vector(7 downto 0); 
        AddressA: in  std_logic_vector(9 downto 0); 
        AddressB: in  std_logic_vector(9 downto 0); 
        ClockA: in  std_logic; ClockB: in  std_logic; 
        ClockEnA: in  std_logic; ClockEnB: in  std_logic; 
        WrA: in  std_logic; WrB: in  std_logic; ResetA: in  std_logic; 
        ResetB: in  std_logic; QA: out  std_logic_vector(7 downto 0); 
        QB: out  std_logic_vector(7 downto 0));
end component;

COMPONENT OSCH
GENERIC  (NOM_FREQ: string :=  "2.08");
PORT (STDBY   : IN  std_logic;
      OSC     : OUT std_logic;
      SEDSTDBY: OUT std_logic);
END COMPONENT;

component efb_core
    port (
        wb_clk_i: in  std_logic;
        wb_rst_i: in  std_logic; 
        wb_cyc_i: in  std_logic;
        wb_stb_i: in  std_logic; 
        wb_we_i: in  std_logic; 
        wb_adr_i: in  std_logic_vector(7 downto 0); 
        wb_dat_i: in  std_logic_vector(7 downto 0); 
        wb_dat_o: out  std_logic_vector(7 downto 0); 
        wb_ack_o: out  std_logic; 
        i2c2_scl: inout  std_logic; 
        i2c2_sda: inout  std_logic;
        i2c2_irqo: out  std_logic; 
        spi_clk: inout  std_logic; 
        spi_miso: inout  std_logic;
        spi_mosi: inout  std_logic; 
        spi_scsn: in  std_logic);
end component;

component SerialInterface
    Port ( 
        CLK_I : in std_logic;
        RST_I : in std_logic;
        PIF_CLK_I : in std_logic;
        DATA_REQ_O : out std_logic;
        DATA_WR_I : in std_logic;
        DATA_I : in std_logic_vector(7 downto 0);
        DATA_VALID_O : out std_logic;
        DATA_O : out std_logic_vector(7 downto 0);
        BUSY_O : out std_logic;
        
        SERIAL_CLK2X_I : in std_logic;
        SERIAL_CLK_I : in std_logic;
        SERIAL_DAT_I : in std_logic;
        SERIAL_DAT_O : out std_logic);
end component SerialInterface;

component CicInterface
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
end component CicInterface;

component prio_arbiter_single_slave
generic (
    NUM_MASTERS : natural := 4;
    ADR_WIDTH   : natural := 32;
    DATA_WIDTH  : natural := 32
);
port (
    CLK_I : in std_logic;
    RST_I : in std_logic;
    
    MSTR_CYC_I  : in  std_logic_vector(NUM_MASTERS - 1 downto 0);
    MSTR_STB_I  : in  std_logic_vector(NUM_MASTERS - 1 downto 0);
    MSTR_WE_I   : in  std_logic_vector(NUM_MASTERS - 1 downto 0);
    MSTR_ACK_O  : out std_logic_vector(NUM_MASTERS - 1 downto 0);
    MSTR_ADR_I  : in  std_logic_vector(NUM_MASTERS * ADR_WIDTH - 1 downto 0);
    MSTR_DAT_I  : in  std_logic_vector(NUM_MASTERS * DATA_WIDTH - 1 downto 0);
    
    SLV_CYC_O   : out std_logic;
    SLV_STB_O   : out std_logic;
    SLV_WE_O    : out std_logic;
    SLV_ACK_I   : in  std_logic;
    SLV_ADR_O   : out std_logic_vector(ADR_WIDTH - 1 downto 0);
    SLV_DAT_O   : out std_logic_vector(DATA_WIDTH - 1 downto 0)
    
);
end component;

constant FPGA_VERSION : std_logic_vector(31 downto 0) := x"01000400";

constant REG_SPICR1     : std_logic_vector(7 downto 0) := x"55";
constant REG_SPITXDR    : std_logic_vector(7 downto 0) := x"59";
constant REG_SPISR      : std_logic_vector(7 downto 0) := x"5A";
constant REG_SPIRXDR    : std_logic_vector(7 downto 0) := x"5B";

constant REGBIT_SPISR_RRDY : natural := 3;
constant REGBIT_SPISR_TRDY : natural := 4;

constant REG_I2C2CR     : std_logic_vector(7 downto 0) := x"4A";
constant REG_I2C2CMD    : std_logic_vector(7 downto 0) := x"4B";
constant REG_I2CBR0     : std_logic_vector(7 downto 0) := x"4C";
constant REG_I2C2BR1    : std_logic_vector(7 downto 0) := x"4D";
constant REG_I2C2TXDR   : std_logic_vector(7 downto 0) := x"4E";
constant REG_I2C2SR     : std_logic_vector(7 downto 0) := x"4F";
constant REG_I2C2GCDR   : std_logic_vector(7 downto 0) := x"50";
constant REG_I2C2RXDR   : std_logic_vector(7 downto 0) := x"51";
constant REG_I2C2IRQ    : std_logic_vector(7 downto 0) := x"52";
constant REG_I2C2IRQEN  : std_logic_vector(7 downto 0) := x"53";

constant REGBIT_SR_TRRDY : natural := 2;
constant REGBIT_SR_SRW   : natural := 4;
constant REGBIT_SR_BUSY  : natural := 6;

constant SPI_ADDR_INT               : std_logic_vector(4 downto 0) := "00000";
constant SPI_ADDR_PIFCMD            : std_logic_vector(4 downto 0) := "00001";
constant SPI_ADDR_GPIO              : std_logic_vector(4 downto 0) := "00010";
constant SPI_ADDR_RED               : std_logic_vector(4 downto 0) := "00011";
constant SPI_ADDR_GREEN             : std_logic_vector(4 downto 0) := "00100";
constant SPI_ADDR_BLUE              : std_logic_vector(4 downto 0) := "00101";
constant SPI_ADDR_I2C_SLAVEADDR     : std_logic_vector(4 downto 0) := "00110";
constant SPI_ADDR_SI_CHANNEL        : std_logic_vector(4 downto 0) := "00111";
constant SPI_ADDR_DMA_MEMADDR       : std_logic_vector(4 downto 0) := "01000";
constant SPI_ADDR_DMA_CNT           : std_logic_vector(4 downto 0) := "01001";
constant SPI_ADDR_DMA_CMD           : std_logic_vector(4 downto 0) := "01010";
constant SPI_ADDR_DMA_STAT          : std_logic_vector(4 downto 0) := "01011";
constant SPI_ADDR_PIFRAMBANK        : std_logic_vector(4 downto 0) := "01100";
constant SPI_ADDR_SI_RXCNT          : std_logic_vector(4 downto 0) := "01101";
constant SPI_ADDR_FPGA_VER0         : std_logic_vector(4 downto 0) := "01110";
constant SPI_ADDR_FPGA_VER1         : std_logic_vector(4 downto 0) := "01111";

constant DMA_CMD_I2C_WRITE          : std_logic_vector(7 downto 0) := x"01";
constant DMA_CMD_I2C_READ           : std_logic_vector(7 downto 0) := x"02";
constant DMA_CMD_SI_TRANSCEIVE      : std_logic_vector(7 downto 0) := x"04";
constant DMA_CMD_CIC_TRANSCEIVE     : std_logic_vector(7 downto 0) := x"08";

type pif_state_t is (
    S_PIF_WAIT_START,
    S_PIF_READ_COMMAND,
    S_PIF_WAIT_CLEARANCE,
    S_PIF_EXEC_COMMAND,
    S_PIF_WAIT_PIF_ACK,
    S_PIF_READ_PIF_DATA,
    S_PIF_WAIT_RCP_START,
    S_PIF_READ_RCP_DATA
);

type spi_state_t is (
    S_SPI_WAIT_CMD,
    S_SPI_SEND_DATA,
    S_SPI_RECEIVE_DATA,
    S_SPI_SEND_REGISTER,
    S_SPI_RECEIVE_REGISTER,
    S_SPI_RECEIVE_EXT_ADDR,
    S_SPI_RECEIVE_EXT_DATA,
    S_SPI_SEND_EXT_DATA,
    S_SPI_INVALID_CMD
);

type slave_state_t is (
    S_SLAVE_IDLE,
    S_SLAVE_WAIT_RX,
    S_SLAVE_READ_RX,
    S_SLAVE_WRITE_TX,
    S_SLAVE_WRITE_TX_DUMMY
);

type dma_state_t is (
    S_DMA_IDLE,
    S_DMA_I2C_SEND_ADDR_TXDR,
    S_DMA_I2C_SEND_ADDR_CMDR,
    S_DMA_I2C_WRITE_WAIT_TRRDY,
    S_DMA_I2C_SEND_DATA_TXDR,
    S_DMA_I2C_SEND_DATA_CMDR,
    S_DMA_I2C_SEND_DATA_STOP,
    S_DMA_I2C_READ_DATA_WAIT_SRW,
    S_DMA_I2C_READ_DATA_CMDR,
    S_DMA_I2C_READ_DATA_CHECKLAST,
    S_DMA_I2C_READ_DATA_DELAY,
    S_DMA_I2C_READ_DATA_WAIT_TRRDY,
    S_DMA_I2C_READ_DATA_READ_RXDR,
    S_DMA_I2C_READ_DATA_STOP_CMDR,
    S_DMA_I2C_DISABLE_CMDR,
    S_DMA_I2C_READ_DATA_WAIT_BUSY,
    S_DMA_SI_SEND_DATA,
    S_DMA_SI_RECEIVE_DATA,
    S_DMA_CIC_RECEIVE_DATA
);

function resize_slv(arg: in std_logic_vector; new_size : in NATURAL) return std_logic_vector is
begin
    return std_logic_vector(resize(unsigned(arg), new_size));
end resize_slv;

function inc_slv(arg: in std_logic_vector) return std_logic_vector is
begin
    return std_logic_vector(unsigned(arg) + 1);
end inc_slv;

function dec_slv(arg: in std_logic_vector) return std_logic_vector is
begin
    return std_logic_vector(unsigned(arg) - 1);
end dec_slv;

signal clk : std_logic;
signal rst : std_logic;

signal serial_clk : std_logic;
signal serial_clk2x : std_logic;
signal serial_clk_div : unsigned(1 downto 0);

signal pif_en_ff1, pif_en_ff2, pif_clk, pif_clk_d1, pif_adr, pif_adr_d1, pif_dat : std_logic := '0';
signal pif_dat_shift_out : std_logic := '0';

signal pifState : pif_state_t := S_PIF_WAIT_START;
signal pifCmd : std_logic_vector(1 downto 0) := (others => '0');
signal pifAddr : std_logic_vector(8 downto 0) := (others => '0');
signal pifData : std_logic_vector(31 downto 0) := (others => '0');
signal pifCmdStart : std_logic := '0';

signal shiftCnt : unsigned(8 downto 0) := (others => '0'); -- TODO: change to unsigned(4 downto 0) and test
signal shiftReg : std_logic_vector(32 downto 0) := (others => '0');
signal dataCnt : unsigned(5 downto 0) := (others => '0');

signal shiftOutReg : std_logic_vector(32 downto 0) := (others => '1');

signal spiState : spi_state_t := S_SPI_WAIT_CMD;
signal spi_di_req : std_logic;
signal spi_di : std_logic_vector(7 downto 0) := (others => '0');
signal spi_wren  : std_logic := '0';
signal spi_di_pending : std_logic;  -- don't write dummy bytes to TXDR if this signal is high
signal spi_do_valid : std_logic;
signal spi_do : std_logic_vector(7 downto 0);
signal spi_nss_ff, spi_nss_ff1 : std_logic := '1';

signal spiCmdRead : std_logic := '0';
signal spiAddr : std_logic_vector(4 downto 0) := (others => '0');
signal pifRamAddr : std_logic_vector(8 downto 0);

signal wea : std_logic := '0';
signal douta : std_logic_vector(31 downto 0);

signal ena : std_logic; 
signal rdya : std_logic;
signal pif_ram_DataInA, pif_ram_DataInB, pif_ram_QA, pif_ram_QB : std_logic_vector(7 downto 0);
signal pif_ram_AddressA, pif_ram_AddressB : std_logic_vector(10 downto 0);
signal pif_ram_AddressA_low : std_logic_vector(1 downto 0);
signal pif_ram_WrA, pif_ram_WrB : std_logic;
signal read_delay : std_logic;

signal dma_mem_DataInA : std_logic_vector(7 downto 0);
signal dma_mem_DataInB : std_logic_vector(7 downto 0);
signal dma_mem_QA : std_logic_vector(7 downto 0);
signal dma_mem_QB : std_logic_vector(7 downto 0);
signal dma_mem_AddressA : std_logic_vector(9 downto 0);
signal dma_mem_AddressB : std_logic_vector(9 downto 0);
signal dma_mem_WrA : std_logic;
signal dma_mem_WrB : std_logic;

signal spiByteSel : unsigned(1 downto 0) := (others => '0');

signal spiLowBuffer : std_logic_vector(7 downto 0) := (others => '0');
signal spiRegPifCommandAddr : std_logic_vector(10 downto 0) := (others => '0');
signal spiRegPifIrq : std_logic_vector(1 downto 0) := (others => '0');
-- 1: Write done
-- 0: Waiting for Clearance

signal spiRegGpio : std_logic_vector(15 downto 0) := (others => '0');
signal spiRegPifRamBank : std_logic_vector(4 downto 0) := (others => '0');

signal spiRegI2cSlaveAddr : std_logic_vector(6 downto 0) := (others => '0');
signal spiRegDmaMemPtr : std_logic_vector(9 downto 0) := (others => '0');
signal spiRegDmaCnt : std_logic_vector(9 downto 0) := (others => '0');
signal spiRegDmaCmd : std_logic_vector(7 downto 0) := (others => '0');
signal spiRegDmaStat : std_logic_vector(0 downto 0) := (others => '0');
signal spiRegSiChannel : std_logic_vector(2 downto 0) := (others => '0');

signal spiSelectedRegister : std_logic_vector(15 downto 0);
signal spiSelectedRegisterByte : std_logic_vector(7 downto 0);

-- wishbone
signal mstr_cyc     : std_logic_vector(1 downto 0);
signal mstr_stb     : std_logic_vector(1 downto 0);
signal mstr_we      : std_logic_vector(1 downto 0);
signal mstr_ack     : std_logic_vector(1 downto 0);
signal mstr_adr     : std_logic_vector(2 * 8 - 1 downto 0);
signal mstr_dat_in  : std_logic_vector(2 * 8 - 1 downto 0);

signal wb_rst : std_logic;
signal wb_cyc : std_logic;
signal wb_stb : std_logic;
signal wb_we : std_logic;
signal wb_adr : std_logic_vector(7 downto 0);
signal wb_dat_i : std_logic_vector(7 downto 0);
signal wb_dat_o : std_logic_vector(7 downto 0);
signal wb_ack : std_logic;

signal rst_btn_ff, rst_btn_ff2 : std_logic;
signal rc_por_ff, rc_por_ff2 : std_logic;

signal slaveState : slave_state_t;
signal spi_di_valid : std_logic;

signal slave_wb_cyc : std_logic;
signal slave_wb_stb : std_logic;
signal slave_wb_adr : std_logic_vector(7 downto 0);
signal slave_wb_dat_o : std_logic_vector(7 downto 0);
signal slave_wb_we : std_logic;
signal slave_wb_ack : std_logic;

signal dma_wb_cyc : std_logic;
signal dma_wb_stb : std_logic;
signal dma_wb_adr : std_logic_vector(7 downto 0);
signal dma_wb_dat_o : std_logic_vector(7 downto 0);
signal dma_wb_we : std_logic;
signal dma_wb_ack : std_logic;

signal rgb_int : std_logic_vector(2 downto 0);
signal counter : integer range 0 to 66500000 - 1 := 0;
signal div_rgb : integer range 0 to 132 := 0;
signal cnt_rgb : integer range 0 to 1023 := 0;

signal val_r : unsigned(9 downto 0) := (others => '0');
signal val_g : unsigned(9 downto 0) := (others => '0');
signal val_b : unsigned(9 downto 0) := (others => '0');

signal dmaState : dma_state_t := S_DMA_IDLE;
signal dmaWr : std_logic := '0';
signal dmaLastData : std_logic := '0';
signal dmaMemAddr : std_logic_vector(9 downto 0) := (others => '0');
signal dmaCnt : unsigned(10 downto 0) := (others => '0');

signal dmaSiRxCnt : unsigned(5 downto 0) := (others => '0');

attribute syn_encoding : string;
attribute syn_encoding of dmaState : signal is "onehot";

signal si_data_o : std_logic_vector(7 downto 0);
signal si_wr : std_logic;
signal si_req : std_logic;
signal si_valid : std_logic;
signal si_busy : std_logic;
signal si_serial_dat_i : std_logic;
signal si_serial_dat_o : std_logic;
signal eep_sdat_in : std_logic;
signal eep_sdat_out : std_logic;

signal cic_start : std_logic;
signal cic_busy : std_logic;
signal cic_data_o : std_logic;
signal cic_do : std_logic;
signal cic_di, cic_di_ff, cic_di_ff2 : std_logic;
signal cic_dclk : std_logic;

signal jumper_ff1, jumper_ff2 : std_logic_vector(3 downto 0);

begin

ResetGen_Inst : ResetGen
port map (
    CLK_I       => clk,
    HOLD_NRST_I => '0',
    NRST_O      => rst
);

PIF_DAT_O <= pif_dat when pif_en_ff2 = '1' else 'Z';

COLD_RESET_O <= spiRegGpio(0);
NMI_CPU_O <= not spiRegGpio(1);
INT_CPU_O <= not spiRegGpio(2);

CTRL_O(0) <= '0' when (spiRegSiChannel = "001") and (si_serial_dat_o = '0') else '1';
CTRL_O(1) <= '0' when (spiRegSiChannel = "010") and (si_serial_dat_o = '0') else '1';
CTRL_O(2) <= '0' when (spiRegSiChannel = "011") and (si_serial_dat_o = '0') else '1';
CTRL_O(3) <= '0' when (spiRegSiChannel = "100") and (si_serial_dat_o = '0') else '1';
eep_sdat_out <= '0' when (spiRegSiChannel = "101") and (si_serial_dat_o = '0') else '1';
CTRL0_AUX_O <= CTRL_I(0);

EEP_DAT_IO <= '0' when eep_sdat_out = '0' else 'Z';
eep_sdat_in <= EEP_DAT_IO;

EEP_CIC_CLK_O <= serial_clk;

CIC_DATA_CLK_O <= '0' when (cic_dclk = '0') or (spiRegGpio(3) = '0') else '1';
CIC_DATA_IO <= '0' when (cic_do = '0') or (spiRegGpio(4) = '0') else 'Z';
cic_di <= CIC_DATA_IO;

RED_O <= rgb_int(2);
GREEN_O <= rgb_int(1);
BLUE_O <= rgb_int(0);

OSCInst0: OSCH
GENERIC MAP (NOM_FREQ => "66.5")
PORT MAP (STDBY => '0',
          OSC   => clk,
          SEDSTDBY => open
);

wb_rst <= not rst;

efb_core_Inst : efb_core
    port map (
        wb_clk_i => clk,
        wb_rst_i => wb_rst,
        wb_cyc_i => wb_cyc,
        wb_stb_i => wb_stb, 
        wb_we_i => wb_we,
        wb_adr_i(7 downto 0) => wb_adr,
        wb_dat_i(7 downto 0) => wb_dat_o, 
        wb_dat_o(7 downto 0) => wb_dat_i,
        wb_ack_o => wb_ack,
        i2c2_scl => SCL_IO,
        i2c2_sda => SDA_IO,
        i2c2_irqo => open,
        spi_clk => SPI_SCK_I,
        spi_miso => SPI_MISO_O, 
        spi_mosi => SPI_MOSI_I,
        spi_scsn => SPI_NSS_I
);
    
    mstr_cyc  <= slave_wb_cyc & dma_wb_cyc;
    mstr_stb  <= slave_wb_stb & dma_wb_stb;
    mstr_we   <= slave_wb_we & dma_wb_we;
    mstr_adr  <= slave_wb_adr & dma_wb_adr;
    mstr_dat_in <= slave_wb_dat_o & dma_wb_dat_o;
    
    slave_wb_ack <= mstr_ack(1);
    dma_wb_ack <= mstr_ack(0);

    arbiter_inst : prio_arbiter_single_slave
    generic map (
        NUM_MASTERS => 2,
        ADR_WIDTH => 8,
        DATA_WIDTH => 8
    )
    port map (
        CLK_I       => clk,
        RST_I       => wb_rst,
        
        MSTR_CYC_I  => mstr_cyc,
        MSTR_STB_I  => mstr_stb,
        MSTR_WE_I   => mstr_we,
        MSTR_ACK_O  => mstr_ack,
        MSTR_ADR_I  => mstr_adr,
        MSTR_DAT_I  => mstr_dat_in,
        
        SLV_CYC_O   => wb_cyc,
        SLV_STB_O   => wb_stb,
        SLV_WE_O    => wb_we,
        SLV_ACK_I   => wb_ack,
        SLV_ADR_O   => wb_adr,
        SLV_DAT_O   => wb_dat_o
    );

   pif_ram_Inst : pif_ram
      port map (
                DataInA  => pif_ram_DataInA,
                DataInB  => pif_ram_DataInB,
                AddressA => pif_ram_AddressA,
                AddressB => pif_ram_AddressB,
                ClockA   => clk,
                ClockB   => clk,
                ClockEnA => '1',
                ClockEnB => '1',
                WrA      => pif_ram_WrA,
                WrB      => pif_ram_WrB,
                ResetA   => wb_rst,
                ResetB   => wb_rst,
                QA       => pif_ram_QA,
                QB       => pif_ram_QB
   );
   
   pif_ram_AddressA <= pifRamAddr & pif_ram_AddressA_low;
   
   bram8_to_32_InstA : bram8_to_32
    port map (
            Clock_I => clk,
            Reset_I => wb_rst,
            
            DataIn_I => pifData,
            En_I => ena,
            Wr_I => wea,
            Q_O => douta,
            Ready_O => rdya,
            
            DataIn_O => pif_ram_DataInA,
            Address_O => pif_ram_AddressA_low,
            Wr_O => pif_ram_WrA,
            Q_I => pif_ram_QA
    );
    
    dma_mem_Inst : dma_mem
        port map (
            DataInA(7 downto 0) => dma_mem_DataInA,
            DataInB(7 downto 0) => dma_mem_DataInB,
            AddressA(9 downto 0) => dma_mem_AddressA,
            AddressB(9 downto 0) => dma_mem_AddressB,
            ClockA => clk,
            ClockB => clk,
            ClockEnA => '1',
            ClockEnB => '1',
            WrA => dma_mem_WrA,
            WrB => dma_mem_WrB,
            ResetA => wb_rst,
            ResetB => wb_rst,
            QA(7 downto 0) => dma_mem_QA,
            QB(7 downto 0) => dma_mem_QB
        );
        
   SerialInterface_Inst : SerialInterface
      port map (
        CLK_I => clk,
        RST_I => wb_rst,
        PIF_CLK_I => PIF_CLK_I,
        DATA_WR_I => si_wr,
        DATA_REQ_O => si_req,
        DATA_I => dma_mem_QA,
        DATA_VALID_O => si_valid,
        DATA_O => si_data_o,
        BUSY_O => si_busy,
        
        SERIAL_CLK2X_I => serial_clk2x,
        SERIAL_CLK_I => serial_clk,
        SERIAL_DAT_I  => si_serial_dat_i,
        SERIAL_DAT_O  => si_serial_dat_o
   );
   
    CicInterface_Inst : CicInterface
    port map (
        CLK_I => clk,
        RST_I => wb_rst,
        START_I => cic_start,
        DATA_I => dma_mem_QA(0),
        DATA_O => cic_data_o,
        BUSY_O => cic_busy,
        
        CIC_DCLK_O => cic_dclk,
        CIC_DATA_I => cic_di_ff2,
        CIC_DATA_O => cic_do
    );
   
with spiRegSiChannel select si_serial_dat_i <=
    CTRL_I(0) when "001",
    CTRL_I(1) when "010",
    CTRL_I(2) when "011",
    CTRL_I(3) when "100",
    eep_sdat_in when "101",
    '1' when others;

spi_di_req <= '1' when (spi_di_valid = '0') and (spi_wren = '0') else '0';
spi_do_valid <= '1' when (slaveState = S_SLAVE_READ_RX) and (slave_wb_ack = '1') else '0';
spi_do <= wb_dat_i;

with spiState select spi_di_pending <= 
    '1' when S_SPI_SEND_DATA,
    '1' when S_SPI_SEND_EXT_DATA,
    '1' when S_SPI_SEND_REGISTER,
    '0' when others;

spi_proc : process (clk)
begin
    if rising_edge(clk) then
        slave_wb_cyc <= '0';
        slave_wb_stb <= '0';
        slave_wb_we <= '0';
        slave_wb_adr <= x"00";
        slave_wb_dat_o <= x"00";
        
        if (rst = '0') then
            slaveState <= S_SLAVE_IDLE;
            spi_di_valid <= '0';
            spi_nss_ff <= '1';
            spi_nss_ff1 <= '1';
        else
            spi_nss_ff <= SPI_NSS_I;
            spi_nss_ff1 <= spi_nss_ff;
            
            if spi_wren = '1' then
                spi_di_valid <= '1';
            end if;
            
            case slaveState is
            when S_SLAVE_IDLE =>
                spi_di_valid <= '0';
                if spi_nss_ff1 = '0' then
                    slaveState <= S_SLAVE_WRITE_TX_DUMMY;
                end if;
            
            when S_SLAVE_WAIT_RX =>
                slave_wb_cyc <= '1';
                slave_wb_stb <= '1';
                slave_wb_adr <= REG_SPISR;
                if slave_wb_ack = '1' then
                    slave_wb_cyc <= '0';
                    slave_wb_stb <= '0';
                    if wb_dat_i(REGBIT_SPISR_RRDY) = '1' then -- RRDY-Flag
                        slaveState <= S_SLAVE_READ_RX;
                    elsif spi_nss_ff1 = '1' then
                        slaveState <= S_SLAVE_IDLE;
                    elsif wb_dat_i(REGBIT_SPISR_TRDY) = '1' then -- TRDY-Flag
                        if spi_di_valid = '1' then
                            slaveState <= S_SLAVE_WRITE_TX;
                        elsif spi_di_pending = '0' then
                            slaveState <= S_SLAVE_WRITE_TX_DUMMY;
                        end if;
                    end if;
                end if;
            
            when S_SLAVE_READ_RX =>
                slave_wb_cyc <= '1';
                slave_wb_stb <= '1';
                slave_wb_adr <= REG_SPIRXDR;
                if slave_wb_ack = '1' then
                    slave_wb_cyc <= '0';
                    slave_wb_stb <= '0';
                    slaveState <= S_SLAVE_WAIT_RX;
                end if;
            
            when S_SLAVE_WRITE_TX =>
                slave_wb_cyc <= '1';
                slave_wb_stb <= '1';
                slave_wb_we <= '1';
                slave_wb_adr <= REG_SPITXDR;
                slave_wb_dat_o <= spi_di;
                if slave_wb_ack = '1' then
                    slave_wb_cyc <= '0';
                    slave_wb_stb <= '0';
                    slave_wb_we <= '0';
                    spi_di_valid <= '0';
                    slaveState <= S_SLAVE_WAIT_RX;
                end if;
                
            when S_SLAVE_WRITE_TX_DUMMY =>
                slave_wb_cyc <= '1';
                slave_wb_stb <= '1';
                slave_wb_we <= '1';
                slave_wb_adr <= REG_SPITXDR;
                slave_wb_dat_o <= x"00";
                if slave_wb_ack = '1' then
                    slave_wb_cyc <= '0';
                    slave_wb_stb <= '0';
                    slave_wb_we <= '0';
                    slaveState <= S_SLAVE_WAIT_RX;
                end if;
            
            when others => slaveState <= S_SLAVE_IDLE;
            end case;
        end if;
    end if;
end process spi_proc;

dma_proc : process (clk)
begin
    if rising_edge(clk) then
        if rst = '0' then
            dmaState <= S_DMA_IDLE;
            dma_wb_we <= '0';
            dma_wb_cyc <= '0';
            dma_wb_stb <= '0';
            dma_wb_dat_o <= x"00";
            dma_mem_WrA <= '0';
            si_wr <= '0';
            cic_start <= '0';
        else
            dma_wb_we <= '0';
            dma_wb_cyc <= '0';
            dma_wb_stb <= '0';
            dma_wb_dat_o <= x"00";
            dma_mem_WrA <= '0';
            si_wr <= '0';
            cic_start <= '0';
            
            case dmaState is
            when S_DMA_IDLE =>
                dmaLastData <= '0';
                dmaCnt <= resize(unsigned(spiRegDmaCnt), dmaCnt'length);
                dma_mem_AddressA <= spiRegDmaMemPtr(9 downto 0);
                dmaWr <= '0';
                case spiRegDmaCmd is
                when DMA_CMD_I2C_WRITE =>
                    dmaState <= S_DMA_I2C_SEND_ADDR_TXDR;
                    
                when DMA_CMD_I2C_READ =>
                    dmaState <= S_DMA_I2C_SEND_ADDR_TXDR;
                    dmaWr <= '1';
                    
                when DMA_CMD_SI_TRANSCEIVE =>
                    dmaState <= S_DMA_SI_SEND_DATA;
                    dmaSiRxCnt <= (others => '0');
                    
                when DMA_CMD_CIC_TRANSCEIVE =>
                    dmaState <= S_DMA_CIC_RECEIVE_DATA;
                    
                when others => dmaState <= S_DMA_IDLE;
                end case;
                
            when S_DMA_I2C_SEND_ADDR_TXDR =>
                dma_wb_cyc <= '1';
                dma_wb_stb <= '1';
                dma_wb_we <= '1';
                dma_wb_adr <= REG_I2C2TXDR;
                dma_wb_dat_o <= spiRegI2cSlaveAddr & dmaWr;
                if dma_wb_ack = '1' then
                    dma_wb_cyc <= '0';
                    dma_wb_stb <= '0';
                    dma_wb_we <= '0';
                    dmaState <= S_DMA_I2C_SEND_ADDR_CMDR;
                end if;
            
            when S_DMA_I2C_SEND_ADDR_CMDR =>
                dma_wb_cyc <= '1';
                dma_wb_stb <= '1';
                dma_wb_we <= '1';
                dma_wb_adr <= REG_I2C2CMD;
                dma_wb_dat_o <= x"94"; -- STA, WR
                if dma_wb_ack = '1' then
                    dma_wb_cyc <= '0';
                    dma_wb_stb <= '0';
                    dma_wb_we <= '0';
                    if dmaWr = '1' then
                        dmaState <= S_DMA_I2C_READ_DATA_WAIT_SRW;
                    else
                        dmaState <= S_DMA_I2C_WRITE_WAIT_TRRDY;
                    end if;
                end if;
                
            when S_DMA_I2C_WRITE_WAIT_TRRDY =>
                dma_wb_cyc <= '1';
                dma_wb_stb <= '1';
                dma_wb_adr <= REG_I2C2SR;
                if dma_wb_ack = '1' then
                    dma_wb_cyc <= '0';
                    dma_wb_stb <= '0';
                    if wb_dat_i(REGBIT_SR_TRRDY) = '1' then
                        if dmaLastData = '1' then
                            dmaState <= S_DMA_I2C_SEND_DATA_STOP;
                        else
                            dmaState <= S_DMA_I2C_SEND_DATA_TXDR;
                            if dmaCnt = to_unsigned(0, dmaCnt'length) then
                                dmaLastData <= '1';
                            end if;
                        end if;
                    end if;
                end if;
                
            when S_DMA_I2C_SEND_DATA_TXDR =>
                dma_wb_cyc <= '1';
                dma_wb_stb <= '1';
                dma_wb_we <= '1';
                dma_wb_adr <= REG_I2C2TXDR;
                dma_wb_dat_o <= dma_mem_QA;
                if dma_wb_ack = '1' then
                    dma_wb_cyc <= '0';
                    dma_wb_stb <= '0';
                    dma_wb_we <= '0';
                    dmaState <= S_DMA_I2C_SEND_DATA_CMDR;
                    dma_mem_AddressA <= inc_slv(dma_mem_AddressA);
                    dmaCnt <= dmaCnt - 1;
                end if;
            
            when S_DMA_I2C_SEND_DATA_CMDR =>
                dma_wb_cyc <= '1';
                dma_wb_stb <= '1';
                dma_wb_we <= '1';
                dma_wb_adr <= REG_I2C2CMD;
                dma_wb_dat_o <= x"14"; -- WR
                if dma_wb_ack = '1' then
                    dma_wb_cyc <= '0';
                    dma_wb_stb <= '0';
                    dma_wb_we <= '0';
                    dmaState <= S_DMA_I2C_WRITE_WAIT_TRRDY;
                end if;
            
            when S_DMA_I2C_SEND_DATA_STOP =>
                dma_wb_cyc <= '1';
                dma_wb_stb <= '1';
                dma_wb_we <= '1';
                dma_wb_adr <= REG_I2C2CMD;
                dma_wb_dat_o <= x"44"; -- STOP
                if dma_wb_ack = '1' then
                    dma_wb_cyc <= '0';
                    dma_wb_stb <= '0';
                    dma_wb_we <= '0';
                    dmaState <= S_DMA_I2C_READ_DATA_WAIT_BUSY;
                end if;
                
            when S_DMA_I2C_READ_DATA_WAIT_SRW =>
                dma_wb_cyc <= '1';
                dma_wb_stb <= '1';
                dma_wb_adr <= REG_I2C2SR;
                if dma_wb_ack = '1' then
                    dma_wb_cyc <= '0';
                    dma_wb_stb <= '0';
                    if wb_dat_i(REGBIT_SR_SRW) = '1' then
                        dmaState <= S_DMA_I2C_READ_DATA_CMDR;
                    end if;
                end if;
            
            when S_DMA_I2C_READ_DATA_CMDR =>
                dma_wb_cyc <= '1';
                dma_wb_stb <= '1';
                dma_wb_we <= '1';
                dma_wb_adr <= REG_I2C2CMD;
                dma_wb_dat_o <= x"24"; -- RD
                if dma_wb_ack = '1' then
                    dma_wb_cyc <= '0';
                    dma_wb_stb <= '0';
                    dma_wb_we <= '0';
                    dmaState <= S_DMA_I2C_READ_DATA_CHECKLAST;
                end if;
                
            when S_DMA_I2C_READ_DATA_CHECKLAST =>
                if dmaCnt = to_unsigned(0, dmaCnt'length) then
                    dmaCnt <= to_unsigned(1330, dmaCnt'length);
                    dmaState <= S_DMA_I2C_READ_DATA_DELAY;
                else
                    dmaCnt <= dmaCnt - 1;
                    dmaState <= S_DMA_I2C_READ_DATA_WAIT_TRRDY;
                end if;
                
            when S_DMA_I2C_READ_DATA_DELAY =>
                dmaLastData <= '1';
                dmaCnt <= dmaCnt - 1; -- delay wait 2*tcl
                if dmaCnt = to_unsigned(0, dmaCnt'length) then
                    dmaState <= S_DMA_I2C_READ_DATA_STOP_CMDR;
                end if;
            
            when S_DMA_I2C_READ_DATA_WAIT_TRRDY =>
                dma_wb_cyc <= '1';
                dma_wb_stb <= '1';
                dma_wb_adr <= REG_I2C2SR;
                if dma_wb_ack = '1' then
                    dma_wb_cyc <= '0';
                    dma_wb_stb <= '0';
                    if wb_dat_i(REGBIT_SR_TRRDY) = '1' then
                        dmaState <= S_DMA_I2C_READ_DATA_READ_RXDR;
                    end if;
                end if;
            
            when S_DMA_I2C_READ_DATA_READ_RXDR =>
                dma_wb_cyc <= '1';
                dma_wb_stb <= '1';
                dma_wb_adr <= REG_I2C2RXDR;
                if dma_wb_ack = '1' then
                    dma_wb_cyc <= '0';
                    dma_wb_stb <= '0';
                    dma_mem_DataInA <= wb_dat_i;
                    dma_mem_WrA <= '1';
                    if dmaLastData = '1' then
                        dmaState <= S_DMA_I2C_DISABLE_CMDR;
                    else
                        dmaState <= S_DMA_I2C_READ_DATA_CHECKLAST;
                    end if;
                end if;
            
            when S_DMA_I2C_READ_DATA_STOP_CMDR =>
                dma_wb_cyc <= '1';
                dma_wb_stb <= '1';
                dma_wb_we <= '1';
                dma_wb_adr <= REG_I2C2CMD;
                dma_wb_dat_o <= x"6C"; -- RD, NACK, STOP
                if dma_wb_ack = '1' then
                    dma_wb_cyc <= '0';
                    dma_wb_stb <= '0';
                    dma_wb_we <= '0';
                    dmaState <= S_DMA_I2C_READ_DATA_WAIT_TRRDY;
                end if;
                
            when S_DMA_I2C_DISABLE_CMDR =>
                dma_wb_cyc <= '1';
                dma_wb_stb <= '1';
                dma_wb_we <= '1';
                dma_wb_adr <= REG_I2C2CMD;
                dma_wb_dat_o <= x"04";
                if dma_wb_ack = '1' then
                    dma_wb_cyc <= '0';
                    dma_wb_stb <= '0';
                    dma_wb_we <= '0';
                    dmaState <= S_DMA_I2C_READ_DATA_WAIT_BUSY;
                end if;
                
            when S_DMA_I2C_READ_DATA_WAIT_BUSY =>
                dma_wb_cyc <= '1';
                dma_wb_stb <= '1';
                dma_wb_adr <= REG_I2C2SR;
                if dma_wb_ack = '1' then
                    dma_wb_cyc <= '0';
                    dma_wb_stb <= '0';
                    if wb_dat_i(REGBIT_SR_BUSY) = '0' then
                        dmaState <= S_DMA_IDLE;
                    end if;
                end if;
                
            when S_DMA_SI_SEND_DATA =>
                if dmaCnt = 0 then
                    if si_busy = '1' then
                        dmaState <= S_DMA_SI_RECEIVE_DATA;
                    end if;
                elsif si_req = '1' then
                    si_wr <= '1';
                    dmaCnt <= dmaCnt - 1;
                    dma_mem_AddressA <= inc_slv(dma_mem_AddressA);
                end if;
                
            when S_DMA_SI_RECEIVE_DATA =>
                if si_valid = '1' then
                    dma_mem_DataInA <= si_data_o;
                    dma_mem_WrA <= '1';
                    dmaSiRxCnt <= dmaSiRxCnt + 1;
                end if;
                if si_busy = '0' then
                    dmaState <= S_DMA_IDLE;
                end if;
                
            when S_DMA_CIC_RECEIVE_DATA =>
                dma_mem_DataInA <= (0 => cic_data_o, others => '0');
                if cic_busy = '0' then
                    if dmaCnt = 0 then
                        dmaState <= S_DMA_IDLE;
                    else
                        dmaCnt <= dmaCnt - 1;
                        cic_start <= '1';
                    end if;
                    dma_mem_WrA <= '1';
                end if;
                
            when others => dmaState <= S_DMA_IDLE;
            end case;
            
            if dma_mem_WrA = '1' then -- Adresse nach Schreibzyklus erhöhen
                dma_mem_AddressA <= inc_slv(dma_mem_AddressA);
            end if;
            
        end if;
    end if;
end process dma_proc;

LED_O(0) <= spiRegGpio(7);
LED_O(1) <= '1' when (pifState /= S_PIF_WAIT_START) else '0';

spiRegGpio(8) <= rst_btn_ff2;
spiRegGpio(9) <= rc_por_ff2;
spiRegGpio(10) <= cic_di_ff2;
spiRegGpio(11) <= pif_en_ff2;
spiRegGpio(15 downto 12) <= jumper_ff2;

spiRegDmaStat(0) <= '0' when (dmaState = S_DMA_IDLE) else '1';

pif_dat <= pif_dat_shift_out;
    
with spiAddr select spiSelectedRegister <=
    resize_slv(spiRegPifIrq, 16) when SPI_ADDR_INT,
    resize_slv(spiRegPifCommandAddr, 16) when SPI_ADDR_PIFCMD,
    resize_slv(spiRegPifRamBank, 16) when SPI_ADDR_PIFRAMBANK,
    spiRegGpio when SPI_ADDR_GPIO,
    std_logic_vector(resize(val_r, 16)) when SPI_ADDR_RED,
    std_logic_vector(resize(val_g, 16)) when SPI_ADDR_GREEN,
    std_logic_vector(resize(val_b, 16)) when SPI_ADDR_BLUE,
    resize_slv(spiRegI2cSlaveAddr, 16) when SPI_ADDR_I2C_SLAVEADDR,
    resize_slv(spiRegSiChannel, 16) when SPI_ADDR_SI_CHANNEL,
    resize_slv(spiRegDmaMemPtr, 16) when SPI_ADDR_DMA_MEMADDR,
    resize_slv(spiRegDmaCnt, 16) when SPI_ADDR_DMA_CNT,
    resize_slv(spiRegDmaStat, 16) when SPI_ADDR_DMA_STAT,
    std_logic_vector(resize(dmaSiRxCnt, 16)) when SPI_ADDR_SI_RXCNT,
    FPGA_VERSION(15 downto 0) when SPI_ADDR_FPGA_VER0,
    FPGA_VERSION(31 downto 16) when SPI_ADDR_FPGA_VER1,
    (others => '1') when others;
    
with spiByteSel(0) select spiSelectedRegisterByte <=
    spiSelectedRegister(7 downto 0) when '0',
    spiSelectedRegister(15 downto 8) when others;
    
NIRQ_O <= '0' when spiRegPifIrq /= (spiRegPifIrq'range => '0') else '1';

RESET_BTN_O <= rst_btn_ff2;
    
process (clk)
begin
    if rising_edge(clk) then
        pif_en_ff1 <= PIF_EN_I;
        pif_en_ff2 <= pif_en_ff1;
        if (pif_en_ff2 = '0') or (rst = '0') then
            pif_clk_d1 <= '1';
            pif_clk <= '1';
            pif_adr <= '1';
            rst_btn_ff <= '0';
            rst_btn_ff2 <= '0';
            rc_por_ff <= '0';
            rc_por_ff2 <= '0';
            cic_di_ff <= '0';
            cic_di_ff2 <= '0';
        else
            pif_clk <= PIF_CLK_I;
            pif_clk_d1 <= pif_clk;
            pif_adr <= PIF_ADR_I;
            rst_btn_ff <= RESET_BTN_I;
            rst_btn_ff2 <= rst_btn_ff;
            rc_por_ff <= RC_POR_I;
            rc_por_ff2 <= rc_por_ff;
            cic_di_ff <= cic_di;
            cic_di_ff2 <= cic_di_ff;
        end if;
        
    end if;
end process;

process (clk)
begin
    if rising_edge(clk) then
        if rst = '0' then
            jumper_ff1 <= (others => '0');
            jumper_ff2 <= (others => '0');
        else
            jumper_ff1 <= JUMPER_I;
            jumper_ff2 <= jumper_ff1;
        end if;
    end if;
end process;

process (clk)
begin

    if rising_edge(clk) then
        
        wea <= '0';
        ena <= '0';
        if (pif_en_ff2 = '0') or (rst = '0') or (spiRegGpio(0) = '0') then
            pif_adr_d1 <= '1';
            pifCmdStart <= '0';
            pifAddr <= "111110000";
            pifState <= S_PIF_WAIT_START;
            shiftOutReg <= (others => '1');
            spiRegPifIrq <= (others => '0');
        else

			if ((pif_clk_d1 = '1') and (pif_clk = '0')) then
                pif_adr_d1 <= pif_adr;
                pifCmdStart <= '0';
                
                case pifState is
                        
                    when S_PIF_WAIT_START =>
                        if (pif_adr_d1 = '1' and (pif_adr = '0')) then
                            pifState <= S_PIF_READ_COMMAND;
                            shiftCnt <= to_unsigned(11, shiftCnt'length);
                            shiftReg <= (others => '0');
                        end if;
                        
                    when S_PIF_READ_COMMAND =>
                        if (shiftCnt = 0) then
                            pifCmd <= shiftReg(10 downto 9);
                            pifAddr <= shiftReg(8 downto 0);
                            pifRamAddr <= shiftReg(8 downto 0);
                            pifState <= S_PIF_WAIT_CLEARANCE;
                            spiRegPifIrq(0) <= '1';
                            pifCmdStart <= '1';
                            spiRegPifCommandAddr <= shiftReg(10 downto 0);
                        else
                            shiftReg <= shiftReg(shiftReg'length - 2 downto 0) & pif_adr;
                            shiftCnt <= shiftCnt - 1;
                        end if;
                        
                    when S_PIF_WAIT_CLEARANCE =>
                        if (spiRegPifIrq(0) = '0') then
                            pifState <= S_PIF_EXEC_COMMAND;
                            ena <= '1';
                        end if;
                        
                    when S_PIF_EXEC_COMMAND =>
                        if rdya = '1' then
                            case (pifCmd) is
                                when "00" => --write64B
                                    pifState <= S_PIF_WAIT_PIF_ACK;
                                    shiftOutReg <= x"feffffff" & '1';
                                    
                                when "10" => --write4B 
                                    pifState <= S_PIF_WAIT_PIF_ACK;
                                    shiftOutReg <= x"feffffff" & '1';
                                
                                when "01" => --read64B
                                    pifState <= S_PIF_WAIT_PIF_ACK;
                                    shiftOutReg <= "0" & douta;
                                    pifRamAddr <= std_logic_vector(unsigned(pifRamAddr) + 1);
                                    ena <= '1';
                                
                                when "11" => --read4B
                                    pifState <= S_PIF_WAIT_PIF_ACK;
                                    shiftOutReg <= "0" & douta;
                            
                                when others => null;
                            end case;
                        end if;
                        
                    when S_PIF_WAIT_PIF_ACK =>
                        if (pif_dat = '0') then
                            case (pifCmd) is
                                when "00" => --write64B
                                    dataCnt <= to_unsigned(15, dataCnt'length);
                                    pifState <= S_PIF_WAIT_RCP_START;
                                    
                                when "10" => --write4B 
                                    dataCnt <= to_unsigned(0, dataCnt'length);
                                    pifState <= S_PIF_WAIT_RCP_START;
                                
                                when "01" => --read64B
                                    dataCnt <= to_unsigned(15, dataCnt'length);
                                    pifState <= S_PIF_READ_PIF_DATA;
                                
                                when "11" => --read4B
                                    dataCnt <= to_unsigned(0, dataCnt'length);
                                    pifState <= S_PIF_READ_PIF_DATA;
                                
                                when others => null;
                            end case;
                        end if;
                        
                    when S_PIF_WAIT_RCP_START =>
                        if (pif_adr = '0') then
                            pifState <= S_PIF_READ_RCP_DATA;
                        end if;
                    
                    when S_PIF_READ_RCP_DATA =>
                        shiftReg <= shiftReg(shiftReg'length - 2 downto 0) & pif_adr;
                        if (shiftCnt = 31) then
                            shiftCnt <= (others => '0');
                            pifData <= shiftReg(30 downto 0) & pif_adr;
                            pifRamAddr <= pifAddr;
                            wea <= '1';
                            ena <= '1';
                            if (dataCnt = 0) then
                                pifState <= S_PIF_WAIT_START;
                                spiRegPifIrq(1) <= '1';
                            else
                                pifAddr <= std_logic_vector(unsigned(pifAddr) + 1);
                                dataCnt <= dataCnt - 1;
                            end if;
                        else
                            shiftCnt <= shiftCnt + 1;
                        end if;
                        
                    when S_PIF_READ_PIF_DATA =>
                        shiftReg <= shiftReg(shiftReg'length - 2 downto 0) & pif_dat;
                        if (shiftCnt = 31) then
                            shiftCnt <= (others => '0');
                            pifData <= shiftReg(30 downto 0) & pif_dat;
                            pifAddr <= std_logic_vector(unsigned(pifAddr) + 1);
                            if (dataCnt = 0) then
                                pifState <= S_PIF_WAIT_START;
                            else
                                dataCnt <= dataCnt - 1;
                                shiftOutReg <= douta & "1";
                                pifRamAddr <= std_logic_vector(unsigned(pifRamAddr) + 1);
                                ena <= '1';
                            end if;
                        else
                            shiftCnt <= shiftCnt + 1;
                        end if;
                    
                    when others => pifState <= S_PIF_WAIT_START;
                    
                end case;
                
            elsif ((pif_clk_d1 = '0') and (pif_clk = '1')) then
                
                pif_dat_shift_out <= shiftOutReg(32);
                shiftOutReg <= shiftOutReg(shiftOutReg'length - 2 downto 0) & "1";
            end if;  
        end if;
        
        spi_wren <= '0';
        pif_ram_WrB <= '0';
        dma_mem_WrB <= '0';
        spiRegDmaCmd <= (others => '0');
        read_delay <= '1';
        if rst = '0' then
            val_r <= to_unsigned(50, val_r'length);
            val_g <= to_unsigned(0, val_g'length);
            val_b <= to_unsigned(0, val_b'length);
        end if;
        if (spi_nss_ff1 = '1') or (rst = '0') then
            spiState <= S_SPI_WAIT_CMD;
            spiByteSel <= (others => '0');
        else
            case spiState is
                when S_SPI_WAIT_CMD =>
                    if (spi_do_valid = '1') then
-- SPI Command structure:
-- 1RAAAAAA Read/Write PIF RAM -> 64 Locations -> PIF RAM bytewise addressable, add 5 bit Bank Register to address PIF ROM
-- 01RAAAAA Read/Write Register -> 32 Registers (16 bit each)
-- 001RAAAA | AAAAAAAA Read/Write ExtMem -> max 16*256 = 4096 (used 1024) Locations
                        spiAddr <= spi_do(4 downto 0);
                        pif_ram_AddressB <= spiRegPifRamBank & spi_do(5 downto 0);
                        spiCmdRead <= spi_do(4);
                        if spi_do(7) = '1' then -- Read/Write PIF RAM
                            if spi_do(6) = '1' then
                                spiState <= S_SPI_SEND_DATA;
                            else
                                spiState <= S_SPI_RECEIVE_DATA;
                            end if;
                        elsif spi_do(6) = '1' then -- Read/Write Register
                            if spi_do(5) = '1' then
                                spiState <= S_SPI_SEND_REGISTER;
                            else
                                spiState <= S_SPI_RECEIVE_REGISTER;
                            end if;
                        elsif spi_do(5) = '1' then -- Read/Write ExtMem
                            spiState <= S_SPI_RECEIVE_EXT_ADDR;
                        end if;
                    end if;
                
                when S_SPI_RECEIVE_DATA =>
                    if (spi_do_valid = '1') then
                        pif_ram_DataInB <= spi_do;
                        pif_ram_WrB <= '1';
                    end if;
                    -- Addresse nach Schreibzyklus erhöhen
                    if pif_ram_WrB = '1' then
                        pif_ram_AddressB <= std_logic_vector(unsigned(pif_ram_AddressB) + 1);
                    end if;
                
                when S_SPI_SEND_DATA =>
                    read_delay <= '0';
                    if (read_delay = '0') and (spi_di_req = '1') then
                        spi_wren <= '1';
                        spi_di <= pif_ram_QB;
                        pif_ram_AddressB <= std_logic_vector(unsigned(pif_ram_AddressB) + 1);
                    end if;
                
                when S_SPI_SEND_REGISTER =>
                    if (spi_di_req = '1') then
                        spi_wren <= '1';
                        spi_di <= spiSelectedRegisterByte;
                        spiByteSel <= spiByteSel + 1;
                        if (spiByteSel(0) = '1') then
                            spiAddr <= std_logic_vector(unsigned(spiAddr) + 1);
                        end if;
                    end if;
                
                when S_SPI_RECEIVE_REGISTER =>
                    if (spi_do_valid = '1') then
                        spiLowBuffer <= spi_do;
                        case spiAddr is
                        when SPI_ADDR_INT =>
                            if spiByteSel(0) = '0' then
                                if (spi_do(0) = '1') then
                                    spiRegPifIrq(0) <= '0';
                                end if;
                                if (spi_do(1) = '1') then
                                    spiRegPifIrq(1) <= '0';
                                end if;
                            end if;
                            
                        when SPI_ADDR_PIFRAMBANK =>
                            if spiByteSel(0) = '0' then
                                spiRegPifRamBank <= spi_do(4 downto 0);
                            end if;
                            
                        when SPI_ADDR_GPIO =>
                            if spiByteSel(0) = '0' then
                                spiRegGpio(7 downto 0) <= spi_do;
                            end if;
                            
                        when SPI_ADDR_RED =>
                            if spiByteSel(0) = '1' then
                                val_r <= unsigned(spi_do(1 downto 0) & spiLowBuffer);
                            end if;
                            
                        when SPI_ADDR_GREEN =>
                            if spiByteSel(0) = '1' then
                                val_g <= unsigned(spi_do(1 downto 0) & spiLowBuffer);
                            end if;
                            
                        when SPI_ADDR_BLUE =>
                            if spiByteSel(0) = '1' then
                                val_b <= unsigned(spi_do(1 downto 0) & spiLowBuffer);
                            end if;
                            
                        when SPI_ADDR_I2C_SLAVEADDR =>
                            if spiByteSel(0) = '0' then
                                spiRegI2cSlaveAddr <= spi_do(6 downto 0);
                            end if;
                            
                        when SPI_ADDR_SI_CHANNEL =>
                            if spiByteSel(0) = '0' then
                                spiRegSiChannel <= spi_do(2 downto 0);
                            end if;
                            
                        when SPI_ADDR_DMA_CMD =>
                            if spiByteSel(0) = '0' then
                                spiRegDmaCmd <= spi_do;
                            end if;
                            
                        when SPI_ADDR_DMA_MEMADDR =>
                            if spiByteSel(0) = '1' then
                                spiRegDmaMemPtr <= spi_do(1 downto 0) & spiLowBuffer;
                            end if;
                            
                        when SPI_ADDR_DMA_CNT =>
                            if spiByteSel(0) = '1' then
                                spiRegDmaCnt <= spi_do(1 downto 0) & spiLowBuffer;
                            end if;
                            
                        when others => null;
                        end case;

                        spiByteSel <= spiByteSel + 1;
                        if (spiByteSel(0) = '1') then
                            spiAddr <= std_logic_vector(unsigned(spiAddr) + 1);
                        end if;
                    end if;
                    
                when S_SPI_RECEIVE_EXT_ADDR =>
                    if (spi_do_valid = '1') then
                        dma_mem_AddressB <= spiAddr(1 downto 0) & spi_do;
                        if spiCmdRead = '1' then
                            spiState <= S_SPI_SEND_EXT_DATA;
                        else
                            spiState <= S_SPI_RECEIVE_EXT_DATA;
                        end if;
                    end if;
                    
                when S_SPI_RECEIVE_EXT_DATA =>
                    if (spi_do_valid = '1') then
                        dma_mem_DataInB <= spi_do;
                        dma_mem_WrB <= '1';
                    end if;
                    
                    -- Adresse nach Schreibzyklus erhöhen
                    if dma_mem_WrB = '1' then
                        dma_mem_AddressB <= std_logic_vector(unsigned(dma_mem_AddressB) + 1);
                    end if;
                
                when S_SPI_SEND_EXT_DATA =>
                    read_delay <= '0';
                    if (spi_di_req = '1') and (read_delay = '0') then
                        spi_wren <= '1';
                        spi_di <= dma_mem_QB;
                        dma_mem_AddressB <= std_logic_vector(unsigned(dma_mem_AddressB) + 1);
                    end if;
                    
                when S_SPI_INVALID_CMD =>
                    -- nichts tun, bis zum nächsten Command
                    
                when others => spiState <= S_SPI_WAIT_CMD;
            end case;
        end if;
    end if;

end process;

rgb_int(2) <= '1' when (cnt_rgb < val_r) else '0';
rgb_int(1) <= '1' when (cnt_rgb < val_g) else '0';
rgb_int(0) <= '1' when (cnt_rgb < val_b) else '0';

process (clk)
begin
    if rising_edge(clk) then
	
        if rst = '0' then
            div_rgb <= 0;
            cnt_rgb <= 0;
        else
            counter <= counter + 1;
            if counter = 66500000/5 - 1 then
                counter <= 0;
            end if;
            
            div_rgb <= div_rgb + 1;
            if div_rgb = 132 then
                div_rgb <= 0;
                cnt_rgb <= cnt_rgb + 1;
            end if;
        end if;
    end if;
end process;


process (PIF_CLK_I, rst)
begin
    if rst = '0' then
        serial_clk2x <= '0';
        serial_clk_div <= (others => '0');
        serial_clk <= '0';
    elsif rising_edge(PIF_CLK_I) then
        serial_clk2x <= '0';
        serial_clk_div <= serial_clk_div + 1;
        
        if serial_clk_div = 0 then
            serial_clk2x <= '1';
        end if;
        
        if serial_clk2x = '1' then
            serial_clk <= not serial_clk;
        end if;
    end if;
end process;


end Behavioral;
