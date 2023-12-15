-- ==============================================================
-- Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2020.2 (64-bit)
-- Copyright 1986-2020 Xilinx, Inc. All Rights Reserved.
-- ==============================================================
library ieee; 
use ieee.std_logic_1164.all; 
use ieee.std_logic_unsigned.all;

entity inference_weights_3_rom is 
    generic(
             DWIDTH     : integer := 32; 
             AWIDTH     : integer := 6; 
             MEM_SIZE    : integer := 48
    ); 
    port (
          addr0      : in std_logic_vector(AWIDTH-1 downto 0); 
          ce0       : in std_logic; 
          q0         : out std_logic_vector(DWIDTH-1 downto 0);
          clk       : in std_logic
    ); 
end entity; 


architecture rtl of inference_weights_3_rom is 

signal addr0_tmp : std_logic_vector(AWIDTH-1 downto 0); 
type mem_array is array (0 to MEM_SIZE-1) of std_logic_vector (DWIDTH-1 downto 0); 
signal mem : mem_array := (
    0 => "10111101111111100100000101000010", 
    1 => "10111111011001111101100111111110", 
    2 => "10111110110000000111011101011100", 
    3 => "10111111111101000000010111111111", 
    4 => "10111111010111100001000100110110", 
    5 => "00111100000011101111010011101101", 
    6 => "10111110110001001111111001111110", 
    7 => "10111110101010100010100111000111", 
    8 => "00111110110100110010001101000101", 
    9 => "10111111011010001010101110110101", 
    10 => "00111110101011010100010111111000", 
    11 => "00111110011011111011011101010111", 
    12 => "10111111100101100001100001111101", 
    13 => "10111101000011000111111100001101", 
    14 => "10111111001001011111011001111000", 
    15 => "10111110110110101001110000001000", 
    16 => "10111111101111000101011000000000", 
    17 => "10111111101001010110000000001111", 
    18 => "10111111101000000110010100111000", 
    19 => "10111111010010000001010111111111", 
    20 => "10111101110001011100001001000101", 
    21 => "10111111010001001011101010000001", 
    22 => "00111101110111011010011101001010", 
    23 => "10111110010010000000101010101110", 
    24 => "00111110101100111000001110110001", 
    25 => "00111110110110111011000100101110", 
    26 => "10111111110111111111000100111010", 
    27 => "00111110101111011000110001000110", 
    28 => "10111110111011111111110110100110", 
    29 => "10111111100100111010100000011001", 
    30 => "00111100110100011000100100110110", 
    31 => "10111110111101100010000111111011", 
    32 => "10111110101000010100011010000001", 
    33 => "00111111000010010100001000110111", 
    34 => "00111111000000001100011011001000", 
    35 => "10111110001011000000011110100010", 
    36 => "00111111011011111111111000110110", 
    37 => "10111110110110010000010100110100", 
    38 => "00111111010011100001110101111100", 
    39 => "10111011111101011110001101011110", 
    40 => "10111110111000100001000010101110", 
    41 => "00111110001010001111110000100010", 
    42 => "10111110111001001010000001101001", 
    43 => "00111110100010011001110100100100", 
    44 => "00111101100111101011001111111000", 
    45 => "00111110000111000001110111000111", 
    46 => "00111110101010110000001001010110", 
    47 => "10111110111010101000101001000100" );


begin 


memory_access_guard_0: process (addr0) 
begin
      addr0_tmp <= addr0;
--synthesis translate_off
      if (CONV_INTEGER(addr0) > mem_size-1) then
           addr0_tmp <= (others => '0');
      else 
           addr0_tmp <= addr0;
      end if;
--synthesis translate_on
end process;

p_rom_access: process (clk)  
begin 
    if (clk'event and clk = '1') then
        if (ce0 = '1') then 
            q0 <= mem(CONV_INTEGER(addr0_tmp)); 
        end if;
    end if;
end process;

end rtl;

Library IEEE;
use IEEE.std_logic_1164.all;

entity inference_weights_3 is
    generic (
        DataWidth : INTEGER := 32;
        AddressRange : INTEGER := 48;
        AddressWidth : INTEGER := 6);
    port (
        reset : IN STD_LOGIC;
        clk : IN STD_LOGIC;
        address0 : IN STD_LOGIC_VECTOR(AddressWidth - 1 DOWNTO 0);
        ce0 : IN STD_LOGIC;
        q0 : OUT STD_LOGIC_VECTOR(DataWidth - 1 DOWNTO 0));
end entity;

architecture arch of inference_weights_3 is
    component inference_weights_3_rom is
        port (
            clk : IN STD_LOGIC;
            addr0 : IN STD_LOGIC_VECTOR;
            ce0 : IN STD_LOGIC;
            q0 : OUT STD_LOGIC_VECTOR);
    end component;



begin
    inference_weights_3_rom_U :  component inference_weights_3_rom
    port map (
        clk => clk,
        addr0 => address0,
        ce0 => ce0,
        q0 => q0);

end architecture;


