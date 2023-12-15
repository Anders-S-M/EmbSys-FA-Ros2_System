-- ==============================================================
-- Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2020.2 (64-bit)
-- Copyright 1986-2020 Xilinx, Inc. All Rights Reserved.
-- ==============================================================
library ieee; 
use ieee.std_logic_1164.all; 
use ieee.std_logic_unsigned.all;

entity inference_bias_layer1_rom is 
    generic(
             DWIDTH     : integer := 32; 
             AWIDTH     : integer := 5; 
             MEM_SIZE    : integer := 32
    ); 
    port (
          addr0      : in std_logic_vector(AWIDTH-1 downto 0); 
          ce0       : in std_logic; 
          q0         : out std_logic_vector(DWIDTH-1 downto 0);
          clk       : in std_logic
    ); 
end entity; 


architecture rtl of inference_bias_layer1_rom is 

signal addr0_tmp : std_logic_vector(AWIDTH-1 downto 0); 
type mem_array is array (0 to MEM_SIZE-1) of std_logic_vector (DWIDTH-1 downto 0); 
signal mem : mem_array := (
    0 => "00111111010000100001111001000010", 
    1 => "10111111010000010100110001111011", 
    2 => "11000000010000001011011010011100", 
    3 => "10111111000000001011010010001011", 
    4 => "00111111011000000001011111001000", 
    5 => "10111111100010011000111010010100", 
    6 => "01000000000111001001101101110100", 
    7 => "01000000000010101110101100101010", 
    8 => "00111111110111000001010110101000", 
    9 => "10111101110101000100110101011111", 
    10 => "10111111000000110101100111001101", 
    11 => "01000000000000010010011010100101", 
    12 => "10111111100100101111010101110111", 
    13 => "00111111110001000111101000010011", 
    14 => "10111110101100010111111111110101", 
    15 => "10111111100000110101001111000111", 
    16 => "00111111010000101100101111000000", 
    17 => "00111111001110001011000110100111", 
    18 => "01000000000101100010001000010100", 
    19 => "10111111010000101110000111001110", 
    20 => "10111111000010110101100000011011", 
    21 => "01000000001001110111110100110100", 
    22 => "00111110101101010111111111101110", 
    23 => "10111111000100111100001100000010", 
    24 => "00111111101011000100010110100111", 
    25 => "01000000000110111100100001000111", 
    26 => "00111101100100000011110111100010", 
    27 => "00111111100011001011011110001100", 
    28 => "00111110001110011110101011111101", 
    29 => "00111110101001101011001100011110", 
    30 => "10111110110011111010100010000000", 
    31 => "00111111100111111111010110001100" );


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

entity inference_bias_layer1 is
    generic (
        DataWidth : INTEGER := 32;
        AddressRange : INTEGER := 32;
        AddressWidth : INTEGER := 5);
    port (
        reset : IN STD_LOGIC;
        clk : IN STD_LOGIC;
        address0 : IN STD_LOGIC_VECTOR(AddressWidth - 1 DOWNTO 0);
        ce0 : IN STD_LOGIC;
        q0 : OUT STD_LOGIC_VECTOR(DataWidth - 1 DOWNTO 0));
end entity;

architecture arch of inference_bias_layer1 is
    component inference_bias_layer1_rom is
        port (
            clk : IN STD_LOGIC;
            addr0 : IN STD_LOGIC_VECTOR;
            ce0 : IN STD_LOGIC;
            q0 : OUT STD_LOGIC_VECTOR);
    end component;



begin
    inference_bias_layer1_rom_U :  component inference_bias_layer1_rom
    port map (
        clk => clk,
        addr0 => address0,
        ce0 => ce0,
        q0 => q0);

end architecture;


