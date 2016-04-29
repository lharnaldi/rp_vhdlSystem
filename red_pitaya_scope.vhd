--
--$Id: red_pitaya_scope.vhd 965 2016-04-24 10:41:52Z lharnaldi $
--
--@brief Red Pitaya oscilloscope application, used for capturing ADC data
--       into BRAMs, which can be later read by SW.
--
--@Author L. Horacio Arnaldi
--
--(c) Red Pitaya  http://www.redpitaya.com
--
--This part of code is written in VHDL hardware description language (HDL).
--Please visit http://en.wikipedia.org/wiki/VHDL
--for more details on the language used herein.
--
--
--
--GENERAL DESCRIPTION:
--
--This is simple data aquisition module, primerly used for scilloscope 
--application. It consists from three main parts.
--
--
--               /--------\      /-----------\            /-----\
--  ADC CHA ---> | DFILT1 | ---> | AVG & DEC | ---------> | BUF | --->  SW
--               \--------/      \-----------/     |      \-----/
--                                                 ˇ         ^
--                                             /------\      |
--  ext trigger -----------------------------> | TRIG | -----+
--                                             \------/      |
--                                                 ^         ˇ
--               /--------\      /-----------\     |      /-----\
--  ADC CHB ---> | DFILT1 | ---> | AVG & DEC | ---------> | BUF | --->  SW
--               \--------/      \-----------/            \-----/ 
--
--
--Input data is optionaly averaged and decimated via average filter.
--
--Trigger section makes triggers from input ADC data or external digital 
--signal. To make trigger from analog signal schmitt trigger is used, external
--trigger goes first over debouncer, which is separate for pos. and neg. edge.
--
--Data capture buffer is realized with BRAM. Writing into ram is done with 
--arm/trig logic. With adc_arm_do signal (SW) writing is enabled, this is active
--until trigger arrives and adc_dly_cnt counts to zero. Value adc_wp_trig
--serves as pointer which shows when trigger arrived. This is used to show
--pre-trigger data.
--
--

library ieee;

use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity red_pitaya_scope is
  generic(
  RSZ : natural := 14  -- RAM size 2**RSZ
);
  port(
   -- ADC
   adc_clk_i       : in std_logic                         ; -- ADC clock
   adc_rstn_i      : in std_logic                         ; -- ADC reset - active low
   adc_a_i         : in std_logic_vector(14-1 downto 0)   ; -- ADC data CHA
   adc_b_i         : in std_logic_vector(14-1 downto 0)   ; -- ADC data CHB
   -- trigger sources
   trig_ext_i      : in std_logic                         ; -- external trigger
   trig_asg_i      : in std_logic                         ; -- ASG trigger

   -- AXI0 master
   axi0_clk_o      : inout std_logic                        ; -- global clock
   axi0_rstn_o     : inout std_logic                        ; -- global reset
   axi0_waddr_o    : out std_logic_vector(32-1 downto 0)  ; -- system write address
   axi0_wdata_o    : out std_logic_vector(64-1 downto 0)  ; -- system write data
   axi0_wsel_o     : out std_logic_vector(8-1 downto 0)   ; -- system write byte select
   axi0_wvalid_o   : inout std_logic                        ; -- system write data valid
   axi0_wlen_o     : out std_logic_vector(4-1 downto 0)   ; -- system write burst length
   axi0_wfixed_o   : out std_logic                        ; -- system write burst type (fixed / incremental)
   axi0_werr_i     : in std_logic                         ; -- system write error
   axi0_wrdy_i     : in std_logic                         ; -- system write ready

   -- AXI1 master
   axi1_clk_o      : out std_logic                        ; -- global clock 
   axi1_rstn_o     : out std_logic                        ; -- global reset
   axi1_waddr_o    : out std_logic_vector(32-1 downto 0)  ; -- system write address
   axi1_wdata_o    : out std_logic_vector(64-1 downto 0)  ; -- system write data
   axi1_wsel_o     : out std_logic_vector(8-1 downto 0)   ; -- system write byte select
   axi1_wvalid_o   : inout std_logic                        ; -- system write data valid
   axi1_wlen_o     : out std_logic_vector(4-1 downto 0)   ; -- system write burst length
   axi1_wfixed_o   : out std_logic                        ; -- system write burst 
   axi1_werr_i     : in std_logic                         ; -- system write error 
   axi1_wrdy_i     : in std_logic			  ;  -- system write ready ;
  
   -- System bus
   sys_addr      : in std_logic_vector(32-1 downto 0)     ; -- bus saddress
   sys_wdata     : in std_logic_vector(32-1 downto 0)     ; -- bus write data
   sys_sel       : in std_logic_vector(4-1 downto 0)      ; -- bus write byte select
   sys_wen       : in std_logic                           ; -- bus write enable
   sys_ren       : in std_logic                           ; -- bus read enable
   sys_rdata     : out std_logic_vector(32-1 downto 0)    ; -- bus read data
   sys_err       : out std_logic                          ; -- bus error indicator
   sys_ack       : out std_logic                            -- bus acknowledge signal
);
end red_pitaya_scope;

architecture rtl of red_pitaya_scope is

-- General signals
signal adc_arm_do_reg, adc_arm_do_next            : std_logic;
signal adc_rst_do_reg, adc_rst_do_next            : std_logic;

----------------------------------------------------------------------------------
--  Input filtering

signal adc_a_filt_in                              : std_logic_vector(14-1 downto 0);
signal adc_a_filt_out                             : std_logic_vector(14-1 downto 0);
signal adc_b_filt_in                              : std_logic_vector(14-1 downto 0);
signal adc_b_filt_out                             : std_logic_vector(14-1 downto 0);
signal set_a_filt_aa_reg, set_a_filt_aa_next      : std_logic_vector(18-1 downto 0);
signal set_a_filt_bb_reg, set_a_filt_bb_next      : std_logic_vector(25-1 downto 0);
signal set_a_filt_kk_reg, set_a_filt_kk_next      : std_logic_vector(25-1 downto 0);
signal set_a_filt_pp_reg, set_a_filt_pp_next      : std_logic_vector(25-1 downto 0);
signal set_b_filt_aa_reg, set_b_filt_aa_next      : std_logic_vector(18-1 downto 0);
signal set_b_filt_bb_reg, set_b_filt_bb_next      : std_logic_vector(25-1 downto 0);
signal set_b_filt_kk_reg, set_b_filt_kk_next      : std_logic_vector(25-1 downto 0);
signal set_b_filt_pp_reg, set_b_filt_pp_next      : std_logic_vector(25-1 downto 0);

-----------------------------------------------------------------------------------
--  Decimate input data

signal adc_a_dat_reg,  adc_a_dat_next             : std_logic_vector(14-1 downto 0);   
signal adc_b_dat_reg,  adc_b_dat_next             : std_logic_vector(14-1 downto 0);   
signal adc_a_sum_reg,  adc_a_sum_next             : signed(32-1 downto 0);   
signal adc_b_sum_reg,  adc_b_sum_next             : signed(32-1 downto 0);   
signal set_dec_reg, set_dec_next                  : std_logic_vector(17-1 downto 0);
signal adc_dec_cnt_reg, adc_dec_cnt_next          : unsigned(17-1 downto 0);
signal set_avg_en_reg, set_avg_en_next            : std_logic;
signal adc_dv_reg, adc_dv_next                    : std_logic;

--hopo
signal set_dec_aux                  							: std_logic_vector(17-1 downto 0);
-----------------------------------------------------------------------------------
--  ADC buffer RAM

type ram_type is array (0 to (2**RSZ)-1) of std_logic_vector(14-1 downto 0);
signal adc_a_buf                                  : ram_type;
signal adc_b_buf                                  : ram_type;

signal adc_a_rd_reg, adc_a_rd_next                : std_logic_vector(14-1 downto 0)      ;
signal adc_b_rd_reg, adc_b_rd_next                : std_logic_vector(14-1 downto 0)      ;
signal adc_wp_reg, adc_wp_next                    : unsigned(RSZ-1 downto 0)        ;
signal adc_raddr_reg, adc_raddr_next              : std_logic_vector(RSZ-1 downto 0)    ;
signal adc_a_raddr_reg, adc_a_raddr_next          : std_logic_vector(RSZ-1 downto 0)   ;
signal adc_b_raddr_reg, adc_b_raddr_next          : std_logic_vector(RSZ-1 downto 0)   ;
signal adc_rval_reg, adc_rval_next                : std_logic_vector(4-1 downto 0)      ;
signal adc_rd_dv                                  : std_logic     ;
signal adc_we_reg, adc_we_next                    : std_logic        ;
signal adc_we_keep_reg, adc_we_keep_next          : std_logic   ;
signal adc_trig_reg, adc_trig_next                : std_logic      ;

signal adc_wp_trig_reg, adc_wp_trig_next          : unsigned(RSZ-1 downto 0)   ;
signal adc_wp_cur_reg, adc_wp_cur_next            : unsigned(RSZ-1 downto 0)    ;
signal set_dly_reg, set_dly_next                  : std_logic_vector(32-1 downto 0)       ;
signal adc_we_cnt_reg, adc_we_cnt_next            : unsigned(32-1 downto 0)    ;
signal adc_dly_cnt_reg, adc_dly_cnt_next          : unsigned(32-1 downto 0)   ;
signal adc_dly_do_reg, adc_dly_do_next            : std_logic    ;
signal set_deb_len_reg, set_deb_len_next          : std_logic_vector(20-1 downto 0)   ; -- debouncing length (glitch free time after a posedge)

-----------------------------------------------------------------------------------
--
--  AXI CHA connection

signal set_a_axi_start_reg, set_a_axi_start_next  : std_logic_vector(32-1 downto 0) ;
signal set_a_axi_stop_reg,  set_a_axi_stop_next   : std_logic_vector(32-1 downto 0) ;
signal set_a_axi_dly_reg,   set_a_axi_dly_next    : std_logic_vector(32-1 downto 0) ;
signal set_a_axi_en_reg,    set_a_axi_en_next     : std_logic ;
signal set_a_axi_trig_reg,  set_a_axi_trig_next   : std_logic_vector(32-1 downto 0) ;
signal set_a_axi_cur_reg,   set_a_axi_cur_next    : std_logic_vector(32-1 downto 0)  ;
signal axi_a_we_reg,        axi_a_we_next         : std_logic ;
signal axi_a_dat_reg,       axi_a_dat_next        : std_logic_vector(64-1 downto 0)  ;
signal axi_a_dat_sel_reg,   axi_a_dat_sel_next    : unsigned(2-1 downto 0) ;
signal axi_a_dat_dv_reg,    axi_a_dat_dv_next     : std_logic_vector(1-1 downto 0) ;
signal axi_a_dly_cnt_reg,   axi_a_dly_cnt_next    : unsigned(32-1 downto 0) ;
signal axi_a_dly_do_reg,    axi_a_dly_do_next     : std_logic       ;
signal axi_a_clr                                  : std_logic          ;
signal axi_a_cur_addr                             : std_logic_vector(32-1 downto 0)     ;

-----------------------------------------------------------------------------------
--
--  AXI CHB connection

signal set_b_axi_start_reg, set_b_axi_start_next  : std_logic_vector(32-1 downto 0) ;
signal set_b_axi_stop_reg,  set_b_axi_stop_next   : std_logic_vector(32-1 downto 0) ;
signal set_b_axi_dly_reg,   set_b_axi_dly_next    : std_logic_vector(32-1 downto 0) ;
signal set_b_axi_en_reg,    set_b_axi_en_next     : std_logic ;
signal set_b_axi_trig_reg,  set_b_axi_trig_next   : std_logic_vector(32-1 downto 0) ;
signal set_b_axi_cur_reg,   set_b_axi_cur_next    : std_logic_vector(32-1 downto 0)  ;
signal axi_b_we_reg,        axi_b_we_next         : std_logic ;
signal axi_b_dat_reg,       axi_b_dat_next        : std_logic_vector(64-1 downto 0)  ;
signal axi_b_dat_sel_reg,   axi_b_dat_sel_next    : std_logic_vector(2-1 downto 0) ;
signal axi_b_dat_dv_reg,    axi_b_dat_dv_next     : std_logic_vector(1-1 downto 0) ;
signal axi_b_dly_cnt_reg,    axi_b_dly_cnt_next    : unsigned(32-1 downto 0) ;
signal axi_b_dly_do_reg,    axi_b_dly_do_next     : std_logic       ;
signal axi_b_clr                                  : std_logic          ;
signal axi_b_cur_addr                             : std_logic     ;

-----------------------------------------------------------------------------------
--  Trigger source selector

signal adc_trig_ap_reg,  adc_trig_ap_next        : std_logic                       ;
signal adc_trig_an_reg,  adc_trig_an_next       : std_logic                       ;
signal adc_trig_bp_reg,  adc_trig_bp_next       : std_logic                       ;
signal adc_trig_bn_reg,  adc_trig_bn_next       : std_logic                       ;
signal adc_trig_sw_reg,  adc_trig_sw_next       : std_logic                       ;
signal set_trig_src_reg, set_trig_src_next      : std_logic_vector(4-1 downto 0)  ;
signal ext_trig_p                               : std_logic                       ;
signal ext_trig_n                               : std_logic                       ;
signal asg_trig_p                               : std_logic                       ;
signal asg_trig_n                               : std_logic                       ;

-----------------------------------------------------------------------------------
--  Trigger created from input signal

signal adc_scht_ap_reg,  adc_scht_ap_next       : std_logic_vector(2-1 downto 0)  ;
signal adc_scht_an_reg,  adc_scht_an_next       : std_logic_vector(2-1 downto 0)  ;
signal adc_scht_bp_reg,  adc_scht_bp_next       : std_logic_vector(2-1 downto 0)  ;
signal adc_scht_bn_reg,  adc_scht_bn_next       : std_logic_vector(2-1 downto 0)  ;
signal set_a_tresh_reg,  set_a_tresh_next       : std_logic_vector(14-1 downto 0) ;
signal set_a_treshp_reg, set_a_treshp_next      : std_logic_vector(14-1 downto 0) ;
signal set_a_treshm_reg, set_a_treshm_next      : std_logic_vector(14-1 downto 0) ;
signal set_b_tresh_reg,  set_b_tresh_next       : std_logic_vector(14-1 downto 0) ;
signal set_b_treshp_reg, set_b_treshp_next      : std_logic_vector(14-1 downto 0) ;
signal set_b_treshm_reg, set_b_treshm_next      : std_logic_vector(14-1 downto 0) ;
signal set_a_hyst_reg,   set_a_hyst_next        : std_logic_vector(14-1 downto 0) ;--todo:verify that it is not signed signal
signal set_b_hyst_reg,   set_b_hyst_next        : std_logic_vector(14-1 downto 0) ;--todo:verify that it is not signed signal

-----------------------------------------------------------------------------------
--  External trigger

signal ext_trig_in_reg,  ext_trig_in_next       : std_logic_vector(3-1 downto 0 ) ;
signal ext_trig_dp_reg,  ext_trig_dp_next       : std_logic_vector(2-1 downto 0 ) ;
signal ext_trig_dn_reg,  ext_trig_dn_next       : std_logic_vector(2-1 downto 0 ) ;
signal ext_trig_debp_reg, ext_trig_debp_next    : std_logic_vector(20-1 downto 0) ;
signal ext_trig_debn_reg, ext_trig_debn_next    : std_logic_vector(20-1 downto 0) ;
signal asg_trig_in_reg,  asg_trig_in_next       : std_logic_vector(3-1 downto 0 ) ;
signal asg_trig_dp_reg,  asg_trig_dp_next       : std_logic_vector(2-1 downto 0 ) ;
signal asg_trig_dn_reg,  asg_trig_dn_next       : std_logic_vector(2-1 downto 0 ) ;
signal asg_trig_debp_reg, asg_trig_debp_next    : std_logic_vector(20-1 downto 0) ;
signal asg_trig_debn_reg, asg_trig_debn_next    : std_logic_vector(20-1 downto 0) ;
                                                           
signal sys_en                                   : std_logic                       ;
signal sys_rdata_i_reg, sys_rdata_i_next	: std_logic_vector(32-1 downto 0) ; 
signal sys_err_i_reg, sys_err_i_next            : std_logic                       ;
signal sys_ack_i_reg, sys_ack_i_next            : std_logic                       ;

begin                                                     

-----------------------------------------------------------------------------------
--  Input filtering
adc_a_filt_in <= adc_a_i ;
adc_b_filt_in <= adc_b_i ;

i_dfilt1_cha: entity work.red_pitaya_dfilt1 
port map(
   -- ADC
  adc_clk_i   => adc_clk_i,      -- ADC clock
  adc_rstn_i  => adc_rstn_i,      -- ADC reset - active low
  adc_dat_i   => adc_a_filt_in,   -- ADC data
  adc_dat_o   => adc_a_filt_out,  -- ADC data
  -- configuration
  cfg_aa_i    => set_a_filt_aa_reg,   -- config AA coefficient
  cfg_bb_i    => set_a_filt_bb_reg,   -- config BB coefficient
  cfg_kk_i    => set_a_filt_kk_reg,   -- config KK coefficient
  cfg_pp_i    => set_a_filt_pp_reg    -- config PP coefficient
);

i_dfilt1_chb: entity work.red_pitaya_dfilt1
port map(
   -- ADC
  adc_clk_i   => adc_clk_i       ,  -- ADC clock
  adc_rstn_i  => adc_rstn_i      ,  -- ADC reset - active low
  adc_dat_i   => adc_b_filt_in   ,  -- ADC data
  adc_dat_o   => adc_b_filt_out  ,  -- ADC data
  -- configuration
  cfg_aa_i    => set_b_filt_aa_reg   ,  -- config AA coefficient
  cfg_bb_i    => set_b_filt_bb_reg   ,  -- config BB coefficient
  cfg_kk_i    => set_b_filt_kk_reg   ,  -- config KK coefficient
  cfg_pp_i    => set_b_filt_pp_reg      -- config PP coefficient
);

-----------------------------------------------------------------------------------
--  Decimate input data
process(adc_clk_i, adc_rstn_i)
begin
if (adc_rstn_i = '0') then
  adc_a_sum_reg   <= (others => '0');
  adc_b_sum_reg   <= (others => '0');
  adc_dec_cnt_reg <= (others => '0') ;
  adc_dv_reg      <=  '0' ;
  adc_a_dat_reg   <= (others => '0'); 
  adc_b_dat_reg   <= (others => '0'); 
elsif (rising_edge(adc_clk_i)) then
  adc_a_sum_reg   <=  adc_a_sum_next;
  adc_b_sum_reg   <=  adc_b_sum_next;
  adc_dec_cnt_reg <=  adc_dec_cnt_next;
  adc_dv_reg      <=  adc_dv_next;
  adc_a_dat_reg   <=  adc_a_dat_next; 
  adc_b_dat_reg   <=  adc_b_dat_next;
end if;
end process;

  --next state logic
  adc_dec_cnt_next  <=  to_unsigned(1, adc_dec_cnt_next'length) when ((adc_dec_cnt_reg >= unsigned(set_dec_reg)) or (adc_arm_do_reg = '1')) else -- start again or arm
                        adc_dec_cnt_reg + 1;
  adc_a_sum_next    <=  signed(adc_a_filt_out) when ((adc_dec_cnt_reg >= unsigned(set_dec_reg)) or (adc_arm_do_reg = '1')) else
                        adc_a_sum_reg + signed(adc_a_filt_out);
  adc_b_sum_next    <=  signed(adc_b_filt_out) when ((adc_dec_cnt_reg >= unsigned(set_dec_reg)) or (adc_arm_do_reg = '1')) else
                        adc_b_sum_reg + signed(adc_b_filt_out);

  adc_dv_next <=  '1' when (adc_dec_cnt_reg >= unsigned(set_dec_reg)) else
                  '0';
  set_dec_aux <= (set_dec_reg and ( set_avg_en_reg & set_avg_en_reg & set_avg_en_reg & set_avg_en_reg & set_avg_en_reg & set_avg_en_reg & set_avg_en_reg & set_avg_en_reg & 
                  set_avg_en_reg & set_avg_en_reg & set_avg_en_reg & set_avg_en_reg & set_avg_en_reg & set_avg_en_reg & set_avg_en_reg & set_avg_en_reg & set_avg_en_reg));

	with set_dec_aux select
--todo: check if the 15 is ok. For my it must be 14
    adc_a_dat_next <= adc_a_filt_out                      when  std_logic_vector(to_unsigned(16#0#, set_dec_reg'length)), --17'h0
                      adc_a_sum_reg(15+0 downto 0)        when  std_logic_vector(to_unsigned(16#1#, set_dec_reg'length)), --17'h1     
                      adc_a_sum_reg(15+3 downto 3)        when  std_logic_vector(to_unsigned(16#8#, set_dec_reg'length)), --17'h8     
                      adc_a_sum_reg(15+6 downto  6)       when  std_logic_vector(to_unsigned(16#40#, set_dec_reg'length)),      
                      adc_a_sum_reg(15+10 downto 10)      when  std_logic_vector(to_unsigned(16#400#, set_dec_reg'length)),       
                      adc_a_sum_reg(15+13 downto 13)      when  std_logic_vector(to_unsigned(16#2000#, set_dec_reg'length)),     
                      adc_a_sum_reg(15+16 downto 16)      when  std_logic_vector(to_unsigned(16#10000#, set_dec_reg'length)),     
                      adc_a_sum_reg(15+0 downto  0)       when  others ;      

	with set_dec_aux select
--todo: check if the 15 is ok. For my it must be 14
    adc_b_dat_next <= adc_b_filt_out                      when  std_logic_vector(to_unsigned(16#0#, set_dec_reg'length)), --17'h0
                      adc_b_sum_reg(15+0 downto 0)        when  std_logic_vector(to_unsigned(16#1#, set_dec_reg'length)), --17'h1     
                      adc_b_sum_reg(15+3 downto 3)        when  std_logic_vector(to_unsigned(16#8#, set_dec_reg'length)), --17'h8     
                      adc_b_sum_reg(15+6 downto  6)       when  std_logic_vector(to_unsigned(16#40#, set_dec_reg'length)),      
                      adc_b_sum_reg(15+10 downto 10)      when  std_logic_vector(to_unsigned(16#400#, set_dec_reg'length)),       
                      adc_b_sum_reg(15+13 downto 13)      when  std_logic_vector(to_unsigned(16#2000#, set_dec_reg'length)),     
                      adc_b_sum_reg(15+16 downto 16)      when  std_logic_vector(to_unsigned(16#10000#, set_dec_reg'length)),     
                      adc_b_sum_reg(15+0 downto  0)       when  others ;      

-----------------------------------------------------------------------------------
--  ADC buffer RAM

-- Write
process(adc_clk_i, adc_rstn_i)
begin
if (adc_rstn_i = '0') then
  adc_wp_reg      <= (others => '0');
  adc_we_reg      <=  '0'    ;
  adc_wp_trig_reg <= (others => '0');
  adc_wp_cur_reg  <= (others => '0');
  adc_we_cnt_reg  <= (others => '0')     ;
  adc_dly_cnt_reg <= (others => '0')     ;
  adc_dly_do_reg  <=  '0'     ;
elsif (rising_edge(adc_clk_i)) then
  adc_wp_reg      <= adc_wp_next;     
  adc_we_reg      <= adc_we_next;     
  adc_wp_trig_reg <= adc_wp_trig_next;
  adc_wp_cur_reg  <= adc_wp_cur_next; 
  adc_we_cnt_reg  <= adc_we_cnt_next; 
  adc_dly_cnt_reg <= adc_dly_cnt_next;
  adc_dly_do_reg  <= adc_dly_do_next; 
end if;
end process;

  -- next state logic
  adc_we_next <= '1' when (adc_arm_do_reg = '1') else
                 '0' when (((adc_dly_do_reg = '1') or (adc_trig_reg = '1')) and (adc_dly_cnt_reg = 0) and ((not adc_we_keep_reg = '1') or (adc_rst_do_reg = '1'))) else --delayed reached or reset
                  adc_we_reg; --mantain value
  
  -- count how much data was written into the buffer before trigger
  adc_we_cnt_next <=  (others => '0') when ((adc_rst_do_reg = '1') or (adc_arm_do_reg = '1')) else 
                      adc_we_cnt_reg + 1 when ((adc_we_reg = '1') and ( not adc_dly_do_reg = '1') and (adc_dv_reg = '1') and (adc_we_cnt_reg /= (others => '1'))) else 
                      adc_we_cnt_reg; -- mantain value
  
  adc_wp_next <= (others => '0') when (adc_rst_do_reg = '1') else
                adc_wp_reg + 1 when ((adc_we_reg = '1') and (adc_dv_reg = '1')) else
                adc_wp_reg; -- mantain value

  adc_wp_trig_next <= (others => '0') when (adc_rst_do_reg = '1') else
                  adc_wp_cur_reg when ((adc_trig_reg = '1') and  (not adc_dly_do_reg = '1')) else -- save write pointer at trigger arrival
                  adc_wp_trig_reg; -- mantain value 
  
  adc_wp_cur_next <= (others => '0') when (adc_rst_do_reg = '1') else
                      adc_wp_reg when ((adc_we_reg = '1') and (adc_dv_reg = '1')) else -- save current write pointer
                      adc_wp_cur_reg; -- mantain value  
  
  adc_dly_do_next  <= '1' when (adc_trig_reg = '1') else
                      '0' when (((adc_dly_do_reg = '1') and (adc_dly_cnt_reg = 0)) or (adc_rst_do_reg = '1') or (adc_arm_do_reg = '1')) else -- delayed reached or reset
                      adc_dly_do_reg;
  
  adc_dly_cnt_next <= adc_dly_cnt_reg - 1 when ((adc_dly_do_reg = '1') and (adc_we_reg = '1') and (adc_dv_reg = '1')) else
                      unsigned(set_dly_reg) when (not adc_dly_do_reg = '1') else
                      adc_dly_cnt_reg; -- mantain value

--todo: check that part
process(adc_clk_i)
begin
if (rising_edge(adc_clk_i)) then
   if ((adc_we_reg = '1') and (adc_dv_reg = '1')) then
      adc_a_buf(to_integer(adc_wp_reg)) <= adc_a_dat_reg ;
      adc_b_buf(to_integer(adc_wp_reg)) <= adc_b_dat_reg ;
   end if;
end if;
end process;

-- Read
process(adc_clk_i, adc_rstn_i)
begin
if (rising_edge(adc_clk_i)) then
  if (adc_rstn_i = '0') then
    adc_rval_reg <= (others => '0') ;
  else
    adc_rval_reg <= adc_rval_next;
  end if;
end if;
end process;

  --next state logic
  adc_rval_next <= adc_rval_reg(2 downto 0) & (sys_ren or sys_wen);

  adc_rd_dv <= adc_rval_reg(3);

process(adc_clk_i)
begin
  if (rising_edge(adc_clk_i)) then
   adc_raddr_reg   <= adc_raddr_next;  
   adc_a_raddr_reg <= adc_a_raddr_next;
   adc_b_raddr_reg <= adc_b_raddr_next;
   adc_a_rd_reg    <= adc_a_rd_next;
   adc_b_rd_reg    <= adc_b_rd_next;
  end if;
end process;

-- next state logic
  adc_raddr_next   <= sys_addr(RSZ+1 downto 2)                          ; -- address synchronous to clock
  adc_a_raddr_next <= adc_raddr_reg                                     ; -- double register 
  adc_b_raddr_next <= adc_raddr_reg                                     ; -- otherwise memory corruption at reading
  adc_a_rd_next    <= adc_a_buf(to_integer(unsigned(adc_a_raddr_reg)))  ;
  adc_b_rd_next    <= adc_b_buf(to_integer(unsigned(adc_b_raddr_reg)))  ;

-----------------------------------------------------------------------------------
--
--  AXI CHA connection

axi_a_clr <= adc_rst_do_reg ;

process(axi0_clk_o, axi0_rstn_o)
begin
 if (axi0_rstn_o = '0') then
    axi_a_we_reg      <= '0' ;
    axi_a_dat_reg     <= (others => '0');
    axi_a_dat_sel_reg <=  (others => '0');
    axi_a_dat_dv_reg  <= (others => '0') ;
    axi_a_dly_cnt_reg <= (others => '0') ;
    axi_a_dly_do_reg  <= '0' ;
    set_a_axi_cur_reg <= (others => '0');
 elsif (rising_edge(axi0_clk_o)) then
    axi_a_we_reg      <= axi_a_we_next;      
    axi_a_dat_reg     <= axi_a_dat_next;     
    axi_a_dat_sel_reg <= axi_a_dat_sel_next; 
    axi_a_dat_dv_reg  <= axi_a_dat_dv_next;  
    axi_a_dly_cnt_reg <= axi_a_dly_cnt_next; 
    axi_a_dly_do_reg  <= axi_a_dly_do_next;  
    set_a_axi_cur_reg <= set_a_axi_cur_next;
  end if;
end process;

  --next state logic
  axi_a_we_next <=  '1' when ((adc_arm_do_reg = '1') and (set_a_axi_en_reg = '1')) else
                    '0' when ((((axi_a_dly_do_reg = '1') or (adc_trig_reg = '1')) and (axi_a_dly_cnt_reg = 0)) or (adc_rst_do_reg = '1')) else --delayed reached or reset
                    axi_a_we_reg; -- mantain value

  axi_a_dly_do_next  <= '1' when ((adc_trig_reg = '1') and (axi_a_we_reg = '1')) else
                        '0' when (((axi_a_dly_do_reg = '1') and (axi_a_dly_cnt_reg = 0)) or (axi_a_clr = '1') or (adc_arm_do_reg = '1')) else --delayed reached or reset
                        axi_a_dly_do_reg;

  axi_a_dly_cnt_next <= axi_a_dly_cnt_reg - 1        when ((axi_a_dly_do_reg = '1') and (axi_a_we_reg = '1') and (adc_dv_reg = '1')) else
                        unsigned(set_a_axi_dly_reg)  when (not axi_a_dly_do_reg = '1') else
                        axi_a_dly_cnt_reg; -- mantain value

  axi_a_dat_sel_next <= (others => '0')       when (axi_a_clr = '1') else
                        axi_a_dat_sel_reg + 1 when ((axi_a_we_reg = '1') and (adc_dv_reg = '1')) else
                        axi_a_dat_sel_reg; -- mantain value

  axi_a_dat_dv_next <=  (others => '1') when ((axi_a_we_reg = '1') and (axi_a_dat_sel_reg = 3) and (adc_dv_reg = '1')) else 
                        (others => '0');

  axi_a_dat_next(16-1 downto 0) <=  adc_a_dat_reg when ((axi_a_we_reg = '1') and (adc_dv_reg = '1') and (axi_a_dat_sel_reg = "00")) else axi_a_dat_reg(16-1 downto 0); --mantain value 
  axi_a_dat_next(32-1 downto 16) <= adc_a_dat_reg when ((axi_a_we_reg = '1') and (adc_dv_reg = '1') and (axi_a_dat_sel_reg = "01")) else axi_a_dat_reg(32-1 downto 16); --mantain value  
  axi_a_dat_next(48-1 downto 32) <= adc_a_dat_reg when ((axi_a_we_reg = '1') and (adc_dv_reg = '1') and (axi_a_dat_sel_reg = "10")) else axi_a_dat_reg(48-1 downto 32); --mantain value  
  axi_a_dat_next(64-1 downto 48) <= adc_a_dat_reg when ((axi_a_we_reg = '1') and (adc_dv_reg = '1') and (axi_a_dat_sel_reg = "11")) else axi_a_dat_reg(64-1 downto 48); --mantain value  

  set_a_axi_trig_next <=  (others => '0')                                       when (axi_a_clr = '1') else
                          (31-3 => axi_a_cur_addr(32-1 downto 3), 2-1 => std_logic_vector(axi_a_dat_sel_reg), 0 => '0') when ((adc_trig_reg = '1') and (not axi_a_dly_do_reg = '1') and (axi_a_we_reg = '1')) else -- save write pointer at trigger arrival
                          set_a_axi_trig_reg; --mantain value

  set_a_axi_cur_next <= set_a_axi_start_reg when (axi_a_clr = '1')     else
                        axi_a_cur_addr      when (axi0_wvalid_o = '1') else
                        set_a_axi_cur_reg; --mantain value

i_wr0: entity work.axi_wr_fifo 
generic map(
  DW  =>  64    , -- data width (8,16,...,1024)
  AW  =>  32    , -- address width
  FW  =>   8      -- address width of FIFO pointers
) 
port map(
   -- global signals
  axi_clk_i          =>  axi0_clk_o        , -- global clock
  axi_rstn_i         =>  axi0_rstn_o       , -- global reset

  -- Connection to AXI master
  axi_waddr_o        =>  axi0_waddr_o      , -- write address
  axi_wdata_o        =>  axi0_wdata_o      , -- write data
  axi_wsel_o         =>  axi0_wsel_o       , -- write byte select
  axi_wvalid_o       =>  axi0_wvalid_o     , -- write data valid
  axi_wlen_o         =>  axi0_wlen_o       , -- write burst length
  axi_wfixed_o       =>  axi0_wfixed_o     , -- write burst type (fixed / incremental)
  axi_werr_i         =>  axi0_werr_i       , -- write error
  axi_wrdy_i         =>  axi0_wrdy_i       , -- write ready

  -- data and configuration
  wr_data_i          =>  axi_a_dat_reg         , -- write data
  wr_val_i           =>  axi_a_dat_dv_reg      , -- write data valid
  ctrl_start_addr_i  =>  set_a_axi_start_reg   , -- range start address
  ctrl_stop_addr_i   =>  set_a_axi_stop_reg    , -- range stop address
  ctrl_trig_size_i   =>  (others => '1')       , -- trigger level
  ctrl_wrap_i        =>  "1" 	             , -- start from begining when reached stop
  ctrl_clr_i         =>  axi_a_clr         , -- clear / flush
  stat_overflow_o    =>  open                  , -- overflow indicator
  stat_cur_addr_o    =>  axi_a_cur_addr    , -- current write address
  stat_write_data_o  =>  open                    -- write data indicator
);

 axi0_clk_o  <= adc_clk_i ;
 axi0_rstn_o <= adc_rstn_i;

-----------------------------------------------------------------------------------
--
--  AXI CHB connection

axi_b_clr = adc_rst_do_reg ;
process(axi1_clk_o, axi1_rstn_o)
begin
 if (axi1_rstn_o = '0') then
    axi_b_we_reg      <= '0' ;
    axi_b_dat_reg     <= (others => '0');
    axi_b_dat_sel_reg <=  (others => '0');
    axi_b_dat_dv_reg  <= (others => '0') ;
    axi_b_dly_cnt_reg <= (others => '0') ;
    axi_b_dly_do_reg  <= '0' ;
    set_b_axi_cur_reg <= (others => '0');
 elsif (rising_edge(axi1_clk_o)) then
    axi_b_we_reg      <= axi_b_we_next;      
    axi_b_dat_reg     <= axi_b_dat_next;     
    axi_b_dat_sel_reg <= axi_b_dat_sel_next; 
    axi_b_dat_dv_reg  <= axi_b_dat_dv_next;  
    axi_b_dly_cnt_reg <= axi_b_dly_cnt_next; 
    axi_b_dly_do_reg  <= axi_b_dly_do_next;  
    set_b_axi_cur_reg <= set_b_axi_cur_next;
  end if;
end process;

  --next state logic
  axi_b_we_next <=  '1' when ((adc_arm_do_reg = '1') and (set_b_axi_en_reg = '1')) else
      	          	'0' when ((axi_b_dly_do_reg = '1') or (adc_trig_reg = '1') and 
                             (axi_b_dly_cnt_reg = 0) or (adc_rst_do_reg = '1')) else --delayed reached or reset
                   	axi_b_we_reg; -- mantain value

  axi_b_dly_do_next  <= '1' when ((adc_trig_reg = '1') and (axi_b_we_reg = '1')) else
                       	'0' when ((axi_b_dly_do_reg = '1') and (axi_b_dly_cnt_reg = 0) or 
                                 ((axi_b_clr = '1') or (adc_arm_do_reg = '1'))) else --delayed reached or reset
                       	axi_b_dly_do_reg;

  axi_b_dly_cnt_next <= axi_b_dly_cnt_reg - 1 when ((axi_b_dly_do_reg = '1') and ((axi_b_we_reg = '1') and (adc_dv_reg = '1'))) else
                       	unsigned(set_b_axi_dly_reg)     when (not axi_b_dly_do = '1') else
                       	axi_b_dly_cnt_reg; -- mantain value

  axi_b_dat_sel_next <= (others => '0')       when (axi_b_clr = '1') else
                        axi_b_dat_sel_reg + 1 when ((axi_b_we_reg = '1') and (adc_dv_reg = '1')) else
                        axi_b_dat_sel_reg; -- mantain value

  axi_b_dat_dv_next <=  (others => '1') when ((axi_b_we_reg = '1') and (axi_b_dat_sel_reg = 3) and (adc_dv_reg = '1')) else 
                        (others => '0');

  axi_b_dat_next(16-1 downto 0) <=  signed(adc_b_dat) when ((axi_b_we_reg = '1') and (adc_dv_reg = '1') and (axi_b_dat_sel = "00")) else
                                    axi_b_dat_reg(16-1 downto 0); --mantain value 
  axi_b_dat_next(32-1 downto 16) <= signed(adc_b_dat) when ((axi_b_we_reg = '1') and (adc_dv_reg = '1') and (axi_b_dat_sel = "01")) else
                                    axi_b_dat_reg(32-1 downto 16); --mantain value  
  axi_b_dat_next(48-1 downto 32) <= signed(adc_b_dat) when ((axi_b_we_reg = '1') and (adc_dv_reg = '1') and (axi_b_dat_sel = "10")) else
                                    axi_b_dat_reg(48-1 downto 32); --mantain value  
  axi_b_dat_next(64-1 downto 48) <= signed(adc_b_dat) when ((axi_b_we_reg = '1') and (adc_dv_reg = '1') and (axi_b_dat_sel = "11")) else
                                    axi_b_dat_reg(64-1 downto 48); --mantain value  

  set_b_axi_trig_next <=  (others => '0')                                       when (axi_b_clr = '1') else
                          (axi_b_cur_addr(32-1 downto 3) & axi_b_dat_sel & '0') when ((adc_trig_reg = '1') and 
                                                                                      (not axi_b_dly_do = '1') and (axi_b_we_reg = '1')) else -- save write pointer at trigger arrival
                          set_b_axi_trig_reg; --mantain value

  set_b_axi_cur_next <= set_b_axi_start when (axi_b_clr = '1') else
                        axi_b_cur_addr  when (axi1_wvalid_o = '1') else
                        set_b_axi_cur_reg; --mantain value


i_wr1: entity work.axi_wr_fifo 
generic map(
  DW  =>  64    , -- data width (8,16,...,1024)
  AW  =>  32    , -- address width
  FW  =>   8      -- address width of FIFO pointers
) 
port map(
   -- global signals
  axi_clk_i          =>  axi1_clk_o        , -- global clock
  axi_rstn_i         =>  axi1_rstn_o       , -- global reset

  -- Connection to AXI master
  axi_waddr_o        =>  axi1_waddr_o      , -- write address
  axi_wdata_o        =>  axi1_wdata_o      , -- write data
  axi_wsel_o         =>  axi1_wsel_o       , -- write byte select
  axi_wvalid_o       =>  axi1_wvalid_o     , -- write data valid
  axi_wlen_o         =>  axi1_wlen_o       , -- write burst length
  axi_wfixed_o       =>  axi1_wfixed_o     , -- write burst type (fixed / incremental)
  axi_werr_i         =>  axi1_werr_i       , -- write error
  axi_wrdy_i         =>  axi1_wrdy_i       , -- write ready

  -- data and configuration
  wr_data_i          =>  axi_b_dat_reg         , -- write data
  wr_val_i           =>  axi_b_dat_dv_reg      , -- write data valid
  ctrl_start_addr_i  =>  set_b_axi_start_reg   , -- range start address
  ctrl_stop_addr_i   =>  set_b_axi_stop_reg    , -- range stop address
  ctrl_trig_size_i   =>  (others => '1')       , -- trigger level
  ctrl_wrap_i        =>  "1"              , -- start from begining when reached stop
  ctrl_clr_i         =>  axi_b_clr         , -- clear / flush
  stat_overflow_o    =>  open              , -- overflow indicator
  stat_cur_addr_o    =>  axi_b_cur_addr    , -- current write address
  stat_write_data_o  =>  open                -- write data indicator
);

axi1_clk_o  <= adc_clk_i ;
axi1_rstn_o <= adc_rstn_i;

-----------------------------------------------------------------------------------
--  Trigger source selector

process(adc_clk_i, adc_rstn_i)
begin
  if (adc_rstn_i = '0') then
    adc_arm_do_reg    <= '0' ;
    adc_rst_do_reg    <= '0' ;
    adc_trig_sw_reg   <= '0' ;
    set_trig_src_reg  <= (others => '0');
    adc_trig_reg      <= '0' ;
  elsif (rising_edge(adc_clk_i)) then
    adc_arm_do_reg    <=  adc_arm_do_next;
    adc_rst_do_reg    <=  adc_rst_do_next;
    adc_trig_sw_reg   <=  adc_trig_sw_next;  
    set_trig_src_reg  <=  adc_trig_src_next;  
    adc_trig_reg      <=  adc_trig_next;  
  end if;
end process;

  -- next state logic
  adc_arm_do_next  <= '1' when ((sys_wen = '1') and (sys_addr(19 downto 0) = std_logic_vector(to_unsigned(0, sys_addr(19 downto 0)'length)) and (sys_wdata(0) = '1')) else -- SW ARM
                      '0';
  
  adc_rst_do_next  <= '1' when ((sys_wen = '1') and (sys_addr(19 downto 0) = std_logic_vector(to_unsigned(0, sys_addr(19 downto 0)'length)) and (sys_wdata(1) = '1')) else
                      '0';
  adc_trig_sw_next <= '1' when ((sys_wen = '1') and (sys_addr(19 downto 0) = std_logic_vector(to_unsigned(16#4#, sys_addr(19 downto 0)'length)) and
                                (sys_wdata(3 downto 0) = std_logic_vector(to_unsigned(1, sys_wdata(3 downto 0)'length))))) else -- SW trigger
                      '0';

  set_trig_src_next <=  sys_wdata(4-1 downto 0) when ((sys_wen = '1') and (sys_addr(20-1 downto 0) = std_logic_vector(to_unsigned(16#4#, sys_addr(19 downto 0)'length)))) else 
                        (others => '0')       when ((adc_dly_do_reg = '1') or (adc_trig_reg = '1') and (adc_dly_cnt_reg = 0)) or (adc_rst_do_reg = '1')) else --delayed reached or reset
                        set_trig_src_reg ;

  with (set_trig_src_reg) select
    adc_trig_next <=  adc_trig_sw_reg when std_logic_vector(to_unsigned(1, set_trig_src_reg'length), -- manual
                      adc_trig_ap_reg when std_logic_vector(to_unsigned(2, set_trig_src_reg'length), -- A ch rising edge
                      adc_trig_an_reg when std_logic_vector(to_unsigned(3, set_trig_src_reg'length), -- A ch falling edge
                      adc_trig_bp_reg when std_logic_vector(to_unsigned(4, set_trig_src_reg'length), -- B ch rising edge
                      adc_trig_bn_reg when std_logic_vector(to_unsigned(5, set_trig_src_reg'length), -- B ch falling edge
                      ext_trig_p      when std_logic_vector(to_unsigned(6, set_trig_src_reg'length),  -- external - rising edge
                      ext_trig_n      when std_logic_vector(to_unsigned(7, set_trig_src_reg'length), -- external - falling edge
                      asg_trig_p      when std_logic_vector(to_unsigned(8, set_trig_src_reg'length), -- ASG - rising edge
                      asg_trig_n      when std_logic_vector(to_unsigned(9, set_trig_src_reg'length), -- ASG - falling edge
                      '0'             when others;

-----------------------------------------------------------------------------------
--  Trigger created from input signal

process(adc_clk_i, adc_rstn_i)
begin
if (adc_rstn_i = '0') then
  adc_scht_ap_reg  <=  (others => '0') ;
  adc_scht_an_reg  <=  (others => '0') ;
  adc_scht_bp_reg  <=  (others => '0') ;
  adc_scht_bn_reg  <=  (others => '0') ;
  adc_trig_ap_reg  <=  '0' ;
  adc_trig_an_reg  <=  '0' ;
  adc_trig_bp_reg  <=  '0' ;
  adc_trig_bn_reg  <=  '0' ;
  set_a_treshp_reg <=  (others => '0') ;
  set_a_treshm_reg <=  (others => '0') ;
  set_b_treshp_reg <=  (others => '0') ;
  set_b_treshm_reg <=  (others => '0') ;
elsif (rising_edge(adc_clk_i)) then
  adc_scht_ap_reg  <= adc_scht_ap_next; 
  adc_scht_an_reg  <= adc_scht_an_next; 
  adc_scht_bp_reg  <= adc_scht_bp_next; 
  adc_scht_bn_reg  <= adc_scht_bn_next; 
  adc_trig_ap_reg  <= adc_trig_ap_next; 
  adc_trig_an_reg  <= adc_trig_an_next; 
  adc_trig_bp_reg  <= adc_trig_bp_next; 
  adc_trig_bn_reg  <= adc_trig_bn_next; 
  set_a_treshp_reg <= set_a_treshp_next;
  set_a_treshm_reg <= set_a_treshm_next;
  set_b_treshp_reg <= set_b_treshp_next;
  set_b_treshm_reg <= set_b_treshm_next;
end if;
end process;

  --next state logic
  set_a_treshp_next <= unsigned(set_a_tresh_reg) + unsigned(set_a_hyst_reg) ; -- calculate positive
  set_a_treshm_next <= unsigned(set_a_tresh_reg) - unsigned(set_a_hyst_reg) ; -- and negative treshold
  set_b_treshp_next <= unsigned(set_b_tresh_reg) + unsigned(set_b_hyst_reg) ;
  set_b_treshm_next <= unsigned(set_b_tresh_reg) - unsigned(set_b_hyst_reg) ;

  adc_scht_ap_next(0) <= 	'1' when ((adc_dv_reg = '1') and (signed(adc_a_dat_reg) >= signed(set_a_tresh_reg))) else  -- treshold reached
													'0' when ((adc_dv_reg = '1') and (signed(adc_a_dat_reg) <  signed(set_a_treshm_reg))) else  -- wait until it goes under hysteresis
													adc_scht_ap_reg(0); --mantain value
 	adc_scht_an_next(0) <= 	'1' when ((adc_dv_reg = '1') and (signed(adc_a_dat_reg) <= signed(set_a_tresh_reg))) else        -- treshold reached
     											'0' when ((adc_dv_reg = '1') and (signed(adc_a_dat_reg) > (signed(set_a_treshp_reg))) else  -- wait until it goes over hysteresis
													adc_scht_an_reg(0); --mantain value
  adc_scht_bp_next(0) <= 	'1' when ((adc_dv_reg = '1') and (signed(adc_b_dat_reg) >= signed(set_b_tresh_reg))) else  -- treshold reached
													'0' when ((adc_dv_reg = '1') and (signed(adc_b_dat_reg) <  signed(set_b_treshm_reg))) else  -- wait until it goes under hysteresis
													adc_scht_bp_reg(0); --mantain value
 	adc_scht_bn_next(0) <= 	'1' when ((adc_dv_reg = '1') and (signed(adc_b_dat_reg) <= signed(set_b_tresh_reg))) else        -- treshold reached
     											'0' when ((adc_dv_reg = '1') and (signed(adc_b_dat_reg) > (signed(set_b_treshp_reg))) else  -- wait until it goes over hysteresis
													adc_scht_bn_reg(0); --mantain value

  adc_scht_ap_next(1) <= adc_scht_ap_reg(0) ;
  adc_scht_an_next(1) <= adc_scht_an_reg(0) ;
  adc_scht_bp_next(1) <= adc_scht_bp_reg(0) ;
  adc_scht_bn_next(1) <= adc_scht_bn_reg(0) ;

  adc_trig_ap_next <= ((adc_scht_ap_reg(0)) and (not adc_scht_ap_reg(1)) ; -- make 1 cyc pulse 
  adc_trig_an_next <= ((adc_scht_an_reg(0)) and (not adc_scht_an_reg(1)) ;
  adc_trig_bp_next <= ((adc_scht_bp_reg(0)) and (not adc_scht_bp_reg(1)) ;
  adc_trig_bn_next <= ((adc_scht_bn_reg(0)) and (not adc_scht_bn_reg(1)) ;

-----------------------------------------------------------------------------------
--  External trigger

process(adc_clk_i, adc_rstn_i)
begin
if (adc_rstn_i = '0') then
   ext_trig_in_reg   <= (others => '0');
   ext_trig_dp_reg   <= (others => '0') ;
   ext_trig_dn_reg   <= (others => '0') ;
   ext_trig_debp_reg <= (others => '0') ;
   ext_trig_debn_reg <= (others => '0') ;
   asg_trig_in_reg   <= (others => '0') ;
   asg_trig_dp_reg   <= (others => '0') ;
   asg_trig_dn_reg   <= (others => '0') ;
   asg_trig_debp_reg <= (others => '0') ;
   asg_trig_debn_reg <= (others => '0');
elsif (rising_edge(adc_clk_i) then
   ext_trig_in_reg   <= ext_trig_in_next;
   ext_trig_dp_reg   <= ext_trig_dp_next; 
   ext_trig_dn_reg   <= ext_trig_dn_next; 
   ext_trig_debp_reg <= ext_trig_debp_next; 
   ext_trig_debn_reg <= ext_trig_debn_next; 
   asg_trig_in_reg   <= asg_trig_in_next; 
   asg_trig_dp_reg   <= asg_trig_dp_next; 
   asg_trig_dn_reg   <= asg_trig_dn_next; 
   asg_trig_debp_reg <= asg_trig_debp_next; 
   asg_trig_debn_reg <= asg_trig_debn_next; 
end if;
end process;

  -- next state logic
   ------------- External trigger
   -- synchronize FFs
   ext_trig_in_next <= ext_trig_in_reg(1 downto 0) & trig_ext_i ;

   -- look for input changes
	ext_trig_debp_next <= set_deb_len_reg when ((ext_trig_debp_reg = std_logic_vector(to_unsigned(0, ext_trig_debp_reg'length))) and ((ext_trig_in(1) = '1') and (not ext_trig_in(2) = '1'))) else -- ~0.5ms
												std_logic_vector(unsigned(ext_trig_debp_reg) - 1) when (ext_trig_debp_reg /= std_logic_vector(to_unsigned(0, ext_trig_debp_reg'length))) else
												ext_trig_debp_reg; --mantain value
	ext_trig_debn_next <= set_deb_len_reg when ((ext_trig_debn_reg = std_logic_vector(to_unsigned(0, ext_trig_debn_reg'length))) and ((not ext_trig_in(1) = '1') and (ext_trig_in(2) = '1'))) else -- ~0.5ms
												std_logic_vector(unsigned(ext_trig_debn_reg) - 1) when (ext_trig_debn_reg /= std_logic_vector(to_unsigned(0, ext_trig_debn_reg'length))) else
												ext_trig_debn_reg; --mantain value

   -- update output values
   ext_trig_dp_next(1) <= ext_trig_dp_reg(0) ;
   ext_trig_dp_next(0) <= ext_trig_in_reg(1) when (ext_trig_debp_reg = std_logic_vector(to_unsigned(0, ext_trig_debp_reg'length))) else
													ext_trig_dp_reg(0);	--mantain value

   ext_trig_dn_next(1) <= ext_trig_dn_reg(0) ;
   ext_trig_dn_next(0) <= ext_trig_in_reg(1) when (ext_trig_debn_reg = std_logic_vector(to_unsigned(0, ext_trig_debn_reg'length))) else
													ext_trig_dn_reg(0);	--mantain value

   ------------- ASG trigger
   -- synchronize FFs
   asg_trig_in_next <= (asg_trig_in_reg(1 downto 0) & trig_asg_i) ;

   -- look for input changes
	asg_trig_debp_next <= set_deb_len_reg when ((asg_trig_debp_reg = std_logic_vector(to_unsigned(0, asg_trig_debp_reg'length))) and ((asg_trig_in(1) = '1') and (not asg_trig_in(2) = '1'))) else -- ~0.5ms
												std_logic_vector(unsigned(asg_trig_debp_reg) - 1) when (asg_trig_debp_reg /= std_logic_vector(to_unsigned(0, asg_trig_debp_reg'length))) else
												asg_trig_debp_reg; --mantain value
	asg_trig_debn_next <= set_deb_len_reg when ((asg_trig_debn_reg = std_logic_vector(to_unsigned(0, asg_trig_debn_reg'length))) and ((not asg_trig_in(1) = '1') and (asg_trig_in(2) = '1'))) else -- ~0.5ms
												std_logic_vector(unsigned(asg_trig_debn_reg) - 1) when (asg_trig_debn_reg /= std_logic_vector(to_unsigned(0, asg_trig_debn_reg'length))) else
												asg_trig_debn_reg; --mantain value

   -- update output values
   asg_trig_dp_next(1) <= asg_trig_dp_reg(0) ;
   asg_trig_dp_next(0) <= asg_trig_in_reg(1) when (asg_trig_debp_reg = std_logic_vector(to_unsigned(0, asg_trig_debp_reg'length))) else
	  											asg_trig_dp_reg(0);	--mantain value

   asg_trig_dn_next(1) <= asg_trig_dn_reg(0) ;
   asg_trig_dn_next(0) <= asg_trig_in_reg(1) when (asg_trig_debn_reg = std_logic_vector(to_unsigned(0, asg_trig_debn_reg'length))) else
													asg_trig_dn_reg(0);	--mantain value

  ext_trig_p <= '1' when (ext_trig_dp_reg = "01") else
								'0';
  ext_trig_n <= '1' when (ext_trig_dn_reg = "10") else
								'0';
  asg_trig_p <= '1' when (asg_trig_dp_reg = "01") else
								'0';
  asg_trig_n <= '1' when (asg_trig_dn_reg = "10") else
								'0';

-----------------------------------------------------------------------------------
--  System bus connection

process(adc_clk_i, adc_rstn_i)
begin
if (adc_rstn_i = '0') then
   adc_we_keep_reg   <= '0'      ;
   set_a_tresh_reg   <= std_logic_vector(to_unsigned(5000,set_a_tresh_reg'length))  ;
   set_b_tresh_reg   <= std_logic_vector(to_unsigned(5000,set_b_tresh_reg'length));--todo: verificar esta asignacion -14'd5000   ;
   set_dly_reg       <= (others => '0')      ;
   set_dec_reg       <= std_logic_vector(to_unsigned(1,set_dec_reg'length));--17'd1      ;
   set_a_hyst_reg    <= std_logic_vector(to_unsigned(20,set_a_hyst_reg'length))     ;
   set_b_hyst_reg    <= std_logic_vector(to_unsigned(20,set_b_hyst_reg'length));--14'd20     ;
   set_avg_en_reg    <= '1'      ;
   set_a_filt_aa_reg <= (others => '0');--18'h0      ;
   set_a_filt_bb_reg <= (others => '0');--25'h0      ;
   set_a_filt_kk_reg <= (others => '1');--25'hFFFFFF ;
   set_a_filt_pp_reg <= (others => '0');--25'h0      ;
   set_b_filt_aa_reg <= (others => '0');--18'h0      ;
   set_b_filt_bb_reg <= (others => '0');--25'h0      ;
   set_b_filt_kk_reg <= (others => '1');--25'hFFFFFF ;
   set_b_filt_pp_reg <= (others => '0');--25'h0      ;
   set_deb_len_reg   <= std_logic_vector(to_unsigned(62500,set_deb_len_reg'length))  ;
   set_a_axi_en_reg  <= '0';-- 1'b0      ;
   set_b_axi_en_reg  <= '0';-- 1'b0      ;
elsif (rising_edge(adc_clk_i)) then
  adc_we_keep_reg   <=  adc_we_keep_next;  
  set_a_tresh_reg   <=  set_a_tresh_next;
  set_b_tresh_reg   <=  set_b_tresh_next;
  set_dly_reg       <=  set_dly_reg_next;      
  set_dec_reg       <=  set_dec_reg_next;      
  set_a_hyst_reg    <=  set_a_hyst_next;
  set_b_hyst_reg    <=  set_b_hyst_next;
  set_avg_en_reg    <=  set_avg_en_next;
  set_a_filt_aa_reg <=  set_a_filt_aa_next;
  set_a_filt_bb_reg <=  set_a_filt_bb_next;
  set_a_filt_kk_reg <=  set_a_filt_kk_next;
  set_a_filt_pp_reg <=  set_a_filt_pp_next;
  set_b_filt_aa_reg <=  set_b_filt_aa_next;
  set_b_filt_bb_reg <=  set_b_filt_bb_next;
  set_b_filt_kk_reg <=  set_b_filt_kk_next;
  set_b_filt_pp_reg <=  set_b_filt_pp_next;
  set_deb_len_reg   <=  set_deb_len_next; 
  set_a_axi_en_reg  <=  set_a_axi_en_next; 
  set_b_axi_en_reg  <=  set_b_axi_en_next;
end if; 
end process;

	--next state logic
	adc_we_keep_next   <= 	sys_wdata(     3) 	when ((sys_wen = '1') and (sys_addr(19 downto 0) = std_logic_vector(to_unsigned(16#00#,sys_addr(19 downto 0)'length)))) else adc_we_keep_reg;
                                                                           
  set_a_tresh_next   <= sys_wdata(14-1 downto 0) when ((sys_wen = '1') and (sys_addr(19 downto 0) = std_logic_vector(to_unsigned(16#08#,sys_addr(19 downto 0)'length)))) else set_a_tresh_reg;
  set_b_tresh_next   <= sys_wdata(14-1 downto 0) when ((sys_wen = '1') and (sys_addr(19 downto 0) = std_logic_vector(to_unsigned(16#0C#,sys_addr(19 downto 0)'length)))) else 	set_b_tresh_reg;
  set_dly_next       <= sys_wdata(32-1 downto 0) when ((sys_wen = '1') and (sys_addr(19 downto 0) = std_logic_vector(to_unsigned(16#10#,sys_addr(19 downto 0)'length)))) else set_dly_reg;
  set_dec_next       <= sys_wdata(17-1 downto 0) when ((sys_wen = '1') and (sys_addr(19 downto 0) = std_logic_vector(to_unsigned(16#14#,sys_addr(19 downto 0)'length)))) else set_dec_reg;
  set_a_hyst_next    <= sys_wdata(14-1 downto 0) when ((sys_wen = '1') and (sys_addr(19 downto 0) = std_logic_vector(to_unsigned(16#20#,sys_addr(19 downto 0)'length)))) else set_a_hyst_reg;
  set_b_hyst_next    <= sys_wdata(14-1 downto 0) when ((sys_wen = '1') and (sys_addr(19 downto 0) = std_logic_vector(to_unsigned(16#24#,sys_addr(19 downto 0)'length)))) else set_b_hyst_reg;
  set_avg_en_next    <= sys_wdata(     0)     	when ((sys_wen = '1') and (sys_addr(19 downto 0) = std_logic_vector(to_unsigned(16#28#,sys_addr(19 downto 0)'length)))) else set_avg_en_reg;
                                                                          
  set_a_filt_aa_next <= sys_wdata(18-1 downto 0) when ((sys_wen = '1') and (sys_addr(19 downto 0) = std_logic_vector(to_unsigned(16#30#,sys_addr(19 downto 0)'length)))) else set_a_filt_aa_reg;
  set_a_filt_bb_next <= sys_wdata(25-1 downto 0) when ((sys_wen = '1') and (sys_addr(19 downto 0) = std_logic_vector(to_unsigned(16#34#,sys_addr(19 downto 0)'length)))) else set_a_filt_bb_reg;
  set_a_filt_kk_next <= sys_wdata(25-1 downto 0) when ((sys_wen = '1') and (sys_addr(19 downto 0) = std_logic_vector(to_unsigned(16#38#,sys_addr(19 downto 0)'length)))) else set_a_filt_kk_reg;
  set_a_filt_pp_next <= sys_wdata(25-1 downto 0) when ((sys_wen = '1') and (sys_addr(19 downto 0) = std_logic_vector(to_unsigned(16#3C#,sys_addr(19 downto 0)'length)))) else set_a_filt_pp_reg;
  set_b_filt_aa_next <= sys_wdata(18-1 downto 0) when ((sys_wen = '1') and (sys_addr(19 downto 0) = std_logic_vector(to_unsigned(16#40#,sys_addr(19 downto 0)'length)))) else set_b_filt_aa_reg;
  set_b_filt_bb_next <= sys_wdata(25-1 downto 0) when ((sys_wen = '1') and (sys_addr(19 downto 0) = std_logic_vector(to_unsigned(16#44#,sys_addr(19 downto 0)'length)))) else set_b_filt_bb_reg;
  set_b_filt_kk_next <= sys_wdata(25-1 downto 0) when ((sys_wen = '1') and (sys_addr(19 downto 0) = std_logic_vector(to_unsigned(16#48#,sys_addr(19 downto 0)'length)))) else set_b_filt_kk_reg;
  set_b_filt_pp_next <= sys_wdata(25-1 downto 0) when ((sys_wen = '1') and (sys_addr(19 downto 0) = std_logic_vector(to_unsigned(16#4C#,sys_addr(19 downto 0)'length)))) else set_b_filt_pp_reg;
                                                                           
  set_a_axi_start_next <= sys_wdata(32-1 downto 0) when ((sys_wen = '1') and (sys_addr(19 downto 0) = std_logic_vector(to_unsigned(16#50#,sys_addr(19 downto 0)'length)))) else set_a_axi_start_reg;
  set_a_axi_stop_next  <= sys_wdata(32-1 downto 0) when ((sys_wen = '1') and (sys_addr(19 downto 0) = std_logic_vector(to_unsigned(16#54#,sys_addr(19 downto 0)'length)))) else set_a_axi_stop_reg;
  set_a_axi_dly_next   <= sys_wdata(32-1 downto 0) when ((sys_wen = '1') and (sys_addr(19 downto 0) = std_logic_vector(to_unsigned(16#58#,sys_addr(19 downto 0)'length)))) else set_a_axi_dly_reg;
  set_a_axi_en_next    <= sys_wdata(     0)        when ((sys_wen = '1') and (sys_addr(19 downto 0) = std_logic_vector(to_unsigned(16#5C#,sys_addr(19 downto 0)'length)))) else set_a_axi_en_reg;
                                                                          
  set_b_axi_start_next <= sys_wdata(32-1 downto 0) when ((sys_wen = '1') and (sys_addr(19 downto 0) = std_logic_vector(to_unsigned(16#70#,sys_addr(19 downto 0)'length)))) else set_b_axi_start_reg;
  set_b_axi_stop_next  <= sys_wdata(32-1 downto 0) when ((sys_wen = '1') and (sys_addr(19 downto 0) = std_logic_vector(to_unsigned(16#74#,sys_addr(19 downto 0)'length)))) else set_b_axi_stop_reg;
  set_b_axi_dly_next   <= sys_wdata(32-1 downto 0) when ((sys_wen = '1') and (sys_addr(19 downto 0) = std_logic_vector(to_unsigned(16#78#,sys_addr(19 downto 0)'length)))) else set_b_axi_dly_reg;
  set_b_axi_en_next    <= sys_wdata(     0)         when ((sys_wen = '1') and (sys_addr(19 downto 0) = std_logic_vector(to_unsigned(16#7C#,sys_addr(19 downto 0)'length)))) else set_b_axi_en_reg;
                                                                            
  set_deb_len_next <= sys_wdata(20-1 downto 0)	when ((sys_wen = '1') and (sys_addr(19 downto 0) = std_logic_vector(to_unsigned(16#90#,sys_addr(19 downto 0)'length)))) else set_deb_len_reg;

sys_en <= '1' when ((sys_wen = '1') or (sys_ren = '1')) else '0';


process(adc_clk_i, adc_rstn_i)
begin
if (adc_rstn_i = '0') then
  sys_err_i_reg <= '0' ;
  sys_ack_i_reg <= '0' ;
	sys_rdata_i_reg <= (others => '0');
elsif (rising_edge(adc_clk_i)) then
  sys_err_i_reg <= sys_err_i_next;
  sys_ack_i_reg <= sys_ack_i_next;
	sys_rdata_i_reg <= sys_rdata_i_next;
end if;
end process;

	--next state logic

process(sys_addr(19 downto 0))
begin
	case sys_addr(19 downto 0) is 
     when x"00000" => sys_ack_i_next <= sys_en;          sys_rdata_i_next <= (others => '0', 3 => adc_we_keep_reg, 		 -- do not disarm on 
                                                                              2 => adc_dly_do_reg,                		 -- trigger status
                                                                              1 => '0',                   				 -- reset
                                                                              0 => adc_we_reg)             					 ; -- arm

     when x"00004" => sys_ack_i_next <= sys_en;          sys_rdata_i_next <= (others => '0', 3-0 => set_trig_src_reg ; 

     when x"00008" => sys_ack_i_next <= sys_en;          sys_rdata_i_next <= (others => '0', 13-0 => set_a_tresh_reg)    ; 
     when x"0000C" => sys_ack_i_next <= sys_en;          sys_rdata_i_next <= (others => '0', 13-0 => set_b_tresh_reg)    ; 
     when x"00010" => sys_ack_i_next <= sys_en;          sys_rdata_i_next <= set_dly_reg            ; 
     when x"00014" => sys_ack_i_next <= sys_en;          sys_rdata_i_next <= (others => '0', 16-0 => set_dec_reg)        ; 

     when x"00018" => sys_ack_i_next <= sys_en;          sys_rdata_i_next <= (others => '0', (RSZ-1)-0 => std_logic_vector(adc_wp_cur))        ; 
     when x"0001C" => sys_ack_i_next <= sys_en;          sys_rdata_i_next <= (others => '0', (RSZ-1)-0 => adc_wp_trig)       ; 

     when x"00020" => sys_ack_i_next <= sys_en;          sys_rdata_i_next <= (others => '0', 13-0 => set_a_hyst_reg)     ; 
     when x"00024" => sys_ack_i_next <= sys_en;          sys_rdata_i_next <= (others => '0', 13-0 => set_b_hyst_reg)     ; 

     when x"00028" => sys_ack_i_next <= sys_en;          sys_rdata_i_next <= (others => '0', 0 => set_avg_en_reg)         ; 

     when x"0002C" => sys_ack_i_next <= sys_en;          sys_rdata_i_next <=                 adc_we_cnt_reg          ; 

     when x"00030" => sys_ack_i_next <= sys_en;          sys_rdata_i_next <= (others => '0', 17-0 => set_a_filt_aa_reg)      ; 
     when x"00034" => sys_ack_i_next <= sys_en;          sys_rdata_i_next <= (others => '0', 24-0 => set_a_filt_bb_reg)      ; 
     when x"00038" => sys_ack_i_next <= sys_en;          sys_rdata_i_next <= (others => '0', 24-0 => set_a_filt_kk_reg)      ; 
     when x"0003C" => sys_ack_i_next <= sys_en;          sys_rdata_i_next <= (others => '0', 24-0 => set_a_filt_pp_reg)      ; 
     when x"00040" => sys_ack_i_next <= sys_en;          sys_rdata_i_next <= (others => '0', 17-0 => set_b_filt_aa_reg)      ; 
     when x"00044" => sys_ack_i_next <= sys_en;          sys_rdata_i_next <= (others => '0', 24-0 => set_b_filt_bb_reg)      ; 
     when x"00048" => sys_ack_i_next <= sys_en;          sys_rdata_i_next <= (others => '0', 24-0 => set_b_filt_kk_reg)      ; 
     when x"0004C" => sys_ack_i_next <= sys_en;          sys_rdata_i_next <= (others => '0', 24-0 => set_b_filt_pp_reg)      ; 

     when x"00050" => sys_ack_i_next <= sys_en;          sys_rdata_i_next <=                 set_a_axi_star_reg      ;  
     when x"00054" => sys_ack_i_next <= sys_en;          sys_rdata_i_next <=                 set_a_axi_stop_reg      ; 
     when x"00058" => sys_ack_i_next <= sys_en;          sys_rdata_i_next <=                 set_a_axi_dly_reg       ; 
     when x"0005C" => sys_ack_i_next <= sys_en;          sys_rdata_i_next <= (others => '0', 0 => set_a_axi_en_reg       ; 
     when x"00060" => sys_ack_i_next <= sys_en;          sys_rdata_i_next <=                 set_a_axi_trig_reg      ; 
     when x"00064" => sys_ack_i_next <= sys_en;          sys_rdata_i_next <=                 set_a_axi_cur_reg       ; 

     when x"00070" => sys_ack_i_next <= sys_en;          sys_rdata_i_next <=                 set_b_axi_start_reg     ; 
     when x"00074" => sys_ack_i_next <= sys_en;          sys_rdata_i_next <=                 set_b_axi_stop_reg      ; 
     when x"00078" => sys_ack_i_next <= sys_en;          sys_rdata_i_next <=                 set_b_axi_dly_reg       ; 
     when x"0007C" => sys_ack_i_next <= sys_en;          sys_rdata_i_next <= (others => '0', 0 => set_b_axi_en_reg       ; 
     when x"00080" => sys_ack_i_next <= sys_en;          sys_rdata_i_next <=                 set_b_axi_trig_reg      ; 
     when x"00084" => sys_ack_i_next <= sys_en;          sys_rdata_i_next <=                 set_b_axi_cur_reg       ; 

     when x"00090" => sys_ack_i_next <= sys_en;          sys_rdata_i_next <= (others => '0', 19-0 => set_deb_len_reg)    ; 

     when (19-16 => x"1", others => "-") => sys_ack_i_next <= adc_rd_dv;       sys_rdata_i_next <= (others => '0', 13-0 => adc_a_rd_reg)              ; 
     when (19-16 => x"2", others => "-") => sys_ack_i_next <= adc_rd_dv;       sys_rdata_i_next <= (others => '0', 13-0 => adc_b_rd_reg)              ; 

     when  others  =>  sys_ack_i_next <= sys_en;          sys_rdata_i_next <= (others => 0)                      ; 
   end case;
end process;

-- assign outputs
sys_err <= sys_err_i_reg;
sys_ack <= sys_ack_i_reg;
sys_rdata <= sys_rdata_i_reg;

end rtl;
