--------------------------------------------------------------------------------
--
--   FileName:         pmod_color_sensor.vhd
--   Dependencies:     i2c_master.vhd (Version 2.2)
--   Design Software:  Quartus Prime Version 17.0.0 Build 595 SJ Lite Edition
--
--   HDL CODE IS PROVIDED "AS IS."  DIGI-KEY EXPRESSLY DISCLAIMS ANY
--   WARRANTY OF ANY KIND, WHETHER EXPRESS OR IMPLIED, INCLUDING BUT NOT
--   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
--   PARTICULAR PURPOSE, OR NON-INFRINGEMENT. IN NO EVENT SHALL DIGI-KEY
--   BE LIABLE FOR ANY INCIDENTAL, SPECIAL, INDIRECT OR CONSEQUENTIAL
--   DAMAGES, LOST PROFITS OR LOST DATA, HARM TO YOUR EQUIPMENT, COST OF
--   PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS
--   BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF),
--   ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER SIMILAR COSTS.
--
--   Version History
--   Version 1.0 02/21/2020 Scott Larson
--     Initial Public Release
-- 
--------------------------------------------------------------------------------

LIBRARY ieee;
USE ieee.std_logic_1164.all;
USE ieee.numeric_std.all;

ENTITY pmod_color_sensor IS
  GENERIC(
    sys_clk_freq : INTEGER := 50_000_000;                        --input clock speed from user logic in Hz
    gain         : INTEGER := 1;                                 --analog gain (1,4, 16, or 60)
    atime        : STD_LOGIC_VECTOR(7 DOWNTO 0) := "11111111");  --ADC integration time, see TCS3472 datasheet                            
  PORT(
    clk           : IN    STD_LOGIC;                       --system clock
    reset_n       : IN    STD_LOGIC;                       --asynchronous active-low reset
    scl           : INOUT STD_LOGIC;                       --I2C serial clock
    sda           : INOUT STD_LOGIC;                       --I2C serial data
    i2c_ack_err   : OUT   STD_LOGIC;                       --I2C slave acknowledge error flag
    clear         : OUT   STD_LOGIC_VECTOR(15 DOWNTO 0);   --clear color value obtained
    red           : OUT   STD_LOGIC_VECTOR(15 DOWNTO 0);   --red color value obtained
    green         : OUT   STD_LOGIC_VECTOR(15 DOWNTO 0);   --green color value obtained
    blue          : OUT   STD_LOGIC_VECTOR(15 DOWNTO 0);   --blue color value obtained
    sensor_select : IN    STD_LOGIC_VECTOR(7 DOWNTO 0);
    portA_output_select : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
    portB_output_select : IN STD_LOGIC_VECTOR(7 DOWNTO 0));
END pmod_color_sensor;

ARCHITECTURE behavior OF pmod_color_sensor IS
  CONSTANT color_sensor_addr   : STD_LOGIC_VECTOR(6 DOWNTO 0) := "0101001"; --I2C address of the color sensor pmod
  CONSTANT I2C_multiplexer_addr: STD_LOGIC_VECTOR(6 DOWNTO 0) := "1110000"; --I2C address of the I2C multiplexer
  CONSTANT I2C_expander_addr   : STD_LOGIC_VECTOR(6 DOWNTO 0) := "0100000"; --I2C address of the I2C GPIO expander
  CONSTANT sensor_num          : INTEGER := 2;                              --Number of color sensors to read and write from (Change if needed)
  CONSTANT bit_shift           : STD_LOGIC_VECTOR(7 DOWNTO 0) := "00000001";--Base value for the selection of the sensor
  TYPE machine IS(start, set_gain, set_atime, set_pon_aen, pause, read_data, output_result, change_sensor, ports_to_output, ports_control); --needed states
  SIGNAL state       : machine;                       --state machine
  SIGNAL gain_bits   : STD_LOGIC_VECTOR(1 DOWNTO 0);  --code to set the sensor's analog gain
  SIGNAL i2c_ena     : STD_LOGIC;                     --i2c enable signal
  SIGNAL i2c_addr    : STD_LOGIC_VECTOR(6 DOWNTO 0);  --i2c address signal
  SIGNAL i2c_rw      : STD_LOGIC;                     --i2c read/write command signal
  SIGNAL i2c_data_wr : STD_LOGIC_VECTOR(7 DOWNTO 0);  --i2c write data
  SIGNAL i2c_data_rd : STD_LOGIC_VECTOR(7 DOWNTO 0);  --i2c read data
  SIGNAL i2c_busy    : STD_LOGIC;                     --i2c busy signal
  SIGNAL busy_prev   : STD_LOGIC;                     --previous value of i2c busy signal
  SIGNAL clear_data  : STD_LOGIC_VECTOR(15 DOWNTO 0); --clear color data buffer
  SIGNAL red_data    : STD_LOGIC_VECTOR(15 DOWNTO 0); --red color data buffer
  SIGNAL green_data  : STD_LOGIC_VECTOR(15 DOWNTO 0); --green color data buffer
  SIGNAL blue_data   : STD_LOGIC_VECTOR(15 DOWNTO 0); --blue color data buffer

  COMPONENT i2c_master IS
    GENERIC(
      input_clk : INTEGER;  --input clock speed from user logic in Hz
      bus_clk   : INTEGER); --speed the i2c bus (scl) will run at in Hz
    PORT(
      clk       : IN     STD_LOGIC;                    --system clock
      reset_n   : IN     STD_LOGIC;                    --active low reset
      ena       : IN     STD_LOGIC;                    --latch in command
      addr      : IN     STD_LOGIC_VECTOR(6 DOWNTO 0); --address of target slave
      rw        : IN     STD_LOGIC;                    --'0' is write, '1' is read
      data_wr   : IN     STD_LOGIC_VECTOR(7 DOWNTO 0); --data to write to slave
      busy      : OUT    STD_LOGIC;                    --indicates transaction in progress
      data_rd   : OUT    STD_LOGIC_VECTOR(7 DOWNTO 0); --data read from slave
      ack_error : BUFFER STD_LOGIC;                    --flag if improper acknowledge from slave
      sda       : INOUT  STD_LOGIC;                    --serial data output of i2c bus
      scl       : INOUT  STD_LOGIC);                   --serial clock output of i2c bus
  END COMPONENT;

BEGIN

  --instantiate the i2c master
  i2c_master_0:  i2c_master
    GENERIC MAP(input_clk => sys_clk_freq, bus_clk => 400_000)
    PORT MAP(clk => clk, reset_n => reset_n, ena => i2c_ena, addr => i2c_addr,
             rw => i2c_rw, data_wr => i2c_data_wr, busy => i2c_busy,
             data_rd => i2c_data_rd, ack_error => i2c_ack_err, sda => sda,
             scl => scl);
         
  --set the analog gain code for the sensor's Control Register value
  WITH gain SELECT
    gain_bits <= "01" WHEN 4,      --4x gain
                 "10" WHEN 16,     --16x gain
                 "11" WHEN 60,     --60x gain
                 "00" WHEN OTHERS; --1x gain (default)

  PROCESS(clk, reset_n)
    VARIABLE busy_cnt  : INTEGER RANGE 0 TO 9 := 0;               --counts the I2C busy signal transistions
    VARIABLE counter   : INTEGER RANGE 0 TO sys_clk_freq/10 := 0; --counts 100ms to wait before communicating
    VARIABLE pause_cnt : INTEGER RANGE 0 TO 240000 := 0;          --counter to execute wait periods
    VARIABLE sensor    : INTEGER RANGE 0 TO sensor_num-1 := 0; 
  BEGIN
    IF(reset_n = '0') THEN               --reset activated
      counter := 0;                        --clear wait counter
      i2c_ena <= '0';                      --clear I2C enable
      busy_cnt := 0;   
      sensor := 0;                    --clear busy counter
      clear <= (OTHERS => '0');            --clear the clear color result output
      red <= (OTHERS => '0');              --clear the red color result output
      green <= (OTHERS => '0');            --clear the green color result output
      blue <= (OTHERS => '0');             --clear the blue color result output
      state <= start;                      --return to start state
    ELSIF(clk'EVENT AND clk = '1') THEN  --rising edge of system clock
      CASE state IS                        --state machine
      
        --give color sensor 100ms to power up before communicating
        WHEN start =>
          IF(counter < sys_clk_freq/10) THEN  --100ms not yet reached
            counter := counter + 1;             --increment counter
          ELSE                                --100ms reached
            counter := 0;                       --clear counter
            state <= set_gain;                  --advance to setting the gain
          END IF;
        
        --set analog gain
        WHEN set_gain =>
          busy_prev <= i2c_busy;                       --capture the value of the previous i2c busy signal
          IF(busy_prev = '0' AND i2c_busy = '1') THEN  --i2c busy just went high
            busy_cnt := busy_cnt + 1;                    --counts the times busy has gone from low to high during transaction
          END IF;
          CASE busy_cnt IS                             --busy_cnt keeps track of which command we are on
            WHEN 0 =>
              i2c_ena <='1';
              i2c_addr <= I2C_multiplexer_addr;
              i2c_rw <= '0';
              i2c_data_wr <=  std_logic_vector(unsigned(bit_shift) sll sensor); --   "00000001" sll sensor;
            WHEN 1 =>                                    --no command latched in yet
              i2c_ena <= '1';                              --initiate the transaction
              i2c_addr <= color_sensor_addr;               --set the address of the color sensor
              i2c_rw <= '0';                               --command 1 is a write
              i2c_data_wr <= "10101111";                   --set the register pointer to the Control Register
            WHEN 2 =>                                    --1st busy high: command 1 latched, okay to issue command 2
              i2c_data_wr <= "000000" & gain_bits;         --write data to the Control Register to set analog gain code
            WHEN 3 =>                                    --2nd busy high: command 2 latched
              i2c_ena <= '0';                              --deassert enable to stop transaction after command 2
              IF(i2c_busy = '0') THEN                      --transaction complete
                IF(sensor = sensor_num-1) THEN
                  busy_cnt := 0;                               --reset busy_cnt for next transaction
                  state <= set_atime;                          --advance to setting the ADC integration time
                  sensor := 0;
                ELSE
                  busy_cnt := 0;
                  sensor := sensor + 1;
                END IF;
              END IF;
            WHEN OTHERS => NULL;
          END CASE;      

        --set integration time
        WHEN set_atime =>
          busy_prev <= i2c_busy;                       --capture the value of the previous i2c busy signal
          IF(busy_prev = '0' AND i2c_busy = '1') THEN  --i2c busy just went high
            busy_cnt := busy_cnt + 1;                    --counts the times busy has gone from low to high during transaction
          END IF;
          CASE busy_cnt IS                             --busy_cnt keeps track of which command we are on
             WHEN 0 =>
              i2c_ena <='1';
              i2c_addr <= I2C_multiplexer_addr;
              i2c_rw <= '0';
              i2c_data_wr <= std_logic_vector(unsigned(bit_shift) sll sensor); --   "00000001" sll sensor;
            WHEN 1 =>                                    --no command latched in yet
              i2c_ena <= '1';                              --initiate the transaction
              i2c_addr <= color_sensor_addr;               --set the address of the color sensor
              i2c_rw <= '0';                               --command 1 is a write
              i2c_data_wr <= "10100001";                   --set the register pointer to the Atime Register
            WHEN 2 =>                                    --1st busy high: command 1 latched, okay to issue command 2
              i2c_data_wr <= atime;                        --write data to the Atime Register to set integration time
            WHEN 3 =>                                    --2nd busy high: command 2 latched
              i2c_ena <= '0';                              --deassert enable to stop transaction after command 2
              IF(i2c_busy = '0') THEN                      --transaction complete
               IF(sensor = sensor_num-1) THEN
                busy_cnt := 0;                               --reset busy_cnt for next transaction
                state <= set_pon_aen;                          --advance to setting the ADC integration time
                sensor := 0;
                ELSE
                busy_cnt := 0;
                sensor := sensor + 1;
                END IF;
              END IF;
            WHEN OTHERS => NULL;
          END CASE;
       
        --set the PON bit to turn on the oscillators and the AEN bit to enable the sensor's ADCs   
        WHEN set_pon_aen =>            
          busy_prev <= i2c_busy;                       --capture the value of the previous i2c busy signal
          IF(busy_prev = '0' AND i2c_busy = '1') THEN  --i2c busy just went high
            busy_cnt := busy_cnt + 1;                    --counts the times busy has gone from low to high during transaction
          END IF;
          CASE busy_cnt IS                             --busy_cnt keeps track of which command we are on
            WHEN 0 =>
              i2c_ena <='1';
              i2c_addr <= I2C_multiplexer_addr;
              i2c_rw <= '0';
              i2c_data_wr <= std_logic_vector(unsigned(bit_shift) sll sensor); --   "00000001" sll sensor;
            WHEN 1 =>                                    --no command latched in yet
              i2c_ena <= '1';                              --initiate the transaction
              i2c_addr <= color_sensor_addr;               --set the address of the color sensor
              i2c_rw <= '0';                               --command 1 is a write
              i2c_data_wr <= "10100000";                   --set the register pointer to the Enable Register
            WHEN 2 =>                                    --1st busy high: command 1 latched, okay to issue command 2
              i2c_data_wr <= "00000011";                   --write data to the Enable Register to set AEN and PON bits
            WHEN 3 =>                                    --2nd busy high: command 2 latched
              i2c_ena <= '0';                              --deassert enable to stop transaction after command 2
              IF(i2c_busy = '0') THEN                      --transaction complete
               IF(sensor = sensor_num-1) THEN
                busy_cnt := 0;                               --reset busy_cnt for next transaction
                state <= ports_to_output;                              --advance to setting the ADC integration time
                sensor := 0;
               ELSE
                busy_cnt := 0;
                sensor := sensor + 1;
               END IF;
              END IF;
            WHEN OTHERS => NULL;
          END CASE; 


        --This state is used to put all the output of the GPIO expader to output mode
        WHEN ports_to_output =>                        
          busy_prev <= i2c_busy;                       --capture the value of the previous i2c busy signal
          IF(busy_prev = '0' AND i2c_busy = '1') THEN  --i2c busy just went high
            busy_cnt := busy_cnt + 1;                   --counts the times busy has gone from low to high during transaction
          END IF;  
          CASE busy_cnt IS 
            WHEN 0 =>                                  --This step set the register pointer to Port A of the MCP23017
              i2c_ena <= '1';
              i2c_addr <= I2C_expander_addr;
              i2c_rw <= '0';
              i2c_data_wr <= "00000000";
            WHEN 1 =>                                 --Set the GPIO to Output mode
              i2c_data_wr <= "00000000";              
            WHEN 2 =>                                 --This step set the register pointer to Port B of the MCP23017
              i2c_ena <= '1';
              i2c_addr <= I2C_expander_addr;
              i2c_rw <= '0';
              i2c_data_wr <= "00000001";
            WHEN 3 =>                                --Set the GPIO to Output mode
              i2c_data_wr <= "00000000";
            WHEN 4 =>
              i2c_ena <= '0';
              IF(i2c_busy = '0') THEN
                busy_cnt := 0;
                state <= pause;
              END IF;
            WHEN OTHERS => NULL;
          END CASE;


        --wait 2.4ms
        WHEN pause =>
          IF(pause_cnt < sys_clk_freq/417) THEN  --2.4ms wait time not met
            pause_cnt := pause_cnt + 1;            --increment counter
          ELSE                                   --2.4ms wait time met
            pause_cnt := 0;                        --reset counter
            state <= read_data;                    --advance to reading data
          END IF;
       
        --all color read data  
        WHEN read_data =>
          busy_prev <= i2c_busy;                       --capture the value of the previous i2c busy signal
          IF(busy_prev = '0' AND i2c_busy = '1') THEN  --i2c busy just went high
            busy_cnt := busy_cnt + 1;                    --counts the times busy has gone from low to high during transaction
          END IF;
          CASE busy_cnt IS                             --busy_cnt keeps track of which command we are on
            WHEN 0 =>                                    --no command latched in yet
              i2c_ena <= '1';                              --initiate the transaction
              i2c_addr <= color_sensor_addr;               --set the address of the color sensor
              i2c_rw <= '0';                               --command 1 is a write
              i2c_data_wr <= "10110100";                   --set the register pointer to the CDATAL (clear low-byte data) Register
            WHEN 1 =>                                    --1st busy high: command 1 latched
              i2c_rw <= '1';                               --command 2 is a read (addr stays the same)
            WHEN 2 =>                                    --2nd busy high: command 2 latched, okay to issue command 3
              IF(i2c_busy = '0') THEN                      --indicates data read in command 1 is ready
                clear_data(7 DOWNTO 0) <= i2c_data_rd;       --retrieve clear low-byte data from command 1
              END IF;
            WHEN 3 =>                                    --3rd busy high: command 3 latched, okay to issue command 4
              IF(i2c_busy = '0') THEN                      --indicates data read in command 2 is ready
                clear_data(15 DOWNTO 8) <= i2c_data_rd;      --retrieve clear high-byte data from command 2
              END IF;
            WHEN 4 =>                                    --4th busy high: command 4 latched, okay to issue command 5
              IF(i2c_busy = '0') THEN                      --indicates data read in command 3 is ready
                red_data(7 DOWNTO 0) <= i2c_data_rd;         --retrieve red low-byte data from command 3
              END IF;
            WHEN 5 =>                                    --5th busy high: command 5 latched, okay to issue command 6
              IF(i2c_busy = '0') THEN                      --indicates data read in command 4 is ready
                red_data(15 DOWNTO 8) <= i2c_data_rd;        --retrieve red high-byte data from command 4
              END IF;
            WHEN 6 =>                                    --6th busy high: command 6 latched, okay to issue command 7
              IF(i2c_busy = '0') THEN                      --indicates data read in command 5 is ready
                green_data(7 DOWNTO 0) <= i2c_data_rd;       --retrieve green low-byte data from command 5
              END IF;
            WHEN 7 =>                                    --7th busy high: command 7 latched, okay to issue command 8
              IF(i2c_busy = '0') THEN                      --indicates data read in command 6 is ready
                green_data(15 DOWNTO 8) <= i2c_data_rd;      --retrieve green high-byte data from command 6
              END IF;
            WHEN 8 =>                                    --8th busy high: command 8 latched, okay to issue command 9
              IF(i2c_busy = '0') THEN                      --indicates data read in command 7 is ready
                blue_data(7 DOWNTO 0) <= i2c_data_rd;        --retrieve blue low-byte data from command 7
              END IF;
            WHEN 9 =>                                    --9th busy high: command 9 latched
              i2c_ena <= '0';                              --deassert enable to stop transaction after command 8
              IF(i2c_busy = '0') THEN                      --indicates data read in command 8 is ready
                blue_data(15 DOWNTO 8) <= i2c_data_rd;       --retrieve blue high-byte data from command 8
                busy_cnt := 0;                               --reset busy_cnt for next transaction
                state <= output_result;                      --advance to output the result
              END IF;
            WHEN OTHERS => NULL;
          END CASE;



          --Change the color sensor in the multiplexer
          WHEN change_sensor =>
            busy_prev <= i2c_busy;                       --capture the value of the previous i2c busy signal
            IF(busy_prev = '0' AND i2c_busy = '1') THEN  --i2c busy just went high
              busy_cnt := busy_cnt + 1;                    --counts the times busy has gone from low to high during transaction
            END IF;
            CASE busy_cnt IS
              WHEN 0 =>  
                i2c_ena <='1';                           --start transaction
                i2c_addr <= I2C_multiplexer_addr;        --Multiplexer address   
                i2c_rw <= '0';                           --Write type transaction
                i2c_data_wr <= sensor_select;            --Input the targeted color sensor
              WHEN 1 =>
                i2c_ena <= '0';
                IF(i2c_busy = '0') THEN
                  busy_cnt := 0;
                  state <= ports_control;
                END IF;
              WHEN OTHERS => NULL;
            END CASE;


          --This state is used to control the output state of the GPIO of the MCP23017
          WHEN ports_control =>
            busy_prev <= i2c_busy;                       --capture the value of the previous i2c busy signal
            IF(busy_prev = '0' AND i2c_busy = '1') THEN  --i2c busy just went high
              busy_cnt := busy_cnt + 1;                    --counts the times busy has gone from low to high during transaction
            END IF;
            CASE busy_cnt IS 
              WHEN 0 =>                                  --This step set the register pointer to Port A of the MCP23017
                i2c_ena <= '1';
                i2c_addr <= I2C_expander_addr;
                i2c_rw <= '0';
                i2c_data_wr <= "00010010";               
              WHEN 1 =>                                  --Writes the desired states of the GPIO's of port A of the MCP23017
                i2c_data_wr <= portA_output_select;
              WHEN 2 =>                                  --This step set the register pointer to Port B of the MCP23017
                i2c_ena <= '1';
                i2c_addr <= I2C_expander_addr;
                i2c_rw <= '0';
                i2c_data_wr <= "00010011";
              WHEN 3 =>                                  --Writes the desired states of the GPIO's of port B of the MCP23017
                i2c_data_wr <= portB_output_select;
              WHEN 4 =>
                i2c_ena <= '0';
                IF(i2c_busy = '0') THEN
                  busy_cnt := 0;
                  state <= pause;
                END IF;
              WHEN OTHERS => NULL;
            END CASE;





        --output the color data
        WHEN output_result =>
          clear <= clear_data;  --write clear color data to output
          red <= red_data;      --write red color data to output
          green <= green_data;  --write green color data to output
          blue <= blue_data;    --write blue color data to output
          state <= change_sensor;       --retrieve the next set of color data

        --default to start state
        WHEN OTHERS =>
          state <= start;

      END CASE;
    END IF;
  END PROCESS;   
END behavior;
