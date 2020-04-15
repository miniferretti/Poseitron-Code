library ieee;
use ieee.std_logic_1164.all;

entity intercept_open_drain_bus2 is
  port (
    clock : in std_logic;
    wire1 : inout std_logic;
    wire2 : inout std_logic);
end entity intercept_open_drain_bus2;

architecture rtl of intercept_open_drain_bus2 is
  type state_t is (IDLE, PULLDOWN1, WAIT1, PULLDOWN2, WAIT2);
  signal state : state_t := IDLE;
begin

  -- Moore outputs from finite state machine
  wire1 <= '0' when state = PULLDOWN1 else 'Z';
  wire2 <= '0' when state = PULLDOWN2 else 'Z';

  -- finite state machine
  process(clock)
  begin
    if rising_edge(clock) then
      case state is
        when IDLE =>
          if wire2 = '0' then
            state <= PULLDOWN1;
          elsif wire1 = '0' then
            state <= PULLDOWN2;
          end if;

        when PULLDOWN1 =>
          if wire2 /= '0' then -- 'H' or '1'
            state <= WAIT1;
          end if;

        when WAIT1 => -- wait until wire1 is pulled-up by the resistor
          if wire1 /= '0' then
            state <= IDLE;
          end if;

        when PULLDOWN2 =>
          if wire1 /= '0' then -- 'H' or '1'
            state <= WAIT2;
          end if;

        when WAIT2 => -- wait until wire2 is pulled-up by the resistor
          if wire2 /= '0' then
            state <= IDLE;
          end if;
      end case;
    end if;
  end process;
end architecture rtl;