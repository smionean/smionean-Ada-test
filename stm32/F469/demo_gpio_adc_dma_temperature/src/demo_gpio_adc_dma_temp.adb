------------------------------------------------------------------------------
--                                                                          --
--                  Copyright (C) 2015-2017, AdaCore                        --
--                                                                          --
--  Redistribution and use in source and binary forms, with or without      --
--  modification, are permitted provided that the following conditions are  --
--  met:                                                                    --
--     1. Redistributions of source code must retain the above copyright    --
--        notice, this list of conditions and the following disclaimer.     --
--     2. Redistributions in binary form must reproduce the above copyright --
--        notice, this list of conditions and the following disclaimer in   --
--        the documentation and/or other materials provided with the        --
--        distribution.                                                     --
--     3. Neither the name of the copyright holder nor the names of its     --
--        contributors may be used to endorse or promote products derived   --
--        from this software without specific prior written permission.     --
--                                                                          --
--   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS    --
--   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT      --
--   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR  --
--   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT   --
--   HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, --
--   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT       --
--   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,  --
--   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY  --
--   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT    --
--   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE  --
--   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.   --
--                                                                          --
------------------------------------------------------------------------------

--   ########################################################
--   Modification of demo_adc_vbat_dma.adb,
--   the operations on the gpio port have been taken from
--                               demo_adc_gpio_polling.adb
--   ########################################################


--  This program demonstrates reading the voltage value on a gpio pin from
--  an ADC unit, using DMA.

--  The programs displays the voltage value so it assumes a display of
--  some sort.

with Ada.Real_Time; use Ada.Real_Time;

with Last_Chance_Handler;  pragma Unreferenced (Last_Chance_Handler);

with STM32.Board;  use STM32.Board;
with STM32.Device; use STM32.Device;

with HAL;         use HAL;
with STM32.ADC;   use STM32.ADC;
with STM32.DMA;   use STM32.DMA;
with STM32.GPIO;  use STM32.GPIO;

with LCD_Std_Out;

procedure Demo_gpio_adc_dma_temp is
   --   gpio pin
   Converter : Analog_To_Digital_Converter renames ADC_1;
   Input_Channel : constant Analog_Input_Channel := 9;
   Input : constant GPIO_Point := PB1;

   Controller : DMA_Controller renames DMA_2;
   Stream     : constant DMA_Stream_Selector := Stream_0;

   Counts  : UInt16 with Volatile;
   --  The raw sample from the ADC conversion of the input voltage on Pin PA5

   Voltage : UInt32;  -- in millivolts
   --  the converted voltage representing the voltage on Pin PA5, in millivolts

   procedure Print (X, Y : Natural; Value : UInt32; Suffix : String := "");

   --   gpio pin
   procedure Configure_Analog_Input;

   procedure Initialize_DMA;
   procedure Initialize_ADC;

   -----------
   -- Print --
   -----------

   procedure Print (X, Y : Natural; Value : UInt32; Suffix : String := "") is
      Value_Image : constant String := Value'Img;
   begin
      LCD_Std_Out.Put (X, Y, Value_Image (2 .. Value_Image'Last) & Suffix & "   ");
   end Print;

   -- -------------------------
   -- Configure_Analog_Input --
   -- -------------------------
   procedure Configure_Analog_Input is
   begin
      Enable_Clock (Input);
      Configure_IO (Input,
                    (Mode => Mode_Analog,
                     Resistors => Floating,
                   others => <>));
   end Configure_Analog_Input;


   --------------------
   -- Initialize_DMA --
   --------------------

   procedure Initialize_DMA is
      Config : DMA_Stream_Configuration;
   begin
      Enable_Clock (Controller);

      Reset (Controller, Stream);

      Config.Channel                      := Channel_0;
      Config.Direction                    := Peripheral_To_Memory;
      Config.Memory_Data_Format           := HalfWords;
      Config.Peripheral_Data_Format       := HalfWords;
      Config.Increment_Peripheral_Address := False;
      Config.Increment_Memory_Address     := False;
      Config.Operation_Mode               := Circular_Mode;
      Config.Priority                     := Priority_Very_High;
      Config.FIFO_Enabled                 := False;
      Config.Memory_Burst_Size            := Memory_Burst_Single;
      Config.Peripheral_Burst_Size        := Peripheral_Burst_Single;

      Configure (Controller, Stream, Config);

      Clear_All_Status (Controller, Stream);
   end Initialize_DMA;

   --------------------
   -- Initialize_ADC --
   --------------------

   procedure Initialize_ADC is
      All_Regular_Conversions : constant Regular_Channel_Conversions :=
        (1 => (Channel => Input_Channel, Sample_Time => Sample_480_Cycles));
   begin
      Enable_Clock (Converter);

      Reset_All_ADC_Units;

      Configure_Common_Properties
        (Mode           => Independent,
         Prescalar      => PCLK2_Div_4,
         DMA_Mode       => Disabled,
         Sampling_Delay => Sampling_Delay_5_Cycles);

      Configure_Unit
        (Converter,
         Resolution => ADC_Resolution_10_Bits,
         Alignment  => Right_Aligned);

      Configure_Regular_Conversions
        (Converter,
         Continuous  => True,    --  True,
         Trigger     => Software_Triggered,
         Enable_EOC  => True,    --  False,
         Conversions => All_Regular_Conversions);

      Enable_DMA (Converter);

      Enable_DMA_After_Last_Transfer (Converter);
   end Initialize_ADC;

   Temperature : UInt32 := 0;
   --  the computed temperature

begin
   Initialize_LEDs;

   --  gpio
   Configure_Analog_Input;

   delay until Clock + Milliseconds (500);
   LCD_Std_Out.Clear_Screen;

   Initialize_DMA;

   Initialize_ADC;

   Enable (Converter);

   Start_Conversion (Converter);

   Start_Transfer
     (Controller,
      Stream,
      Source      => Data_Register_Address (Converter),
      Destination => Counts'Address,
      Data_Count  => 1);  -- ie, 1 halfword

   loop
      Voltage := UInt32 (Counts);

      Print (0, 0, Voltage, " raw value");

      Temperature := ((UInt32 (Counts) * VBat_Bridge_Divisor) * ADC_Supply_Voltage / 16#FFF#);

      Print (0, 24, Temperature, " degrees C (x10)");

      Green_LED.Toggle;
      Temperature := 0;

      delay until Clock + Milliseconds (100);
   end loop;
end Demo_gpio_adc_dma_temp;
