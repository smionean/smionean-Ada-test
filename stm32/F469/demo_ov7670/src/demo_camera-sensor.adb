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

with STM32.DCMI;
with STM32.DMA;     use STM32.DMA;
with Ada.Real_Time; use Ada.Real_Time;
with OV2640;        use OV2640;
with OV7725;        use OV7725;
with OV7670;        use OV7670;
with HAL.I2C;       use HAL.I2C;
with HAL.Bitmap;    use HAL.Bitmap;
with HAL;           use HAL;
with STM32.PWM;     use STM32.PWM;
with STM32.Setup;
with Demo_Camera; use Demo_Camera;
with LCD_Std_Out;

package body Demo_Camera.Sensor is
  package DCMI renames STM32.DCMI;
   function Probe (Cam_Addr : out UInt10) return Boolean;

   REG_PID : constant := 16#0A#;
   --  REG_VER : constant := 16#0B#;

   CLK_PWM_Mod    : PWM_Modulator;
   Camera_PID     : HAL.UInt8 := 0;
   Camera_2640    : OV2640_Camera (Sensor_I2C'Access);
   Camera_7670    : OV7670_Camera (Sensor_I2C'Access);
   Camera_7725    : OV7725_Camera (Sensor_I2C'Access);
   Is_Initialized : Boolean := False;

   -----------------
   -- Initialized --
   -----------------

   function Initialized return Boolean is (Is_Initialized);

   -----------
   -- Probe --
   -----------

   function Probe (Cam_Addr : out UInt10) return Boolean is
      Status : I2C_Status := Ok;
   begin
      LCD_Std_Out.Clear_Screen;
      for Addr in UInt10 range 0 .. 126 loop
--           LCD_Std_Out.Put_Line("* Probe Addr "&Addr'Img);
         Sensor_I2C.Master_Transmit (Addr    => Addr,
                                     Data    => (0 => 0),
                                     Status  => Status,
                                     Timeout => 5_000);
         if Addr mod 20 = 0 then
            LCD_Std_Out.Clear_Screen;
         end if;

         LCD_Std_Out.Put_Line("* Probe Addr "&Addr'Img&"-> Status : "&Status'Img);
         if Status = Ok then
            Cam_Addr := Addr;
          return True;
         end if;
         delay until Clock + Milliseconds (1);
      end loop;
      return False;
   end Probe;

   ----------------
   -- Initialize --
   ----------------

   procedure Initialize is
      procedure Initialize_Clock;
      procedure Initialize_Camera;
      procedure Initialize_IO;
      procedure Initialize_DCMI;
      procedure Initialize_DMA;

      ----------------------
      -- Initialize_Clock --
      ----------------------

      procedure Initialize_Clock is
      begin
         LCD_Std_Out.Put_Line("           Initialize_Clock");
         Configure_PWM_Timer (SENSOR_CLK_TIM'Access, SENSOR_CLK_FREQ);
         LCD_Std_Out.Put_Line("             Configure_PWM_Timer done");
         CLK_PWM_Mod.Attach_PWM_Channel
          (Generator => SENSOR_CLK_TIM'Access,
           Channel   => SENSOR_CLK_CHAN,
           Point     => SENSOR_CLK_IO,
           PWM_AF    => SENSOR_CLK_AF);
         LCD_Std_Out.Put_Line("             CLK_PWM_Mod.Attach_PWM_Channel done");
         CLK_PWM_Mod.Set_Duty_Cycle (Value => 50);
         LCD_Std_Out.Put_Line("             CLK_PWM_Mod.Set_Duty_Cycle done");

         CLK_PWM_Mod.Enable_Output;
         LCD_Std_Out.Put_Line("             CLK_PWM_Mod.Enable_Output done");

      end Initialize_Clock;

      -----------------------
      -- Initialize_Camera --
      -----------------------

      procedure Initialize_Camera is
         Cam_Addr : UInt10;
         Data : I2C_Data (1 .. 1);
         Status : I2C_Status;
      begin

         --  Power cycle
         LCD_Std_Out.Put_Line("    Initialize_Camera");
         Set (DCMI_PWDN);
         LCD_Std_Out.Put_Line("      DCMI_PWDN set");
         delay until Clock + Milliseconds (10);
         Clear (DCMI_PWDN);
         LCD_Std_Out.Put_Line("      DCMI_PWDN cleared");
         delay until Clock + Milliseconds (10);

         Initialize_Clock;
         LCD_Std_Out.Put_Line("      Initialize_Clock");

         Set (DCMI_RST);
         LCD_Std_Out.Put_Line("      DCMI_RST set");
         delay until Clock + Milliseconds (10);
         Clear (DCMI_RST);
         LCD_Std_Out.Put_Line("      DCMI_RST cleared");
         delay until Clock + Milliseconds (10);

         if  not Probe (Cam_Addr) then
            LCD_Std_Out.Put_Line("        no probe");
            --  Retry with reversed reset polarity
            Set (DCMI_RST);
            LCD_Std_Out.Put_Line("          DCMI_RST set");
            delay until Clock + Milliseconds (10);

            if  not Probe (Cam_Addr) then
               LCD_Std_Out.Put_Line("        Initialize_Camera Error 001 raised");
               raise Program_Error;
            end if;
         end if;

         delay until Clock + Milliseconds (10);

         --  Select sensor bank
         LCD_Std_Out.Put_Line("      Sensor_I2C.Mem_Write started");
         Sensor_I2C.Mem_Write (Addr          => Cam_Addr,
                               Mem_Addr      => 16#FF#,
                               Mem_Addr_Size => Memory_Size_8b,
                               Data          => (0 => 1),
                               Status        => Status);
         LCD_Std_Out.Put_Line("      Sensor_I2C.Mem_Write done");
         if Status /= Ok then
            LCD_Std_Out.Put_Line("        Initialize_Camera Error 002 raised");
            raise Program_Error;
         end if;

         delay until Clock + Milliseconds (10);

         Sensor_I2C.Master_Transmit (Addr    => Cam_Addr,
                                     Data    => (1 => REG_PID),
                                     Status  => Status);
         LCD_Std_Out.Put_Line("      Sensor_I2C.Master_Transmit");
         if Status /= Ok then
            LCD_Std_Out.Put_Line("        Initialize_Camera Error 003 raised");
            raise Program_Error;
         end if;

         Sensor_I2C.Master_Receive (Addr    => Cam_Addr,
                                    Data    => Data,
                                    Status  => Status);
         LCD_Std_Out.Put_Line("      Sensor_I2C.Master_Receive");
         if Status /= Ok then
            LCD_Std_Out.Put_Line("        Initialize_Camera Error 004 raised");
            raise Program_Error;
         end if;

         if Status /= Ok then
            LCD_Std_Out.Put_Line("        Initialize_Camera Error 005 raised");
            raise Program_Error;
         end if;
         Camera_PID := Data (Data'First);
         LCD_Std_Out.Put_Line("       Camera_PID "&Camera_PID'Img);
         case Camera_PID is
            when OV2640_PID =>
               Initialize (Camera_2640, Cam_Addr);
               Set_Pixel_Format (Camera_2640, Pix_RGB565);
               Set_Frame_Size (Camera_2640, QQVGA2);
            when OV7670_PID =>
               Initialize (Camera_7670, Cam_Addr);
               Set_Pixel_Format (Camera_7670, Pix_RGB565);
               Set_Frame_Size (Camera_7670, QQVGA2);
            when OV7725_PID =>
               Initialize (Camera_7725, Cam_Addr);
               Set_Pixel_Format (Camera_7725, Pix_RGB565);
               Set_Frame_Size (Camera_7725, QQVGA2);
            when others =>
               LCD_Std_Out.Put_Line("        Initialize_Camera Error 006 raised");
               raise Program_Error with "Unknown camera module";
         end case;

      end Initialize_Camera;

      -------------------
      -- Initialize_IO --
      -------------------

      procedure Initialize_IO is
         GPIO_Conf : GPIO_Port_Configuration;
         DCMI_AF_Points : constant GPIO_Points :=
           GPIO_Points'(DCMI_D0, DCMI_D1, DCMI_D2, DCMI_D3, DCMI_D4,
                        DCMI_D5, DCMI_D6, DCMI_D7, DCMI_VSYNC, DCMI_HSYNC,
                        DCMI_PCLK);
         DCMI_Out_Points : GPIO_Points :=
           GPIO_Points'(DCMI_PWDN, DCMI_RST, FS_IN);
      begin

         STM32.Setup.Setup_I2C_Master (Port        => Sensor_I2C,
                                       SDA         => Sensor_I2C_SDA,
                                       SCL         => Sensor_I2C_SCL,
                                       SDA_AF      => Sensor_I2C_AF,
                                       SCL_AF      => Sensor_I2C_AF,
                                       Clock_Speed => 100_000);

         Enable_Clock (DCMI_Out_Points);
         --  Sensor PowerDown, Reset and FSIN
         GPIO_Conf := (Mode        => Mode_Out,
                       Output_Type => Push_Pull,
                       Speed       => Speed_100MHz,  -- not specified in previous version of this module
                       Resistors   => Pull_Down);
         Configure_IO (DCMI_Out_Points, GPIO_Conf);

         Clear (DCMI_Out_Points);

         GPIO_Conf := (Mode           => Mode_AF,
                       AF             => GPIO_AF_DCMI_13,
                       AF_Speed       => Speed_100MHz,  -- not specified in previous version of this module
                       AF_Output_Type => Push_Pull,
                       Resistors      => Pull_Down);

         Configure_IO (DCMI_AF_Points, GPIO_Conf);
      end Initialize_IO;

      ---------------------
      -- Initialize_DCMI --
      ---------------------

      procedure Initialize_DCMI is
         Vertical    : DCMI.DCMI_Polarity;
         Horizontal  : DCMI.DCMI_Polarity;
         Pixel_Clock : DCMI.DCMI_Polarity;
      begin
         case Camera_PID is
            when OV2640_PID =>
               Vertical    := DCMI.Active_Low;
               Horizontal  := DCMI.Active_Low;
               Pixel_Clock := DCMI.Active_High;
            when OV7670_PID =>
               Vertical    := DCMI.Active_High;
               Horizontal  := DCMI.Active_Low;
               Pixel_Clock := DCMI.Active_High;
            when OV7725_PID =>
               Vertical    := DCMI.Active_High;
               Horizontal  := DCMI.Active_Low;
               Pixel_Clock := DCMI.Active_High;
            when others =>
               raise Program_Error with "Unknown camera module";
         end case;

         Enable_DCMI_Clock;
         DCMI.Configure (Data_Mode            => DCMI.DCMI_8bit,
                         Capture_Rate         => DCMI.Capture_All,
                         Vertical_Polarity    => Vertical,
                         Horizontal_Polarity  => Horizontal,
                         Pixel_Clock_Polarity => Pixel_Clock,

                         Hardware_Sync        => True,
                         JPEG                 => False);
         DCMI.Disable_Crop;
         DCMI.Enable_DCMI;
      end Initialize_DCMI;

      --------------------
      -- Initialize_DMA --
      --------------------

      procedure Initialize_DMA is
         Config : DMA_Stream_Configuration;
      begin
         Enable_Clock (Sensor_DMA);
         Config.Channel := Sensor_DMA_Chan;
         Config.Direction := Peripheral_To_Memory;
         Config.Increment_Peripheral_Address := False;
         Config.Increment_Memory_Address := True;
         Config.Peripheral_Data_Format := Words;
         Config.Memory_Data_Format := Words;
         Config.Operation_Mode := Normal_Mode;
         Config.Priority := Priority_High;
         Config.FIFO_Enabled := True;
         Config.FIFO_Threshold := FIFO_Threshold_Full_Configuration;
         Config.Memory_Burst_Size := Memory_Burst_Inc4;
         Config.Peripheral_Burst_Size := Peripheral_Burst_Single;
         Configure (Sensor_DMA, Sensor_DMA_Stream, Config);
      end Initialize_DMA;
   begin
      LCD_Std_Out.Put_Line("Initializing Camera");
      Initialize_IO;
      LCD_Std_Out.Put_Line("  IO Initialized");
      Initialize_Camera;
      LCD_Std_Out.Put_Line("  Camera Initialized");
      Initialize_DCMI;
      LCD_Std_Out.Put_Line("  DCMI Initialized");
      Initialize_DMA;
      LCD_Std_Out.Put_Line("  DMA Initialized");
      Is_Initialized := True;
      LCD_Std_Out.Put_Line("Initializing Finished");
   end Initialize;

   --------------
   -- Snapshot --
   --------------

   procedure Snapshot (BM : not null HAL.Bitmap.Any_Bitmap_Buffer) is
      Status : DMA_Error_Code;
      Cnt : constant UInt16 := UInt16 ((BM.Width * BM.Height) / 2);
   begin
      if BM.Width /= Image_Width
        or else
          BM.Height /= Image_Height
        or else
          not BM.Mapped_In_RAM
      then
         LCD_Std_Out.Put_Line("Program_Error 001");
         raise Program_Error;
      end if;

      LCD_Std_Out.Put_Line("Snapshot Width -> "&BM.Width'Img&" / Height -> "&BM.Height'Img);
      if not Compatible_Alignments (Sensor_DMA,
                                    Sensor_DMA_Stream,
                                    DCMI.Data_Register_Address,
                                    BM.Memory_Address)
      then
         LCD_Std_Out.Put_Line("Program_Error 002");
         raise Program_Error;
      end if;

      Sensor_DMA_Int.Start_Transfer
        (Source      => DCMI.Data_Register_Address,
         Destination => BM.Memory_Address,
         Data_Count  => Cnt);

      DCMI.Start_Capture (DCMI.Snapshot);

      Sensor_DMA_Int.Wait_For_Completion (Status);

      if Status /= DMA_No_Error then
         if Status = DMA_Timeout_Error then
            LCD_Std_Out.Put_Line("Program_Error 003");
            raise Program_Error with "DMA timeout! Transferred: " &
              Items_Transferred (Sensor_DMA, Sensor_DMA_Stream)'Img;
         else
            LCD_Std_Out.Put_Line("Program_Error 004");
            raise Program_Error;
         end if;
      end if;
   end Snapshot;
end Demo_Camera.Sensor;
