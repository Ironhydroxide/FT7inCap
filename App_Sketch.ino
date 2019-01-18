/*****************************************************************************
* Copyright (c) Future Technology Devices International 2014
* propriety of Future Technology devices International.
*
* Software License Agreement
*
* This code is provided as an example only and is not guaranteed by FTDI. 
* FTDI accept no responsibility for any issues resulting from its use. 
* The developer of the final application incorporating any parts of this 
* sample project is responsible for ensuring its safe and correct operation 
* and for any consequences resulting from its use.
*****************************************************************************/
/**
* @file                           App_Sketch.c
* @brief                          Sample application to demonstrate FT800 primitives and built-in widgets 


version 4.0 - July/01/2014 - Support for FT81X chips.
version 3.1 - Jun/13/2014 - Application now uses CMD_CSKETCH for FT801 platform.
Version 3.0 - Support for FT801 platform.
Version 2.0 - Support for FT800 emulator platform.
Version 1.0 - Final version based on the requirements.
Version 0.1 - intial draft of the release notes
*
*/

#include <stdio.h>
#include <Arduino.h>
#include <EEPROM.h>
#include <SPI.h>
#include <avr/pgmspace.h>
#include "Platform.h"


//#define SAMAPP_DELAY_BTW_APIS (1000)
//#define SAMAPP_ENABLE_DELAY() /*Gpu_Hal_Sleep*/ delay(SAMAPP_DELAY_BTW_APIS)
//#define SAMAPP_ENABLE_DELAY_VALUE(x) /*Gpu_Hal_Sleep*/ delay(x)


/* Global variables for display resolution to support various display panels */
/* Default is WQVGA - 480x272 */
/*int16_t DispWidth = 480;
int16_t DispHeight = 272;
int16_t DispHCycle =  548;
int16_t DispHOffset = 43;
int16_t DispHSync0 = 0;
int16_t DispHSync1 = 41;
int16_t DispVCycle = 292;
int16_t DispVOffset = 12;
int16_t DispVSync0 = 0;
int16_t DispVSync1 = 10;
uint8_t DispPCLK = 5;
char8_t DispSwizzle = 0;
char8_t DispPCLKPol = 1;
char8_t DispCSpread = 1;
char8_t DispDither = 1;*/
//non default settings, default is above
int16_t	DispWidth = 800;
int16_t	DispHeight = 480;
int16_t	DispHCycle =  928;
int16_t	DispHOffset = 88;
int16_t	DispHSync0 = 0;
int16_t	DispHSync1 = 48;
int16_t	DispVCycle = 525;
int16_t	DispVOffset = 32;
int16_t	DispVSync0 = 0;
int16_t	DispVSync1 = 3;
uint8_t	DispPCLK = 2;
char8_t	DispSwizzle = 0;
char8_t	DispPCLKPol = 1;
char8_t	DispCSpread = 0;
char8_t	DispDither = 1;

/* Global used for buffer optimization */
Gpu_Hal_Context_t host,*phost;

uint32_t CmdBuffer_Index;
uint32_t DlBuffer_Index;

/*#ifdef BUFFER_OPTIMIZATION
uint8_t  DlBuffer[DL_SIZE];
uint8_t  CmdBuffer[CMD_FIFO_SIZE];
#endif*/
/* Boot up for FT800 followed by graphics primitive sample cases */
/* Initial boot up DL - make the back ground green color */
const uint8_t DLCODE_BOOTUP[12] = 
{
    0,0,0,2,//GPU instruction CLEAR_COLOR_RGB
    7,0,0,38, //GPU instruction CLEAR
    0,0,0,0,  //GPU instruction DISPLAY
};
void App_WrCoCmd_Buffer(Gpu_Hal_Context_t *phost,uint32_t cmd)
{
/*#ifdef  BUFFER_OPTIMIZATION
  // Copy the command instruction into buffer 
  uint32_t *pBuffcmd;
  pBuffcmd =(uint32_t*)&CmdBuffer[CmdBuffer_Index];
  *pBuffcmd = cmd;
#endif*/
#ifdef ARDUINO_PLATFORM
  Gpu_Hal_WrCmd32(phost,cmd);
#endif
/*#ifdef FT900_PLATFORM
   Gpu_Hal_WrCmd32(phost,cmd);
#endif*/
   /* Increment the command index */
   CmdBuffer_Index += CMD_SIZE;  
}

void App_WrDlCmd_Buffer(Gpu_Hal_Context_t *phost,uint32_t cmd)
{
/*#ifdef BUFFER_OPTIMIZATION  
  // Copy the command instruction into buffer 
  uint32_t *pBuffcmd;
  pBuffcmd =(uint32_t*)&DlBuffer[DlBuffer_Index];
  *pBuffcmd = cmd;
#endif*/

#ifdef ARDUINO_PLATFORM
  Gpu_Hal_Wr32(phost,(RAM_DL+DlBuffer_Index),cmd);
#endif
/*#ifdef FT900_PLATFORM
   Gpu_Hal_Wr32(phost,(RAM_DL+DlBuffer_Index),cmd);
#endif*/
   /* Increment the command index */
   DlBuffer_Index += CMD_SIZE;  
}

void App_WrCoStr_Buffer(Gpu_Hal_Context_t *phost,const char8_t *s)
{
/*#ifdef  BUFFER_OPTIMIZATION  
  uint16_t length = 0;
  length = strlen(s) + 1;//last for the null termination

  strcpy(&CmdBuffer[CmdBuffer_Index],s);  

  // increment the length and align it by 4 bytes 
  CmdBuffer_Index += ((length + 3) & ~3);  
#endif*/  
}

void App_Flush_DL_Buffer(Gpu_Hal_Context_t *phost)
{
/*#ifdef  BUFFER_OPTIMIZATION    
  if (DlBuffer_Index > 0)
    Gpu_Hal_WrMem(phost,RAM_DL,DlBuffer,DlBuffer_Index);
#endif*/     
  DlBuffer_Index = 0;

}

void App_Flush_Co_Buffer(Gpu_Hal_Context_t *phost)
{
/*#ifdef  BUFFER_OPTIMIZATION    
  if (CmdBuffer_Index > 0)
    Gpu_Hal_WrCmdBuf(phost,CmdBuffer,CmdBuffer_Index);
#endif*/     
  CmdBuffer_Index = 0;
}


/* API to give fadeout effect by changing the display PWM from 100 till 0 */
void SAMAPP_fadeout()
{
  int32_t i;

  for (i = 100; i >= 0; i -= 3) 
  {
    Gpu_Hal_Wr8(phost,REG_PWM_DUTY,i);

    /*Gpu_Hal_Sleep*/ delay(2);//sleep for 2 ms
  }
}

/* API to perform display fadein effect by changing the display PWM from 0 till 100 and finally 128 */
void SAMAPP_fadein()
{
  int32_t i;

  for (i = 0; i <=100 ; i += 3) 
  {
    Gpu_Hal_Wr8(phost,REG_PWM_DUTY,i);
    /*Gpu_Hal_Sleep*/ delay(2);//sleep for 2 ms
  }
  /* Finally make the PWM 100% */
  i = 128;
  Gpu_Hal_Wr8(phost,REG_PWM_DUTY,i);
}


/* API to check the status of previous DLSWAP and perform DLSWAP of new DL */
/* Check for the status of previous DLSWAP and if still not done wait for few ms and check again */
void SAMAPP_GPU_DLSwap(uint8_t DL_Swap_Type)
{
  uint8_t Swap_Type = DLSWAP_FRAME,Swap_Done = DLSWAP_FRAME;

  if(DL_Swap_Type == DLSWAP_LINE)
  {
    Swap_Type = DLSWAP_LINE;
  }

  /* Perform a new DL swap */
  Gpu_Hal_Wr8(phost,REG_DLSWAP,Swap_Type);

  /* Wait till the swap is done */
  while(Swap_Done)
  {
    Swap_Done = Gpu_Hal_Rd8(phost,REG_DLSWAP);

    if(DLSWAP_DONE != Swap_Done)
    {
      /*Gpu_Hal_Sleep*/ delay(10);//wait for 10ms
    }
  }	
}

void BootupConfig()
{
	Gpu_Hal_Powercycle(phost,TRUE);//cycles PD pin (low/high for TRUE, high/low for FALSE)

	/* Access address 0 to wake up the FT800 */
	Gpu_HostCommand(phost,GPU_ACTIVE_M);//sends cmd on SPI Gpu_HostCommand(*host, cmd) GPU_ACTIVE_M is 0x00,
	/*Gpu_Hal_Sleep*/ delay(20);

	/* Set the clk to external clock */
/*#if (!defined(ME800A_HV35R) && !defined(ME810A_HV35R))
		Gpu_HostCommand(phost,GPU_EXTERNAL_OSC);
		//Gpu_Hal_Sleep delay(10);
#endif*/

	uint8_t chipid;
	//Read Register ID to check if FT800 is ready.
	chipid = Gpu_Hal_Rd8(phost, REG_ID);//sends REG_ID on SPI then empty byte, returns 8 bits
	while(chipid != 0x7C)
	{
		chipid = Gpu_Hal_Rd8(phost, REG_ID); //REG_ID is 3153920UL //Offset 0x00
		delay(100);
	}
	/*#if defined(MSVC_PLATFORM)*//* || defined (FT900_PLATFORM)*//*
			printf("VC1 register ID after wake up %x\n",chipid);
	#endif*/
	
	/* Configuration of LCD display */
/*#ifdef DISPLAY_RESOLUTION_QVGA
	// Values specific to QVGA LCD display 
	DispWidth = 320;
	DispHeight = 240;
	DispHCycle =  408;
	DispHOffset = 70;
	DispHSync0 = 0;
	DispHSync1 = 10;
	DispVCycle = 263;
	DispVOffset = 13;
	DispVSync0 = 0;
	DispVSync1 = 2;
	DispPCLK = 8;
	DispSwizzle = 2;
	DispPCLKPol = 0;
	DispCSpread = 1;
	DispDither = 1;

#endif*/
#ifdef DISPLAY_RESOLUTION_WVGA
	/* Values specific to QVGA LCD display */
	DispWidth = 800;
	DispHeight = 480;
	DispHCycle =  928;
	DispHOffset = 88;
	DispHSync0 = 0;
	DispHSync1 = 48;
	DispVCycle = 525;
	DispVOffset = 32;
	DispVSync0 = 0;
	DispVSync1 = 3;
	DispPCLK = 2;
	DispSwizzle = 0;
	DispPCLKPol = 1;
	DispCSpread = 0;
	DispDither = 1;
#endif
/*#ifdef DISPLAY_RESOLUTION_HVGA_PORTRAIT
	/// Values specific to HVGA LCD display 

	DispWidth = 320;
	DispHeight = 480;
	DispHCycle =  400;
	DispHOffset = 40;
	DispHSync0 = 0;
	DispHSync1 = 10;
	DispVCycle = 500;
	DispVOffset = 10;
	DispVSync0 = 0;
	DispVSync1 = 5;
	DispPCLK = 4;
	DispSwizzle = 2;
	DispPCLKPol = 1;
	DispCSpread = 1;
	DispDither = 1;

*//*#ifdef ME810A_HV35R
	DispPCLK = 5;
#endif*//*

#endif*/

/*#if (defined(ME800A_HV35R) || defined(ME810A_HV35R))
	// After recognizing the type of chip, perform the trimming if necessary 
    Gpu_ClockTrimming(phost,LOW_FREQ_BOUND);
#endif*/

	Gpu_Hal_Wr16(phost, REG_HCYCLE, DispHCycle);
	Gpu_Hal_Wr16(phost, REG_HOFFSET, DispHOffset);
	Gpu_Hal_Wr16(phost, REG_HSYNC0, DispHSync0);
	Gpu_Hal_Wr16(phost, REG_HSYNC1, DispHSync1);
	Gpu_Hal_Wr16(phost, REG_VCYCLE, DispVCycle);
	Gpu_Hal_Wr16(phost, REG_VOFFSET, DispVOffset);
	Gpu_Hal_Wr16(phost, REG_VSYNC0, DispVSync0);
	Gpu_Hal_Wr16(phost, REG_VSYNC1, DispVSync1);
	Gpu_Hal_Wr8(phost, REG_SWIZZLE, DispSwizzle);
	Gpu_Hal_Wr8(phost, REG_PCLK_POL, DispPCLKPol);
	Gpu_Hal_Wr16(phost, REG_HSIZE, DispWidth);
	Gpu_Hal_Wr16(phost, REG_VSIZE, DispHeight);
	Gpu_Hal_Wr16(phost, REG_CSPREAD, DispCSpread);
	Gpu_Hal_Wr16(phost, REG_DITHER, DispDither);

#if (/*defined(800_ENABLE) || *//*defined(810_ENABLE) ||*/ defined(812_ENABLE))
    /* Touch configuration - configure the resistance value to 1200 - this value is specific to customer requirement and derived by experiment */
    Gpu_Hal_Wr16(phost, REG_TOUCH_RZTHRESH,RESISTANCE_THRESHOLD);
#endif
    Gpu_Hal_Wr8(phost, REG_GPIO_DIR,0xff);
    Gpu_Hal_Wr8(phost, REG_GPIO,0xff);


    /*It is optional to clear the screen here*/
    Gpu_Hal_WrMem(phost, RAM_DL,(uint8_t *)DLCODE_BOOTUP,sizeof(DLCODE_BOOTUP));
    Gpu_Hal_Wr8(phost, REG_DLSWAP,DLSWAP_FRAME);


    Gpu_Hal_Wr8(phost, REG_PCLK,DispPCLK);//after this display is visible on the LCD


/*#ifdef ENABLE_ILI9488_HVGA_PORTRAIT
	// to cross check reset pin 
	Gpu_Hal_Wr8(phost, REG_GPIO,0xff);
	delay(120);
	Gpu_Hal_Wr8(phost, REG_GPIO,0x7f);
	delay(120);
	Gpu_Hal_Wr8(phost, REG_GPIO,0xff);

	ILI9488_Bootup();

	//Reconfigure the SPI 
*//*#ifdef FT900_PLATFORM
	printf("after ILI9488 bootup \n");
	//spi
	// Initialize SPIM HW
	sys_enable(sys_device_spi_master);
	gpio_function(27, pad_spim_sck); // GPIO27 to SPIM_CLK 
	gpio_function(28, pad_spim_ss0); // GPIO28 as CS 
	gpio_function(29, pad_spim_mosi); // GPIO29 to SPIM_MOSI 
	gpio_function(30, pad_spim_miso); // GPIO30 to SPIM_MISO 

	gpio_write(28, 1);
	spi_init(SPIM, spi_dir_master, spi_mode_0, 4);
#endif*//*

#endif*/





	/* make the spi to quad mode - addition 2 bytes for silicon */
#ifdef 81X_ENABLE
	/* api to set quad and numbe of dummy bytes */
/*#ifdef ENABLE_SPI_QUAD
	Gpu_Hal_SetSPI(phost,GPU_SPI_QUAD_CHANNEL,GPU_SPI_TWODUMMY);
#elif ENABLE_SPI_DUAL
	Gpu_Hal_SetSPI(phost,GPU_SPI_DUAL_CHANNEL,GPU_SPI_TWODUMMY);
#else*/
	Gpu_Hal_SetSPI(phost,GPU_SPI_SINGLE_CHANNEL,GPU_SPI_ONEDUMMY);
/*#endif*/

#endif

/*#ifdef FT900_PLATFORM
    // Change clock frequency to 25mhz 
	spi_init(SPIM, spi_dir_master, spi_mode_0, 4);

#if (defined(ENABLE_SPI_QUAD))
    // Initialize IO2 and IO3 pad/pin for dual and quad settings 
    gpio_function(31, pad_spim_io2);
    gpio_function(32, pad_spim_io3);
    gpio_write(31, 1);
    gpio_write(32, 1);
#endif
	// Enable FIFO of QSPI 
	spi_option(SPIM,spi_option_fifo_size,64);
	spi_option(SPIM,spi_option_fifo,1);
	spi_option(SPIM,spi_option_fifo_receive_trigger,1);
#endif*/

/*#ifdef ENABLE_SPI_QUAD
*//*#ifdef FT900_PLATFORM
	spi_option(SPIM,spi_option_bus_width,4);
#endif*//*
#elif ENABLE_SPI_DUAL	
*//*#ifdef FT900_PLATFORM
	spi_option(SPIM,spi_option_bus_width,2);
#endif*//*
#else
*//*#ifdef FT900_PLATFORM
	spi_option(SPIM,spi_option_bus_width,1);
#endif*//*

#endif*/
phost->cmd_fifo_wp = Gpu_Hal_Rd16(phost,REG_CMD_WRITE);

}




/*****************************************************************************/
/* Example code to display few points at various offsets with various colors */



/* Boot up for FT800 followed by graphics primitive sample cases */
/* Initial boot up DL - make the back ground green color */

static uint8_t home_star_icon[] = {0x78,0x9C,0xE5,0x94,0xBF,0x4E,0xC2,0x40,0x1C,0xC7,0x7F,0x2D,0x04,0x8B,0x20,0x45,0x76,0x14,0x67,0xA3,0xF1,0x0D,0x64,0x75,0xD2,0xD5,0x09,0x27,0x17,0x13,0xE1,0x0D,0xE4,0x0D,0x78,0x04,0x98,0x5D,0x30,0x26,0x0E,0x4A,0xA2,0x3E,0x82,0x0E,0x8E,0x82,0xC1,0x38,0x62,0x51,0x0C,0x0A,0x42,0x7F,0xDE,0xB5,0x77,0xB4,0x77,0x17,0x28,0x21,0x26,0x46,0xFD,0x26,0xCD,0xE5,0xD3,0x7C,0xFB,0xBB,0xFB,0xFD,0xB9,0x02,0xCC,0xA4,0xE8,0x99,0x80,0x61,0xC4,0x8A,0x9F,0xCB,0x6F,0x31,0x3B,0xE3,0x61,0x7A,0x98,0x84,0x7C,0x37,0xF6,0xFC,0xC8,0xDD,0x45,0x00,0xDD,0xBA,0xC4,0x77,0xE6,0xEE,0x40,0xEC,0x0E,0xE6,0x91,0xF1,0xD2,0x00,0x42,0x34,0x5E,0xCE,0xE5,0x08,0x16,0xA0,0x84,0x68,0x67,0xB4,0x86,0xC3,0xD5,0x26,0x2C,0x20,0x51,0x17,0xA2,0xB8,0x03,0xB0,0xFE,0x49,0xDD,0x54,0x15,0xD8,0xEE,0x73,0x37,0x95,0x9D,0xD4,0x1A,0xB7,0xA5,0x26,0xC4,0x91,0xA9,0x0B,0x06,0xEE,0x72,0xB7,0xFB,0xC5,0x16,0x80,0xE9,0xF1,0x07,0x8D,0x3F,0x15,0x5F,0x1C,0x0B,0xFC,0x0A,0x90,0xF0,0xF3,0x09,0xA9,0x90,0xC4,0xC6,0x37,0xB0,0x93,0xBF,0xE1,0x71,0xDB,0xA9,0xD7,0x41,0xAD,0x46,0xEA,0x19,0xA9,0xD5,0xCE,0x93,0xB3,0x35,0x73,0x0A,0x69,0x59,0x91,0xC3,0x0F,0x22,0x1B,0x1D,0x91,0x13,0x3D,0x91,0x73,0x43,0xF1,0x6C,0x55,0xDA,0x3A,0x4F,0xBA,0x25,0xCE,0x4F,0x04,0xF1,0xC5,0xCF,0x71,0xDA,0x3C,0xD7,0xB9,0xB2,0x48,0xB4,0x89,0x38,0x20,0x4B,0x2A,0x95,0x0C,0xD5,0xEF,0x5B,0xAD,0x96,0x45,0x8A,0x41,0x96,0x7A,0x1F,0x60,0x0D,0x7D,0x22,0x75,0x82,0x2B,0x0F,0xFB,0xCE,0x51,0x3D,0x2E,0x3A,0x21,0xF3,0x1C,0xD9,0x38,0x86,0x2C,0xC6,0x05,0xB6,0x7B,0x9A,0x8F,0x0F,0x97,0x1B,0x72,0x6F,0x1C,0xEB,0xAE,0xFF,0xDA,0x97,0x0D,0xBA,0x43,0x32,0xCA,0x66,0x34,0x3D,0x54,0xCB,0x24,0x9B,0x43,0xF2,0x70,0x3E,0x42,0xBB,0xA0,0x95,0x11,0x37,0x46,0xE1,0x4F,0x49,0xC5,0x1B,0xFC,0x3C,0x3A,0x3E,0xD1,0x65,0x0E,0x6F,0x58,0xF8,0x9E,0x5B,0xDB,0x55,0xB6,0x41,0x34,0xCB,0xBE,0xDB,0x87,0x5F,0xA9,0xD1,0x85,0x6B,0xB3,0x17,0x9C,0x61,0x0C,0x9B,0xA2,0x5D,0x61,0x10,0xED,0x2A,0x9B,0xA2,0x5D,0x61,0x10,0xED,0x2A,0x9B,0xA2,0x5D,0x61,0x10,0xED,0x2A,0x9B,0xED,0xC9,0xFC,0xDF,0x14,0x54,0x8F,0x80,0x7A,0x06,0xF5,0x23,0xA0,0x9F,0x41,0xF3,0x10,0x30,0x4F,0x41,0xF3,0x18,0x30,0xCF,0xCA,0xFC,0xFF,0x35,0xC9,0x79,0xC9,0x89,0xFA,0x33,0xD7,0x1D,0xF6,0x5E,0x84,0x5C,0x56,0x6E,0xA7,0xDA,0x1E,0xF9,0xFA,0xAB,0xF5,0x97,0xFF,0x2F,0xED,0x89,0x7E,0x29,0x9E,0xB4,0x9F,0x74,0x1E,0x69,0xDA,0xA4,0x9F,0x81,0x94,0xEF,0x4F,0xF6,0xF9,0x0B,0xF4,0x65,0x51,0x08};

char* const info[] PROGMEM = {  "FT800 Sketch Application",
                          "APP to demonstrate interactive Sketch,",
                          "using Sketch, Slider",
                          "& Buttons"
                       }; 

/***********************API used to SET the ICON******************************************/
/*Refer the code flow in the flowchart availble in the Application Note */

void home_setup()
{
  /*Icon  file is deflated use J1 Command to inflate the file and write into the GRAM*/
  Gpu_Hal_WrCmd32(phost,CMD_INFLATE);
  Gpu_Hal_WrCmd32(phost,250*1024L);
  Gpu_Hal_WrCmdBuf(phost,home_star_icon,sizeof(home_star_icon));
  /*Set the Bitmap properties for the ICONS*/ 	
  Gpu_CoCmd_Dlstart(phost);        // start
  App_WrCoCmd_Buffer(phost,CLEAR(1,1,1));
  App_WrCoCmd_Buffer(phost,COLOR_RGB(255, 255, 255));
  App_WrCoCmd_Buffer(phost,BITMAP_HANDLE(13));    // handle for background stars
  App_WrCoCmd_Buffer(phost,BITMAP_SOURCE(250*1024L));      // Starting address in gram
  App_WrCoCmd_Buffer(phost,BITMAP_LAYOUT(L4, 16, 32));  // format 
  App_WrCoCmd_Buffer(phost,BITMAP_SIZE(NEAREST, REPEAT, REPEAT, 512, 512  ));
  App_WrCoCmd_Buffer(phost,BITMAP_HANDLE(14));    // handle for background stars
  App_WrCoCmd_Buffer(phost,BITMAP_SOURCE(250*1024L));      // Starting address in gram
  App_WrCoCmd_Buffer(phost,BITMAP_LAYOUT(L4, 16, 32));  // format 
  App_WrCoCmd_Buffer(phost,BITMAP_SIZE(NEAREST, BORDER, BORDER, 32, 32  ));
  App_WrCoCmd_Buffer(phost,DISPLAY());
  Gpu_CoCmd_Swap(phost);
  App_Flush_Co_Buffer(phost);
  Gpu_Hal_WaitCmdfifo_empty(phost);
}
/********API to return the assigned TAG value when penup,for the primitives/widgets******/


static uint8_t sk=0;
uint8_t Read_Keys()
{
  static uint8_t Read_tag=0,temp_tag=0,ret_tag=0;	
  Read_tag = Gpu_Hal_Rd8(phost,REG_TOUCH_TAG);
  ret_tag = NULL;
  if(Read_tag!=NULL)								// Allow if the Key is released
  {
    if(temp_tag!=Read_tag)
    {
      temp_tag = Read_tag;	
      sk = Read_tag;										// Load the Read tag to temp variable	
    }  
  }
  else
  {
    if(temp_tag!=0)
    {
      ret_tag = temp_tag;
    }  
    sk = 0;
  }
  return ret_tag;
}

void Play_Sound(uint8_t sound,uint8_t vol,uint8_t midi)
{
  uint16_t val = (midi << 8) | sound; 
  Gpu_Hal_Wr8(phost,REG_SOUND,val);
  Gpu_Hal_Wr8(phost,REG_PLAY,1); 
}

// Info Screen//
void Info()
{
  uint16_t dloffset = 0,z;
  CmdBuffer_Index = 0;
  
// Touch Screen Calibration
  
  Gpu_CoCmd_Dlstart(phost); 
  App_WrCoCmd_Buffer(phost,CLEAR(1,1,1));
  App_WrCoCmd_Buffer(phost,COLOR_RGB(255,255,255));
  Gpu_CoCmd_Text(phost,DispWidth/2,DispHeight/2,26,OPT_CENTERX|OPT_CENTERY,"Please tap on a dot");
  Gpu_CoCmd_Calibrate(phost,0);
  App_Flush_Co_Buffer(phost);
  Gpu_Hal_WaitCmdfifo_empty(phost);
// Ftdi Logo animation 
  Gpu_CoCmd_Logo(phost);
  App_Flush_Co_Buffer(phost);
  Gpu_Hal_WaitCmdfifo_empty(phost);
  while(0!=Gpu_Hal_Rd16(phost,REG_CMD_READ)); 
  dloffset = Gpu_Hal_Rd16(phost,REG_CMD_DL);
  dloffset -= 4;

#ifdef 81X_ENABLE
  dloffset -= 2*4;//remove two more instructions in case of 81x
#endif


  Gpu_Hal_WrCmd32(phost,CMD_MEMCPY);
  Gpu_Hal_WrCmd32(phost,100000L);
  Gpu_Hal_WrCmd32(phost,RAM_DL);
  Gpu_Hal_WrCmd32(phost,dloffset);
  //Enter into Info Screen
  do
  {
    Gpu_CoCmd_Dlstart(phost);   
    Gpu_CoCmd_Append(phost,100000L,dloffset);
    //Reset the BITMAP properties used during Logo animation
    App_WrCoCmd_Buffer(phost,BITMAP_TRANSFORM_A(256));
    App_WrCoCmd_Buffer(phost,BITMAP_TRANSFORM_A(256));
    App_WrCoCmd_Buffer(phost,BITMAP_TRANSFORM_B(0));
    App_WrCoCmd_Buffer(phost,BITMAP_TRANSFORM_C(0));
    App_WrCoCmd_Buffer(phost,BITMAP_TRANSFORM_D(0));
    App_WrCoCmd_Buffer(phost,BITMAP_TRANSFORM_E(256));
    App_WrCoCmd_Buffer(phost,BITMAP_TRANSFORM_F(0));  
    App_WrCoCmd_Buffer(phost,SAVE_CONTEXT());	
    //Display the information with transparent Logo using Edge Strip  
    App_WrCoCmd_Buffer(phost,COLOR_RGB(219,180,150));
    App_WrCoCmd_Buffer(phost,COLOR_A(220));
    App_WrCoCmd_Buffer(phost,BEGIN(EDGE_STRIP_A));
    App_WrCoCmd_Buffer(phost,VERTEX2F(0,DispHeight*16));
    App_WrCoCmd_Buffer(phost,VERTEX2F(DispWidth*16,DispHeight*16));
    App_WrCoCmd_Buffer(phost,COLOR_A(255));
    App_WrCoCmd_Buffer(phost,RESTORE_CONTEXT());	
    App_WrCoCmd_Buffer(phost,COLOR_RGB(0,0,0));
   // INFORMATION 
    Gpu_CoCmd_Text(phost,DispWidth/2,20,28,OPT_CENTERX|OPT_CENTERY,(char*)pgm_read_word(&info[0]));
    Gpu_CoCmd_Text(phost,DispWidth/2,60,26,OPT_CENTERX|OPT_CENTERY,(char*)pgm_read_word(&info[1]));
    Gpu_CoCmd_Text(phost,DispWidth/2,90,26,OPT_CENTERX|OPT_CENTERY,(char*)pgm_read_word(&info[2]));  
    Gpu_CoCmd_Text(phost,DispWidth/2,120,26,OPT_CENTERX|OPT_CENTERY,(char*)pgm_read_word(&info[3]));  
    Gpu_CoCmd_Text(phost,DispWidth/2,DispHeight-30,26,OPT_CENTERX|OPT_CENTERY,"Click to play");
    //Check if the Play key enter then change the color
    if(sk!='P')
    App_WrCoCmd_Buffer(phost,COLOR_RGB(255,255,255));
    else
    App_WrCoCmd_Buffer(phost,COLOR_RGB(100,100,100));
    App_WrCoCmd_Buffer(phost,BEGIN(FTPOINTS));   
    App_WrCoCmd_Buffer(phost,POINT_SIZE(20*16));
    App_WrCoCmd_Buffer(phost,TAG('P'));
    App_WrCoCmd_Buffer(phost,VERTEX2F((DispWidth/2)*16,(DispHeight-60)*16));
    App_WrCoCmd_Buffer(phost,COLOR_RGB(180,35,35));
    App_WrCoCmd_Buffer(phost,BEGIN(BITMAPS));


#ifdef DISPLAY_RESOLUTION_WVGA
    App_WrCoCmd_Buffer(phost, BITMAP_HANDLE(14));
    App_WrCoCmd_Buffer(phost, CELL(4));
    App_WrCoCmd_Buffer(phost,VERTEX2F((DispWidth/2-14)*16,(DispHeight-75)*16));
/*#else
    App_WrCoCmd_Buffer(phost,VERTEX2II((DispWidth/2)-14,(DispHeight-75),14,4));*/
#endif
    App_WrCoCmd_Buffer(phost,DISPLAY());
    Gpu_CoCmd_Swap(phost);
    App_Flush_Co_Buffer(phost);
    Gpu_Hal_WaitCmdfifo_empty(phost);
  }while(Read_Keys()!='P');
  Play_Sound(0x50,255,0xc0);
}

/*#ifdef FT900_PLATFORM
void FT900_Config()
{
	sys_enable(sys_device_uart0);
		    gpio_function(48, pad_uart0_txd); // UART0 TXD 
		    gpio_function(49, pad_uart0_rxd); // UART0 RXD 
		    uart_open(UART0,                    // Device 
		              1,                        // Prescaler = 1 
		              UART_DIVIDER_115200_BAUD,  // Divider = 1302 
		              uart_data_bits_8,         // No. Data Bits 
		              uart_parity_none,         // Parity 
		              uart_stop_bits_1);        // No. Stop Bits 

		    // Print out a welcome message... 
		    uart_puts(UART0,

		        "(C) Copyright 2014-2015, Future Technology Devices International Ltd. \r\n"
		        "--------------------------------------------------------------------- \r\n"
		        "Welcome to Sketch Example ... \r\n"
		        "\r\n"
		        "--------------------------------------------------------------------- \r\n"
		        );

		    //init_printf(UART0,myputc);

	#ifdef ENABLE_ILI9488_HVGA_PORTRAIT
		// asign all the respective pins to gpio and set them to default values 
		gpio_function(34, pad_gpio34);
		gpio_dir(34, pad_dir_output);
	    gpio_write(34,1);

		gpio_function(27, pad_gpio27);
		gpio_dir(27, pad_dir_output);
	    gpio_write(27,1);

		gpio_function(29, pad_gpio29);
		gpio_dir(29, pad_dir_output);
	    gpio_write(29,1);

	    gpio_function(33, pad_gpio33);
	    gpio_dir(33, pad_dir_output);
	    gpio_write(33,1);


	    gpio_function(30, pad_gpio30);
	    gpio_dir(30, pad_dir_output);
	    gpio_write(30,1);

		gpio_function(28, pad_gpio28);
		gpio_dir(28, pad_dir_output);
	    gpio_write(28,1);


	  	gpio_function(43, pad_gpio43);
	  	gpio_dir(43, pad_dir_output);
	    gpio_write(43,1);
		gpio_write(34,1);
		gpio_write(28,1);
		gpio_write(43,1);
		gpio_write(33,1);
		gpio_write(33,1);

	#endif
		// useful for timer 
		millis_init();
		interrupt_enable_globally();
		//printf("ft900 config done \n");
	}
#endif*/

/* Application*/
void Sketch()
{
  uint32_t  tracker,color=0;
  uint16_t  val=32768;
  uint8_t tag =0;
//  Set the bitmap properties , sketch properties and Tracker for the sliders
  Gpu_CoCmd_Dlstart(phost);
  Gpu_CoCmd_FgColor(phost,0xffffff);        // Set the bg color
  Gpu_CoCmd_Track(phost,(DispWidth-30),40,8,DispHeight-100,1);
/*#if defined 801_ENABLE
  Gpu_CoCmd_CSketch(phost,0,10,DispWidth-40,DispHeight-30,0,L8,1500L);
#elif defined 81X_ENABLE*/
  Gpu_CoCmd_Sketch(phost,0,10,DispWidth-40,DispHeight-30,0,L8);
/*#else
  Gpu_CoCmd_Sketch(phost,0,10,DispWidth-40,DispHeight-30,0,L8);
#endif*/
/*
#if defined 801_ENABLE
  Gpu_CoCmd_CSketch(phost,0,10,DispWidth-40,DispHeight-20,0,L8,1500L);
#elif defined 81X_ENABLE
  Gpu_CoCmd_Sketch(phost,0,10,DispWidth-40,DispHeight-20,0,L8);
#else
  Gpu_CoCmd_Sketch(phost,0,10,DispWidth-40,DispHeight-20,0,L8);
#endif
  */
  Gpu_CoCmd_MemZero(phost,0L,(DispWidth-40)*(DispHeight-20L));  
  App_WrCoCmd_Buffer(phost,BITMAP_HANDLE(1));
  App_WrCoCmd_Buffer(phost,BITMAP_SOURCE(0));
  App_WrCoCmd_Buffer(phost,BITMAP_LAYOUT(L8,DispWidth-40,DispHeight-20));
#ifdef 81X_ENABLE
  App_WrCoCmd_Buffer(phost,BITMAP_LAYOUT_H((DispWidth-40)>>10,(DispHeight-20)>>9));
#endif
  App_WrCoCmd_Buffer(phost,BITMAP_SIZE(NEAREST,BORDER,BORDER,(DispWidth-40),(DispHeight-20)));
#ifdef 81X_ENABLE
  App_WrCoCmd_Buffer(phost,BITMAP_SIZE_H((DispWidth-40)>>9,(DispHeight-20)>>9));
#endif
  Gpu_CoCmd_Swap(phost);
  App_Flush_Co_Buffer(phost);
  Gpu_Hal_WaitCmdfifo_empty(phost);				
  while(1)
  {
    // Check the tracker
    tracker = Gpu_Hal_Rd32(phost,REG_TRACKER);	
        // Check the Tag 
    tag = Gpu_Hal_Rd8(phost,REG_TOUCH_TAG);
    //  clear the GRAM when user enter the Clear button
    if(tag==2)
    {
  	Gpu_CoCmd_Dlstart(phost);  
 	Gpu_CoCmd_MemZero(phost,0,(DispWidth-40)*(DispHeight-20L)); // Clear the gram frm 1024 		
	App_Flush_Co_Buffer(phost);
        Gpu_Hal_WaitCmdfifo_empty(phost);	
    }
    // compute the color from the tracker
    if((tracker&0xff)==1)      // check the tag val
    {
      val = (tracker>>16);		
    }
    color = val*255;
    // Start the new display list
    Gpu_CoCmd_Dlstart(phost);                  // Start the display list
    App_WrCoCmd_Buffer(phost,CLEAR(1,1,1));	  // clear the display     
    App_WrCoCmd_Buffer(phost,COLOR_RGB(255,255,255));  // color	
    Gpu_CoCmd_BgColor(phost,color);   
    App_WrCoCmd_Buffer(phost,TAG_MASK(1));
    App_WrCoCmd_Buffer(phost,TAG(1));          // assign the tag value 
    Gpu_CoCmd_FgColor(phost,color);
   // draw the sliders 
    Gpu_CoCmd_Slider(phost,(DispWidth-30),40,8,(DispHeight-100),0,val,65535);	 // slide j1 cmd  
    Gpu_CoCmd_FgColor(phost,(tag==2)?0x0000ff:color);
    App_WrCoCmd_Buffer(phost,TAG(2));          // assign the tag value 
    Gpu_CoCmd_Button(phost,(DispWidth-35),(DispHeight-45),35,25,26,0,"CLR");
    App_WrCoCmd_Buffer(phost,TAG_MASK(0));
    
    Gpu_CoCmd_Text(phost,DispWidth-35,10,26,0,"Color");
    
    App_WrCoCmd_Buffer(phost,LINE_WIDTH(1*16));
    App_WrCoCmd_Buffer(phost,BEGIN(RECTS));
    App_WrCoCmd_Buffer(phost,VERTEX2F(0,10*16));
    App_WrCoCmd_Buffer(phost,VERTEX2F((int16_t)(DispWidth-40)*16,(int16_t)(DispHeight-20)*16));			
    
     
    App_WrCoCmd_Buffer(phost,COLOR_RGB((color>>16)&0xff,(color>>8)&0xff,(color)&0xff));
    App_WrCoCmd_Buffer(phost,BEGIN(BITMAPS));
    App_WrCoCmd_Buffer(phost,VERTEX2II(0,10,1,0));
    App_WrCoCmd_Buffer(phost,END());
    App_WrCoCmd_Buffer(phost,DISPLAY());
    Gpu_CoCmd_Swap(phost);
    App_Flush_Co_Buffer(phost);
    Gpu_Hal_WaitCmdfifo_empty(phost);	
  }
}


/*#if defined MSVC_PLATFORM*//* | defined FT900_PLATFORM*//*
// Main entry point 
int32_t main(int32_t argc,char8_t *argv[])
#endif*/
//#if defined(ARDUINO_PLATFORM)/*||defined(MSVC_FT800EMU)*/
void setup()
//#endif
{

	 uint8_t chipid;
/*#ifdef FT900_PLATFORM
	FT900_Config();
#endif*/
	Gpu_HalInit_t halinit;
	
	halinit.TotalChannelNum = 1;

              
	//Gpu_Hal_Init(&halinit); Only returns true for arduino
	host.hal_config.channel_no = 0;
	host.hal_config.pdn_pin_no = FT800_PD_N;
	host.hal_config.spi_cs_pin_no = FT800_SEL_PIN;
/*#ifdef MSVC_PLATFORM_SPI
  host.hal_config.spi_clockrate_khz = 12000; //in KHz
#endif*/
#ifdef ARDUINO_PLATFORM_SPI
  host.hal_config.spi_clockrate_khz = 4000; //in KHz//30000kHz is max for FT813
#endif
  Gpu_Hal_Open(&host);//pulls PD pin high, Pulls CS pin High, Starts SPI (badly), SPI Settings

  phost = &host;

    BootupConfig();//initialize screen, cycle PD pin, send wake, send configs

/*#if (*//*(defined FT900_PLATFORM) || *//*defined(MSVC_PLATFORM))
	printf("\n reg_touch_rz =0x%x ", Gpu_Hal_Rd16(phost, REG_TOUCH_RZ));
	printf("\n reg_touch_rzthresh =0x%x ", Gpu_Hal_Rd32(phost, REG_TOUCH_RZTHRESH));
  printf("\n reg_touch_tag_xy=0x%x",Gpu_Hal_Rd32(phost, REG_TOUCH_TAG_XY));
	printf("\n reg_touch_tag=0x%x",Gpu_Hal_Rd32(phost, REG_TOUCH_TAG));
#endif*/

    /*It is optional to clear the screen here*/	
    Gpu_Hal_WrMem(phost, RAM_DL,(uint8_t *)DLCODE_BOOTUP,sizeof(DLCODE_BOOTUP));
    Gpu_Hal_Wr8(phost, REG_DLSWAP,DLSWAP_FRAME);
    
    /*Gpu_Hal_Sleep*/ delay(1000);//Show the booting up screen.


  home_setup();
  Info();
	Sketch();   
	/* Close all the opened handles */
    Gpu_Hal_Close(phost);
    Gpu_Hal_DeInit();
/*#ifdef MSVC_PLATFORM
	return 0;
#endif*/
}

void loop()
{
}



/* Nothing beyond this */













