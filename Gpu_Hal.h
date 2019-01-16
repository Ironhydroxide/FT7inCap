/*

Copyright (c) Future Technology Devices International 2014

THIS SOFTWARE IS PROVIDED BY FUTURE TECHNOLOGY DEVICES INTERNATIONAL LIMITED "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
FUTURE TECHNOLOGY DEVICES INTERNATIONAL LIMITED BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES LOSS OF USE, DATA, OR PROFITS OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

FTDI DRIVERS MAY BE USED ONLY IN CONJUNCTION WITH PRODUCTS BASED ON FTDI PARTS.

FTDI DRIVERS MAY BE DISTRIBUTED IN ANY FORM AS LONG AS LICENSE INFORMATION IS NOT MODIFIED.

IF A CUSTOM VENDOR ID AND/OR PRODUCT ID OR DESCRIPTION STRING ARE USED, IT IS THE
RESPONSIBILITY OF THE PRODUCT MANUFACTURER TO MAINTAIN ANY CHANGES AND SUBSEQUENT WHQL
RE-CERTIFICATION AS A RESULT OF MAKING THESE CHANGES.

Author : FTDI 

Revision History: 
0.1 - date 2013.04.24 - Initial Version
0.2 - date 2013.08.19 - few inclusions
1.0 - date 2015.03.16 - Update with new APIs for FT81X

File Description: 
    This file defines the generic APIs of host access layer for the FT800 or EVE compatible silicon. 
    Application shall access FT800 or EVE resources over these APIs,regardless of I2C or SPI protocol.   
    I2C and SPI is selected by compiler switch "I2C_MODE"  and "SPI_MODE". In addition, there are 
    some helper functions defined for FT800 coprocessor engine as well as host commands.
 
 */
#ifndef GPU_HAL_H
#define GPU_HAL_H



typedef enum {
	GPU_I2C_MODE = 0,
	GPU_SPI_MODE,

	GPU_MODE_COUNT,
	GPU_MODE_UNKNOWN = GPU_MODE_COUNT
}GPU_HAL_MODE_E;

typedef enum {
	GPU_HAL_OPENED,
	GPU_HAL_READING,
	GPU_HAL_WRITING,
	GPU_HAL_CLOSED,

	GPU_HAL_STATUS_COUNT,
	GPU_HAL_STATUS_ERROR = GPU_HAL_STATUS_COUNT
}GPU_HAL_STATUS_E;

typedef struct {
	union {
		uint8_t spi_cs_pin_no;		//spi chip select number of ft8xx chip
		uint8_t i2c_addr;			//i2c address of ft8xx chip
	};
	union {
		uint16_t spi_clockrate_khz;  //In KHz
		uint16_t i2c_clockrate_khz;  //In KHz
	};
	uint8_t channel_no;				//mpsse channel number
	uint8_t pdn_pin_no;				//ft8xx power down pin number
}Gpu_Hal_Config_t;

typedef struct {
	uint8_t reserved;
}Gpu_App_Context_t;

typedef struct {
	/* Total number channels for libmpsse */
	uint32_t TotalChannelNum;
}Gpu_HalInit_t;

typedef enum {
	GPU_READ = 0,
	GPU_WRITE,
}GPU_TRANSFERDIR_T;


typedef struct {	
	uint32_t length; //IN and OUT
	uint32_t address;
	uint8_t  *buffer;
}Gpu_App_Transfer_t;

typedef struct {
	Gpu_App_Context_t    app_header;
	Gpu_Hal_Config_t     hal_config;

    uint16_t 			cmd_fifo_wp; //coprocessor fifo write pointer
    uint16_t 			dl_buff_wp;  //display command memory write pointer

	GPU_HAL_STATUS_E 	status;        //OUT
	void*          		hal_handle;        //IN/OUT
	/* Additions specific to ft81x */
	uint8_t				spichannel;			//variable to contain single/dual/quad channels
	uint8_t				spinumdummy;		//number of dummy bytes as 1 or 2 for spi read
}Gpu_Hal_Context_t;

/* FIFO buffer management */
typedef struct Fifo_t{
	uint32_t 		fifo_buff; 	//fifo buffer address
	int32_t 		fifo_len; 	//fifo length
	int32_t 		fifo_wp; 	//fifo write pointer - maintained by host
	int32_t 		fifo_rp;  	//fifo read point - maintained by device

	/* FT800 series specific registers */
	uint32_t 		HW_Read_Reg;	//hardware fifo read register
	uint32_t 		HW_Write_Reg;	//hardware fifo write register
}Fifo_t;

/*The basic APIs Level 1*/
bool              	 Gpu_Hal_Init(Gpu_HalInit_t *halinit);
bool              	 Gpu_Hal_Open(Gpu_Hal_Context_t *host);

/*The APIs for reading/writing transfer continuously only with small buffer system*/
void				Gpu_Hal_StartTransfer(Gpu_Hal_Context_t *host,GPU_TRANSFERDIR_T rw,uint32_t addr);
uint8_t             Gpu_Hal_Transfer8(Gpu_Hal_Context_t *host,uint8_t value);
uint16_t            Gpu_Hal_Transfer16(Gpu_Hal_Context_t *host,uint16_t value);
uint32_t            Gpu_Hal_Transfer32(Gpu_Hal_Context_t *host,uint32_t value);
void               	Gpu_Hal_EndTransfer(Gpu_Hal_Context_t *host);

/*Close and deinit apis*/
void              	Gpu_Hal_Close(Gpu_Hal_Context_t *host);
void              	Gpu_Hal_DeInit();

/*Helper function APIs Read*/
uint8_t  			Gpu_Hal_Rd8(Gpu_Hal_Context_t *host,uint32_t addr);
uint16_t 			Gpu_Hal_Rd16(Gpu_Hal_Context_t *host,uint32_t addr);
uint32_t 			Gpu_Hal_Rd32(Gpu_Hal_Context_t *host,uint32_t addr);

/*Helper function APIs Write*/
void 				Gpu_Hal_Wr8(Gpu_Hal_Context_t *host,uint32_t addr, uint8_t v);
void 				Gpu_Hal_Wr16(Gpu_Hal_Context_t *host,uint32_t addr, uint16_t v);
void 				Gpu_Hal_Wr32(Gpu_Hal_Context_t *host,uint32_t addr, uint32_t v);

/*******************************************************************************/
/*******************************************************************************/
/*APIs for coprocessor Fifo read/write and space management*/
void Gpu_Hal_Updatecmdfifo(Gpu_Hal_Context_t *host,uint32_t count);
void Gpu_Hal_WrCmd32(Gpu_Hal_Context_t *host,uint32_t cmd);
void Gpu_Hal_WrCmdBuf(Gpu_Hal_Context_t *host,uint8_t *buffer,uint32_t count);
void Gpu_Hal_WaitCmdfifo_empty(Gpu_Hal_Context_t *host);
void Gpu_Hal_ResetCmdFifo(Gpu_Hal_Context_t *host);
void Gpu_Hal_CheckCmdBuffer(Gpu_Hal_Context_t *host,uint32_t count);

void Gpu_Hal_ResetDLBuffer(Gpu_Hal_Context_t *host);

void Gpu_Hal_StartCmdTransfer(Gpu_Hal_Context_t *host,GPU_TRANSFERDIR_T rw, uint16_t count);

void Gpu_Hal_Powercycle(Gpu_Hal_Context_t *host,bool up);


/*******************************************************************************/
/* APIs related to fifo buffer management */
/* API to update the read pointer into the structure */
void Fifo_Init(Fifo_t *pFifo,uint32_t StartAddress,uint32_t Length,uint32_t HWReadRegAddress,uint32_t HWWriteRegAddress);//Init all the parameters of fifo buffer
void Fifo_Update(Gpu_Hal_Context_t *host,Fifo_t *pFifo);//update both the read and write pointers
uint32_t Fifo_Write(Gpu_Hal_Context_t *host,Fifo_t *pFifo,uint8_t *buffer,uint32_t NumbytetoWrite);//just write and update the write register
void Fifo_Write32(Gpu_Hal_Context_t *host,Fifo_t *pFifo,uint32_t WriteWord);//just write one word and update the write register
void Fifo_WriteWait(Gpu_Hal_Context_t *host,Fifo_t *pFifo,uint8_t *buffer,uint32_t Numbyte);//write and wait for the fifo to be empty
uint32_t Fifo_GetFreeSpace(Gpu_Hal_Context_t *host,Fifo_t *pFifo);//get the free space in the fifo - make sure the return value is maximum of (LENGTH - 4)


/*******************************************************************************/
/*******************************************************************************/
/*APIs for Host Commands*/
typedef enum {
	GPU_INTERNAL_OSC = 0x48, //default
	GPU_EXTERNAL_OSC = 0x44,
}GPU_PLL_SOURCE_T;


typedef enum {
	GPU_PLL_48M = 0x62,  //default
	GPU_PLL_36M = 0x61,
	GPU_PLL_24M = 0x64,
}GPU_PLL_FREQ_T;

typedef enum {
	GPU_ACTIVE_M =  0x00,  
	GPU_STANDBY_M = 0x41,//default
	GPU_SLEEP_M =   0x42,
	GPU_POWERDOWN_M = 0x50,
}GPU_POWER_MODE_T;


#ifdef 81X_ENABLE
	typedef enum {
		GPU_SYSCLK_DEFAULT = 0x00,  //default 60mhz
		GPU_SYSCLK_72M = 0x06, 
		GPU_SYSCLK_60M = 0x05,  
		GPU_SYSCLK_48M = 0x04,  
		GPU_SYSCLK_36M = 0x03,
		GPU_SYSCLK_24M = 0x02,
	}GPU_81X_PLL_FREQ_T;

	typedef enum{
		GPU_MAIN_ROM = 0x80,  //main graphicas ROM used 
		GPU_RCOSATAN_ROM = 0x40,  //line slope table used for 
		GPU_SAMPLE_ROM = 0x20,  //JA samples
		GPU_JABOOT_ROM = 0x10, //JA microcode
		GPU_J1BOOT_ROM = 0x08, //J1 microcode
		GPU_ADC = 0x01,  //
		GPU_POWER_ON_ROM_AND_ADC = 0x00,  //specify this element to power on all ROMs and ADCs
	}GPU_81X_ROM_AND_ADC_T;

	typedef enum{
		GPU_5MA = 0x00,  //default current
		GPU_10MA = 0x01,
		GPU_15MA = 0x02,
		GPU_20MA = 0x03,
	}GPU_81X_GPIO_DRIVE_STRENGTH_T;

	typedef enum{
		GPU_GPIO0 = 0x00,
		GPU_GPIO1 = 0x04,
		GPU_GPIO2 = 0x08,
		GPU_GPIO3 = 0x0C,
		GPU_GPIO4 = 0x10,
		GPU_DISP = 0x20,
		GPU_DE = 0x24,
		GPU_VSYNC_HSYNC = 0x28,
		GPU_PCLK = 0x2C,
		GPU_BACKLIGHT = 0x30,
		GPU_R_G_B = 0x34,
		GPU_AUDIO_L = 0x38,
		GPU_INT_N = 0x3C,
		GPU_TOUCHWAKE = 0x40,
		GPU_SCL = 0x44,
		GPU_SDA = 0x48,
		GPU_SPI_MISO_MOSI_IO2_IO3 = 0x4C,
	}GPU_81X_GPIO_GROUP_T;

	#define GPU_81X_RESET_ACTIVE 0x000268

	#define GPU_81X_RESET_REMOVAL 0x002068
#endif

#define GPU_CORE_RESET  (0x68)

/* Enums for number of SPI dummy bytes and number of channels */
typedef enum {
	GPU_SPI_SINGLE_CHANNEL = 0,
	GPU_SPI_DUAL_CHANNEL = 1,
	GPU_SPI_QUAD_CHANNEL = 2,
}GPU_SPI_NUMCHANNELS_T;
typedef enum {
	GPU_SPI_ONEDUMMY = 1,
	GPU_SPI_TWODUMMY = 2,
}GPU_SPI_NUMDUMMYBYTES;

#define SPI_ONE_DUMMY_BYTE	(0x00)
#define SPI_TWO_DUMMY_BYTE	(0x04)
#define SPI_SINGLE_CHANNEL	(0x00)
#define SPI_DUAL_CHANNEL	(0x01)
#define SPI_QUAD_CHANNEL	(0x02)

int32_t hal_strlen(const char8_t *s);
//void Gpu_Hal_Sleep(uint32_t ms);
void Gpu_ClockSelect(Gpu_Hal_Context_t *host,GPU_PLL_SOURCE_T pllsource);
void Gpu_PLL_FreqSelect(Gpu_Hal_Context_t *host,GPU_PLL_FREQ_T freq);
void Gpu_PowerModeSwitch(Gpu_Hal_Context_t *host,GPU_POWER_MODE_T pwrmode);
void Gpu_CoreReset(Gpu_Hal_Context_t *host);
void Gpu_Hal_StartTransfer(Gpu_Hal_Context_t *host,GPU_TRANSFERDIR_T rw,uint32_t addr);
void Gpu_Hal_WrMem(Gpu_Hal_Context_t *host,uint32_t addr, const uint8_t *buffer, uint32_t length);
void Gpu_Hal_WrMemFromFlash(Gpu_Hal_Context_t *host,uint32_t addr,const prog_uchar8_t *buffer, uint32_t length);
void Gpu_Hal_WrCmdBufFromFlash(Gpu_Hal_Context_t *host,PROGMEM prog_uchar8_t *buffer,uint32_t count);
void Gpu_Hal_RdMem(Gpu_Hal_Context_t *host,uint32_t addr, uint8_t *buffer, uint32_t length);
void Gpu_Hal_WaitLogo_Finish(Gpu_Hal_Context_t *host);
uint8_t Gpu_Hal_TransferString(Gpu_Hal_Context_t *host,const char8_t *string);
void Gpu_HostCommand(Gpu_Hal_Context_t *host,uint8_t cmd);
void Gpu_HostCommand_Ext3(Gpu_Hal_Context_t *host,uint32_t cmd);
int32_t Gpu_Hal_Dec2Ascii(char8_t *pSrc,int32_t value);
uint8_t Gpu_Hal_WaitCmdfifo_empty_status(Gpu_Hal_Context_t *host);
void Gpu_Hal_WrCmdBuf_nowait(Gpu_Hal_Context_t *host,uint8_t *buffer,uint32_t count);
uint16_t Gpu_Cmdfifo_Freespace(Gpu_Hal_Context_t *host);
int16_t Gpu_Hal_SetSPI(Gpu_Hal_Context_t *host,GPU_SPI_NUMCHANNELS_T numchnls,GPU_SPI_NUMDUMMYBYTES numdummy);

#ifdef 81X_ENABLE
void Gpu_81X_SelectSysCLK(Gpu_Hal_Context_t *host, GPU_81X_PLL_FREQ_T freq);
void GPU_81X_PowerOffComponents(Gpu_Hal_Context_t *host, uint8_t val);
void GPU_81X_PadDriveStrength(Gpu_Hal_Context_t *host, GPU_81X_GPIO_DRIVE_STRENGTH_T strength, GPU_81X_GPIO_GROUP_T group);
void Gpu_81X_ResetActive(Gpu_Hal_Context_t *host);
void Gpu_81X_ResetRemoval(Gpu_Hal_Context_t *host);
#endif

uint32_t Gpu_CurrentFrequency(Gpu_Hal_Context_t *host);
int32_t Gpu_ClockTrimming(Gpu_Hal_Context_t *host,int32_t LowFreq);

#endif  /* GPU_HAL_H */
