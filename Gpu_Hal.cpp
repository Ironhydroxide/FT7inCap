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
0.2 - date 2013.08.19 - added few APIs

*/

#include "Platform.h"

/* API to initialize the SPI interface */
bool  Gpu_Hal_Init(Gpu_HalInit_t *halinit)
{
/*#ifdef FT900_PLATFORM
	
	
	// Initialize SPIM HW
    sys_enable(sys_device_spi_master);

    gpio_function(27, pad_spim_sck); // GPIO27 to SPIM_CLK 
    gpio_function(28, pad_spim_ss0); // GPIO28 as CS 
    gpio_function(29, pad_spim_mosi); // GPIO29 to SPIM_MOSI 
    gpio_function(30, pad_spim_miso); // GPIO30 to SPIM_MISO 

    gpio_dir(27, pad_dir_output);
    gpio_dir(28, pad_dir_output);
    gpio_dir(29, pad_dir_output);
    gpio_dir(30, pad_dir_input);
#if (defined(ENABLE_SPI_QUAD))
    // Initialize IO2 and IO3 pad/pin for quad settings 
    gpio_function(31, pad_spim_io2); // GPIO31 to IO2 
    gpio_function(32, pad_spim_io3); // GPIO32 to IO3 
    gpio_dir(31, pad_dir_output);
    gpio_dir(32, pad_dir_output);
#endif
    gpio_write(28, 1);

	spi_init(SPIM, spi_dir_master, spi_mode_0, 16);//SPISysInit(SPIM);



#endif*/



/*#ifdef MSVC_PLATFORM_SPI
	// Initialize the libmpsse 
    Init_libMPSSE();
	SPI_GetNumChannels(&halinit->TotalChannelNum);
	// By default i am assuming only one mpsse cable is connected to PC and channel 0 of that mpsse cable is used for spi transactions 
	if(halinit->TotalChannelNum > 0)
	{
        DEVICE_LIST_INFO_NODE devList;
		SPI_GetChannelInfo(0,&devList);
		printf("Information on channel number %d:\n",0);
		// print the dev info 
		printf(" Flags=0x%x\n",devList.Flags);
		printf(" Type=0x%x\n",devList.Type);
		printf(" ID=0x%x\n",devList.ID);
		printf(" LocId=0x%x\n",devList.LocId);
		printf(" SerialNumber=%s\n",devList.SerialNumber);
		printf(" Description=%s\n",devList.Description);
		printf(" ftHandle=0x%x\n",devList.ftHandle);//is 0 unless open
	}
#endif*/
	return TRUE;
}
bool    Gpu_Hal_Open(Gpu_Hal_Context_t *host)
{
/*#ifdef FT900_PLATFORM

	gpio_function(host->hal_config.spi_cs_pin_no, pad_spim_ss0); /// GPIO28 as CS
	gpio_write(host->hal_config.spi_cs_pin_no, 1);

	gpio_function(host->hal_config.pdn_pin_no, pad_gpio43);
	gpio_dir(host->hal_config.pdn_pin_no, pad_dir_output);

    gpio_write(host->hal_config.pdn_pin_no,1);


#endif*/
/*#ifdef MSVC_FT800EMU
	GpuEmu_SPII2C_begin();
#endif*/
#ifdef ARDUINO_PLATFORM_SPI
	pinMode(host->hal_config.pdn_pin_no, OUTPUT);
	digitalWrite(host->hal_config.pdn_pin_no, HIGH);
    pinMode(host->hal_config.spi_cs_pin_no, OUTPUT);
    digitalWrite(host->hal_config.spi_cs_pin_no, HIGH);
	SPI.begin();
	SPI.setClockDivider(SPI_CLOCK_DIV2);
	SPI.setBitOrder(MSBFIRST);
	SPI.setDataMode(SPI_MODE0);
#endif
/*#ifdef MSVC_PLATFORM_SPI
    ChannelConfig channelConf;			//channel configuration
	STATUS status;					
		
	// configure the spi settings
	channelConf.ClockRate = host->hal_config.spi_clockrate_khz * 1000; 
	channelConf.LatencyTimer= 2;       
	channelConf.configOptions = SPI_CONFIG_OPTION_MODE0 | SPI_CONFIG_OPTION_CS_DBUS3 | SPI_CONFIG_OPTION_CS_ACTIVELOW;
	channelConf.Pin = 0x00000000;	//FinalVal-FinalDir-InitVal-InitDir (for dir 0=in, 1=out)

	// Open the first available channel 
	SPI_OpenChannel(host->hal_config.channel_no,(HANDLE *)&host->hal_handle);
	status = SPI_InitChannel((HANDLE)host->hal_handle,&channelConf);
	printf("\nhandle=0x%x status=0x%x\n",host->hal_handle,status);	
#endif*/

	/* Initialize the context valriables */
	host->cmd_fifo_wp = host->dl_buff_wp = 0;
	host->spinumdummy = 1;//by default ft800/801/810/811 goes with single dummy byte for read
	host->spichannel = 0;
	host->status = GPU_HAL_OPENED;

	return TRUE;
}
void  Gpu_Hal_Close(Gpu_Hal_Context_t *host)
{
	host->status = GPU_HAL_CLOSED;
/*#ifdef MSVC_PLATFORM_SPI	
	// Close the channel
	SPI_CloseChannel(host->hal_handle);
#endif*/
#ifdef ARDUINO_PLATFORM_SPI
        SPI.end();
#endif
/*#ifdef MSVC_FT800EMU
	GpuEmu_SPII2C_end();
#endif*/

/*#ifdef FT900_PLATFORM
	//spi_close(SPIM,0);
#endif*/
}

void Gpu_Hal_DeInit()
{
/*#ifdef MSVC_PLATFORM_SPI
   //Cleanup the MPSSE Lib
   Cleanup_libMPSSE();
#endif*/
/*#ifdef FT900_PLATFORM
   spi_uninit(SPIM);
#endif*/
}

/*The APIs for reading/writing transfer continuously only with small buffer system*/
void  Gpu_Hal_StartTransfer(Gpu_Hal_Context_t *host,GPU_TRANSFERDIR_T rw,uint32_t addr)
{
	if (GPU_READ == rw){

/*#ifdef FT900_PLATFORM
		uint8_t spidata[4];
		spidata[0] = (addr >> 16);
		spidata[1] = (addr >> 8);
		spidata[2] =  addr &0xff;
		spi_open(SPIM, host->hal_config.spi_cs_pin_no);


		spi_writen(SPIM,spidata,3);
#endif*/

/*#ifdef MSVC_PLATFORM_SPI
		uint8_t Transfer_Array[4];
		uint32_t SizeTransfered;

		// Compose the read packet 
		Transfer_Array[0] = addr >> 16;
		Transfer_Array[1] = addr >> 8;
		Transfer_Array[2] = addr;

		Transfer_Array[3] = 0; //Dummy Read byte
		SPI_Write((HANDLE)host->hal_handle,Transfer_Array,sizeof(Transfer_Array),&SizeTransfered,SPI_TRANSFER_OPTIONS_SIZE_IN_BYTES | SPI_TRANSFER_OPTIONS_CHIPSELECT_ENABLE);
#endif*/
#ifdef ARDUINO_PLATFORM_SPI
		digitalWrite(host->hal_config.spi_cs_pin_no, LOW);
		SPI.transfer(addr >> 16);
		SPI.transfer(highByte(addr));
		SPI.transfer(lowByte(addr));

		SPI.transfer(0); //Dummy Read Byte
#endif

/*#ifdef MSVC_FT800EMU
		GpuEmu_SPII2C_StartRead(addr);
#endif*/
		host->status = GPU_HAL_READING;
	}else{
	
/*#ifdef FT900_PLATFORM
		uint8_t spidata[4];
		spidata[0] = (0x80|(addr >> 16));
		spidata[1] = (addr >> 8);
		spidata[2] = addr;


		spi_open(SPIM, host->hal_config.spi_cs_pin_no);
		spi_writen(SPIM,spidata,3);

#endif*/
	
/*#ifdef MSVC_PLATFORM_SPI
		uint8_t Transfer_Array[3];
		uint32_t SizeTransfered;

		// Compose the read packet 
		Transfer_Array[0] = (0x80 | (addr >> 16));
		Transfer_Array[1] = addr >> 8;
		Transfer_Array[2] = addr;
		SPI_Write((HANDLE)host->hal_handle,Transfer_Array,3,&SizeTransfered,SPI_TRANSFER_OPTIONS_SIZE_IN_BYTES | SPI_TRANSFER_OPTIONS_CHIPSELECT_ENABLE);		
#endif*/
#ifdef ARDUINO_PLATFORM_SPI
		digitalWrite(host->hal_config.spi_cs_pin_no, LOW);
		SPI.transfer(0x80 | (addr >> 16));
		SPI.transfer(highByte(addr));
		SPI.transfer(lowByte(addr));
#endif
/*#ifdef MSVC_FT800EMU
		GpuEmu_SPII2C_StartWrite(addr);
#endif*/
		host->status = GPU_HAL_WRITING;
	}
}



/*The APIs for writing transfer continuously only*/
void  Gpu_Hal_StartCmdTransfer(Gpu_Hal_Context_t *host,GPU_TRANSFERDIR_T rw, uint16_t count)
{
	Gpu_Hal_StartTransfer(host,rw,host->cmd_fifo_wp + RAM_CMD);
}

uint8_t    Gpu_Hal_TransferString(Gpu_Hal_Context_t *host,const char8_t *string)
{
    uint16_t length = strlen(string);
    while(length --){
       Gpu_Hal_Transfer8(host,*string);
       string ++;
    }
    //Append one null as ending flag
    Gpu_Hal_Transfer8(host,0);
}


uint8_t    Gpu_Hal_Transfer8(Gpu_Hal_Context_t *host,uint8_t value)
{

/*#ifdef FT900_PLATFORM
	uint8_t ReadByte;

	if (host->status == GPU_HAL_WRITING)
	{
		spi_write(SPIM,value);
	}
	else
	{
		spi_read(SPIM,ReadByte);
	}
	return ReadByte;

#endif*/
#ifdef ARDUINO_PLATFORM_SPI
        return SPI.transfer(value);
#endif
/*#ifdef MSVC_PLATFORM_SPI
	uint32_t SizeTransfered;
	if (host->status == GPU_HAL_WRITING){
		SPI_Write(host->hal_handle,&value,sizeof(value),&SizeTransfered,SPI_TRANSFER_OPTIONS_SIZE_IN_BYTES);
	}else{
		SPI_Read(host->hal_handle,&value,sizeof(value),&SizeTransfered,SPI_TRANSFER_OPTIONS_SIZE_IN_BYTES);
	}

	if (SizeTransfered != sizeof(value))
		host->status = GPU_HAL_STATUS_ERROR;
        return value;
#endif*/

/*#ifdef MSVC_FT800EMU
	return GpuEmu_SPII2C_transfer(value);
#endif*/
}


uint16_t  Gpu_Hal_Transfer16(Gpu_Hal_Context_t *host,uint16_t value)
{
	uint16_t retVal = 0;

        if (host->status == GPU_HAL_WRITING){
		Gpu_Hal_Transfer8(host,value & 0xFF);//LSB first
		Gpu_Hal_Transfer8(host,(value >> 8) & 0xFF);
	}else{
		retVal = Gpu_Hal_Transfer8(host,0);
		retVal |= (uint16_t)Gpu_Hal_Transfer8(host,0) << 8;
	}

	return retVal;
}
uint32_t  Gpu_Hal_Transfer32(Gpu_Hal_Context_t *host,uint32_t value)
{
	uint32_t retVal = 0;
	if (host->status == GPU_HAL_WRITING){
		Gpu_Hal_Transfer16(host,value & 0xFFFF);//LSB first
		Gpu_Hal_Transfer16(host,(value >> 16) & 0xFFFF);
	}else{
		retVal = Gpu_Hal_Transfer16(host,0);
		retVal |= (uint32_t)Gpu_Hal_Transfer16(host,0) << 16;
	}
	return retVal;
}

void   Gpu_Hal_EndTransfer(Gpu_Hal_Context_t *host)
{

/*#ifdef FT900_PLATFORM
	spi_close(SPIM, host->hal_config.spi_cs_pin_no);
#endif*/
/*#ifdef MSVC_PLATFORM_SPI  
	//just disbale the CS - send 0 bytes with CS disable
	SPI_ToggleCS((HANDLE)host->hal_handle,FALSE);
#endif*/
#ifdef ARDUINO_PLATFORM_SPI
	digitalWrite(host->hal_config.spi_cs_pin_no, HIGH);
#endif

/*#ifdef MSVC_FT800EMU
	GpuEmu_SPII2C_csHigh();
#endif*/
	host->status = GPU_HAL_OPENED;
}


uint8_t  Gpu_Hal_Rd8(Gpu_Hal_Context_t *host,uint32_t addr)
{
	uint8_t value;
/*#ifdef FT900_PLATFORM
	uint8_t spiData[4] = {0};
	Gpu_Hal_StartTransfer(host,GPU_READ,addr);
	spi_readn(SPIM,spiData,host->spinumdummy + 1);
	value = spiData[host->spinumdummy];
#else*/
	Gpu_Hal_StartTransfer(host,GPU_READ,addr);//drops CS pin sends address on SPI, sends empty bit
	value = Gpu_Hal_Transfer8(host,0);//returns SPI buss data
/*#endif*/
	Gpu_Hal_EndTransfer(host);//pulls CS Pin high
	return value;
}
uint16_t Gpu_Hal_Rd16(Gpu_Hal_Context_t *host,uint32_t addr)
{
	uint16_t value;
/*#ifdef FT900_PLATFORM
	uint8_t spiData[4] = {0};
	Gpu_Hal_StartTransfer(host,GPU_READ,addr);
	spi_readn(SPIM,spiData,host->spinumdummy + 2);
	value = spiData[host->spinumdummy] |(spiData[host->spinumdummy+1] << 8) ;
#else*/
	Gpu_Hal_StartTransfer(host,GPU_READ,addr);
	value = Gpu_Hal_Transfer16(host,0);
/*#endif*/
	Gpu_Hal_EndTransfer(host);
	return value;
}
uint32_t Gpu_Hal_Rd32(Gpu_Hal_Context_t *host,uint32_t addr)
{
	uint32_t value;
/*#ifdef FT900_PLATFORM
	uint8_t spiData[8] = {0};
	Gpu_Hal_StartTransfer(host,GPU_READ,addr);
	spi_readn(SPIM,spiData,host->spinumdummy + 4);
	value = (spiData[host->spinumdummy+3] << 24) | (spiData[host->spinumdummy+2] << 16) | (spiData[host->spinumdummy+1] << 8) | spiData[host->spinumdummy];
#else*/
	Gpu_Hal_StartTransfer(host,GPU_READ,addr);
	value = Gpu_Hal_Transfer32(host,0);
/*#endif*/
	Gpu_Hal_EndTransfer(host);
	return value;
}

void Gpu_Hal_Wr8(Gpu_Hal_Context_t *host,uint32_t addr, uint8_t v)
{	
	Gpu_Hal_StartTransfer(host,GPU_WRITE,addr);
	Gpu_Hal_Transfer8(host,v);
	Gpu_Hal_EndTransfer(host);
}
void Gpu_Hal_Wr16(Gpu_Hal_Context_t *host,uint32_t addr, uint16_t v)
{
	Gpu_Hal_StartTransfer(host,GPU_WRITE,addr);
	Gpu_Hal_Transfer16(host,v);
	Gpu_Hal_EndTransfer(host);
}
void Gpu_Hal_Wr32(Gpu_Hal_Context_t *host,uint32_t addr, uint32_t v)
{
	Gpu_Hal_StartTransfer(host,GPU_WRITE,addr);
	Gpu_Hal_Transfer32(host,v);
	Gpu_Hal_EndTransfer(host);
}

void Gpu_HostCommand(Gpu_Hal_Context_t *host,uint8_t cmd)
{

/*#ifdef FT900_PLATFORM
	uint8_t hcmd[4] = {0};
	hcmd[0] = cmd;
	hcmd[1] = 0;
	hcmd[2] = 0;
	hcmd[3] = 0;

	spi_open(SPIM,host->hal_config.spi_cs_pin_no);
	spi_writen(SPIM,hcmd,3);
	spi_close(SPIM,host->hal_config.spi_cs_pin_no);

#endif*/
/*#ifdef MSVC_PLATFORM_SPI
  uint8_t Transfer_Array[3];
  uint32_t SizeTransfered;

  Transfer_Array[0] = cmd;
  Transfer_Array[1] = 0;
  Transfer_Array[2] = 0;

  SPI_Write(host->hal_handle,Transfer_Array,sizeof(Transfer_Array),&SizeTransfered,SPI_TRANSFER_OPTIONS_SIZE_IN_BYTES | SPI_TRANSFER_OPTIONS_CHIPSELECT_ENABLE | SPI_TRANSFER_OPTIONS_CHIPSELECT_DISABLE);
#endif*/
#ifdef ARDUINO_PLATFORM_SPI
  digitalWrite(host->hal_config.spi_cs_pin_no, LOW);
  SPI.transfer(cmd);
  SPI.transfer(0);
  SPI.transfer(0);
  digitalWrite(host->hal_config.spi_cs_pin_no, HIGH);
#endif
/*#ifdef MSVC_FT800EMU
  //Not implemented in FT800EMU
#endif*/
}

void Gpu_ClockSelect(Gpu_Hal_Context_t *host,GPU_PLL_SOURCE_T pllsource)
{
   Gpu_HostCommand(host,pllsource);
}
void Gpu_PLL_FreqSelect(Gpu_Hal_Context_t *host,GPU_PLL_FREQ_T freq)
{
   Gpu_HostCommand(host,freq);
}
void Gpu_PowerModeSwitch(Gpu_Hal_Context_t *host,GPU_POWER_MODE_T pwrmode)
{
   Gpu_HostCommand(host,pwrmode);
}
void Gpu_CoreReset(Gpu_Hal_Context_t *host)
{
   Gpu_HostCommand(host,GPU_CORE_RESET);
}


#ifdef 81X_ENABLE
//This API can only be called when PLL is stopped(SLEEP mode).  For compatibility, set frequency to the GPU_12MHZ option in the GPU_SETPLLSP1_T table.
void Gpu_81X_SelectSysCLK(Gpu_Hal_Context_t *host, GPU_81X_PLL_FREQ_T freq){
		if(GPU_SYSCLK_72M == freq)
			Gpu_HostCommand_Ext3(host, (uint32_t)0x61 | (0x40 << 8) | (0x06 << 8)); 
		else if(GPU_SYSCLK_60M == freq)
			Gpu_HostCommand_Ext3(host, (uint32_t)0x61 | (0x40 << 8) | (0x05 << 8)); 
		else if(GPU_SYSCLK_48M == freq)
			Gpu_HostCommand_Ext3(host, (uint32_t)0x61 | (0x40 << 8) | (0x04 << 8)); 
		else if(GPU_SYSCLK_36M == freq)
			Gpu_HostCommand_Ext3(host, (uint32_t)0x61 | (0x03 << 8)); 
		else if(GPU_SYSCLK_24M == freq)
			Gpu_HostCommand_Ext3(host, (uint32_t)0x61 | (0x02 << 8)); 
		else if(GPU_SYSCLK_DEFAULT == freq)//default clock
			Gpu_HostCommand_Ext3(host, 0x61); 
}

//Power down or up ROMs and ADCs.  Specified one or more elements in the GPU_81X_ROM_AND_ADC_T table to power down, unspecified elements will be powered up.  The application must retain the state of the ROMs and ADCs as they're not readable from the device.
void GPU_81X_PowerOffComponents(Gpu_Hal_Context_t *host, uint8_t val){
		Gpu_HostCommand_Ext3(host, (uint32_t)0x49 | (val<<8));
}

//this API sets the current strength of supported GPIO/IO group(s)
void GPU_81X_PadDriveStrength(Gpu_Hal_Context_t *host, GPU_81X_GPIO_DRIVE_STRENGTH_T strength, GPU_81X_GPIO_GROUP_T group){
		Gpu_HostCommand_Ext3(host, (uint32_t)0x70 | (group << 8) | (strength << 8));
}

//this API will hold the system reset active, Gpu_81X_ResetRemoval() must be called to release the system reset.
void Gpu_81X_ResetActive(Gpu_Hal_Context_t *host){
	Gpu_HostCommand_Ext3(host, GPU_81X_RESET_ACTIVE); 
}

//This API will release the system reset, and the system will exit reset and behave as after POR, settings done through SPI commands will not be affected.
void Gpu_81X_ResetRemoval(Gpu_Hal_Context_t *host){
	Gpu_HostCommand_Ext3(host, GPU_81X_RESET_REMOVAL); 
}
#endif


//This API sends a 3byte command to the host
void Gpu_HostCommand_Ext3(Gpu_Hal_Context_t *host,uint32_t cmd)
{
	/*#ifdef FT900_PLATFORM
		uint8_t hcmd[4] = {0};
		hcmd[0] = cmd & 0xff;
		hcmd[1] = (cmd>>8) & 0xff;
		hcmd[2] = (cmd>>16) & 0xff;
		hcmd[3] = 0;
	spi_open(SPIM,host->hal_config.spi_cs_pin_no);
	spi_writen(SPIM,hcmd,3);
	spi_close(SPIM,host->hal_config.spi_cs_pin_no);
		
	#endif*/
/*	#ifdef MSVC_PLATFORM_SPI
	  uint8_t Transfer_Array[3];
	  uint32_t SizeTransfered;
	
	  Transfer_Array[0] = cmd;
	  Transfer_Array[1] = (cmd>>8) & 0xff;
	  Transfer_Array[2] = (cmd>>16) & 0xff;
	
	  SPI_Write(host->hal_handle,Transfer_Array,sizeof(Transfer_Array),&SizeTransfered,SPI_TRANSFER_OPTIONS_SIZE_IN_BYTES | SPI_TRANSFER_OPTIONS_CHIPSELECT_ENABLE | SPI_TRANSFER_OPTIONS_CHIPSELECT_DISABLE);
	#endif*/
	#ifdef ARDUINO_PLATFORM_SPI
	  digitalWrite(host->hal_config.spi_cs_pin_no, LOW);
	  SPI.transfer(cmd);
	  SPI.transfer((cmd>>8) & 0xff);
	  SPI.transfer((cmd>>16) & 0xff);
	  digitalWrite(host->hal_config.spi_cs_pin_no, HIGH);
	#endif
	/*#ifdef MSVC_FT800EMU
	  //Not implemented in FT800EMU
	#endif*/
}



void Gpu_Hal_Updatecmdfifo(Gpu_Hal_Context_t *host,uint32_t count)
{
	host->cmd_fifo_wp  = (host->cmd_fifo_wp + count) & 4095;

	//4 byte alignment
	host->cmd_fifo_wp = (host->cmd_fifo_wp + 3) & 0xffc;
	Gpu_Hal_Wr16(host,REG_CMD_WRITE,host->cmd_fifo_wp);
}


uint16_t Gpu_Cmdfifo_Freespace(Gpu_Hal_Context_t *host)
{
	uint16_t fullness,retval;

	//host->cmd_fifo_wp = Gpu_Hal_Rd16(host,REG_CMD_WRITE);

	fullness = (host->cmd_fifo_wp - Gpu_Hal_Rd16(host,REG_CMD_READ)) & 4095;
	retval = (CMD_FIFO_SIZE - 4) - fullness;
	return (retval);
}

void Gpu_Hal_WrCmdBuf(Gpu_Hal_Context_t *host,uint8_t *buffer,uint32_t count)
{
	int32_t length =0, SizeTransfered = 0, availablefreesize;   

#define MAX_CMD_FIFO_TRANSFER   Gpu_Cmdfifo_Freespace(host)  
	do {                
		length = count;
		availablefreesize = MAX_CMD_FIFO_TRANSFER;

		if (length > availablefreesize)
		{
		    length = availablefreesize;
		}
      	        Gpu_Hal_CheckCmdBuffer(host,length);

                Gpu_Hal_StartCmdTransfer(host,GPU_WRITE,length);
/*#ifdef FT900_PLATFORM
        spi_writen(SPIM,buffer,length);
        buffer += length;
#endif*/
#if defined(ARDUINO_PLATFORM_SPI)/* || defined(MSVC_FT800EMU)*/
                SizeTransfered = 0;
		while (length--) {
                    Gpu_Hal_Transfer8(host,*buffer);
		    buffer++;
                    SizeTransfered ++;
		}
                length = SizeTransfered;
#endif

/*#ifdef MSVC_PLATFORM_SPI
		{   
		    SPI_Write(host->hal_handle,buffer,length,&SizeTransfered,SPI_TRANSFER_OPTIONS_SIZE_IN_BYTES);
                    length = SizeTransfered;
   		    buffer += SizeTransfered;
		}
#endif*/

		Gpu_Hal_EndTransfer(host);
		Gpu_Hal_Updatecmdfifo(host,length);

		Gpu_Hal_WaitCmdfifo_empty(host);

		count -= length;
	}while (count > 0);
}

#if defined (ARDUINO_PLATFORM_SPI)/* || defined (FT900_PLATFORM)*/
void Gpu_Hal_WrCmdBufFromFlash(Gpu_Hal_Context_t *host,PROGMEM prog_uchar8_t *buffer,uint32_t count)
{
	uint32_t length =0, SizeTransfered = 0;   

#define MAX_CMD_FIFO_TRANSFER   Gpu_Cmdfifo_Freespace(host)  
	do {                
		length = count;
		if (length > MAX_CMD_FIFO_TRANSFER){
		    length = MAX_CMD_FIFO_TRANSFER;
		}
      	        Gpu_Hal_CheckCmdBuffer(host,length);

                Gpu_Hal_StartCmdTransfer(host,GPU_WRITE,length);


                SizeTransfered = 0;
		while (length--) {
                    Gpu_Hal_Transfer8(host,pgm_read_byte_near(buffer));
		    buffer++;
                    SizeTransfered ++;
		}
                length = SizeTransfered;

    	        Gpu_Hal_EndTransfer(host);
		Gpu_Hal_Updatecmdfifo(host,length);

		Gpu_Hal_WaitCmdfifo_empty(host);

		count -= length;
	}while (count > 0);
}
#endif


void Gpu_Hal_CheckCmdBuffer(Gpu_Hal_Context_t *host,uint32_t count)
{
   uint16_t getfreespace;
   do{
        getfreespace = Gpu_Cmdfifo_Freespace(host);
   }while(getfreespace < count);
}
void Gpu_Hal_WaitCmdfifo_empty(Gpu_Hal_Context_t *host)
{
   while(Gpu_Hal_Rd16(host,REG_CMD_READ) != Gpu_Hal_Rd16(host,REG_CMD_WRITE));
   
   host->cmd_fifo_wp = Gpu_Hal_Rd16(host,REG_CMD_WRITE);
}

void Gpu_Hal_WrCmdBuf_nowait(Gpu_Hal_Context_t *host,uint8_t *buffer,uint32_t count)
{
	uint32_t length =0, SizeTransfered = 0;   

#define MAX_CMD_FIFO_TRANSFER   Gpu_Cmdfifo_Freespace(host)  
	do {                
		length = count;
		if (length > MAX_CMD_FIFO_TRANSFER){
		    length = MAX_CMD_FIFO_TRANSFER;
		}
      	        Gpu_Hal_CheckCmdBuffer(host,length);

                Gpu_Hal_StartCmdTransfer(host,GPU_WRITE,length);
/*#ifdef FT900_PLATFORM
            spi_writen(SPIM,buffer,length);
		    buffer += length;


#endif*/
//#ifdef ARDUINO_PLATFORM_SPI
#if defined(ARDUINO_PLATFORM_SPI)/* || defined(MSVC_FT800EMU)*/
                SizeTransfered = 0;
		while (length--) {
                    Gpu_Hal_Transfer8(host,*buffer);
		    buffer++;
                    SizeTransfered ++;
		}
                length = SizeTransfered;
#endif

/*#ifdef MSVC_PLATFORM_SPI
		{   
		    SPI_Write(host->hal_handle,buffer,length,&SizeTransfered,SPI_TRANSFER_OPTIONS_SIZE_IN_BYTES);
                    length = SizeTransfered;
   		    buffer += SizeTransfered;
		}
#endif*/

		Gpu_Hal_EndTransfer(host);
		Gpu_Hal_Updatecmdfifo(host,length);

	//	Gpu_Hal_WaitCmdfifo_empty(host);

		count -= length;
	}while (count > 0);
}

uint8_t Gpu_Hal_WaitCmdfifo_empty_status(Gpu_Hal_Context_t *host)
{
   if(Gpu_Hal_Rd16(host,REG_CMD_READ) != Gpu_Hal_Rd16(host,REG_CMD_WRITE))
   {
     return 0;
   }
   else
   {
     host->cmd_fifo_wp = Gpu_Hal_Rd16(host,REG_CMD_WRITE);
     return 1;
   }  
}

void Gpu_Hal_WaitLogo_Finish(Gpu_Hal_Context_t *host)
{
    int16_t cmdrdptr,cmdwrptr;

    do{
         cmdrdptr = Gpu_Hal_Rd16(host,REG_CMD_READ);
         cmdwrptr = Gpu_Hal_Rd16(host,REG_CMD_WRITE);
    }while ((cmdwrptr != cmdrdptr) || (cmdrdptr != 0));
    host->cmd_fifo_wp = 0;
}


void Gpu_Hal_ResetCmdFifo(Gpu_Hal_Context_t *host)
{
   host->cmd_fifo_wp = 0;
}


void Gpu_Hal_WrCmd32(Gpu_Hal_Context_t *host,uint32_t cmd)
{
         Gpu_Hal_CheckCmdBuffer(host,sizeof(cmd));
      
         Gpu_Hal_Wr32(host,RAM_CMD + host->cmd_fifo_wp,cmd);
      
         Gpu_Hal_Updatecmdfifo(host,sizeof(cmd));
}


void Gpu_Hal_ResetDLBuffer(Gpu_Hal_Context_t *host)
{
           host->dl_buff_wp = 0;
}
/* Toggle PD_N pin of FT800 board for a power cycle*/
void Gpu_Hal_Powercycle(Gpu_Hal_Context_t *host, bool up)
{
	if (up)
	{
/*#ifdef MSVC_PLATFORM
            //WriteGPIO(host->hal_handle, 0xBB, 0x08);//PDN set to 0 ,connect BLUE wire of MPSSE to PDN# of FT800 board
	        WriteGPIO(host->hal_handle, (1 << host->hal_config.pdn_pin_no) | 0x3B, (0<<host->hal_config.pdn_pin_no)|0x08);//PDN set to 0 ,connect BLUE wire of MPSSE to PDN# of FT800 board
			
            //Gpu_Hal_Sleep delay(20);

            //WriteGPIO(host->hal_handle, 0xBB, 0x88);//PDN set to 1
	        WriteGPIO(host->hal_handle, (1 << host->hal_config.pdn_pin_no) | 0x3B, (1<<host->hal_config.pdn_pin_no)|0x08);//PDN set to 0 ,connect BLUE wire of MPSSE to PDN# of FT800 board
            //Gpu_Hal_Sleep delay(20);
#endif*/
#ifdef ARDUINO_PLATFORM      
            digitalWrite(host->hal_config.pdn_pin_no, LOW);
            /*Gpu_Hal_Sleep*/ delay(20);

            digitalWrite(host->hal_config.pdn_pin_no, HIGH);
            /*Gpu_Hal_Sleep*/ delay(20);
#endif
/*#ifdef FT900_PLATFORM
            gpio_write(host->hal_config.pdn_pin_no, 0);
            delay(20);
            gpio_write(host->hal_config.pdn_pin_no, 1);
            delay(20);
#endif*/
	}else
	{
/*#ifdef MSVC_PLATFORM
	        //WriteGPIO(host->hal_handle, 0xBB, 0x88);//PDN set to 1
	        WriteGPIO(host->hal_handle, (1 << host->hal_config.pdn_pin_no) | 0x3B, (1<<host->hal_config.pdn_pin_no)|0x08);//PDN set to 0 ,connect BLUE wire of MPSSE to PDN# of FT800 board
            //Gpu_Hal_Sleep delay(20);
            
            //WriteGPIO(host->hal_handle, 0xBB, 0x08);//PDN set to 0 ,connect BLUE wire of MPSSE to PDN# of FT800 board
	        WriteGPIO(host->hal_handle, (1 << host->hal_config.pdn_pin_no) | 0x3B, (0<<host->hal_config.pdn_pin_no)|0x08);//PDN set to 0 ,connect BLUE wire of MPSSE to PDN# of FT800 board
			
            //Gpu_Hal_Sleep delay(20);
#endif*/
#ifdef ARDUINO_PLATFORM
            digitalWrite(host->hal_config.pdn_pin_no, HIGH);
            /*Gpu_Hal_Sleep*/ delay(20);
            
            digitalWrite(host->hal_config.pdn_pin_no, LOW);
            /*Gpu_Hal_Sleep*/ delay(20);
#endif
/*#ifdef FT900_PLATFORM
            gpio_write(host->hal_config.pdn_pin_no, 1);
            delay(20);
            gpio_write(host->hal_config.pdn_pin_no, 0);
            delay(20);
#endif*/

	}
}
void Gpu_Hal_WrMemFromFlash(Gpu_Hal_Context_t *host,uint32_t addr,const prog_uchar8_t *buffer, uint32_t length)
{
	uint32_t SizeTransfered = 0;      

	Gpu_Hal_StartTransfer(host,GPU_WRITE,addr);

#if defined(ARDUINO_PLATFORM_SPI)/* || defined(MSVC_FT800EMU) || defined(FT900_PLATFORM*/)
	while (length--) {
            Gpu_Hal_Transfer8(host,pgm_read_byte_near(buffer));
	    buffer++;
	}
#endif

/*#ifdef MSVC_PLATFORM_SPI
	{
	    SPI_Write((HANDLE)host->hal_handle,buffer,length,&SizeTransfered,SPI_TRANSFER_OPTIONS_SIZE_IN_BYTES);
	}
#endif*/


	Gpu_Hal_EndTransfer(host);
}

void Gpu_Hal_WrMem(Gpu_Hal_Context_t *host,uint32_t addr,const uint8_t *buffer, uint32_t length)
{
	uint32_t SizeTransfered = 0;      

	Gpu_Hal_StartTransfer(host,GPU_WRITE,addr);
/*#ifdef FT900_PLATFORM

	spi_writen(SPIM,buffer,length);

#endif*/
#if defined(ARDUINO_PLATFORM_SPI)/* || defined(MSVC_FT800EMU)*/
	while (length--) {
            Gpu_Hal_Transfer8(host,*buffer);
	    buffer++;
	}
#endif

/*#ifdef MSVC_PLATFORM_SPI
	{
	    SPI_Write((HANDLE)host->hal_handle,buffer,length,&SizeTransfered,SPI_TRANSFER_OPTIONS_SIZE_IN_BYTES);
	}
#endif*/


	Gpu_Hal_EndTransfer(host);
}


void Gpu_Hal_RdMem(Gpu_Hal_Context_t *host,uint32_t addr, uint8_t *buffer, uint32_t length)
{
	uint32_t SizeTransfered = 0;      

	Gpu_Hal_StartTransfer(host,GPU_READ,addr);
/*#ifdef FT900_PLATFORM
	unsigned char spiData[2] = {0};
	spi_readn(SPIM,spiData,host->spinumdummy);
	spi_readn(SPIM,buffer,length);
#endif*/
#if defined(ARDUINO_PLATFORM_SPI)/* || defined(MSVC_FT800EMU)*/
	while (length--) {
	   *buffer = Gpu_Hal_Transfer8(host,0);
	   buffer++;
	}
#endif

/*#ifdef MSVC_PLATFORM_SPI
	{
	   SPI_Read((HANDLE)host->hal_handle,buffer,length,&SizeTransfered,SPI_TRANSFER_OPTIONS_SIZE_IN_BYTES);
	}
#endif*/

	Gpu_Hal_EndTransfer(host);
}

/* Helper api for dec to ascii */
int32_t Gpu_Hal_Dec2Ascii(char8_t *pSrc,int32_t value)
{
	int16_t Length;
	char8_t *pdst,charval;
	int32_t CurrVal = value,tmpval,i;
	char8_t tmparray[16],idx = 0;

	Length = strlen(pSrc);
	pdst = pSrc + Length;

	if(0 == value)
	{
		*pdst++ = '0';
		*pdst++ = '\0';
		return 0;
	}

	if(CurrVal < 0)
	{
		*pdst++ = '-';
		CurrVal = - CurrVal;
	}
	/* insert the value */
	while(CurrVal > 0){
		tmpval = CurrVal;
		CurrVal /= 10;
		tmpval = tmpval - CurrVal*10;
		charval = '0' + tmpval;
		tmparray[idx++] = charval;
	}

	for(i=0;i<idx;i++)
	{
		*pdst++ = tmparray[idx - i - 1];
	}
	*pdst++ = '\0';

	return 0;
}



/*void Gpu_Hal_Sleep(uint32_t ms)
{
#ifdef FT900_PLATFORM
	delayms(ms);
#endif
#if defined(MSVC_PLATFORM) || defined(MSVC_FT800EMU)
	Sleep(ms);
#endif
#ifdef ARDUINO_PLATFORM
	delay(ms);
#endif
}*/
#ifdef 81X_ENABLE
int16_t Gpu_Hal_SetSPI(Gpu_Hal_Context_t *host,GPU_SPI_NUMCHANNELS_T numchnls,GPU_SPI_NUMDUMMYBYTES numdummy)
{
	uint8_t writebyte = 0;
	/* error check */
	if((numchnls > GPU_SPI_QUAD_CHANNEL) || (numdummy > GPU_SPI_TWODUMMY) || (numdummy < GPU_SPI_ONEDUMMY))
	{
		return -1;//error
	}

	host->spichannel = numchnls;
	writebyte = host->spichannel;
	host->spinumdummy = numdummy;

	if(GPU_SPI_TWODUMMY == host->spinumdummy)
	{
		writebyte |= SPI_TWO_DUMMY_BYTE;
	}
	Gpu_Hal_Wr8(host,REG_SPI_WIDTH,writebyte);
	/* set the parameters in hal context and also set into ft81x */
	return 0;
}
#endif

/*#ifdef FT900_PLATFORM
// Helper api for millis
// api to return the time in ms. 0 after reset

// Globals for polling implementation 
uint32_t millis_curr = 0,millis_prev = 0;

// Globals for interrupt implementation 
uint32_t TotalMilliseconds = 0;

void millis_ticker()
{
	timer_disable_interrupt(FT900_MILLIS_TIMER);
	// Clear the interrupt and increment the counter 
	timer_is_interrupted(FT900_MILLIS_TIMER);

	TotalMilliseconds += 1;
	timer_enable_interrupt(FT900_MILLIS_TIMER);
}
void millis_init()
{
*//*#ifdef FT900_PLATFORM
	millis_curr = 0;
	millis_prev = 0;

	sys_enable(sys_device_timer_wdt);
	timer_prescaler(FT900_TIMER_PRESCALE_VALUE);
	timer_init(FT900_MILLIS_TIMER,FT900_TIMER_OVERFLOW_VALUE,timer_direction_up,timer_prescaler_select_on,timer_mode_continuous);

	interrupt_attach(interrupt_timers, 17, millis_ticker);


	// enabling the interrupts for timer 
	timer_enable_interrupt(FT900_MILLIS_TIMER);

	timer_start(FT900_MILLIS_TIMER);
#endif*//*
}
// Need to ensure that below api is called at least once in 6.5 seconds duration for FT900 platform as this module doesnt use timer for context update 
// global counter to loopback after ~49.71 days 
uint32_t millis()
{
#if defined(ARDUINO_PLATFORM) ||defined(MSVC_PLATFORM) || defined(MSVC_FT800EMU)
	return millis();
#endif
*//*#ifdef FT900_PLATFORM

	// Polling implementation
#if 0
	uint32_t currtime;
	currtime = ft900_timer_get_value(FT900_MILLIS_TIMER);

	if(millis_prev > currtime)
	{
		// loop back condition 
		millis_curr += ((FT900_TIMER_MAX_VALUE - millis_prev + currtime)/10);
	}
	else
	{
		millis_curr += ((currtime - millis_prev)/10);
	}
	millis_prev = currtime;
	//printf("current time %d \n",millis_curr);
	return millis_curr;
#endif

	// Interrupt implementation 
	return (TotalMilliseconds);
#endif *//*
}

void millis_exit()
{
*//*#ifdef FT900_PLATFORM
	timer_stop(FT900_MILLIS_TIMER);
	timer_disable_interrupt(FT900_MILLIS_TIMER);
#endif*//*
}
#endif*/

/* FIFO related apis */
//Init all the parameters of fifo buffer
void Fifo_Init(Fifo_t *pFifo,uint32_t StartAddress,uint32_t Length,uint32_t HWReadRegAddress,uint32_t HWWriteRegAddress)
{
	/* update the context parameters */
	pFifo->fifo_buff = StartAddress;
	pFifo->fifo_len = Length;
	pFifo->fifo_rp = pFifo->fifo_wp = 0;

	/* update the hardware register addresses - specific to FT800 series chips */
	pFifo->HW_Read_Reg = HWReadRegAddress;
	pFifo->HW_Write_Reg = HWWriteRegAddress;
}

//update both the read and write pointers
void Fifo_Update(Gpu_Hal_Context_t *host,Fifo_t *pFifo)
{
	pFifo->fifo_rp = Gpu_Hal_Rd32(host,pFifo->HW_Read_Reg);
	//Gpu_Hal_Wr32(host,pFifo->HW_Write_Reg,pFifo->fifo_wp);
}

//just write and update the write register
uint32_t Fifo_Write(Gpu_Hal_Context_t *host,Fifo_t *pFifo,uint8_t *buffer,uint32_t NumbytetoWrite)
{
	uint32_t FreeSpace = Fifo_GetFreeSpace(host,pFifo),TotalBytes = NumbytetoWrite;

	if(NumbytetoWrite > FreeSpace)
	{
		/* update the read pointer and get the free space */
		Fifo_Update(host,pFifo);
		FreeSpace = Fifo_GetFreeSpace(host,pFifo);

		if(NumbytetoWrite > FreeSpace)
		{
			TotalBytes = FreeSpace;
		}
	}

	/* sanity check */
	if(TotalBytes <= 0)
	{
		//printf("no space in fifo write %d %d %d %d\n",TotalBytes,FreeSpace,pFifo->fifo_wp,pFifo->fifo_rp);
		return 0;//error condition
	}
	/* check for the loopback conditions */
	if(pFifo->fifo_wp + TotalBytes >= pFifo->fifo_len)
	{
		uint32_t partialchunk = pFifo->fifo_len - pFifo->fifo_wp,secpartialchunk = TotalBytes - partialchunk;

		Gpu_Hal_WrMem(host,pFifo->fifo_buff + pFifo->fifo_wp,buffer,partialchunk);
		if(secpartialchunk > 0)
		{
			Gpu_Hal_WrMem(host,pFifo->fifo_buff,buffer + partialchunk,secpartialchunk);
		}
		pFifo->fifo_wp = secpartialchunk;
		//printf("partial chunks %d %d %d %d\n",partialchunk,secpartialchunk,pFifo->fifo_wp,pFifo->fifo_rp);

	}
	else
	{
		Gpu_Hal_WrMem(host,pFifo->fifo_buff + pFifo->fifo_wp,buffer,TotalBytes);
		pFifo->fifo_wp += TotalBytes;
	}

	/* update the write pointer address in write register */
	Gpu_Hal_Wr32(host,pFifo->HW_Write_Reg,pFifo->fifo_wp);

	return TotalBytes;
}
//just write one word and update the write register
void Fifo_Write32(Gpu_Hal_Context_t *host,Fifo_t *pFifo,uint32_t WriteWord)
{
	Fifo_Write(host,pFifo,(uint8_t *)&WriteWord,4);
}
//write and wait for the fifo to be empty. handle cases even if the Numbytes are more than freespace
void Fifo_WriteWait(Gpu_Hal_Context_t *host,Fifo_t *pFifo,uint8_t *buffer,uint32_t Numbyte)
{
	uint32_t TotalBytes = Numbyte,currchunk = 0,FreeSpace;
	uint8_t *pbuff = buffer;
	/* blocking call, manage to check for the error case and break in case of error */
	while(TotalBytes > 0)
	{
		currchunk = TotalBytes;
		FreeSpace = Fifo_GetFreeSpace(host,pFifo);
		if(currchunk > FreeSpace)
		{
			currchunk = FreeSpace;
		}

		Fifo_Write(host,pFifo,pbuff,currchunk);
		pbuff += currchunk;
		TotalBytes -= currchunk;


	}
}

/*#if defined(FT900_PLATFORM)
void getFlashTextString(char __flash__ *str, uchar8_t *destArray, uint16_t numOfChars){
		uint16_t i;
		for(i=0;i<numOfChars;i++)
			destArray[i] = str[i];
}
#endif*/

//get the free space in the fifo - make sure the return value is maximum of (LENGTH - 4)
uint32_t Fifo_GetFreeSpace(Gpu_Hal_Context_t *host,Fifo_t *pFifo)
{
	uint32_t FreeSpace = 0;

	Fifo_Update(host,pFifo);

	if(pFifo->fifo_wp >= pFifo->fifo_rp)
	{
		FreeSpace = pFifo->fifo_len - pFifo->fifo_wp + pFifo->fifo_rp;
	}
	else
	{
		FreeSpace = pFifo->fifo_rp - pFifo->fifo_wp;
	}

	if(FreeSpace >= 4)
	{
		FreeSpace -= 4;//make sure 1 word space is maintained between rd and wr pointers
	}
	return FreeSpace;
}



uint32_t Gpu_CurrentFrequency(Gpu_Hal_Context_t *host)
{
    uint32_t t0, t1;
    uint32_t addr = REG_CLOCK;
    uint8_t spidata[4];
	int32_t r = 15625;

    t0 = Gpu_Hal_Rd32(host,REG_CLOCK); /* t0 read */
               
/*
#ifdef FT900_PLATFORM
    __asm__
    (
                    "   move.l  $r0,%0"             "\n\t"
                    "   mul.l   $r0,$r0,100"                                                  "\n\t"
                    "1:"               "\n\t"
                    "   sub.l   $r0,$r0,3"          "\n\t" // Subtract the loop time = 4 cycles 
                    "   cmp.l   $r0,0"              "\n\t" // Check that the counter is equal to 0
                    "   jmpc    gt, 1b"  "\n\t"
                    // Outputs :
                    // Inputs  : "r"(r)
                    // Using   : "$r0"

    );

    //usleep(15625);
	//delay(15625);
#endif */


/*#if (defined(MSVC_PLATFORM) || defined(MSVC_FT800EMU))
	//may not be precise
	Sleep(15625/1000);
#endif*/
#ifdef ARDUINO_PLATFORM
	delayMicroseconds(15625);
#endif

    t1 = Gpu_Hal_Rd32(host,REG_CLOCK); /* t1 read */
    return ((t1 - t0) * 64); /* bitshift 6 places is the same as multiplying 64 */
}

int32_t Gpu_ClockTrimming(Gpu_Hal_Context_t *host,int32_t LowFreq)
{
   uint32_t f;
   uint8_t i;

  /* Trim the internal clock by increase the REG_TRIM register till the measured frequency is within the acceptable range.*/
   for (i=0; (i < 31) && ((f= Gpu_CurrentFrequency(host)) < LowFreq); i++)
   {
	   Gpu_Hal_Wr8(host,REG_TRIM, i);  /* increase the REG_TRIM register value automatically increases the internal clock */

   }

   Gpu_Hal_Wr32(host,REG_FREQUENCY,f);  /* Set the final frequency to be used for internal operations */

   return f;
}


