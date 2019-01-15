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

#include "FT_Platform.h"

/* API to initialize the SPI interface */
ft_bool_t  Ft_Gpu_Hal_Init(Ft_Gpu_HalInit_t *halinit)
{
	return TRUE;
}
ft_bool_t    Ft_Gpu_Hal_Open(Ft_Gpu_Hal_Context_t *host)
{

	gpio_function(host->hal_config.spi_cs_pin_no, pad_spim_ss0); /* GPIO28 as CS */
	gpio_write(host->hal_config.spi_cs_pin_no, 1);

	gpio_function(host->hal_config.pdn_pin_no, pad_gpio43);
	gpio_dir(host->hal_config.pdn_pin_no, pad_dir_output);

    gpio_write(host->hal_config.pdn_pin_no,1);

	pinMode(host->hal_config.pdn_pin_no, OUTPUT);
	digitalWrite(host->hal_config.pdn_pin_no, HIGH);
    pinMode(host->hal_config.spi_cs_pin_no, OUTPUT);
    digitalWrite(host->hal_config.spi_cs_pin_no, HIGH);
	SPI.begin();
	SPI.setClockDivider(SPI_CLOCK_DIV2);
	SPI.setBitOrder(MSBFIRST);
	SPI.setDataMode(SPI_MODE0);


    ChannelConfig channelConf;			//channel configuration
	FT_STATUS status;					
		
	/* configure the spi settings */
	channelConf.ClockRate = host->hal_config.spi_clockrate_khz * 1000; 
	channelConf.LatencyTimer= 2;       
	channelConf.configOptions = SPI_CONFIG_OPTION_MODE0 | SPI_CONFIG_OPTION_CS_DBUS3 | SPI_CONFIG_OPTION_CS_ACTIVELOW;
	channelConf.Pin = 0x00000000;	/*FinalVal-FinalDir-InitVal-InitDir (for dir 0=in, 1=out)*/

	/* Open the first available channel */
	SPI_OpenChannel(host->hal_config.channel_no,(FT_HANDLE *)&host->hal_handle);
	status = SPI_InitChannel((FT_HANDLE)host->hal_handle,&channelConf);
	printf("\nhandle=0x%x status=0x%x\n",host->hal_handle,status);	

	/* Initialize the context valriables */
	host->ft_cmd_fifo_wp = host->ft_dl_buff_wp = 0;
	host->spinumdummy = 1;//by default ft800/801/810/811 goes with single dummy byte for read
	host->spichannel = 0;
	host->status = FT_GPU_HAL_OPENED;

	return TRUE;
}
void  Ft_Gpu_Hal_Close(Ft_Gpu_Hal_Context_t *host)
{
	host->status = FT_GPU_HAL_CLOSED;

	/* Close the channel*/
	SPI_CloseChannel(host->hal_handle);
        SPI.end();


}

void Ft_Gpu_Hal_DeInit()
{

   //Cleanup the MPSSE Lib
   Cleanup_libMPSSE();

   spi_uninit(SPIM);

}

/*The APIs for reading/writing transfer continuously only with small buffer system*/
void  Ft_Gpu_Hal_StartTransfer(Ft_Gpu_Hal_Context_t *host,FT_GPU_TRANSFERDIR_T rw,uint32_t addr)
{
	if (FT_GPU_READ == rw){



		uint8_t Transfer_Array[4];
		uint32_t SizeTransfered;

		/* Compose the read packet */
		Transfer_Array[0] = addr >> 16;
		Transfer_Array[1] = addr >> 8;
		Transfer_Array[2] = addr;

		Transfer_Array[3] = 0; //Dummy Read byte
		SPI_Write((FT_HANDLE)host->hal_handle,Transfer_Array,sizeof(Transfer_Array),&SizeTransfered,SPI_TRANSFER_OPTIONS_SIZE_IN_BYTES | SPI_TRANSFER_OPTIONS_CHIPSELECT_ENABLE);

		digitalWrite(host->hal_config.spi_cs_pin_no, LOW);
		SPI.transfer(addr >> 16);
		SPI.transfer(highByte(addr));
		SPI.transfer(lowByte(addr));

		SPI.transfer(0); //Dummy Read Byte



		host->status = FT_GPU_HAL_READING;
	}else{
	
	

		uint8_t Transfer_Array[3];
		uint32_t SizeTransfered;

		/* Compose the read packet */
		Transfer_Array[0] = (0x80 | (addr >> 16));
		Transfer_Array[1] = addr >> 8;
		Transfer_Array[2] = addr;
		SPI_Write((FT_HANDLE)host->hal_handle,Transfer_Array,3,&SizeTransfered,SPI_TRANSFER_OPTIONS_SIZE_IN_BYTES | SPI_TRANSFER_OPTIONS_CHIPSELECT_ENABLE);		

		digitalWrite(host->hal_config.spi_cs_pin_no, LOW);
		SPI.transfer(0x80 | (addr >> 16));
		SPI.transfer(highByte(addr));
		SPI.transfer(lowByte(addr));


		host->status = FT_GPU_HAL_WRITING;
	}
}



/*The APIs for writing transfer continuously only*/
void  Ft_Gpu_Hal_StartCmdTransfer(Ft_Gpu_Hal_Context_t *host,FT_GPU_TRANSFERDIR_T rw, uint16_t count)
{
	Ft_Gpu_Hal_StartTransfer(host,rw,host->ft_cmd_fifo_wp + RAM_CMD);
}

uint8_t    Ft_Gpu_Hal_TransferString(Ft_Gpu_Hal_Context_t *host,const char *string)
{
    uint16_t length = strlen(string);
    while(length --){
       Ft_Gpu_Hal_Transfer8(host,*string);
       string ++;
    }
    //Append one null as ending flag
    Ft_Gpu_Hal_Transfer8(host,0);
}


uint8_t    Ft_Gpu_Hal_Transfer8(Ft_Gpu_Hal_Context_t *host, uint8_t value)
{


        return SPI.transfer(value);


	uint32_t SizeTransfered;
	if (host->status == FT_GPU_HAL_WRITING){
		SPI_Write(host->hal_handle,&value,sizeof(value),&SizeTransfered,SPI_TRANSFER_OPTIONS_SIZE_IN_BYTES);
	}else{
		SPI_Read(host->hal_handle,&value,sizeof(value),&SizeTransfered,SPI_TRANSFER_OPTIONS_SIZE_IN_BYTES);
	}

	if (SizeTransfered != sizeof(value))
		host->status = FT_GPU_HAL_STATUS_ERROR;
        return value;
	
}


uint16_t  Ft_Gpu_Hal_Transfer16(Ft_Gpu_Hal_Context_t *host,uint16_t value)
{
	uint16_t retVal = 0;

        if (host->status == FT_GPU_HAL_WRITING){
		Ft_Gpu_Hal_Transfer8(host,value & 0xFF);//LSB first
		Ft_Gpu_Hal_Transfer8(host,(value >> 8) & 0xFF);
	}else{
		retVal = Ft_Gpu_Hal_Transfer8(host,0);
		retVal |= (uint16_t)Ft_Gpu_Hal_Transfer8(host,0) << 8;
	}

	return retVal;
}
uint32_t  Ft_Gpu_Hal_Transfer32(Ft_Gpu_Hal_Context_t *host,uint32_t value)
{
	uint32_t retVal = 0;
	if (host->status == FT_GPU_HAL_WRITING){
		Ft_Gpu_Hal_Transfer16(host,value & 0xFFFF);//LSB first
		Ft_Gpu_Hal_Transfer16(host,(value >> 16) & 0xFFFF);
	}else{
		retVal = Ft_Gpu_Hal_Transfer16(host,0);
		retVal |= (uint32_t)Ft_Gpu_Hal_Transfer16(host,0) << 16;
	}
	return retVal;
}

void   Ft_Gpu_Hal_EndTransfer(Ft_Gpu_Hal_Context_t *host)
{
	digitalWrite(host->hal_config.spi_cs_pin_no, HIGH);
	host->status = FT_GPU_HAL_OPENED;
}


uint8_t  Ft_Gpu_Hal_Rd8(Ft_Gpu_Hal_Context_t *host,uint32_t addr)
{
	uint8_t value;
Ft_Gpu_Hal_EndTransfer(host);
	return value;
}
uint16_t Ft_Gpu_Hal_Rd16(Ft_Gpu_Hal_Context_t *host,uint32_t addr)
{
	uint16_t value;
Ft_Gpu_Hal_EndTransfer(host);
	return value;
}
uint32_t Ft_Gpu_Hal_Rd32(Ft_Gpu_Hal_Context_t *host,uint32_t addr)
{
	uint32_t value;
Ft_Gpu_Hal_EndTransfer(host);
	return value;
}

void Ft_Gpu_Hal_Wr8(Ft_Gpu_Hal_Context_t *host,uint32_t addr, uint8_t v)
{	
	Ft_Gpu_Hal_StartTransfer(host,FT_GPU_WRITE,addr);
	Ft_Gpu_Hal_Transfer8(host,v);
	Ft_Gpu_Hal_EndTransfer(host);
}
void Ft_Gpu_Hal_Wr16(Ft_Gpu_Hal_Context_t *host,uint32_t addr, uint16_t v)
{
	Ft_Gpu_Hal_StartTransfer(host,FT_GPU_WRITE,addr);
	Ft_Gpu_Hal_Transfer16(host,v);
	Ft_Gpu_Hal_EndTransfer(host);
}
void Ft_Gpu_Hal_Wr32(Ft_Gpu_Hal_Context_t *host,uint32_t addr, uint32_t v)
{
	Ft_Gpu_Hal_StartTransfer(host,FT_GPU_WRITE,addr);
	Ft_Gpu_Hal_Transfer32(host,v);
	Ft_Gpu_Hal_EndTransfer(host);
}

void Ft_Gpu_HostCommand(Ft_Gpu_Hal_Context_t *host, uint8_t cmd)
{


 uint8_t Transfer_Array[3];
  uint32_t SizeTransfered;

  Transfer_Array[0] = cmd;
  Transfer_Array[1] = 0;
  Transfer_Array[2] = 0;

  SPI_Write(host->hal_handle,Transfer_Array,sizeof(Transfer_Array),&SizeTransfered,SPI_TRANSFER_OPTIONS_SIZE_IN_BYTES | SPI_TRANSFER_OPTIONS_CHIPSELECT_ENABLE | SPI_TRANSFER_OPTIONS_CHIPSELECT_DISABLE);

  digitalWrite(host->hal_config.spi_cs_pin_no, LOW);
  SPI.transfer(cmd);
  SPI.transfer(0);
  SPI.transfer(0);
  digitalWrite(host->hal_config.spi_cs_pin_no, HIGH);

}

void Ft_Gpu_ClockSelect(Ft_Gpu_Hal_Context_t *host,FT_GPU_PLL_SOURCE_T pllsource)
{
   Ft_Gpu_HostCommand(host,pllsource);
}
void Ft_Gpu_PLL_FreqSelect(Ft_Gpu_Hal_Context_t *host,FT_GPU_PLL_FREQ_T freq)
{
   Ft_Gpu_HostCommand(host,freq);
}
void Ft_Gpu_PowerModeSwitch(Ft_Gpu_Hal_Context_t *host,FT_GPU_POWER_MODE_T pwrmode)
{
   Ft_Gpu_HostCommand(host,pwrmode);
}
void Ft_Gpu_CoreReset(Ft_Gpu_Hal_Context_t *host)
{
   Ft_Gpu_HostCommand(host,FT_GPU_CORE_RESET);
}



//This API can only be called when PLL is stopped(SLEEP mode).  For compatibility, set frequency to the FT_GPU_12MHZ option in the FT_GPU_SETPLLSP1_T table.
void Ft_Gpu_81X_SelectSysCLK(Ft_Gpu_Hal_Context_t *host, FT_GPU_81X_PLL_FREQ_T freq){
		if(FT_GPU_SYSCLK_72M == freq)
			Ft_Gpu_HostCommand_Ext3(host, (uint32_t)0x61 | (0x40 << 8) | (0x06 << 8)); 
		else if(FT_GPU_SYSCLK_60M == freq)
			Ft_Gpu_HostCommand_Ext3(host, (uint32_t)0x61 | (0x40 << 8) | (0x05 << 8)); 
		else if(FT_GPU_SYSCLK_48M == freq)
			Ft_Gpu_HostCommand_Ext3(host, (uint32_t)0x61 | (0x40 << 8) | (0x04 << 8)); 
		else if(FT_GPU_SYSCLK_36M == freq)
			Ft_Gpu_HostCommand_Ext3(host, (uint32_t)0x61 | (0x03 << 8)); 
		else if(FT_GPU_SYSCLK_24M == freq)
			Ft_Gpu_HostCommand_Ext3(host, (uint32_t)0x61 | (0x02 << 8)); 
		else if(FT_GPU_SYSCLK_DEFAULT == freq)//default clock
			Ft_Gpu_HostCommand_Ext3(host, 0x61); 
}

//Power down or up ROMs and ADCs.  Specified one or more elements in the FT_GPU_81X_ROM_AND_ADC_T table to power down, unspecified elements will be powered up.  The application must retain the state of the ROMs and ADCs as they're not readable from the device.
void Ft_GPU_81X_PowerOffComponents(Ft_Gpu_Hal_Context_t *host, uint8_t val){
		Ft_Gpu_HostCommand_Ext3(host, (uint32_t)0x49 | (val<<8));
}

//this API sets the current strength of supported GPIO/IO group(s)
void Ft_GPU_81X_PadDriveStrength(Ft_Gpu_Hal_Context_t *host, FT_GPU_81X_GPIO_DRIVE_STRENGTH_T strength, FT_GPU_81X_GPIO_GROUP_T group){
		Ft_Gpu_HostCommand_Ext3(host, (uint32_t)0x70 | (group << 8) | (strength << 8));
}

//this API will hold the system reset active, Ft_Gpu_81X_ResetRemoval() must be called to release the system reset.
void Ft_Gpu_81X_ResetActive(Ft_Gpu_Hal_Context_t *host){
	Ft_Gpu_HostCommand_Ext3(host, FT_GPU_81X_RESET_ACTIVE); 
}

//This API will release the system reset, and the system will exit reset and behave as after POR, settings done through SPI commands will not be affected.
void Ft_Gpu_81X_ResetRemoval(Ft_Gpu_Hal_Context_t *host){
	Ft_Gpu_HostCommand_Ext3(host, FT_GPU_81X_RESET_REMOVAL); 
}



//This API sends a 3byte command to the host
void Ft_Gpu_HostCommand_Ext3(Ft_Gpu_Hal_Context_t *host,uint32_t cmd)
{

	 uint8_t Transfer_Array[3];
	  uint32_t SizeTransfered;
	
	  Transfer_Array[0] = cmd;
	  Transfer_Array[1] = (cmd>>8) & 0xff;
	  Transfer_Array[2] = (cmd>>16) & 0xff;
	
	  SPI_Write(host->hal_handle,Transfer_Array,sizeof(Transfer_Array),&SizeTransfered,SPI_TRANSFER_OPTIONS_SIZE_IN_BYTES | SPI_TRANSFER_OPTIONS_CHIPSELECT_ENABLE | SPI_TRANSFER_OPTIONS_CHIPSELECT_DISABLE);

	  digitalWrite(host->hal_config.spi_cs_pin_no, LOW);
	  SPI.transfer(cmd);
	  SPI.transfer((cmd>>8) & 0xff);
	  SPI.transfer((cmd>>16) & 0xff);
	  digitalWrite(host->hal_config.spi_cs_pin_no, HIGH);

}



void Ft_Gpu_Hal_Updatecmdfifo(Ft_Gpu_Hal_Context_t *host,uint32_t count)
{
	host->ft_cmd_fifo_wp  = (host->ft_cmd_fifo_wp + count) & 4095;

	//4 byte alignment
	host->ft_cmd_fifo_wp = (host->ft_cmd_fifo_wp + 3) & 0xffc;
	Ft_Gpu_Hal_Wr16(host,REG_CMD_WRITE,host->ft_cmd_fifo_wp);
}


uint16_t Ft_Gpu_Cmdfifo_Freespace(Ft_Gpu_Hal_Context_t *host)
{
	uint16_t fullness,retval;

	//host->ft_cmd_fifo_wp = Ft_Gpu_Hal_Rd16(host,REG_CMD_WRITE);

	fullness = (host->ft_cmd_fifo_wp - Ft_Gpu_Hal_Rd16(host,REG_CMD_READ)) & 4095;
	retval = (FT_CMD_FIFO_SIZE - 4) - fullness;
	return (retval);
}

void Ft_Gpu_Hal_WrCmdBuf(Ft_Gpu_Hal_Context_t *host, uint8_t *buffer,uint32_t count)
{
	int32_t length =0, SizeTransfered = 0, availablefreesize;   

#define MAX_CMD_FIFO_TRANSFER   Ft_Gpu_Cmdfifo_Freespace(host)  
	do {                
		length = count;
		availablefreesize = MAX_CMD_FIFO_TRANSFER;

		if (length > availablefreesize)
		{
		    length = availablefreesize;
		}
      	        Ft_Gpu_Hal_CheckCmdBuffer(host,length);

                Ft_Gpu_Hal_StartCmdTransfer(host,FT_GPU_WRITE,length);
                SizeTransfered = 0;
		while (length--) {
                    Ft_Gpu_Hal_Transfer8(host,*buffer);
		    buffer++;
                    SizeTransfered ++;
		}
                length = SizeTransfered;




		Ft_Gpu_Hal_EndTransfer(host);
		Ft_Gpu_Hal_Updatecmdfifo(host,length);

		Ft_Gpu_Hal_WaitCmdfifo_empty(host);

		count -= length;
	}while (count > 0);
}

void Ft_Gpu_Hal_WrCmdBufFromFlash(Ft_Gpu_Hal_Context_t *host,PROGMEM const unsigned char *buffer,uint32_t count)
{
	uint32_t length =0, SizeTransfered = 0;   

#define MAX_CMD_FIFO_TRANSFER   Ft_Gpu_Cmdfifo_Freespace(host)  
	do {                
		length = count;
		if (length > MAX_CMD_FIFO_TRANSFER){
		    length = MAX_CMD_FIFO_TRANSFER;
		}
      	        Ft_Gpu_Hal_CheckCmdBuffer(host,length);

                Ft_Gpu_Hal_StartCmdTransfer(host,FT_GPU_WRITE,length);


                SizeTransfered = 0;
		while (length--) {
                    Ft_Gpu_Hal_Transfer8(host,pgm_read_byte_near(buffer));
		    buffer++;
                    SizeTransfered ++;
		}
                length = SizeTransfered;

    	        Ft_Gpu_Hal_EndTransfer(host);
		Ft_Gpu_Hal_Updatecmdfifo(host,length);

		Ft_Gpu_Hal_WaitCmdfifo_empty(host);

		count -= length;
	}while (count > 0);
}

void Ft_Gpu_Hal_CheckCmdBuffer(Ft_Gpu_Hal_Context_t *host,uint32_t count)
{
   uint16_t getfreespace;
   do{
        getfreespace = Ft_Gpu_Cmdfifo_Freespace(host);
   }while(getfreespace < count);
}
void Ft_Gpu_Hal_WaitCmdfifo_empty(Ft_Gpu_Hal_Context_t *host)
{
   while(Ft_Gpu_Hal_Rd16(host,REG_CMD_READ) != Ft_Gpu_Hal_Rd16(host,REG_CMD_WRITE));
   
   host->ft_cmd_fifo_wp = Ft_Gpu_Hal_Rd16(host,REG_CMD_WRITE);
}

void Ft_Gpu_Hal_WrCmdBuf_nowait(Ft_Gpu_Hal_Context_t *host, uint8_t  *buffer,uint32_t count)
{
	uint32_t length =0, SizeTransfered = 0;   

#define MAX_CMD_FIFO_TRANSFER   Ft_Gpu_Cmdfifo_Freespace(host)  
	do {                
		length = count;
		if (length > MAX_CMD_FIFO_TRANSFER){
		    length = MAX_CMD_FIFO_TRANSFER;
		}
      	        Ft_Gpu_Hal_CheckCmdBuffer(host,length);

                Ft_Gpu_Hal_StartCmdTransfer(host,FT_GPU_WRITE,length);
                SizeTransfered = 0;
		while (length--) {
                    Ft_Gpu_Hal_Transfer8(host,*buffer);
		    buffer++;
                    SizeTransfered ++;
		}
                length = SizeTransfered;



		Ft_Gpu_Hal_EndTransfer(host);
		Ft_Gpu_Hal_Updatecmdfifo(host,length);

	//	Ft_Gpu_Hal_WaitCmdfifo_empty(host);

		count -= length;
	}while (count > 0);
}

uint8_t Ft_Gpu_Hal_WaitCmdfifo_empty_status(Ft_Gpu_Hal_Context_t *host)
{
   if(Ft_Gpu_Hal_Rd16(host,REG_CMD_READ) != Ft_Gpu_Hal_Rd16(host,REG_CMD_WRITE))
   {
     return 0;
   }
   else
   {
     host->ft_cmd_fifo_wp = Ft_Gpu_Hal_Rd16(host,REG_CMD_WRITE);
     return 1;
   }  
}

void Ft_Gpu_Hal_WaitLogo_Finish(Ft_Gpu_Hal_Context_t *host)
{
    int16_t cmdrdptr,cmdwrptr;

    do{
         cmdrdptr = Ft_Gpu_Hal_Rd16(host,REG_CMD_READ);
         cmdwrptr = Ft_Gpu_Hal_Rd16(host,REG_CMD_WRITE);
    }while ((cmdwrptr != cmdrdptr) || (cmdrdptr != 0));
    host->ft_cmd_fifo_wp = 0;
}


void Ft_Gpu_Hal_ResetCmdFifo(Ft_Gpu_Hal_Context_t *host)
{
   host->ft_cmd_fifo_wp = 0;
}


void Ft_Gpu_Hal_WrCmd32(Ft_Gpu_Hal_Context_t *host,uint32_t cmd)
{
         Ft_Gpu_Hal_CheckCmdBuffer(host,sizeof(cmd));
      
         Ft_Gpu_Hal_Wr32(host,RAM_CMD + host->ft_cmd_fifo_wp,cmd);
      
         Ft_Gpu_Hal_Updatecmdfifo(host,sizeof(cmd));
}


void Ft_Gpu_Hal_ResetDLBuffer(Ft_Gpu_Hal_Context_t *host)
{
           host->ft_dl_buff_wp = 0;
}
/* Toggle PD_N pin of FT800 board for a power cycle*/
void Ft_Gpu_Hal_Powercycle(Ft_Gpu_Hal_Context_t *host, ft_bool_t up)
{
	if (up)
	{
            //FT_WriteGPIO(host->hal_handle, 0xBB, 0x08);//PDN set to 0 ,connect BLUE wire of MPSSE to PDN# of FT800 board
	        FT_WriteGPIO(host->hal_handle, (1 << host->hal_config.pdn_pin_no) | 0x3B, (0<<host->hal_config.pdn_pin_no)|0x08);//PDN set to 0 ,connect BLUE wire of MPSSE to PDN# of FT800 board
			
            Ft_Gpu_Hal_Sleep(20);

            //FT_WriteGPIO(host->hal_handle, 0xBB, 0x88);//PDN set to 1
	        FT_WriteGPIO(host->hal_handle, (1 << host->hal_config.pdn_pin_no) | 0x3B, (1<<host->hal_config.pdn_pin_no)|0x08);//PDN set to 0 ,connect BLUE wire of MPSSE to PDN# of FT800 board
            Ft_Gpu_Hal_Sleep(20); 
            digitalWrite(host->hal_config.pdn_pin_no, LOW);
            Ft_Gpu_Hal_Sleep(20);

            digitalWrite(host->hal_config.pdn_pin_no, HIGH);
            Ft_Gpu_Hal_Sleep(20);
}else
	{
	        //FT_WriteGPIO(host->hal_handle, 0xBB, 0x88);//PDN set to 1
	        FT_WriteGPIO(host->hal_handle, (1 << host->hal_config.pdn_pin_no) | 0x3B, (1<<host->hal_config.pdn_pin_no)|0x08);//PDN set to 0 ,connect BLUE wire of MPSSE to PDN# of FT800 board
            Ft_Gpu_Hal_Sleep(20);
            
            //FT_WriteGPIO(host->hal_handle, 0xBB, 0x08);//PDN set to 0 ,connect BLUE wire of MPSSE to PDN# of FT800 board
	        FT_WriteGPIO(host->hal_handle, (1 << host->hal_config.pdn_pin_no) | 0x3B, (0<<host->hal_config.pdn_pin_no)|0x08);//PDN set to 0 ,connect BLUE wire of MPSSE to PDN# of FT800 board
			
            Ft_Gpu_Hal_Sleep(20);

            digitalWrite(host->hal_config.pdn_pin_no, HIGH);
            Ft_Gpu_Hal_Sleep(20);
            
            digitalWrite(host->hal_config.pdn_pin_no, LOW);
            Ft_Gpu_Hal_Sleep(20);

	}
}
void Ft_Gpu_Hal_WrMemFromFlash(Ft_Gpu_Hal_Context_t *host,uint32_t addr,const const unsigned char *buffer, uint32_t length)
{
	uint32_t SizeTransfered = 0;      

	Ft_Gpu_Hal_StartTransfer(host,FT_GPU_WRITE,addr);

	while (length--) {
            Ft_Gpu_Hal_Transfer8(host,pgm_read_byte_near(buffer));
	    buffer++;
	}




	Ft_Gpu_Hal_EndTransfer(host);
}

void Ft_Gpu_Hal_WrMem(Ft_Gpu_Hal_Context_t *host,uint32_t addr,const uint8_t *buffer, uint32_t length)
{
	uint32_t SizeTransfered = 0;      

	Ft_Gpu_Hal_StartTransfer(host,FT_GPU_WRITE,addr);
	while (length--) {
            Ft_Gpu_Hal_Transfer8(host,*buffer);
	    buffer++;
	}




	Ft_Gpu_Hal_EndTransfer(host);
}


void Ft_Gpu_Hal_RdMem(Ft_Gpu_Hal_Context_t *host,uint32_t addr, uint8_t  *buffer, uint32_t length)
{
	uint32_t SizeTransfered = 0;      

	Ft_Gpu_Hal_StartTransfer(host,FT_GPU_READ,addr);
	while (length--) {
	   *buffer = Ft_Gpu_Hal_Transfer8(host,0);
	   buffer++;
	}



	Ft_Gpu_Hal_EndTransfer(host);
}

/* Helper api for dec to ascii */
int32_t Ft_Gpu_Hal_Dec2Ascii(char *pSrc,int32_t value)
{
	int16_t Length;
	char *pdst,charval;
	int32_t CurrVal = value,tmpval,i;
	char tmparray[16],idx = 0;

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



void Ft_Gpu_Hal_Sleep(uint32_t ms)
{
	delay(ms);
}

int16_t Ft_Gpu_Hal_SetSPI(Ft_Gpu_Hal_Context_t *host,FT_GPU_SPI_NUMCHANNELS_T numchnls,FT_GPU_SPI_NUMDUMMYBYTES numdummy)
{
	uint8_t writebyte = 0;
	/* error check */
	if((numchnls > FT_GPU_SPI_QUAD_CHANNEL) || (numdummy > FT_GPU_SPI_TWODUMMY) || (numdummy < FT_GPU_SPI_ONEDUMMY))
	{
		return -1;//error
	}

	host->spichannel = numchnls;
	writebyte = host->spichannel;
	host->spinumdummy = numdummy;

	if(FT_GPU_SPI_TWODUMMY == host->spinumdummy)
	{
		writebyte |= FT_SPI_TWO_DUMMY_BYTE;
	}
	Ft_Gpu_Hal_Wr8(host,REG_SPI_WIDTH,writebyte);
	/* set the parameters in hal context and also set into ft81x */
	return 0;
}

/* FIFO related apis */
//Init all the parameters of fifo buffer
void Ft_Fifo_Init(Ft_Fifo_t *pFifo,uint32_t StartAddress,uint32_t Length,uint32_t HWReadRegAddress,uint32_t HWWriteRegAddress)
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
void Ft_Fifo_Update(Ft_Gpu_Hal_Context_t *host,Ft_Fifo_t *pFifo)
{
	pFifo->fifo_rp = Ft_Gpu_Hal_Rd32(host,pFifo->HW_Read_Reg);
	//Ft_Gpu_Hal_Wr32(host,pFifo->HW_Write_Reg,pFifo->fifo_wp);
}

//just write and update the write register
uint32_t Ft_Fifo_Write(Ft_Gpu_Hal_Context_t *host,Ft_Fifo_t *pFifo, uint8_t  *buffer,uint32_t NumbytetoWrite)
{
	uint32_t FreeSpace = Ft_Fifo_GetFreeSpace(host,pFifo),TotalBytes = NumbytetoWrite;

	if(NumbytetoWrite > FreeSpace)
	{
		/* update the read pointer and get the free space */
		Ft_Fifo_Update(host,pFifo);
		FreeSpace = Ft_Fifo_GetFreeSpace(host,pFifo);

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

		Ft_Gpu_Hal_WrMem(host,pFifo->fifo_buff + pFifo->fifo_wp,buffer,partialchunk);
		if(secpartialchunk > 0)
		{
			Ft_Gpu_Hal_WrMem(host,pFifo->fifo_buff,buffer + partialchunk,secpartialchunk);
		}
		pFifo->fifo_wp = secpartialchunk;
		//printf("partial chunks %d %d %d %d\n",partialchunk,secpartialchunk,pFifo->fifo_wp,pFifo->fifo_rp);

	}
	else
	{
		Ft_Gpu_Hal_WrMem(host,pFifo->fifo_buff + pFifo->fifo_wp,buffer,TotalBytes);
		pFifo->fifo_wp += TotalBytes;
	}

	/* update the write pointer address in write register */
	Ft_Gpu_Hal_Wr32(host,pFifo->HW_Write_Reg,pFifo->fifo_wp);

	return TotalBytes;
}
//just write one word and update the write register
void Ft_Fifo_Write32(Ft_Gpu_Hal_Context_t *host,Ft_Fifo_t *pFifo,uint32_t WriteWord)
{
	Ft_Fifo_Write(host,pFifo,(uint8_t *)&WriteWord,4);
}
//write and wait for the fifo to be empty. handle cases even if the Numbytes are more than freespace
void Ft_Fifo_WriteWait(Ft_Gpu_Hal_Context_t *host,Ft_Fifo_t *pFifo, uint8_t  *buffer,uint32_t Numbyte)
{
	uint32_t TotalBytes = Numbyte,currchunk = 0,FreeSpace;
	uint8_t *pbuff = buffer;
	/* blocking call, manage to check for the error case and break in case of error */
	while(TotalBytes > 0)
	{
		currchunk = TotalBytes;
		FreeSpace = Ft_Fifo_GetFreeSpace(host,pFifo);
		if(currchunk > FreeSpace)
		{
			currchunk = FreeSpace;
		}

		Ft_Fifo_Write(host,pFifo,pbuff,currchunk);
		pbuff += currchunk;
		TotalBytes -= currchunk;


	}
}

//get the free space in the fifo - make sure the return value is maximum of (LENGTH - 4)
uint32_t Ft_Fifo_GetFreeSpace(Ft_Gpu_Hal_Context_t *host,Ft_Fifo_t *pFifo)
{
	uint32_t FreeSpace = 0;

	Ft_Fifo_Update(host,pFifo);

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



uint32_t Ft_Gpu_CurrentFrequency(Ft_Gpu_Hal_Context_t *host)
{
    uint32_t t0, t1;
    uint32_t addr = REG_CLOCK;
   uint8_t spidata[4];
	int32_t r = 15625;

    t0 = Ft_Gpu_Hal_Rd32(host,REG_CLOCK); /* t0 read */
               
	//may not be precise
	Sleep(15625/1000);
	delayMicroseconds(15625);


    t1 = Ft_Gpu_Hal_Rd32(host,REG_CLOCK); /* t1 read */
    return ((t1 - t0) * 64); /* bitshift 6 places is the same as multiplying 64 */
}

int32_t Ft_Gpu_ClockTrimming(Ft_Gpu_Hal_Context_t *host,int32_t LowFreq)
{
   uint32_t f;
  uint8_t i;

  /* Trim the internal clock by increase the REG_TRIM register till the measured frequency is within the acceptable range.*/
   for (i=0; (i < 31) && ((f= Ft_Gpu_CurrentFrequency(host)) < LowFreq); i++)
   {
	   Ft_Gpu_Hal_Wr8(host,REG_TRIM, i);  /* increase the REG_TRIM register value automatically increases the internal clock */

   }

   Ft_Gpu_Hal_Wr32(host,REG_FREQUENCY,f);  /* Set the final frequency to be used for internal operations */

   return f;
}


