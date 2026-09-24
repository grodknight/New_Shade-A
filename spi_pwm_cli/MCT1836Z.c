/***************************************************************************//**
 * @file
 * @brief handler for the TI MCT8316ZR motor controller 
 *******************************************************************************
 * # License
 * <b>Copyright Crestron
 *******************************************************************************
 * The licensor of this software is Crestron Inc. Your use of this
 * software is governed by 
 *
 ******************************************************************************/
// #include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "spidrv.h"  
#include "spidrv_usart_master_baremetal.h"
#include "sl_spidrv_instances.h"

// #include "sl_sleeptimer.h"
#include "MCT18316Z_REGS.h"

 bool mct8316z_init(SPIDRV_Callback_t callback);
 bool mct8316z_write_reg(uint8_t reg, uint8_t value);
 bool mct8316z_read_reg(uint8_t reg, uint8_t *value);
 void MCT8316_ReadAllRegs(void);
 void mct8316z_UnlockRegs(void);
 void mct8316z_disableSleep(void);
 void setClockwise(bool clockwise);
 bool isClockwise(void);
 bool motorOn(void);
 bool motorOff(void);


//==++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define SPI_HANDLE                  sl_spidrv_usart_exp_handle

extern bool transfer_complete;  // volatile
// static char rx_buffer[APP_BUFFER_SIZE];

static uint16_t build_write_frame(uint8_t reg, uint8_t data); 
static uint16_t build_read_frame(uint8_t reg);
static bool parityCheck_write(uint8_t addr, uint8_t data);
static bool parityCheck_read(uint8_t addr);

volatile unsigned char dataMSB, dataLSB;
volatile uint16_t frame = 0;

uint8_t  mct8316z_ctl_reg_set[] = {  // from Dan 9/23/26
0x03,  // MCT_REG_CTRL1
0x32,  // MCT_REG_CTRL2
0x42,  // MCT_REG_CTRL3
0x90,  // MCT_REG_CTRL4
0x38,  // MCT_REG_CTRL5
0x00,  // MCT_REG_CTRL6
0x10,  // MCT_REG_CTRL7
0x00,  // MCT_REG_CTRL8
0x00,  // MCT_REG_CTRL9
0x00,  // MCT_REG_CTRL10
 };


void write_ctl_reg_set(void)
{
  uint8_t reg;
  for(reg=0; reg < CONTROL_REGISTER_COUNT; reg++)
  {
    mct8316z_write_reg(reg+MCT_REG_CTRL1, mct8316z_ctl_reg_set[reg]);
  }
}

#if 0
const ALL_MCT_CTL_REGS all_mct_ctl_regs_1 = {
		
  .ctrl1.fields.REG_LOCK = 3,       // uint8_t REG_LOCK :3; // Register Lock
  .ctrl1.fields.RESERVED = 0,       // uint8_t RESERVED :5; // RESERVED

  .ctrl2.fields.CLR_FLAG = true,    // bool CLR_FLAG    :1; / Clear Fault
  .ctrl2.fields.PWM_MODE = 0,       // uint8_t PWM_MODE :2; // Device Mode Selection
  .ctrl2.fields.SLEW     = 0,       // uint8_t SLEW     :2; // Slew Rrate Settings
  .ctrl2.fields.SDO_MODE = 0,		    // bool SDO_MODE    :1; // SDO Mode Setting
  .ctrl2.fields.RESERVED = 0,       // uint8_t RESERVED :2; // RESERVED

  .ctrl3.fields.OTW_REP  = true,    // bool OTW_REP     :1; // Overtemperature Waring Reporting Bit
	.ctrl3.fields.OVP_EN	 = true,    // bool OVP_EN      :1; // Overvoltage Enable Bit
	.ctrl3.fields.OVP_SEL  = true, 	  // bool OVP_SEL     :1; // Overvoltage Level Bit
	.ctrl3.fields.PWM_100_DUTY_SEL= 1,// bool PWM_100_DUTY_SEL :1; // frequency of PWM 20 / 40 % duty

  .ctrl4.fields.OCP_MODE  = 0x0,   // uint8_t OCP_MODE  :2; // OCP Fault Options
	.ctrl4.fields.OCP_LVL   = 0,     // bool OCP_LVL      :1; // Overcurrent Level Setting
	.ctrl4.fields.OCP_RETRY = 0,     // bool OCP_RETRY    :1; // OCP Retry Time Settings
	.ctrl4.fields.OCP_DEG	  = 0,     // uint8_t OCP_DEG   :2; // OCP Deglitch Time Settings
	.ctrl4.fields.OCP_CBC   = 0,     //	bool OCP_CBC      :1; // OCP PWM Cycle Operation Bit
	.ctrl4.fields.DRV_OFF   = 0,     // bool DRV_OFF      :1; // Driver Off Bit
  
  .ctrl5.fields.CSA_GAIN   = 0,  // uint8_t CSA_GAIN     :2; // Current Sense Amplifiers Gain Settings
  .ctrl5.fields.EN_ASRnn   = 0,  // uint8_t EN_ASR       :1; // Active Synchronus Rectification Enable Bit
  .ctrl5.fields.EN_AAR     = 0,  // bool EN_AAR          :1; // Active Asynchronus Rectification Enable Bit
  .ctrl5.fields.RESERVED_2 = 0,  // bool RESERVED_2      :1; // RESERVED
  .ctrl5.fields.RESERVED_1 = 0,  // bool RESERVED        :1; // RESERVED
  .ctrl5.fields.ILIM_RECIR = 0,  // bool ILIM_RECIR      :1; // Current Limit Recicurcaltion Settings
  .ctrl5.fields.RESERVED   = 0,  // bool RESERVED        :1; // RESERVED

  .ctrl6.fields.BUCK_DIS   = 0,  // bool BUCK_DIS       :1; // Buck Disable Bit
  .ctrl6.fields.BUCK_SEL   = 0,  // uint8_t BUCK_SEL    :2; // Buck Voltage Selection
  .ctrl6.fields.BUCK_CL    = 0,  // bool BUCK_CL        :1; // Buck Current Limit Setting
  .ctrl6.fields.BUCK_PS_DIS= 0,  // bool BUCK_PS_DIS    :1; // Buck Power Sequencing Disable Bit
  .ctrl6.fields.RESERVED_1 = 0,  // bool RESERVED_1     :1; // RESERVED
  .ctrl6.fields.RESERVED   = 0,  // uint8_t RESERVED    :2; // RESERVED

 .ctrl7.fields.DIR         = 0,   // bool DIR :1; // Direction Bit
 .ctrl7.fields.BRAKE       = 0,   // ool BRAKE :1; // Brake Bit
 .ctrl7.fields.COAST       = 0,   // bool COAST :1; // Coast Bit
 .ctrl7.fields.BRAKE_MODE  = 0,   // bool BRAKE_MODE :1; //Brake Mode Setting
 .ctrl7.fields.HALL_HYS    = 0,   // bool HALL_HYS :1; // Hall Comparator Hysteresis Settings
 .ctrl7.fields.RESERVED    = 0,   // uint8_t RESERVED :3; // RESERVED

  .ctrl8.fields.MTR_LOCK_MODE  = 0,  // uint8_t MTR_LOCK_MODE :2; // Motor Lock Fault Options
  .ctrl8.fields.MTR_LOCK_TDET  = 0,  // uint8_t MTR_LOCK_TDET :2; // Motor Lock Detection Time Settings
  .ctrl8.fields.MTR_LOCK_RETRY = 0,  // bool MTR_LOCK_RETRY   :1; // Motor Lock Retry Time Settings
  .ctrl8.fields.RESERVED       = 0,  // bool RESERVED         :1; // RESERVED
  .ctrl8.fields.FGOUT_SEL      = 0,  // uint8_t FGOUT_SEL     :2; // Electrical Frequency Generation Output Mode Bits

  .ctrl9.fields.ADVANCED_LVL   = 0,  // uint8_t ADVANCED_LVL :3; // Phase Advance Settings
  .ctrl9.fields.RESERVED       = 0,  // uint8_t RESERVED :5;     // RESERVED
  
  .ctrl10.fields.DLY_TARGET    = 0,  // uint8_t DLY_TARGET :4; // Driver Delay Compensation Target
  .ctrl10.fields.DLYCMP_EN     = 0,  // bool DLYCMP_EN     :1; // Driver Delay Compensation enable
  .ctrl10.fields.RESERVED      = 0,  // uint8_t RESERVED   :3; // RESERVED

};
#endif


const uint8_t MCTregMasks[] = {
  STATUS_REGISTER_MASK,
  STATUS_REGISTER_MASK,
  STATUS_REGISTER_MASK,
  CONTROL_REGISTER_1_MASK,
  CONTROL_REGISTER_2_MASK, 
  CONTROL_REGISTER_2_MASK, 
  CONTROL_REGISTER_3_MASK, 
  CONTROL_REGISTER_4_MASK, 
  CONTROL_REGISTER_5_MASK, 
  CONTROL_REGISTER_6_MASK, 
  CONTROL_REGISTER_7_MASK, 
  CONTROL_REGISTER_8_MASK, 
  CONTROL_REGISTER_9_MASK, 
  CONTROL_REGISTER_10_MASK,
};

struct {
	bool  regAccess:1; // print on reg read or write
	bool  b:1; //  add debug flags per line
	bool  c:1; // 
	bool  d:1; // 
	bool  e:1; // 
	bool  f:1; // 
	bool  g:1; // 
	bool  h:1; // 
} mct_debug_flags;


typedef struct
{
  SPIDRV_HandleData_t *spi_handle;

  GPIO_PORT_TypeDef cs_port;
  uint8_t cs_pin;

} mct8316z_t;

bool waitTransferComplete(void)
{
  //  to do: put timeout return false
  while(!transfer_complete);
  
  return true;

}

bool motorOn(void)
{
    IC_Control_Register4 reg4;

    if (!mct8316z_read_reg(MCT_REG_CTRL4,&reg4.data))
      return false;
    
  reg4.fields.DRV_OFF &= ~MCT_DRV_ENABLE_MASK;

  return mct8316z_write_reg(MCT_REG_CTRL4,reg4.data);
}

bool motorOff(void)
{
    IC_Control_Register4 reg4;

    if (!mct8316z_read_reg(MCT_REG_CTRL4,&reg4.data))
      return false;
    
  reg4.fields.DRV_OFF |= MCT_DRV_ENABLE_MASK;

  return mct8316z_write_reg(MCT_REG_CTRL4,reg4.data);
}

void setClockwise(bool clockwise)
{
    IC_Control_Register7 reg7;

    if (!mct8316z_read_reg(MCT_REG_CTRL7,&reg7.data))
      return ;

  if(clockwise)
  {
    printf("Set clockwise\r\n");
    reg7.fields.DIR = CONTROL_REGISTER_7_DIR_CW;
  }
  else {
    printf("Set Counter-clockwise\r\n");
    reg7.fields.DIR = CONTROL_REGISTER_7_DIR_CCW;
  }

  mct8316z_write_reg( MCT_REG_CTRL7,  reg7.data);
}
  
 bool isClockwise(void)
{
  IC_Control_Register7 reg7;

  mct8316z_read_reg ( MCT_REG_CTRL7,  &reg7.data);  
  if(reg7.fields.DIR & 1) {
    printf("Direction Read = %s\r\n","Counter Clockwise");
    return false;
  }
  else {
   printf("Direction Read = %s\r\n","Clockwise" );
   return true;
  }
 
}

void test(void)
{
  MCT8316_ReadAllRegs(); // before tests

#if 0   // Clockwise tests  
  isClockwise();
  setClockwise(MCT_CLOCKWISE);
  isClockwise();
  setClockwise(MCT_COUNTER_CLOCKWISE);
  isClockwise();

  setClockwise(MCT_CLOCKWISE);
  isClockwise(); 
#endif  // Clockwise tests  

  MCT8316_ReadAllRegs(); // after tests

}

static SPIDRV_Callback_t transfer_callback;

bool mct8316z_init(SPIDRV_Callback_t callback)
{
  transfer_callback = callback;

   mct8316z_disableSleep();
   mct8316z_UnlockRegs();
  
   write_ctl_reg_set();  // write default values to all control registers

   #if 0
   // SDO Push/Pull, PWM Sync/Digital
   mct8316z_write_reg( MCT_REG_CTRL2,  0x26); 
   // 20 KHz, 32V
   mct8316z_write_reg( MCT_REG_CTRL3,  0x42); 
#endif

   MCT8316_ReadAllRegs();

   return true;
}

void mct8316z_disableSleep(void)
{
  printf("nSLEEP SET HI\r\n");
  sl_gpio_set_pin(PB6);     // disable sleep
}

static uint8_t valBefore;
static uint8_t valAfter;

 void mct8316z_UnlockRegs(void)
{ 
	mct8316z_read_reg( MCT_REG_CTRL1,&valBefore); 

	mct8316z_write_reg(MCT_REG_CTRL1, 3); 
	mct8316z_read_reg( MCT_REG_CTRL1, &valAfter); 

	printf("\r\nUnlock Regs: MCT_REG_CTRL1\tBefore= %x\tAfter= %x\r\n\n",
		valBefore, valAfter);
}

void MCT8316_ReadAllRegs(void)
{
  uint8_t reg, result;

  for(reg=0; reg < MCT8316_NUM_REGS; 
    mct8316z_read_reg(reg++,&result));

}


bool mct8316z_write_reg(uint8_t reg, uint8_t value) {
   uint16_t frame;
   Ecode_t result;    // Ecode_t
  uint16_t rx;

	if (reg >= MCT8316_NUM_REGS)
	{
		printf("mct8316z_write_reg: Bad Reg number\n\r"); 
		return false;
	}


  frame = build_write_frame(reg, value);
  result =  SPIDRV_MTransfer(SPI_HANDLE, &frame, &rx, 1, transfer_callback);

  waitTransferComplete();


  if(true || mct_debug_flags.regAccess)
    printf("MCT8316 Set Reg %02d = %02xh    Mask= %02xh  Frame=%xh  status= %08lxh\r\n", 
                        reg,       value, MCTregMasks[reg], frame, result );

  return (result == ECODE_EMDRV_SPIDRV_OK);
}

bool mct8316z_read_reg(uint8_t reg, uint8_t *value) 
{
  uint16_t frame;
  uint16_t rx;
  Ecode_t result;    // Ecode_t

   if (reg >= MCT8316_NUM_REGS)
   {
		printf("mct8316z_read_reg: Bad Reg number\n\r"); 
  		return false;
   }

   
    frame = build_read_frame( reg);

    result =  SPIDRV_MTransfer(SPI_HANDLE, &frame, &rx, 1, transfer_callback);

    waitTransferComplete();


  *value = rx & 0xFF;

  if(true || mct_debug_flags.regAccess)
    printf("MCT8316 Get Reg %02d = %02xh    Mask= %02xh  Frame=%xh  status= %08lxh\r\n",
 		                    reg,     *value, MCTregMasks[reg], frame,   result);
 
  return true;
}

static uint16_t build_write_frame(uint8_t reg,
                                  uint8_t data)
{
   volatile unsigned char addr, dat;

   frame = 0;
   dat = data;
   addr = reg;

   addr = ((addr << 1) & 0x7E);        //line up A5-A0 in MSB
   dat = (dat & 0xFF);                 //line up D7-D0 in LSB

   bool parity = parityCheck_write(addr,dat);

   dataLSB = data;                      //LSB = D7-D0
   dataMSB = (addr | parity);           //MSB = W0=0, A5-A0, P

   frame  =  ((uint16_t) dataMSB << 8) | dataLSB;
 //  frame |= MCTregMasks[reg];         // set masked bits

  return frame;
}

static uint16_t build_read_frame(uint8_t reg)
{
    reg = ((reg << 1) & 0x7E);           //line up A5-A0 in MSB

    bool parity = parityCheck_read(reg);

    dataLSB = 0xFF;                     //LSB = dummy data (D7-D0)
    dataMSB = (0x80 | reg | parity);    //MSB = W0=1, A5-A0, P

    frame  =  ((uint16_t) dataMSB << 8) | dataLSB;

    return frame;

}

static bool parityCheck_write(uint8_t addr, uint8_t data)
{
    volatile unsigned char parity = 0, parity_check = 0, i = 0;
    volatile unsigned int parity_word = (addr << 8) | data;
    for (i=0;i<14;i++)
    {
        parity_check = (parity_word >> i) & 1;
        if (parity_check == 1)
        {
            parity ^= 1;
        }
    }
    return parity;
}

static bool parityCheck_read(uint8_t addr)
{
    volatile unsigned char parity = 0, parity_check = 0, i = 0;
    volatile unsigned int parity_word = (addr << 8);
    for (i=0;i<14;i++)
    {
        parity_check = (parity_word >> i) & 1;
        if (parity_check == 1)
        {
            parity ^= 1;
        }
    }
    return parity;
}


