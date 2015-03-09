// -------------------------------------------------------------------------------------------------------------------
//
//  File: main.c -
//
//  Copyright 2011 (c) DecaWave Ltd, Dublin, Ireland.
//
//  All rights reserved.
//
//  Author: Zoran Skrba, March 2012
//
// -------------------------------------------------------------------------------------------------------------------

/* Includes */
#include "compiler.h"
#include "port.h"
#include "instance.h"
#include "deca_types.h"
#include "deca_spi.h"

//yavari
#include "arm_math.h"
#define R 100				//Radius of the circle which source nodes are located on
#define variance 10			//JUST a guess!!! //Variance of measurements
#define threshold 5			//This threshold is used to finish NLLS iterations
#define dimensions 2		//Number of dimensions of the positioning space
#define source_node_num 3	//Number of the source nodes (stationary nodes) in the system
#define start_x 0 			//Initial estimate of position to be used in NLLS algorithm
#define start_y 61
#define SNode1X	61 			//Source nodes position
#define SNode1Y	213.36
#define SNode2X 91.44
#define SNode2Y	0
#define SNode3X 274.32
#define SNode3Y	121.92
#define I2C1_GYRO_ADDRESS7 0x68 //address for GYRO
#define I2C1_ACCEL_ADDRESS7 0x53 //address for ACCELEROMETER
#define I2C_SPEED 50000 //500Khz speed for I2C

double GYRO_X,GYRO_Y,GYRO_Z,ACCEL_X,ACCEL_Y,ACCEL_Z;
double GYRO_X_OFFSET,GYRO_Y_OFFSET,GYRO_Z_OFFSET,ACCEL_X_OFFSET,ACCEL_Y_OFFSET,ACCEL_Z_OFFSET;
uint8 next_anchor=0;
bool first=true;
dwt_rxdiag_t _diag;
arm_matrix_instance_f32 x,p;
float32_t xvec[(dimensions*2) * (1)];
float32_t pvec[(dimensions*2)*(dimensions*2)];
double f1,f2,f3,n,a,c;
bool Range1_received = false;
bool Range2_received = false;
bool Range3_received = false;
bool RangeALL_received = false;
double sx[3];				//position of source nodes
double sy[3];
double k[3];
float tx,ty;			//position of target node
double var = variance;
double d_estimate[3];
float d[3];
extern void usb_run(void);
extern int usb_init(void);
extern void usb_printconfig(void);
extern void send_usbmessage(uint8*, int);

#define SOFTWARE_VER_STRING    "Version 2.25 DG" //

//yavari Determine anchor address here!
int instance_anchaddr = 2; //0 = 0xDECA020000000001; 1 = 0xDECA020000000002; 2 = 0xDECA020000000003
int dr_mode = 0;
//if instance_mode = TAG_TDOA then the device cannot be selected as anchor
int instance_mode = ANCHOR;
//int instance_mode = TAG;
//int instance_mode = TAG_TDOA;
//int instance_mode = LISTENER;
int paused = 0;

double antennaDelay  ;                          // This is system effect on RTD subtracted from local calculation.


char reset_request;

typedef struct
{
    uint8 channel ;
    uint8 prf ;
    uint8 datarate ;
    uint8 preambleCode ;
    uint8 preambleLength ;
    uint8 pacSize ;
    uint8 nsSFD ;
    uint16 sfdTO ;
} chConfig_t ;


//Configuration for DecaRanging Modes (8 default use cases selectable by the switch S1 on EVK)
chConfig_t chConfig[8] ={
                    //mode 1 - S1: 7 off, 6 off, 5 off
                    {
                        2,              // channel
                        DWT_PRF_16M,    // prf
                        DWT_BR_110K,    // datarate
                        3,             // preambleCode
                        DWT_PLEN_1024,  // preambleLength
                        DWT_PAC32,      // pacSize
                        1,       // non-standard SFD
                        (1025 + 64 - 32) //SFD timeout
                    },
                    //mode 2
                    {
                        2,              // channel
                        DWT_PRF_16M,    // prf
                        DWT_BR_6M8,    // datarate
                        3,             // preambleCode
                        DWT_PLEN_128,   // preambleLength
                        DWT_PAC8,       // pacSize
                        0,       // non-standard SFD
                        (129 + 8 - 8) //SFD timeout
                    },
                    //mode 3
                    {
                        2,              // channel
                        DWT_PRF_64M,    // prf
                        DWT_BR_110K,    // datarate
                        9,             // preambleCode
                        DWT_PLEN_1024,  // preambleLength
                        DWT_PAC32,      // pacSize
                        1,       // non-standard SFD
                        (1025 + 64 - 32) //SFD timeout
                    },
                    //mode 4
                    {
                        2,              // channel
                        DWT_PRF_64M,    // prf
                        DWT_BR_6M8,    // datarate
                        9,             // preambleCode
                        DWT_PLEN_128,   // preambleLength
                        DWT_PAC8,       // pacSize
                        0,       // non-standard SFD
                        (129 + 8 - 8) //SFD timeout
                    },
                    //mode 5
                    {
                        5,              // channel
                        DWT_PRF_16M,    // prf
                        DWT_BR_110K,    // datarate
                        3,             // preambleCode
                        DWT_PLEN_1024,  // preambleLength
                        DWT_PAC32,      // pacSize
                        1,       // non-standard SFD
                        (1025 + 64 - 32) //SFD timeout
                    },
                    //mode 6
                    {
                        5,              // channel
                        DWT_PRF_16M,    // prf
                        DWT_BR_6M8,    // datarate
                        3,             // preambleCode
                        DWT_PLEN_128,   // preambleLength
                        DWT_PAC8,       // pacSize
                        0,       // non-standard SFD
                        (129 + 8 - 8) //SFD timeout
                    },
                    //mode 7
                    {
                        5,              // channel
                        DWT_PRF_64M,    // prf
                        DWT_BR_110K,    // datarate
                        9,             // preambleCode
                        DWT_PLEN_1024,  // preambleLength
                        DWT_PAC32,      // pacSize
                        1,       // non-standard SFD
                        (1025 + 64 - 32) //SFD timeout
                    },
                    //mode 8
					{
						5,              // channel
						DWT_PRF_64M,    // prf
						DWT_BR_6M8,    // datarate
						9,             // preambleCode
						DWT_PLEN_128,   // preambleLength
						DWT_PAC8,       // pacSize
						0,       // non-standard SFD
						(129 + 8 - 8) //SFD timeout
					}
};


#if (DR_DISCOVERY == 0)
//Tag address list
uint64 tagAddressList[3] =
{
     0xDECA010000001001,         // First tag
     0xDECA010000000002,         // Second tag
     0xDECA010000000003          // Third tag
} ;

//Anchor address list
uint64 anchorAddressList[ANCHOR_LIST_SIZE] =
{
     0xDECA020000000001 ,       // First anchor
     0xDECA020000000002 ,       // Second anchor
     0xDECA020000000003 ,       // Third anchor
     0xDECA020000000004         // Fourth anchor
} ;

//ToF Report Forwarding Address
uint64 forwardingAddress[1] =
{
     0xDECA030000000001
} ;


// ======================================================
//
//  Configure instance tag/anchor/etc... addresses
//
void addressconfigure(void)
{
    instanceAddressConfig_t ipc ;

    ipc.forwardToFRAddress = forwardingAddress[0];
    ipc.anchorAddress = anchorAddressList[instance_anchaddr];
    ipc.anchorAddressList = anchorAddressList;
    ipc.anchorListSize = ANCHOR_LIST_SIZE ;
    //yavari anchorPollMask determines the number of anchors which should be considered for ranging
//    ipc.anchorPollMask = 0x1; //0x7;              // anchor poll mask
    ipc.anchorPollMask = 0x7;
    ipc.sendReport = 1 ;  //1 => anchor sends TOF report to tag
    //ipc.sendReport = 2 ;  //2 => anchor sends TOF report to listener

    instancesetaddresses(&ipc);
}
#endif

uint32 inittestapplication(void);

// Restart and re-configure
void restartinstance(void)
{
    instance_close() ;                          //shut down instance, PHY, SPI close, etc.

    spi_peripheral_init();                      //re initialise SPI...

    inittestapplication() ;                     //re-initialise instance/device
} // end restartinstance()



int decarangingmode(void)
{
    int mode = 0;

    if(is_switch_on(TA_SW1_5))
    {
        mode = 1;
    }

    if(is_switch_on(TA_SW1_6))
    {
        mode = mode + 2;
    }

    if(is_switch_on(TA_SW1_7))
    {
        mode = mode + 4;
    }

    return mode;
}

uint32 inittestapplication(void)
{
    uint32 devID ;
    instanceConfig_t instConfig;
    int i , result;

    SPI_ConfigFastRate(SPI_BaudRatePrescaler_16);  //max SPI before PLLs configured is ~4M

    i = 10;

    //this is called here to wake up the device (i.e. if it was in sleep mode before the restart)
    devID = instancereaddeviceid() ;
    if(DWT_DEVICE_ID != devID) //if the read of devide ID fails, the DW1000 could be asleep
    {
        port_SPIx_clear_chip_select();  //CS low
        Sleep(1);   //200 us to wake up then waits 5ms for DW1000 XTAL to stabilise
        port_SPIx_set_chip_select();  //CS high
        Sleep(7);
        devID = instancereaddeviceid() ;
        // SPI not working or Unsupported Device ID
        if(DWT_DEVICE_ID != devID)
            return(-1) ;
        //clear the sleep bit - so that after the hard reset below the DW does not go into sleep
        dwt_softreset();
    }

    //reset the DW1000 by driving the RSTn line low
    reset_DW1000();

    result = instance_init() ;
    if (0 > result) return(-1) ; // Some failure has occurred

    SPI_ConfigFastRate(SPI_BaudRatePrescaler_4); //increase SPI to max
    devID = instancereaddeviceid() ;

    if (DWT_DEVICE_ID != devID)   // Means it is NOT MP device
    {
        // SPI not working or Unsupported Device ID
        return(-1) ;
    }

    if(port_IS_TAG_pressed() == 0)
    {
        instance_mode = TAG;
        led_on(LED_PC7);
    }
    else
    {
        instance_mode = ANCHOR;
#if (DR_DISCOVERY == 1)
        led_on(LED_PC6);
#else
        if(instance_anchaddr & 0x1)
            led_on(LED_PC6);

        if(instance_anchaddr & 0x2)
            led_on(LED_PC8);
#endif
    }

    instancesetrole(instance_mode) ;     // Set this instance role


    //yavari I need SW1_2 button for choosing between two side ranging and positioning. We always need the default ranging mode since in the fast
    // mode the range is not reported to the tag.

    //if(is_fastrng_on(0) == S1_SWITCH_ON) //if fast ranging then initialise instance for fast ranging application
    if(false)
    {
      	instance_init_f(instance_mode); //initialise Fast 2WR specific data
      	//when using fast ranging the channel config is either mode 2 or mode 6
      	//default is mode 2
      	dr_mode = decarangingmode();

      	if((dr_mode & 0x1) == 0)
      		dr_mode = 1;
    }
    else
    {
    	instance_init_s(instance_mode);
    dr_mode = decarangingmode();
    }

    instConfig.channelNumber = chConfig[dr_mode].channel ;
    instConfig.preambleCode = chConfig[dr_mode].preambleCode ;
    instConfig.pulseRepFreq = chConfig[dr_mode].prf ;
    instConfig.pacSize = chConfig[dr_mode].pacSize ;
    instConfig.nsSFD = chConfig[dr_mode].nsSFD ;
    instConfig.sfdTO = chConfig[dr_mode].sfdTO ;
    instConfig.dataRate = chConfig[dr_mode].datarate ;
    instConfig.preambleLen = chConfig[dr_mode].preambleLength ;

    instance_config(&instConfig) ;                  // Set operating channel etc

#if (DR_DISCOVERY == 0)
    addressconfigure() ;                            // set up initial payload configuration
#endif
    instancesettagsleepdelay(POLL_SLEEP_DELAY, BLINK_SLEEP_DELAY); //set the Tag sleep time


    //if TA_SW1_2 is on use fast ranging (fast 2wr)
    //if(is_fastrng_on(0) == S1_SWITCH_ON)

    //yavari I need SW1_2 button for choosing between two side ranging and positioning. We always need the default ranging mode since in the fast
    // mode the range is not reported to the tag.
    if(false)
    {
    	//Fast 2WR specific config
    	//configure the delays/timeouts
    	instance_config_f();
    }
    else //use default ranging modes
    {
    	// NOTE: this is the delay between receiving the blink and sending the ranging init message
    	// The anchor ranging init response delay has to match the delay the tag expects
    	// the tag will then use the ranging response delay as specified in the ranging init message
    	// use this to set the long blink response delay (e.g. when ranging with a PC anchor that wants to use the long response times != 150ms)
   	    if(is_switch_on(TA_SW1_8) == S1_SWITCH_ON)
   	    {
   	        instancesetblinkreplydelay(FIXED_LONG_BLINK_RESPONSE_DELAY);
   	    }
   	    else //this is for ARM to ARM tag/anchor (using normal response times 150ms)
   	    {
   	    	instancesetblinkreplydelay(FIXED_REPLY_DELAY);
   	    }

   	    //set the default response delays
   	    instancesetreplydelay(FIXED_REPLY_DELAY, 0);
    }

    return devID;
}
/**
**===========================================================================
**
**  Abstract: main program
**
**===========================================================================
*/

void process_deca_irq(void)
{
    do{

        instance_process_irq(0);

    }while(port_CheckEXT_IRQ() == 1); //while IRS line active (ARM can only do edge sensitive interrupts)

}

void initLCD(void)
{
    uint8 initseq[9] = { 0x39, 0x14, 0x55, 0x6D, 0x78, 0x38 /*0x3C*/, 0x0C, 0x01, 0x06 };
    uint8 command = 0x0;
    int j = 100000;

    writetoLCD( 9, 0,  initseq); //init seq
    while(j--);

    command = 0x2 ;  //return cursor home
    writetoLCD( 1, 0,  &command);
    command = 0x1 ;  //clear screen
    writetoLCD( 1, 0,  &command);
}

/*
 * @brief switch_mask  - bitmask of testing switches (currently 7 switches)
 * 		  switchbuff[] - switch name to test
 * 		  *switch_fn[]() - corresponded to switch test function
**/
#define switch_mask   (0x7F)

const uint8	switchbuf[]={0, TA_SW1_3 , TA_SW1_4 , TA_SW1_5 , TA_SW1_6 , TA_SW1_7 , TA_SW1_8 };
const int (* switch_fn[])(uint16)={ &is_button_low, \
								&is_switch_on, &is_switch_on, &is_switch_on,\
							    &is_switch_on, &is_switch_on, &is_switch_on };

/*
 * @fn test_application_run
 * @brief	test application for production pre-test procedure
**/
void test_application_run(void)
{
    char  dataseq[2][40];
    uint8 j, switchStateOn, switchStateOff;

    switchStateOn=0;
    switchStateOff=0;

    led_on(LED_ALL);	// show all LED OK
    Sleep(1000);

    dataseq[0][0] = 0x1 ;  //clear screen
    writetoLCD( 1, 0, (const uint8 *) &dataseq);
    dataseq[0][0] = 0x2 ;  //return cursor home
    writetoLCD( 1, 0, (const uint8 *) &dataseq);

/* testing SPI to DW1000*/
    writetoLCD( 40, 1, (const uint8 *) "TESTING         ");
    writetoLCD( 40, 1, (const uint8 *) "SPI, U2, S2, S3 ");
    Sleep(1000);

    if(inittestapplication() == (uint32)-1)
    {
        writetoLCD( 40, 1, (const uint8 *) "SPI, U2, S2, S3 ");
        writetoLCD( 40, 1, (const uint8 *) "-- TEST FAILS --");
        while(1); //stop
    }

    writetoLCD( 40, 1, (const uint8 *) "SPI, U2, S2, S3 ");
    writetoLCD( 40, 1, (const uint8 *) "    TEST OK     ");
    Sleep(1000);

/* testing of switch S2 */
    dataseq[0][0] = 0x1 ;  //clear screen
    writetoLCD( 1, 0, (const uint8 *) &dataseq);

    while( (switchStateOn & switchStateOff) != switch_mask )
        {
        memset(&dataseq, ' ', sizeof(dataseq));
        strcpy(&dataseq[0][0], (const char *)"SWITCH");
        strcpy(&dataseq[1][0], (const char *)"toggle");
//switch 7-1
		for (j=0;j<sizeof(switchbuf);j++)
        {
			if( switch_fn[j](switchbuf[j]) ) //execute current switch switch_fn
			{
				dataseq[0][8+j]='O';
				switchStateOn |= 0x01<<j;
				switchStateOff &= ~(0x01<<j);//all switches finaly should be in off state
			}else{
				dataseq[1][8+j]='O';
				switchStateOff |=0x01<<j;
        }
        }

        writetoLCD(40, 1, (const uint8 *) &dataseq[0][0]);
        writetoLCD(40, 1, (const uint8 *) &dataseq[1][0]);
        Sleep(100);
        }

    led_off(LED_ALL);

	writetoLCD( 40, 1, (const uint8 *) "  Preliminary   ");
    writetoLCD( 40, 1, (const uint8 *) "   TEST OKAY    ");

    while(1);
    }

//yavari Initializes anch_add array with zero
void compute_estimate_d()
{
	d_estimate[0] = sqrt(powf(sx[0]-tx,2)+powf(sy[0]-ty,2));
	d_estimate[1] = sqrt(powf(sx[1]-tx,2)+powf(sy[1]-ty,2));
	d_estimate[2] = sqrt(powf(sx[2]-tx,2)+powf(sy[2]-ty,2));
}

float compute_residual()
{
	float d1,d2,d3,resid;
	d1 = sqrt(powf(sx[0]-tx,2)+powf(sy[0]-ty,2));
	d2 = sqrt(powf(sx[1]-tx,2)+powf(sy[1]-ty,2));
	d3 = sqrt(powf(sx[2]-tx,2)+powf(sy[2]-ty,2));
	resid = powf(d[0]-d1,2) + powf(d[1]-d2,2) + powf(d[2]-d3,2);
	return resid;
}

void compute_position_lls()
{
	float delta_sz;
	float resid;
	int i;
	arm_status status;
	float32_t Pvec[source_node_num * source_node_num] = {variance,0,0,0,variance,0,0,0,variance} ;
	float32_t Avec[source_node_num * dimensions];
	float32_t Wvec[source_node_num];
	float32_t deltavec[dimensions];
	float32_t tmp2vec[dimensions*source_node_num];
	float32_t tmp1vec[dimensions * source_node_num];
	float32_t tmp3vec[dimensions* dimensions];
	float32_t tmp4vec[dimensions* dimensions];
	float32_t tmp5vec[dimensions * source_node_num];
	float32_t tmp6vec[dimensions * source_node_num];
	arm_matrix_instance_f32 A;
	arm_matrix_instance_f32 tmp1,tmp2,tmp3,tmp4,tmp5,tmp6;
	arm_matrix_instance_f32 W;
	arm_matrix_instance_f32 delta;
	arm_matrix_instance_f32 P;

	arm_mat_init_f32(&delta,dimensions,1,deltavec);
	arm_mat_init_f32(&P,source_node_num,source_node_num,(float32_t*)Pvec);
	arm_mat_init_f32(&tmp1,dimensions,source_node_num,tmp1vec);
	arm_mat_init_f32(&tmp2,dimensions,source_node_num,tmp2vec);
	arm_mat_init_f32(&tmp3,dimensions,dimensions,tmp3vec);
	arm_mat_init_f32(&tmp4,dimensions,dimensions,tmp4vec);
	arm_mat_init_f32(&tmp5,dimensions,source_node_num,tmp5vec);
	arm_mat_init_f32(&tmp6,dimensions,source_node_num,tmp6vec);

	resid = compute_residual();
	do
	{
		for(i=0 ; i<source_node_num; i++)
		{
			Avec[2*i] = (tx - sx[i])/d[i];
			Avec[(2*i)+1] = (ty - sy[i])/d[i];
		}
		arm_mat_init_f32(&A,source_node_num,dimensions,Avec);
		compute_estimate_d();
		for(i=0 ; i<source_node_num; i++)
			Wvec[i] = d_estimate[i] - d[i];
		arm_mat_init_f32(&W,source_node_num,1,Wvec);
		status = arm_mat_trans_f32(&A,&tmp1);
		status = arm_mat_mult_f32(&tmp1,&P,&tmp2);
		status = arm_mat_mult_f32(&tmp2,&A,&tmp3);
		status = arm_mat_inverse_f32(&tmp3,&tmp4);
		status = arm_mat_trans_f32(&A,&tmp1);
		status = arm_mat_mult_f32(&tmp4,&tmp1,&tmp5);
		status = arm_mat_mult_f32(&tmp5,&P,&tmp6);
		status = arm_mat_mult_f32(&tmp6,&W,&delta);
		// delta = (A^T * P * A)^-1 * A^T * P * W
		tx = tx - delta.pData[0];
		ty = ty - delta.pData[1];
		delta_sz = sqrt(powf(delta.pData[0],2)+powf(delta.pData[1],2));
		resid = compute_residual();
	} while (delta_sz >threshold);

}

int get_closest(float dis[])
{
	int closest;
	if (dis[0]<dis[1])
	{
		closest = 0;
		if (dis[2]<dis[0])
			closest = 2;
	}
	else
	{
		closest = 1;
		if (dis[2]<dis[1])
			closest = 2;
	}
	return closest;
}

void compute_position_mle()
{
	float delta_sz;
	float resid;
	int i,close_node;;
	arm_status status;
	float32_t Cvec[(source_node_num-1) * (source_node_num-1)];
	float32_t Avec[(source_node_num-1) * dimensions];
	float32_t Pvec[(source_node_num-1)];
	float32_t deltavec[dimensions];
	float32_t tmp2vec[dimensions*(source_node_num-1)];
	float32_t tmp1vec[dimensions * (source_node_num-1)];
	float32_t tmp3vec[dimensions* dimensions];
	float32_t tmp4vec[dimensions* dimensions];
	float32_t tmp5vec[dimensions * (source_node_num-1)];
	float32_t tmp6vec[dimensions * (source_node_num-1)];
	arm_matrix_instance_f32 A;
	arm_matrix_instance_f32 tmp1,tmp2,tmp3,tmp4,tmp5,tmp6;
	arm_matrix_instance_f32 C;
	arm_matrix_instance_f32 delta;
	arm_matrix_instance_f32 P;

	arm_mat_init_f32(&delta,dimensions,1,deltavec);
	arm_mat_init_f32(&tmp1,dimensions,(source_node_num-1),tmp1vec);
	arm_mat_init_f32(&tmp2,dimensions,(source_node_num-1),tmp2vec);
	arm_mat_init_f32(&tmp3,dimensions,dimensions,tmp3vec);
	arm_mat_init_f32(&tmp4,dimensions,dimensions,tmp4vec);
	arm_mat_init_f32(&tmp5,dimensions,(source_node_num-1),tmp5vec);
	arm_mat_init_f32(&tmp6,dimensions,(source_node_num-1),tmp6vec);

	close_node = get_closest(d);




	switch (close_node) {
		case 0:
			Cvec[0] = 4*pow(d[0],2)*var + 4*pow(var,2) + 4*var*pow(d[1],2);
			Cvec[1] = 4*pow(d[0],2)*var + 2*pow(var,2);
			Cvec[2] = 4*pow(d[0],2)*var + 2*pow(var,2);
			Cvec[3] = 4*pow(d[0],2)*var + 4*pow(var,2) + 4*var*pow(d[2],2);

			Avec[0] = 2*(sx[1]-sx[0]);
			Avec[1] = 2*(sy[1]-sy[0]);
			Avec[2] = 2*(sx[2]-sx[0]);
			Avec[3] = 2*(sy[2]-sy[0]);

			Pvec[0] = pow(d[0],2)-pow(d[1],2)-k[0]+k[1];
			Pvec[1] = pow(d[0],2)-pow(d[2],2)-k[0]+k[2];
			break;
		case 1:
			Cvec[0] = 4*pow(d[1],2)*var + 4*pow(var,2) + 4*var*pow(d[0],2);
			Cvec[1] = 4*pow(d[1],2)*var + 2*pow(var,2);
			Cvec[2] = 4*pow(d[1],2)*var + 2*pow(var,2);
			Cvec[3] = 4*pow(d[1],2)*var + 4*pow(var,2) + 4*var*pow(d[2],2);

			Avec[0] = 2*(sx[0]-sx[1]);
			Avec[1] = 2*(sy[0]-sy[1]);
			Avec[2] = 2*(sx[2]-sx[1]);
			Avec[3] = 2*(sy[2]-sy[1]);

			Pvec[0] = pow(d[1],2)-pow(d[0],2)-k[1]+k[0];
			Pvec[1] = pow(d[1],2)-pow(d[2],2)-k[1]+k[2];
			break;
		case 2:
			Cvec[0] = 4*pow(d[2],2)*var + 4*pow(var,2) + 4*var*pow(d[0],2);
			Cvec[1] = 4*pow(d[2],2)*var + 2*pow(var,2);
			Cvec[2] = 4*pow(d[2],2)*var + 2*pow(var,2);
			Cvec[3] = 4*pow(d[2],2)*var + 4*pow(var,2) + 4*var*pow(d[1],2);

			Avec[0] = 2*(sx[0]-sx[2]);
			Avec[1] = 2*(sy[0]-sy[2]);
			Avec[2] = 2*(sx[1]-sx[2]);
			Avec[3] = 2*(sy[1]-sy[2]);

			Pvec[0] = pow(d[2],2)-pow(d[0],2)-k[2]+k[0];
			Pvec[1] = pow(d[2],2)-pow(d[1],2)-k[2]+k[1];
			break;

		}

		arm_mat_init_f32(&A,(source_node_num-1),dimensions,Avec);
		arm_mat_init_f32(&P,(source_node_num-1),1,Pvec);
		arm_mat_init_f32(&C,(source_node_num-1),(source_node_num-1),(float32_t*)Pvec);

		status = arm_mat_trans_f32(&A,&tmp1);
		status = arm_mat_mult_f32(&tmp1,&C,&tmp2);
		status = arm_mat_mult_f32(&tmp2,&A,&tmp3);
		status = arm_mat_inverse_f32(&tmp3,&tmp4);
		status = arm_mat_trans_f32(&A,&tmp1);
		status = arm_mat_mult_f32(&tmp4,&tmp1,&tmp5);
		status = arm_mat_mult_f32(&tmp5,&C,&tmp6);
		status = arm_mat_mult_f32(&tmp6,&P,&delta);
		// delta = (A^T * C * A)^-1 * A^T * C * P
		tx = delta.pData[0];
		ty = delta.pData[1];


}

void kalman_filter()
{
	double delay = POLL_SLEEP_DELAY / 1000;
	int i,j;
	arm_status status;
	arm_matrix_instance_f32 x_est,z_est,v,p_est,s,w,h,r,f,z,q,tmp1,tmp2,tmp3,tmp4,tmp5,tmp6,tmp7,tmp8,tmp9,tmp10,tmp11,tmp12;
	float32_t x_estvec[dimensions*2];
	float32_t tmp1vec[dimensions*2 *dimensions*2];
	float32_t tmp2vec[dimensions*2 *dimensions*2];
	float32_t tmp3vec[dimensions*2 *dimensions*2];
	float32_t tmp4vec[dimensions*2 *dimensions];
	float32_t tmp5vec[dimensions*dimensions*2];
	float32_t tmp6vec[dimensions*dimensions];
	float32_t tmp7vec[dimensions*dimensions];
	float32_t tmp8vec[dimensions*2*dimensions];
	float32_t tmp9vec[2*dimensions];
	float32_t tmp10vec[dimensions*dimensions*2];
	float32_t tmp11vec[dimensions*2*dimensions];
	float32_t tmp12vec[dimensions*2*dimensions*2];

	float32_t z_estvec[dimensions];
	float32_t zvec[dimensions];
	float32_t vvec[dimensions];
	float32_t p_estvec[dimensions*2 *dimensions*2];
	float32_t svec[dimensions*dimensions];
	float32_t hvec[dimensions*dimensions*2];
	float32_t rvec[dimensions*dimensions];
	float32_t fvec[dimensions*2*dimensions*2];
	float32_t qvec[dimensions*2*dimensions*2];
	float32_t wvec[dimensions*2*dimensions];

	arm_mat_init_f32(&tmp1,dimensions*2,dimensions*2,tmp1vec);
	arm_mat_init_f32(&tmp2,dimensions*2,dimensions*2,tmp2vec);
	arm_mat_init_f32(&tmp3,dimensions*2,dimensions*2,tmp3vec);
	arm_mat_init_f32(&tmp4,dimensions*2,dimensions,tmp4vec);
	arm_mat_init_f32(&tmp5,dimensions,dimensions*2,tmp5vec);
	arm_mat_init_f32(&tmp6,dimensions,dimensions,tmp6vec);
	arm_mat_init_f32(&tmp7,dimensions,dimensions,tmp7vec);
	arm_mat_init_f32(&tmp8,dimensions*2,dimensions,tmp8vec);
	arm_mat_init_f32(&tmp9,dimensions*2,1,tmp9vec);
	arm_mat_init_f32(&tmp10,dimensions,dimensions*2,tmp10vec);
	arm_mat_init_f32(&tmp11,dimensions*2,dimensions,tmp11vec);
	arm_mat_init_f32(&tmp12,dimensions*2,dimensions*2,tmp12vec);
	arm_mat_init_f32(&x_est,dimensions*2,1,x_estvec);
	arm_mat_init_f32(&z_est,dimensions,1,z_estvec);
	arm_mat_init_f32(&v,dimensions,1,vvec);
	arm_mat_init_f32(&p_est,dimensions*2,dimensions*2,p_estvec);
	arm_mat_init_f32(&s,dimensions,dimensions,svec);
	arm_mat_init_f32(&w,dimensions*2,dimensions,wvec);

	for(i=0;i<dimensions*2;i++)
		for(j=0;j<dimensions;j++)
			if(i==j)
				hvec[i+(j*dimensions*2)] = 1;
			else
				hvec[i+(j*dimensions*2)] = 0;
	arm_mat_init_f32(&h,dimensions,dimensions*2,hvec);

	for(i=0;i<dimensions;i++)
		for(j=0;j<dimensions;j++)
			if(i==j)
				rvec[i+(j*dimensions)] = var/100;
			else
				rvec[i+(j*dimensions)] = 0;
	arm_mat_init_f32(&r,dimensions,dimensions,rvec);
	for(i=0;i<dimensions*2;i++)
		for(j=0;j<dimensions*2;j++)
			if(i==j)
				fvec[i+(j*dimensions*2)] = 1;
			else
				fvec[i+(j*dimensions*2)] = 0;
	fvec[2]=(delay);
	fvec[7]=(delay);
	arm_mat_init_f32(&f,dimensions*2,dimensions*2,fvec);
	for(i=0;i<dimensions*2;i++)
		for(j=0;j<dimensions*2;j++)
			qvec[i+(j*dimensions*2)] = 0;
	qvec[10]= pow(delay,2);
	qvec[15]= pow(delay,2);
	arm_mat_init_f32(&q,dimensions*2,dimensions*2,qvec);

	zvec[0] = tx / 100;
	zvec[1] = ty / 100;
	arm_mat_init_f32(&z,dimensions,1,zvec);

	status = arm_mat_mult_f32(&f,&x,&x_est);			//x (k +1|k )= F(k ) x (k| k)
	status = arm_mat_mult_f32(&h,&x_est,&z_est);		//z(k +1|k )= H(k )x (k +1|k )
	status = arm_mat_sub_f32(&z,&z_est,&v);			//v(k +1)= z(k +1) - z(k +1|k )
	status = arm_mat_mult_f32(&f,&p,&tmp1);			//P(k +1|k ) = F(k)P(k|k )F(k )'+Q(k )
	status = arm_mat_trans_f32(&f,&tmp2);
	status = arm_mat_mult_f32(&tmp1,&tmp2,&tmp3);
	status = arm_mat_add_f32(&tmp3,&q,&p_est);
	status = arm_mat_trans_f32(&h,&tmp4);			//S(k +1) = H(k +1)P(k +1|k)H(k +1)'+R(k +1)
	status = arm_mat_mult_f32(&h,&p_est,&tmp5);
	status = arm_mat_mult_f32(&tmp5,&tmp4,&tmp6);
	status = arm_mat_add_f32(&tmp6,&r,&s);
	status = arm_mat_inverse_f32(&s,&tmp7);			//W(k +1) = P(k +1|k )H(k +1)' S(k +1)^-1
	status = arm_mat_trans_f32(&h,&tmp4);
	status = arm_mat_mult_f32(&p_est,&tmp4,&tmp8);
	status = arm_mat_mult_f32(&tmp8,&tmp7,&w);
	status = arm_mat_mult_f32(&w,&v,&tmp9);			//x (k +1|k +1)= x (k +1|k ) +W(k +1)v(k +1)
	status = arm_mat_add_f32(&x_est,&tmp9,&x);
	status = arm_mat_trans_f32(&w,&tmp10);			//P(k +1|k +1) = P(k +1|k ) -W(k +1)S(k +1)W(k +1)'
	status = arm_mat_mult_f32(&w,&s,&tmp11);
	status = arm_mat_mult_f32(&tmp11,&tmp10,&tmp12);
	status = arm_mat_sub_f32(&p_est,&tmp12,&p);

	tx = x.pData[0]*100;
	ty = x.pData[1]*100;
}

void init()
{
	int i;
	next_anchor=0;
	tx = start_x;
	ty = start_y;
//	sx[0]=R;
//	sy[0]=2*R;
//	sx[1]=0.134 * R;
//	sy[1]=0.5 * R;
//	sx[2]=1.866 * R;
//	sy[2]=0.5 * R;
	sx[0]= SNode1X;
	sx[1]= SNode2X;
	sx[2]= SNode3X;
	sy[0]= SNode1Y;
	sy[1]= SNode2Y;
	sy[2]= SNode3Y;


	for(i=0; i<3; i++)
			k[i] = pow(sx[i],2)+pow(sy[i],2);
}

void init_kalman()
{
	int i,j;
	xvec[0] = tx / 100;
	xvec[1] = ty / 100;
	xvec[2] = 0;
	xvec[3] = 0;
	arm_mat_init_f32(&x,dimensions*2,1,xvec);
	for(i=0;i<dimensions*2;i++)
		for(j=0;j<dimensions*2;j++)
			if(i==j)
				pvec[i+(j*dimensions*2)] = 1;
			else
				pvec[i+(j*dimensions*2)] = 0;
	arm_mat_init_f32(&p,dimensions*2,dimensions*2,pvec);
}

//yavari Updates the distance array (d) with new measurement
void update_d(float dist, uint64 tag)
{
	if(0xDECA020000000001==tag)
		{
			d[0]=dist * 100;
			Range1_received = true;
			next_anchor = 1;
		}
	else if(0xDECA020000000002==tag)
	{
		d[1]=dist * 100;
		Range2_received = true;
		next_anchor = 2;
	}
	else if(0xDECA020000000003==tag)
	{
		d[2]=dist * 100;
		Range3_received = true;
		next_anchor = 0;
	}
	if(Range1_received && Range2_received && Range3_received)
		RangeALL_received = true;
}

double compute_FPPower()
{
	double fpp;
	dwt_readdignostics(&_diag);
	f1 = _diag.firstPathAmp1;
	f2 = _diag.firstPathAmp2;
	f3 = _diag.firstPathAmp3;
	n = _diag.rxPreamCount;
	if((dr_mode==0)||(dr_mode==1)||(dr_mode==4)||(dr_mode==5))
		a = 115.72;
	else
		a = 121.74;
	fpp = 10*log10((pow(f1,2)  + pow(f2,2) + pow(f3,2))/pow(n,2)) - a;
	//fpp = (pow(f1,2)  + pow(f2,2) + pow(f3,2))/pow(n,2);

	return fpp;

}

double compute_RP()
{
	double rp;
	dwt_readdignostics(&_diag);
	c = _diag.maxGrowthCIR;
	n = _diag.rxPreamCount;
	if((dr_mode==0)||(dr_mode==1)||(dr_mode==4)||(dr_mode==5))
			a = 115.72;
		else
			a = 121.74;
	rp = 10*log10(c * pow(2,17)/pow(n,2)) - a;
	return rp;
}




void I2C_Setup(void)
{

    GPIO_InitTypeDef  GPIO_InitStructure;
    I2C_InitTypeDef  I2C_InitStructure;

    GPIO_StructInit(&GPIO_InitStructure);
    /* Configure I2C1 */
    I2C_DeInit(I2C1);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); // Enable GPIOB Clock
    /* I2C1 clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

    /* Enable AFIO clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    /* I2C1 SDA and SCL configuration */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    /*SCL is pin06 and SDA is pin 07 for I2C1*/

    /* I2C1 Reset */

    RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, ENABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, DISABLE);



    I2C_StructInit(&I2C_InitStructure);
    /* I2C1 configuration */
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = 0x00;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = I2C_SPEED ;

    I2C_Init(I2C1, &I2C_InitStructure);

    /*enable I2C*/
    I2C_Cmd(I2C1,ENABLE);





}

void init_ACCEL(void)
{
	//write 0x09 to to 0x31 of Accelerometer

	/*!< Clear the I2C1 AF flag */
	I2C_ClearFlag(I2C1, I2C_FLAG_AF);

	/*!< Enable I2C1 acknowledgement if it is already disabled by other function */
	I2C_AcknowledgeConfig(I2C1, ENABLE);


	/* initiate start sequence */
    I2C_GenerateSTART(I2C1, ENABLE);
    /* check start bit flag */
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
    /*send write command to chip*/
    I2C_Send7bitAddress(I2C1, I2C1_ACCEL_ADDRESS7<<1, I2C_Direction_Transmitter);
    /*check master is now in Tx mode*/
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
    /*mode register address*/
    I2C_SendData(I2C1, 0x31);
    /*wait for byte send to complete*/
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    /*clear bits*/
    //I2C_SendData(I2C1, 0x09);
    I2C_SendData(I2C1, 0x0B);
    /*wait for byte send to complete*/
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    /*generate stop*/
    I2C_GenerateSTOP(I2C1, ENABLE);
    /*stop bit flag*/
    while(I2C_GetFlagStatus(I2C1, I2C_FLAG_STOPF));

    //write 0x08 to to 0x2D of Accelerometer

	/* initiate start sequence */
	I2C_GenerateSTART(I2C1, ENABLE);
	/* check start bit flag */
	while(!I2C_GetFlagStatus(I2C1, I2C_FLAG_SB));
	/*send write command to chip*/
	I2C_Send7bitAddress(I2C1, I2C1_ACCEL_ADDRESS7<<1, I2C_Direction_Transmitter);
	/*check master is now in Tx mode*/
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	/*mode register address*/
	I2C_SendData(I2C1, 0x2D);
	/*wait for byte send to complete*/
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	/*clear bits*/
	I2C_SendData(I2C1, 0x08);
	/*wait for byte send to complete*/
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	/*generate stop*/
	I2C_GenerateSTOP(I2C1, ENABLE);
	/*stop bit flag*/
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_STOPF));

}


void init_GYRO(void)
{
	//write 0x09 to 0x15

	/*!< Clear the I2C1 AF flag */
	I2C_ClearFlag(I2C1, I2C_FLAG_AF);

	/*!< Enable I2C1 acknowledgement if it is already disabled by other function */
	I2C_AcknowledgeConfig(I2C1, ENABLE);

    /* initiate start sequence */
    I2C_GenerateSTART(I2C1, ENABLE);
    /* check start bit flag */
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
    //while(!I2C_GetFlagStatus(I2C1, I2C_FLAG_SB));
    /*send write command to chip*/
    I2C_Send7bitAddress(I2C1, I2C1_GYRO_ADDRESS7<<1, I2C_Direction_Transmitter);
    /*check master is now in Tx mode*/
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
    /*mode register address*/
    I2C_SendData(I2C1, 0x15);
    /*wait for byte send to complete*/
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    /*clear bits*/
    I2C_SendData(I2C1, 0x09);
    /*wait for byte send to complete*/
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    /*generate stop*/
    I2C_GenerateSTOP(I2C1, ENABLE);
    /*stop bit flag*/
    while(I2C_GetFlagStatus(I2C1, I2C_FLAG_STOPF));

    //write 0x1a to 0x16

    /* initiate start sequence */
    I2C_GenerateSTART(I2C1, ENABLE);
    /* check start bit flag */
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
    /*send write command to chip*/
    I2C_Send7bitAddress(I2C1, I2C1_GYRO_ADDRESS7<<1, I2C_Direction_Transmitter);
    /*check master is now in Tx mode*/
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
    /*mode register address*/
    I2C_SendData(I2C1, 0x16);
    /*wait for byte send to complete*/
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    /*clear bits*/
    I2C_SendData(I2C1, 0x1a);//****************OR***************************I2C_SendData(I2C1, 0x19);
    /*wait for byte send to complete*/
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    /*generate stop*/
    I2C_GenerateSTOP(I2C1, ENABLE);
    /*stop bit flag*/
    while(I2C_GetFlagStatus(I2C1, I2C_FLAG_STOPF));

}

void Receive_GYRO()
{
    u8 XMSB,XLSB,YMSB,YLSB,ZMSB,ZLSB; /* variables to store temporary values in */
    u8 Address = 0x68;
    u8 Register = 0x1d;

    /*left align address*/
    Address = Address<<1;
    /*re-enable ACK bit incase it was disabled last call*/
    I2C_AcknowledgeConfig(I2C1, ENABLE);
    /* Test on BUSY Flag */
    while (I2C_GetFlagStatus(I2C1,I2C_FLAG_BUSY));
    /* Enable the I2C peripheral */
/*======================================================*/
    I2C_GenerateSTART(I2C1, ENABLE);
    /* Test on start flag */
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
    /* Send device address for write */
    I2C_Send7bitAddress(I2C1, Address, I2C_Direction_Transmitter);
    /* Test on master Flag */
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
    /* Send the device's internal address to write to */
    I2C_SendData(I2C1,Register);
    /* Test on TXE FLag (data sent) */
    while (!I2C_GetFlagStatus(I2C1,I2C_FLAG_TXE));
/*=====================================================*/
      /* Send START condition a second time (Re-Start) */
    I2C_GenerateSTART(I2C1, ENABLE);
    /* Test start flag */
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
    /* Send address for read */
    I2C_Send7bitAddress(I2C1, Address, I2C_Direction_Receiver);
    /* Test Receive mode Flag */
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
    /* load in all 6 registers */
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
    XMSB = I2C_ReceiveData(I2C1);
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
    XLSB = I2C_ReceiveData(I2C1);
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
    YMSB = I2C_ReceiveData(I2C1);
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
    YLSB = I2C_ReceiveData(I2C1);
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
    ZMSB = I2C_ReceiveData(I2C1);
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
    ZLSB = I2C_ReceiveData(I2C1);

    /*enable NACK bit */
    I2C_NACKPositionConfig(I2C1, I2C_NACKPosition_Current);
    I2C_AcknowledgeConfig(I2C1, DISABLE);

    /* Send STOP Condition */
    I2C_GenerateSTOP(I2C1, ENABLE);
    while(I2C_GetFlagStatus(I2C1, I2C_FLAG_STOPF)); // stop bit flag

    /*sort into 3 global variables*/
    GYRO_X = ((XMSB<<8) | XLSB);
    GYRO_X = GYRO_X / 14.375;
    GYRO_Y = ((YMSB<<8) | YLSB);
    GYRO_Y = GYRO_Y / 14.375;
    GYRO_Z = ((ZMSB<<8) | ZLSB);
    GYRO_Z = GYRO_Z / 14.375;
}

bool test_ACCEL()
{
    u8 buf; /* variables to store temporary values in */
    u8 Address = 0x53;
    u8 Register = 0x00;

    /*left align address*/
    Address = Address<<1;
    /*re-enable ACK bit incase it was disabled last call*/
    I2C_AcknowledgeConfig(I2C1, ENABLE);
    /* Test on BUSY Flag */
    while (I2C_GetFlagStatus(I2C1,I2C_FLAG_BUSY));
    /* Enable the I2C peripheral */
/*======================================================*/
    I2C_GenerateSTART(I2C1, ENABLE);
    /* Test on start flag */
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
    /* Send device address for write */
    I2C_Send7bitAddress(I2C1, Address, I2C_Direction_Transmitter);
    /* Test on master Flag */
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
    /* Send the device's internal address to write to */
    I2C_SendData(I2C1,Register);
    /* Test on TXE FLag (data sent) */
    while (!I2C_GetFlagStatus(I2C1,I2C_FLAG_TXE));
/*=====================================================*/
      /* Send START condition a second time (Re-Start) */
    I2C_GenerateSTART(I2C1, ENABLE);
    /* Test start flag */
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
    /* Send address for read */
    I2C_Send7bitAddress(I2C1, Address, I2C_Direction_Receiver);
    /* Test Receive mode Flag */
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
    /* load in all 6 registers */
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
    buf = I2C_ReceiveData(I2C1);

    /*enable NACK bit */
    I2C_NACKPositionConfig(I2C1, I2C_NACKPosition_Current);
    I2C_AcknowledgeConfig(I2C1, DISABLE);

    /* Send STOP Condition */
    I2C_GenerateSTOP(I2C1, ENABLE);
    while(I2C_GetFlagStatus(I2C1, I2C_FLAG_STOPF)); // stop bit flag

    if(buf!=229)
    	return false;
    return true;
}

void Receive_ACCEL()
{
    u8 XMSB,XLSB,YMSB,YLSB,ZMSB,ZLSB; /* variables to store temporary values in */
    s16 _ACCEL_X,_ACCEL_Y,_ACCEL_Z;
    u8 Address = 0x53;
    u8 Register = 0x32;

    /*left align address*/
    Address = Address<<1;
    /*re-enable ACK bit incase it was disabled last call*/
    I2C_AcknowledgeConfig(I2C1, ENABLE);
    /* Test on BUSY Flag */
    while (I2C_GetFlagStatus(I2C1,I2C_FLAG_BUSY));
    /* Enable the I2C peripheral */
/*======================================================*/
    I2C_GenerateSTART(I2C1, ENABLE);
    /* Test on start flag */
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
    /* Send device address for write */
    I2C_Send7bitAddress(I2C1, Address, I2C_Direction_Transmitter);
    /* Test on master Flag */
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
    /* Send the device's internal address to write to */
    I2C_SendData(I2C1,Register);
    /* Test on TXE FLag (data sent) */
    while (!I2C_GetFlagStatus(I2C1,I2C_FLAG_TXE));
/*=====================================================*/
      /* Send START condition a second time (Re-Start) */
    I2C_GenerateSTART(I2C1, ENABLE);
    /* Test start flag */
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
    /* Send address for read */
    I2C_Send7bitAddress(I2C1, Address, I2C_Direction_Receiver);
    /* Test Receive mode Flag */
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
    /* load in all 6 registers */
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
    XLSB = I2C_ReceiveData(I2C1);
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
    XMSB = I2C_ReceiveData(I2C1);
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
    YLSB = I2C_ReceiveData(I2C1);
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
    YMSB = I2C_ReceiveData(I2C1);
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
    ZLSB = I2C_ReceiveData(I2C1);
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
    ZMSB = I2C_ReceiveData(I2C1);

    /*enable NACK bit */
    I2C_NACKPositionConfig(I2C1, I2C_NACKPosition_Current);
    I2C_AcknowledgeConfig(I2C1, DISABLE);

    /* Send STOP Condition */
    I2C_GenerateSTOP(I2C1, ENABLE);
    while(I2C_GetFlagStatus(I2C1, I2C_FLAG_STOPF)); // stop bit flag

    /*sort into 3 global variables*/
    _ACCEL_X = ((XMSB<<8) | XLSB);
    ACCEL_X = _ACCEL_X;
//    ACCEL_X = ACCEL_X / 25.6;
    _ACCEL_Y = ((YMSB<<8) | YLSB);
    ACCEL_Y = _ACCEL_Y;
//    ACCEL_Y = ACCEL_Y / 25.6;
    _ACCEL_Z = ((ZMSB<<8) | ZLSB);
    ACCEL_Z = _ACCEL_Z;
//    ACCEL_Z = ACCEL_Z / 25.6;
}

void calib()
{
	uint8 buf[40];
	led_on(LED_PC6);
	led_on(LED_PC9);
	int i=20;
	    while(i--)
	    {
	    	if(instance_mode == TAG)
	    	{
	    		if(i==19)
	    		{
	    			GYRO_X_OFFSET = 0;
	    			GYRO_Y_OFFSET = 0;
	    			GYRO_Z_OFFSET = 0;
	    			ACCEL_X_OFFSET = 0;
	    			ACCEL_Y_OFFSET = 0;
	    			ACCEL_Z_OFFSET = 0;
	    		}
//	    		Receive_GYRO();
	    		Sleep(10);
	    		Receive_ACCEL();
	    		sprintf((char*)&buf[0], "Calib : %4.1f %4.1f %4.1f", ACCEL_X,ACCEL_Y,ACCEL_Z);
	    		send_usbmessage(&buf[0], 35);

	    		Sleep(10);
	    		GYRO_X_OFFSET += GYRO_X;
	    		GYRO_Y_OFFSET += GYRO_Y;
	    		GYRO_Z_OFFSET += GYRO_Z;
	    		ACCEL_X_OFFSET += ACCEL_X;
	    		ACCEL_Y_OFFSET += ACCEL_Y;
	    		ACCEL_Z_OFFSET += ACCEL_Z;
	    		if(i==0)
	    		{
	    			GYRO_X_OFFSET /= 20;
	    			GYRO_Y_OFFSET /= 20;
	    			GYRO_Z_OFFSET /= 20;
	    			ACCEL_X_OFFSET /= 20;
	    			ACCEL_Y_OFFSET /= 20;
	    			ACCEL_Z_OFFSET /= 20;
	    			ACCEL_Z_OFFSET += 250;
	    			sprintf((char*)&buf[0], "OFFSET : %4.1f %4.1f %4.1f", ACCEL_X_OFFSET,ACCEL_Y_OFFSET,ACCEL_Z_OFFSET);
	    			send_usbmessage(&buf[0], 35);
	    		}


	    	}
	    	else{
	    		if (i & 1) led_off(LED_ALL);
	    		else    led_on(LED_ALL);

	    		Sleep(300);
	    	}
	    }
	    led_off(LED_ALL);

}

/*
 * @fn 		main()
 * @brief	main entry point
**/
int main(void)
{
	int i = 0;
	int toggle = 1;
	int ranging = 0;
    uint8 dataseq[40];
	double range_result = 0;
	double avg_result = 0;
	uint8 dataseq1[40];
	//yavari
	double FPP,RP;
	Range1_received = false;
	Range2_received = false;
	Range3_received = false;
	RangeALL_received = false;
	init();
    led_off(LED_ALL); //turn off all the LEDs

    peripherals_init();

    spi_peripheral_init();

    Sleep(1000); //wait for LCD to power on

    initLCD();

    memset(dataseq, 40, 0);
    memcpy(dataseq, (const uint8 *) "DECAWAVE        ", 16);
    writetoLCD( 40, 1, dataseq); //send some data
    memcpy(dataseq, (const uint8 *) SOFTWARE_VER_STRING, 16); // Also set at line #26 (Should make this from single value !!!)
    writetoLCD( 16, 1, dataseq); //send some data

    Sleep(1000);
#ifdef USB_SUPPORT
	// enable the USB functionality
	usb_init();
	Sleep(1000);





	#endif
	

    port_DisableEXT_IRQ(); //disable ScenSor IRQ until we configure the device

    //test EVB1000 - used in EVK1000 production
#if 1
    if((is_button_low(0) == S1_SWITCH_ON) && (is_switch_on(TA_SW1_8) == S1_SWITCH_ON)) //using BOOT1 switch for test
    {
        test_application_run(); //does not return....
    }
    else
#endif
    if(is_switch_on(TA_SW1_3) == S1_SWITCH_OFF)
    {
        int j = 1000000;
        uint8 command;

        memset(dataseq, 0, 40);

        while(j--);
        //command = 0x1 ;  //clear screen
        //writetoLCD( 1, 0,  &command);
        command = 0x2 ;  //return cursor home
        writetoLCD( 1, 0,  &command);

        memcpy(dataseq, (const uint8 *) "DECAWAVE   ", 12);
        writetoLCD( 40, 1, dataseq); //send some data
#ifdef USB_SUPPORT //this is set in the port.h file
        memcpy(dataseq, (const uint8 *) "USB to SPI ", 12);
#else
#endif
        writetoLCD( 16, 1, dataseq); //send some data

        j = 1000000;

        while(j--);

        command = 0x2 ;  //return cursor home
        writetoLCD( 1, 0,  &command);
#ifdef USB_SUPPORT //this is set in the port.h file
        // enable the USB functionality
        //usb_init();

        NVIC_DisableDECAIRQ();

        // Do nothing in foreground -- allow USB application to run, I guess on the basis of USB interrupts?
        while (1)       // loop forever
        {
            usb_run();
        }
#endif
        return 1;
    }
    else //run DecaRanging application
    {
        uint8 dataseq[40];
        uint8 command = 0x0;

        command = 0x2 ;  //return cursor home
        writetoLCD( 1, 0,  &command);
        memset(dataseq, ' ', 40);
        memcpy(dataseq, (const uint8 *) "DECAWAVE  RANGE", 15);
        writetoLCD( 15, 1, dataseq); //send some data

        led_off(LED_ALL);

#ifdef USB_SUPPORT //this is set in the port.h file
        usb_printconfig();
#endif

        if(inittestapplication() == (uint32)-1)
        {
            led_on(LED_ALL); //to display error....
            dataseq[0] = 0x2 ;  //return cursor home
            writetoLCD( 1, 0,  &dataseq[0]);
            memset(dataseq, ' ', 40);
            memcpy(dataseq, (const uint8 *) "ERROR   ", 12);
            writetoLCD( 40, 1, dataseq); //send some data
            memcpy(dataseq, (const uint8 *) "  INIT FAIL ", 12);
            writetoLCD( 40, 1, dataseq); //send some data
            return 0; //error
        }


        led_off(LED_ALL);
        command = 0x2 ;  //return cursor home
        writetoLCD( 1, 0,  &command);

        memset(dataseq, ' ', 40);

        if(port_IS_TAG_pressed() == 0)
        {
            instance_mode = TAG;
            led_on(LED_PC7);
        }
        else
        {
            instance_mode = ANCHOR;
#if (DR_DISCOVERY == 1)
            led_on(LED_PC6);
#else
            if(instance_anchaddr & 0x1)
                led_on(LED_PC6);

            if(instance_anchaddr & 0x2)
                led_on(LED_PC8);
#endif
        }

        if(instance_mode == TAG)
        {
    		//if TA_SW1_2 is on use fast ranging (fast 2wr)
    		//if(is_button_low(0) == S1_SWITCH_ON)

        	//yavari I need SW1_2 button for choosing between two side ranging and positioning. We always need the default ranging mode since in the fast
       	    // mode the range is not reported to the tag.
        	if(false)
			{
				memcpy(&dataseq[2], (const uint8 *) " Fast Tag   ", 12);
				writetoLCD( 40, 1, dataseq); //send some data
				memcpy(&dataseq[2], (const uint8 *) "   Ranging  ", 12);
				writetoLCD( 16, 1, dataseq); //send some data
			}
			else
			{
            	memcpy(&dataseq[2], (const uint8 *) " TAG BLINK  ", 12);
            	writetoLCD( 40, 1, dataseq); //send some data
            	sprintf((char*)&dataseq[0], "%llX", instance_get_addr());
            	writetoLCD( 16, 1, dataseq); //send some data
			}
        }
        else
        {
            memcpy(&dataseq[2], (const uint8 *) "  AWAITING  ", 12);
            writetoLCD( 40, 1, dataseq); //send some data
            memcpy(&dataseq[2], (const uint8 *) "    POLL    ", 12);
            writetoLCD( 16, 1, dataseq); //send some data
        }

        command = 0x2 ;  //return cursor home
        writetoLCD( 1, 0,  &command);
    }

    port_EnableEXT_IRQ(); //enable ScenSor IRQ before starting

    memset(dataseq, ' ', 40);
    memset(dataseq1, ' ', 40);

    if(instance_mode == TAG)
    {
    	I2C_Setup();
//    	init_GYRO();
    	init_ACCEL();
    }
    //sleep for 5 seconds displaying "Decawave"
    calib();
        i = 0;

    if(!test_ACCEL())
    {
    	sprintf((char*)&dataseq1[0], "Accelerometer I2C Connection error!");
    	writetoLCD( 40, 1, dataseq1); //send some data
    	while(1);
    }
    else
    	Sleep(150);
    while(1)
    {
    	if(instance_mode == TAG)
    	        {
//    	        	Receive_GYRO();
    	        	Sleep(10);
    	        	Receive_ACCEL();

    	        	dataseq[0] = 0x2 ;  //return cursor home
    	        	writetoLCD( 1, 0,  dataseq);
    	        	memset(dataseq1, ' ', 40);

    	        	sprintf((char*)&dataseq1[0], "%d, %d, %d", GYRO_X - GYRO_X_OFFSET, GYRO_Y - GYRO_Y_OFFSET, GYRO_Z - GYRO_Z_OFFSET);
//    	        	writetoLCD( 40, 1, dataseq1); //send some data

    	        	sprintf((char*)&dataseq1[0], "%4.1f,%4.1f,%4.1f", ACCEL_X - ACCEL_X_OFFSET, ACCEL_Y - ACCEL_Y_OFFSET, ACCEL_Z - ACCEL_Z_OFFSET);
//    	        	sprintf((char*)&dataseq1[0], "%4.1f,%4.1f,%4.1f", ACCEL_X, ACCEL_Y, ACCEL_Z);
    	        	writetoLCD( 40, 1, dataseq1); //send some data
    	        	sprintf((char*)&dataseq[0], "Measure : %4.1f %4.1f %4.1f", ACCEL_X,ACCEL_Y,ACCEL_Z);
    	        	send_usbmessage(&dataseq[0], 35);
    	        	usb_run();
    	        	sprintf((char*)&dataseq[0], "Offset : %4.1f %4.1f %4.1f", ACCEL_X_OFFSET,ACCEL_Y_OFFSET,ACCEL_Z_OFFSET);
    	        	send_usbmessage(&dataseq[0], 35);
    	        	usb_run();
//    	        	Sleep(150);
    	        }
    }
    // main loop
    while(1)
    {

        instance_run();

        if(instance_mode == TAG)
        {
        	Receive_GYRO();
        	Sleep(10);
        	Receive_ACCEL();
        	memset(dataseq1, ' ', 40);
        	sprintf((char*)&dataseq1[0], "%d, %d, %d,%d, %d, %d", GYRO_X, GYRO_Y, GYRO_Z,ACCEL_X, ACCEL_Y, ACCEL_Z);
        	writetoLCD( 40, 1, dataseq1); //send some data
        	Sleep(10);
        }

        if(instancenewrange()) // have we got range details?
        {
        	ranging = 1;
            //send the new range information to LCD and/or USB
            range_result = instance_get_idist();
            uint64 anchadd = instance_get_anchaddr();


            if(instance_mode == TAG) //yavari: update d[] array which stores distance from each tag
            	{
				//FPP = compute_FPPower();
				//RP = compute_RP();
            	if(is_button_low(0)==S1_SWITCH_ON)
            		instancesetNextAnchId(next_anchor);
				update_d(range_result,anchadd);
				if(RangeALL_received == true)
				{
					compute_position_mle();
					if(first)
					{
						first = false;
						init_kalman();
					}
					else
						kalman_filter();
				}

            	}
#if (DR_DISCOVERY == 0)
            if(instance_mode == ANCHOR)
#endif
                avg_result = instance_get_adist();
            //set_rangeresult(range_result);
            dataseq[0] = 0x2 ;  //return cursor home
            writetoLCD( 1, 0,  dataseq);

            memset(dataseq, ' ', 40);
			memset(dataseq1, ' ', 40);
            sprintf((char*)&dataseq[1], "LAST: %4.2f m", range_result);
            writetoLCD( 40, 1, dataseq); //send some data

#if (DR_DISCOVERY == 0)
            if(instance_mode == ANCHOR)
                sprintf((char*)&dataseq1[1], "AVG8: %4.2f m", avg_result);
            else
            	sprintf((char*)&dataseq1[0], "%3.0f , %3.0f", tx,ty);
            	//sprintf((char*)&dataseq1[0], "%llx", instance_get_anchaddr());
#else
            sprintf((char*)&dataseq1[1], "AVG8: %4.2f m", avg_result);
#endif
            writetoLCD( 16, 1, dataseq1); //send some data
#ifdef USB_SUPPORT //this is set in the port.h file
            if(instance_mode == TAG)
			{
            	//yavari
            	sprintf((char*)&dataseq[0], "t %3.0f %3.0f %3.0f %3.0f %3.0f", d[0],d[1],d[2],tx,ty);
//            	dwt_readdignostics(&_diag);
//            	sprintf((char*)&dataseq[0], "%2.2f %5.2f %u %u %u ", range_result,FPP, _diag.maxGrowthCIR , _diag.maxNoise, _diag.stdNoise);
            }
            else
            {
            	sprintf((char*)&dataseq[0], "a %4f", range_result);
            }

			if(RangeALL_received == true)
			{
				RangeALL_received = false;
				Range1_received = false;
				Range2_received = false;
				Range3_received = false;

				//Only set Data to PC if we have all new ranges.
	            send_usbmessage(&dataseq[0], 35);
			}


//            send_usbmessage(&dataseq[0], 40);
#endif
        }

        if(ranging == 0)
        {
        	if(instance_mode != ANCHOR)
        	{
        		if(instancesleeping())
				{
					dataseq[0] = 0x2 ;  //return cursor home
					writetoLCD( 1, 0,  dataseq);
					if(toggle)
		{
						toggle = 0;
						memcpy(&dataseq[0], (const uint8 *) "    AWAITING    ", 16);
						writetoLCD( 40, 1, dataseq); //send some data
						memcpy(&dataseq[0], (const uint8 *) "    RESPONSE    ", 16);
						writetoLCD( 16, 1, dataseq); //send some data
					}
					else
					{
						toggle = 1;
						memcpy(&dataseq[0], (const uint8 *) "   TAG BLINK    ", 16);
						writetoLCD( 40, 1, dataseq); //send some data
						sprintf((char*)&dataseq[0], "%llX", instance_get_addr());
						writetoLCD( 16, 1, dataseq); //send some data
					}
//#ifdef USB_SUPPORT //this is set in the port.h file
					//send_usbmessage(&dataseq[0], 35);
//#endif
				}

        		if(instanceanchorwaiting() == 2)
				{
        			ranging = 1;
					dataseq[0] = 0x2 ;  //return cursor home
					writetoLCD( 1, 0,  dataseq);
					memcpy(&dataseq[0], (const uint8 *) "    RANGING WITH", 16);
					writetoLCD( 40, 1, dataseq); //send some data
					sprintf((char*)&dataseq[0], "%016llX", instance_get_anchaddr());
					writetoLCD( 16, 1, dataseq); //send some data
				}
        	}
        	else
        	{
				if(instanceanchorwaiting())
				{
					toggle+=2;

					if(toggle > 300000)
					{
						dataseq[0] = 0x2 ;  //return cursor home
						writetoLCD( 1, 0,  dataseq);
						if(toggle & 0x1)
						{
							toggle = 0;
							memcpy(&dataseq[0], (const uint8 *) "    AWAITING    ", 16);
							writetoLCD( 40, 1, dataseq); //send some data
							memcpy(&dataseq[0], (const uint8 *) "      POLL      ", 16);
							writetoLCD( 16, 1, dataseq); //send some data
		}
						else
						{
							toggle = 1;
	#if (DR_DISCOVERY == 1)
							memcpy(&dataseq[0], (const uint8 *) " DISCOVERY MODE ", 16);
	#else
							memcpy(&dataseq[0], (const uint8 *) " NON DISCOVERY  ", 16);
#endif
							writetoLCD( 40, 1, dataseq); //send some data
							sprintf((char*)&dataseq[0], "%llX", instance_get_addr());
							writetoLCD( 16, 1, dataseq); //send some data
						}
					}

				}
				else if(instanceanchorwaiting() == 2)
				{
					dataseq[0] = 0x2 ;  //return cursor home
					writetoLCD( 1, 0,  dataseq);
					memcpy(&dataseq[0], (const uint8 *) "    RANGING WITH", 16);
					writetoLCD( 40, 1, dataseq); //send some data
					sprintf((char*)&dataseq[0], "%llX", instance_get_tagaddr());
					writetoLCD( 16, 1, dataseq); //send some data
				}
        	}
        }
#ifdef USB_SUPPORT //this is set in the port.h file
		usb_run();
#endif
    }


    return 0;
}



