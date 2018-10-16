/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
#include "main.h"
#include "stm32f0xx_hal.h"
#include "stdlib.h"
#include "stdio.h"
#include "string.h"
#define multiplierFactor 0.002
 
#define ENERGY_IC
#define meter_constant 2400.00
#define power_multiplier 0.88
#define energy_multiplier 1.017
#define voltage_threshhold 50000 //5V
#define current_threshhold 50000//change it to 5000 //0.5A
#define temp_threshhold 1
#define energy_threshhold 0.1
#define current_max_threshold 2000000 //20A
#define power_max_threshold 400000000//4kw
#define volt_max_threshold 3000000 //300v , this will also handle relay back emf 400v issue 
#define peak_current_reset_threshold 10000000 //100W
#define SYSCON 0x00
#define SYSCON_len 2
#define EMUCON 0x01
#define EMUCON_len 2
#define EMUCON2 0x17
#define EMUCON2_len 2
#define IARMS 0x22
#define IARMS_len 3
#define IBRMS 0x23
#define IBRMS_len 3
#define URMS 0x24
#define URMS_len 3
#define GPQB 0x06
#define GPQB_len 2
#define HFCONST 2
#define HFCONST_len 2
#define PowerPB 0x27
#define PowerPB_len 4
#define HFConst 0x02
#define HFConst_len 2
#define EnergyP 0x29

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/



uint16_t const tm_dat[2][16]={{'0','1','2','3','4','5',     //the char and its segment code   
            '6','7','8','9','@','-','_','c','*','#'},  
            {0x3F,0x06,0x5B,0x4F,0x66,0x6D,  
            0x7D,0x07,0x7F,0x6F,0x02,0x08,  
            0x05,0x01,0x04,0x01}}; 

float res[]  = {162.31, 154.30, 146.75, 139.61, 132.87, 126.50, 120.48, 114.78, 109.40, 104.30,
		99.47, 94.76, 90.30, 86.08, 78.31, 74.78, 71.44, 68.27, 65.26, 62.40, 59.66, 57.05, 54.58, 52.23,
		50.00, 47.88, 45.87, 43.95, 42.13, 40.40, 38.72, 37.13, 35.62, 34.17, 32.80, 31.46, 30.18, 28.96, 27.80, 
		26.70, 25.63, 24.61, 23.65, 22.72, 21.84, 20.98, 20.17, 19.39, 18.64, 17.93, 17.27, 16.63, 16.03, 15.44, 
		14.89, 14.34, 13.81, 13.31, 12.83, 12.37, 11.92, 11.48,  11.07, 10.68, 10.30, 9.93, 9.57, 9.23, 8.91,
		8.60, 8.30, 8.01, 7.73, 7.46, 7.21, 6.97, 6.74, 6.52, 6.30, 6.10, 5.89, 5.70, 5.51, 5.33, 5.16, 4.99, 4.83, 4.68, 4.53, 4.39  };						
						

int count = 0 ;
int temp = 0;
int temp1 = 0;
int temp2 = 0;	
long sum_temp;
uint16_t value;
unsigned int ADC_raw[4];
unsigned char Index=0;
int R1 = 49200, R2 = 0;	
float tempvolt;
float tempvolt1;
char  Rx_data[2], Rx_Buffer[2], Transfer_cplt;
int Rx_indx;	
char SSD[3];
int ic;
char SSD1[3];
char PHD[3];
int ic1;
int counter=6;
int counter1=80;	
int Flag=0;
int Flag1=0;	
int Flag2=0;
int Flag3=0;
int Flag4=0;
float tds;
float volts_tds;
int gain=1;
int weight;
int ph_w;
int tds_w;
int tc;
int th;
//int data_read2[10];
//float current_gain = 179869 / 33000; //store 24 bytes of current register
//char R_data;
//float Energy_KWH;
//long APower;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
float read_adc ( uint16_t avg, uint16_t channel);
/* USER CODE END PFP */


/////////////////////////////////Multichannel ADC Interrupt///////////////////////////////////////////////////////////////////////
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	
	if (__HAL_ADC_GET_FLAG(hadc, ADC_IT_EOC))
		{
			
		ADC_raw[Index] = HAL_ADC_GetValue(hadc);
		Index++;
		}
 
	if (__HAL_ADC_GET_FLAG(hadc, ADC_IT_EOS))
		{
		Index=0;		
		}		
			
}

//////////////////////////////////UART2 Data Receive Interrupt//////////////////////////////////////////////////////////////////

/*void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart2)
{
	uint8_t i;
	
	if (huart2->Instance == USART2)	//current UART
		{
		if (Rx_indx==0) {
		for (i=0;i<100;i++)
			Rx_Buffer[i]=0;
		}	//clear Rx_Buffer before receiving new data	
		
		
		if (Rx_data[0]!=13)	//if received data different from ascii 13 (enter)
			{
			Rx_Buffer[Rx_indx++]=Rx_data[0];	//add data to Rx_Buffer
			}
		else			//if received data = 13
			{
			Rx_indx=0;
			Transfer_cplt=1;//transfer complete, data is ready to read                                             
			}
 
		HAL_UART_Receive_IT(huart2,(uint8_t *)&Rx_data, 1);	//activate UART receive interrupt every time
		}
 
}


*/

///////////////////////////////Temperature Calculation///////////////////////////////////////////////////////////////////

int temperature_calc(float voltage){
	
	
	R2 = (voltage * R1) / (3.3 - voltage);
  R2 = R2 / 1000;
	 sum_temp = 0;
  for (int j = 0 ; j < 90 ; j++)
  {
    count = 0;
    for (int ii = 0; ii < 90; ii++)
    {
      if (R2 <= res[ii] && R2 > res[ii + 1])
      {
      
        count = count + ii;
      }
    }
  
    sum_temp += count;
  }
  sum_temp = sum_temp / 90;
		
	return(sum_temp);
}



//////////////////////////Cold Thermostat////////////////////////////////////////////////////////////////////////

int cold_thermostat(int cvalue,int temp_cold){
	
	if (temp_cold < cvalue) {
    HAL_Delay(1000);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_RESET);	
  }

 else if (temp_cold > cvalue  ) {
    HAL_Delay(1000);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_SET);

 }	
}

////////////////////////////Hot Thermostat/////////////////////////////////////////////////////////////////////

int hot_thermostat(int hvalue,int temp_hot){
	
	if (temp_hot < hvalue) {
    HAL_Delay(1000);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET);	
  }

 else if (temp_hot > hvalue  ) {
    HAL_Delay(1000);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET);

 }	
}

////////////////////////////////////////////////////SEVEN SEGMENT -HUMAN MACHINE INTERFACE////////////////////////////////////////////////////////////////
void Write_Byte(uint8_t byte){  
    uint8_t i=0;  
    for(i=0;i<8;i++){  
      HAL_GPIO_WritePin(GPIOC,GPIO_PIN_11,GPIO_PIN_RESET);  			
        if(byte&0x01){  
					 HAL_GPIO_WritePin(GPIOC,GPIO_PIN_10,GPIO_PIN_SET); 
        }else{  
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_10,GPIO_PIN_RESET);  
        }  
          HAL_GPIO_WritePin(GPIOC,GPIO_PIN_11,GPIO_PIN_SET); 
        byte>>=1;  
    }  
}  		
		

void Write_Cmd(uint8_t cmd){  
    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_12,GPIO_PIN_SET); 
	 // HAL_Delay(100);
    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_12,GPIO_PIN_RESET);   
    Write_Byte(cmd);  
}  
  
  
void Write_Dat(uint8_t addr,uint8_t dat){  
    Write_Cmd(0x44);  
    Write_Cmd(0xc0|addr);  
    Write_Byte(dat);  
}  	



void TM1638_SendData(uint8_t i,char * str){  
    int j=0,k=0;  
    unsigned char chr;  
    for(;i<8;i++){  
        k=0;  
        for(j=0;j<16;j++){  
            if(*str==tm_dat[0][j]){  
                chr=tm_dat[1][j];  
                k=1;  
                break;  
            }  
        }  
          
        if(k==0){  
            chr=0x00;  
        }  
          
        if(*(str+1)=='.'){  
            chr|=0x80;  
            Write_Dat(i*2,chr);  
            str++;  
        }else{  
            Write_Dat(i*2,chr);  
        }  
        str++;  
        if(*str=='\0')break;  
    }  
}  


void TM1638_Init(){  
    int i=0;    
    Write_Cmd(0x8a);  
    Write_Cmd(0x40);  
    for(i=0;i<16;i++){  
        Write_Byte(0x00);  
    }  
}  
	





//////////////////////////+ increment key-HMI////////////////////////////////////////////////////////////

void number_display(int number)
{   
     SSD[0]=((number/10))+48;
     SSD[1]=(number%10)+48;
		// SSD1[0]=((ic1/10))+48;
  //   SSD1[1]=(ic1%10)+48;
		
		for (int l=0;l<2;l++)
		{
		TM1638_SendData(l,&SSD[l]);
		}

}		
	

///////////////////////////////////////PH Sensor//////////////////////////////////////////////////////////
		
int read_ph (void)
{
  float ph = read_adc( 10000,1);
	int iPH=ph*1000;  
	   
  iPH = ((iPH - 850) * 100);// 1652 is offset voltage added to PH voltage 
  iPH = ((41412 - iPH) * 100) / 5916;// Using slope value 59.16 from datasheet and y intercept from measuring PH 4
 
	
	return(iPH);
	
//	printf("Volts :%d \r\n",iPH);
	/*   PHD[0]=((iPH/10))+48;
     PHD[1]=(iPH%10)+48;
		
		for (int p=0;p<2;p++)
		{
	
		TM1638_SendData(p,&PHD[p]);
		}
	*/
}
////////////////////////////////////////////TDS Sensor /////////////////////////////////////////////////////////////////////
	 int read_tds(void)
	 {		 
    volts_tds=read_adc(10000,0);
	  volts_tds=volts_tds*1000;
		tds = ((((multiplierFactor * volts_tds) - 0.188) / 188) * 640530);
		tds =tds /2.0 ;
		//printf("Value :%f \r\n",tds);
  //  tds=tds*1000;
		return(tds); 
	 }

	int avg_tds (void)
{
  uint32_t accum1 = 0; // averaging accumulator
  uint16_t x1 = 50;
  while (x1--) {
	 
    accum1 += read_tds(); // add analog readings to accumulator
  
	}
  uint32_t tds_value = (accum1 / 50);
   
  return ((tds_value/2)+135); // return averaged voltage reading
} 
	 
	 
	 

///////////////////////////////////////////////Weight Sensor///////////////////////////////////////////////////////////////////////////
	 
	 int HX711_Value()
{
    int buffer;
    buffer = 0;

    while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3)==1);

    for (uint8_t i = 0; i < 24; i++)
    {
    	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);

        buffer = buffer << 1 ;

        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3))
        {
            buffer ++;
        }

        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
    }

    for (int i = 0; i <gain; i++)
    {
    	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
    	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
    }

    //buffer = buffer ^ 0x800000;
 //   buffer=buffer+0x01;
		
    return buffer;
}
	 

int avg_weight (void)
{
  uint32_t accum3 = 0; // averaging accumulator
  uint16_t x3 = 50;
  while (x3--) {
	 
    accum3 +=HX711_Value(); // add analog readings to accumulator
  
	}
  uint32_t weight_avg = (accum3 / 50);
	
	  
    weight_avg=weight_avg/1000;
   // weight=weight-440;	
	//weight=-weight;	
  // weight_avg=weight_avg*25;	
	//  weight_avg=weight_avg*10;	
	   weight_avg=(weight_avg-1390)*2;
	  
	
  return (weight_avg); // return averaged voltage reading
} 
	 











//////////////////////////////////////Energy IC Measurement Code///////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* Private function prototypes -----------------------------------------------*/
#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */



	PUTCHAR_PROTOTYPE
{
	/* Place your implementation of fputc here */
	/* e.g. write a character to the USART */
	HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 100);
	
	
	return ch;
}






/* USER CODE END 0 */
uint8_t testData = '5';
char receivedData[1]; //= "Hello World";
int main(void)
{

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_ADC_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();

 HAL_ADC_Start_IT(&hadc);
  /* USER CODE BEGIN 2 */
// HAL_UART_Receive_IT(&huart2,(uint8_t *)&Rx_data,1);
  /* USER CODE END 2 */
 TM1638_Init();

 //rn8209_init();	
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	
  while (1)
  { 
		
	
		HAL_UART_Receive(&huart2, (uint8_t *)receivedData, 1, 1000);
		
		
	 if (receivedData[0]=='6')
	 {
		 counter = 6;
		 cold_thermostat(counter,ic);
			
		 } else if(receivedData[0]=='7'){
			counter = 7;
		 cold_thermostat(counter,ic); 
		 
		} else if(receivedData[0]=='8'){
			counter = 8;
		 cold_thermostat(counter,ic); 
		  
		} else if(receivedData[0]=='9'){
			counter = 9;
		 cold_thermostat(counter,ic); 
		  
		}  else if(receivedData[0]=='!'){
			counter = 10;
		 cold_thermostat(counter,ic); 
		} 



		else if(receivedData[0]=='0')
			{
			counter1 = 80;
		// hot_thermostat(counter1,ic1); 
		 } 
			else if(receivedData[0]=='1')
				{
			counter1 = 81;
		 hot_thermostat(counter1,ic1);
	    	}  
			else if(receivedData[0]=='2')
				{
			counter1 = 82;
		 hot_thermostat(counter1,ic1);
		   }  
				else if(receivedData[0]=='3')
					{
			counter1 = 83;
		 hot_thermostat(counter1,ic1);
		    } 
					else if(receivedData[0]=='4')
		{
			counter1 = 84;
		 hot_thermostat(counter1,ic1);
		}  
		else if(receivedData[0]=='5')
			{
			counter1 = 85;
		 hot_thermostat(counter1,ic1);
		  }
		  else if(receivedData[0]=='#')
				{
			counter1 = 86;
		// hot_thermostat(counter1,ic1);
	    	}
		
  
	
	//removed comment from here
	
   if (Flag4!=1)
	 {
  		
	 TM1638_SendData(4,"#");
	 }
		
				
	
		/////////////////////////////////////////Cold & Hot Temperature Volts////////////////////////////////////////////////////////////	
		HAL_ADC_Start_IT(&hadc);
	  tempvolt=read_adc(5000,2);
	  tempvolt1=read_adc(5000,5);
	   ic=temperature_calc(tempvolt);
	  ic1=temperature_calc(tempvolt1);
			
		ph_w=read_ph(); 
 
	 
	 // if (ph_w>850 || ph_w<650)
	//	{
	//		ph_w=771;	
	//	}
   	 
	
	  
	  tds_w=avg_tds();
		
		weight=avg_weight();
		
		
		
		/*
    weight=HX711_Value();
    weight=weight/1000;
   // weight=weight-440;	
	//weight=-weight;	
    weight=weight*25;	
	  weight=weight*10;	
   */
		
		
		
	 if (weight>12000 || weight<0)
	 {
		 
		 weight=0;
	 }
	 

	 
	//////////////////////////////////////////////////Data sending to WiFi Module////////////////////////////////////////////////
	
	
	    printf("\n %d %d %d %d %d ",ic,ic1,tds_w,ph_w,weight);
		  //HAL_Delay(5000);
		
		
  //////////////////////////////////////Hot & Cold Temperature Display/////////////////////////////////////////////////////////////////
		
 
	
	 
	  	HAL_Delay(1000);
	    TM1638_SendData(2,"@");
		  TM1638_SendData(3,"c");
		  number_display(ic);
		  HAL_Delay(2000);
		  TM1638_SendData(2,"-");
	    TM1638_SendData(3,"c");
		  number_display(ic1);
	    HAL_Delay(1000);
	   
	/*
		 SSD[0]=((ic/10))+48;
     SSD[1]=(ic%10)+48;
		 SSD1[0]=((ic1/10))+48;
     SSD1[1]=(ic1%10)+48;
		
		for (int l=0;l<2;l++)
		{
	
		TM1638_SendData(l,&SSD[l]);
		}
	  		 
		HAL_Delay(2000);
		
		for (int l=0;l<2;l++)
		{
 	
		TM1638_SendData(l,&SSD1[l]);
		}
			
			
		 *///added comment here
/////////////////////////////////////////////Child Lock KEY-6//////////////////////////////////////////////////////////////		
				if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8)==0)  
		{
    
    HAL_Delay(100);
			
   if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8)==0) 
	{
		 
		Flag4=1;	
	}		
	}			
	
		
//////////////////////////////////////////Heater Setpoint Mode  KEY-4//////////////////////////////////	 
	
 if (Flag4==1)
 {
 
 TM1638_SendData(4,""); 
 HAL_Delay(1000);
	 
	 
 

	if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8)==0)  
		{
    
    HAL_Delay(100);
			
   if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8)==0) 
	{
		 
		Flag2=1;	
	}		
	}			
	
	
	
	
	if (Flag2==1)
		
	{
		TM1638_SendData(2,"_");
		hot_thermostat(counter1,ic1);
		
		
		
		if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8)==0)  
		{ 
		 HAL_Delay(100);  
     if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8)==0)
		 {			 	
			Flag2=0; 
	    // TM1638_SendData(3,"");
			//  HAL_Delay(1000);
		 }	
		}			
	 
		
		
		
		
	}
	
	
	/////////////////////////////////////////////Compressor Setpoint Mode   KEY-5//////////////////////////////////////
		if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9)==0)  
		{
    
    HAL_Delay(100);
			
   if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9)==0) 
	{  
		 Flag3=1;
		 
	}	
		
	}		



if (Flag3==1)
		
	{
		
		 TM1638_SendData(3,"*");
		 cold_thermostat(counter,ic);
		
		
		
		if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9)==0)  
		{ 
		 HAL_Delay(100);  
     if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9)==0)
		 {			 	
			Flag3=0; 
	  //  TM1638_SendData(3,"");
			//  HAL_Delay(1000);
		 }	
		}			
	 
	}
	
	

//////////////////////////////Temperature Setpoint Mode  KEY-3//////////////////////////////////////////////
	
  	if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7)==0)  
		{
    
    HAL_Delay(100);
			
   if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7)==0) 
	{
			Flag=1;
		 // TM1638_SendData(2,"1");
		 HAL_Delay(1000);
	}	
		
	}			

///////////////////////////////////Compressor Setpoint  KEY 1&2/////////////////////////////////////////  
   while (Flag==1)
	 {	
	
	 number_display(counter);
		 
		if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15)==0)  
   {
	 HAL_Delay(10);
		 
	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15))  
	{
		counter++;	
		
		number_display(counter);
		
		if (counter>10)
		{
			counter=6;
	  }
	
	}
   
	}

	
	
	if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6)==0)  
   {
	 HAL_Delay(10);
		 
	if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6))  
	{
		counter--;	
		
		number_display(counter);
		
		if (counter>10)
		{
			counter=6;
		}
	
	}
   
	}
	 
  
	
	
	
		if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7)==0)  
		{ 
		 HAL_Delay(100);
    
     if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7)==0)
		 {			 
			Flag=0;
			Flag1=1; 
	   //  TM1638_SendData(2,"2");
			  HAL_Delay(1000);
		 }	
		}			
	 }
	
	 
	 
		  
////////////////////////////////Heater Setpoint  KEY 1&2////////////////////////////////////////	
	
	while(Flag1==1)
	{
		number_display(counter1);
		
		if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15)==0)  
   {
	 HAL_Delay(10);
		 
	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15))  
	{
		counter1++;	
		
		number_display(counter1);
		
		if (counter1>86)
		{
			counter1=80;
		}
	
	}
   
	}

	
	
	if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6)==0)  
   {
	 HAL_Delay(10);
		 
	if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6))  
	{
		counter1--;	
		
		number_display(counter1);
		
  	if (counter1>86)
		{
			counter1=80;
		}
	} 
	}
	
  
	
	
		if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7)==0)  
		{ 
		 HAL_Delay(100);
    
     if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7)==0)
		 {			 
			 Flag1=0;
	    // TM1638_SendData(2,"2");
			  HAL_Delay(1000);
		 }	
		}	

	}
	
	
	
	
	
	
		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8)==0)  
		{
    
    HAL_Delay(100);
			
   if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8)==0) 
	{
		
		Flag4=0;	
	}		
	}		

 }
	
///////////////////////////////////////////End of HMI///////////////////////////////////////////////////////////////////////////////	
	 
	 
		//temp1=atoi(Rx_Buffer);
		//temp2=atoi(Rx_Buffer);
	//  cold_thermostat(temp1,temperature_calc(tempvolt));
 //   hot_thermostat(temp2,temperature_calc(tempvolt1));
		
		// HAL_Delay(500);
		
		//removed comment from here	

  }
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC init function */
static void MX_ADC_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_4;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_6;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_7;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_9;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Analogue filter 
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Digital filter 
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C2 init function */
static void MX_I2C2_Init(void)
{

  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x2000090E;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Analogue filter 
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Digital filter 
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC2 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC3 PC4 PC5 PC6 
                           PC7 PC8 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6 
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB13 PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC10 PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */
float read_adc ( uint16_t avg,uint16_t channel )
{
  uint32_t accum = 0; // averaging accumulator
  uint16_t x = avg;
  while (x--) {
	  value= ADC_raw[channel];
    accum += value; // add analog readings to accumulator
  }
  uint32_t sensorValue = (accum / avg);
 float voltage_read =  sensorValue * (3.3 / 4095.0);
// uint16_t voltage_probe = voltage_read * 1000;
  return (voltage_read); // return averaged voltage reading
}






/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
