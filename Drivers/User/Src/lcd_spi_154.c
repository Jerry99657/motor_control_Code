/***
	*************************************************************************************************
	*	@file  	main.c
	*	@version V1.0
   *************************************************************************************************
   *  @description
>>>>> ��Ҫ˵����
	*
	*  1.��Ļ����Ϊ16λRGB565��ʽ
	*  2.SPIͨ���ٶ�Ϊ60M
   *
>>>>> ����˵����
	*
	*	1. �����ֿ�ʹ�õ���С�ֿ⣬���õ��˶�Ӧ�ĺ�����ȥȡģ���û����Ը�����������������ɾ��
	*	2. ���������Ĺ��ܺ�ʹ�ÿ��Բο�������˵��
	*
	**************************************************************************************************************************************************************************************************
***/

#include "lcd_spi_154.h"
#include <stdio.h>
#include <string.h>

extern SPI_HandleTypeDef hspi6;

#define  LCD_SPI hspi6           // SPI�ֲ��꣬�����޸ĺ���ֲ

static pFONT *LCD_AsciiFonts;		// Ӣ�����壬ASCII�ַ���
static pFONT *LCD_CHFonts;		   // �������壨ͬʱҲ����Ӣ�����壩

// ��Ϊ����SPI����Ļ��ÿ�θ�����ʾʱ����Ҫ����������������д�Դ棬
// ����ʾ�ַ�ʱ�������һ������ȥд����д�Դ棬��ǳ�����
// ��˿���һƬ���������Ƚ���Ҫ��ʾ������д�������������������д���Դ档
// �û����Ը���ʵ�����ȥ�޸Ĵ˴��������Ĵ�С��
// ���磬�û���Ҫ��ʾ32*32�ĺ���ʱ����Ҫ�Ĵ�СΪ 32*32*2 = 2048 �ֽڣ�ÿ�����ص�ռ2�ֽڣ�
uint16_t  LCD_Buff[1024];        // LCD��������16λ����ÿ�����ص�ռ2�ֽڣ�

struct	//LCD��ز����ṹ��
{
	uint32_t Color;  				//	LCD��ǰ������ɫ
	uint32_t BackColor;			//	����ɫ
   uint8_t  ShowNum_Mode;		// ������ʾģʽ
	uint8_t  Direction;			//	��ʾ����
   uint16_t Width;            // ��Ļ���س���
   uint16_t Height;           // ��Ļ���ؿ���	
   uint8_t  X_Offset;         // X����ƫ�ƣ�����������Ļ���������Դ�д�뷽ʽ
   uint8_t  Y_Offset;         // Y����ƫ�ƣ�����������Ļ���������Դ�д�뷽ʽ
}LCD;

// �ú����޸���HAL��SPI�⺯����רΪ LCD_Clear() ���������޸ģ�
// Ŀ����Ϊ��SPI�������ݲ������ݳ��ȵ�д��
HAL_StatusTypeDef LCD_SPI_Transmit(SPI_HandleTypeDef *hspi, uint16_t pData, uint32_t Size);
HAL_StatusTypeDef LCD_SPI_TransmitBuffer (SPI_HandleTypeDef *hspi, uint16_t *pData, uint32_t Size);

static volatile uint8_t s_lcd_spi_tx_busy = 0U;
static volatile HAL_StatusTypeDef s_lcd_spi_tx_status = HAL_OK;
static LCD_TransferCallback_t s_lcd_spi_tx_callback = NULL;
static void *s_lcd_spi_tx_callback_context = NULL;

/* SPI6 is served by BDMA, which can only access the D3/SRAM4 domain.  Keep
 * two small chunks there and copy arbitrary source frame buffers into them. */
#define LCD_BDMA_STAGE_COUNT       2U
#define LCD_BDMA_STAGE_PIXELS      4096U
#define LCD_BDMA_SLOT_FREE         0U
#define LCD_BDMA_SLOT_READY        1U
#define LCD_BDMA_SLOT_FILLING      2U
#define LCD_BDMA_NO_ACTIVE_SLOT    0xFFU

static uint16_t s_lcd_bdma_stage[LCD_BDMA_STAGE_COUNT][LCD_BDMA_STAGE_PIXELS]
	__attribute__((section(".ram_d3_bdma"), aligned(32)));
static const uint16_t *s_lcd_spi_tx_source = NULL;
static volatile uint32_t s_lcd_spi_tx_total = 0U;
static volatile uint32_t s_lcd_spi_tx_source_pos = 0U;
static volatile uint16_t s_lcd_bdma_slot_pixels[LCD_BDMA_STAGE_COUNT] = {0U, 0U};
static volatile uint8_t s_lcd_bdma_slot_state[LCD_BDMA_STAGE_COUNT] = {LCD_BDMA_SLOT_FREE, LCD_BDMA_SLOT_FREE};
static volatile uint8_t s_lcd_bdma_active_slot = LCD_BDMA_NO_ACTIVE_SLOT;
static volatile uint8_t s_lcd_bdma_hw_busy = 0U;
static volatile uint8_t s_lcd_spi_needs_finalize = 0U;

static void LCD_SPI_CleanDCacheForBuffer(const void *address, uint32_t bytes)
{
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
	uintptr_t start_addr;
	uintptr_t end_addr;

	if ((address == NULL) || (bytes == 0U) || ((SCB->CCR & SCB_CCR_DC_Msk) == 0U))
	{
		return;
	}

	start_addr = ((uintptr_t)address) & ~((uintptr_t)31U);
	end_addr = (((uintptr_t)address + (uintptr_t)bytes + 31U) & ~((uintptr_t)31U));
	SCB_CleanDCache_by_Addr((uint32_t *)start_addr, (int32_t)(end_addr - start_addr));
#else
	(void)address;
	(void)bytes;
#endif
}

static HAL_StatusTypeDef LCD_SPI_SetDataSize(uint32_t data_size)
{
	LCD_SPI.Init.DataSize = data_size;
	if (HAL_SPI_Init(&LCD_SPI) != HAL_OK)
	{
		return HAL_ERROR;
	}

	return HAL_OK;
}

static HAL_StatusTypeDef LCD_SPI_TransmitBufferAsync(SPI_HandleTypeDef *hspi, const uint16_t *pData, uint32_t Size);
static HAL_StatusTypeDef LCD_SPI_StartReadyChunk(void);
static void LCD_SPI_FinalizeTransfer(void);

/****************************************************************************************************************************************
*	�� �� ��: LCD_WriteCommand
*
*	��ڲ���: lcd_command - ��Ҫд��Ŀ���ָ��
*
*	��������: ��������Ļ������д��ָ��
*
****************************************************************************************************************************************/

void  LCD_WriteCommand(uint8_t lcd_command)
{
	LCD_CS_LOW;
   LCD_DC_Command;     // ����ָ��ѡ�� ��������͵�ƽ���������δ��� ָ��

	HAL_SPI_Transmit(&LCD_SPI, &lcd_command, 1, 1000); // ����SPI����
	LCD_CS_HIGH;
}

/****************************************************************************************************************************************
*	�� �� ��: LCD_WriteData_8bit
*
*	��ڲ���: lcd_data - ��Ҫд������ݣ�8λ
*
*	��������: д��8λ����
*	
****************************************************************************************************************************************/

void  LCD_WriteData_8bit(uint8_t lcd_data)
{
	LCD_CS_LOW;
   LCD_DC_Data;     // ����ָ��ѡ�� ��������ߵ�ƽ���������δ��� ����

	HAL_SPI_Transmit(&LCD_SPI, &lcd_data, 1, 1000) ; // ����SPI����
	LCD_CS_HIGH;
}

/****************************************************************************************************************************************
*	�� �� ��: LCD_WriteData_16bit
*
*	��ڲ���: lcd_data - ��Ҫд������ݣ�16λ
*
*	��������: д��16λ����
*	
****************************************************************************************************************************************/

void  LCD_WriteData_16bit(uint16_t lcd_data)
{
   uint8_t lcd_data_buff[2];    // ���ݷ�����
	LCD_CS_LOW;
   LCD_DC_Data;      // ����ָ��ѡ�� ��������ߵ�ƽ���������δ��� ����
 
   lcd_data_buff[0] = lcd_data>>8;  // �����ݲ��
   lcd_data_buff[1] = lcd_data;
		
	HAL_SPI_Transmit(&LCD_SPI, lcd_data_buff, 2, 1000) ;   // ����SPI����
	LCD_CS_HIGH;
}

/****************************************************************************************************************************************
*	�� �� ��: LCD_WriteBuff
*
*	��ڲ���: DataBuff - ��������DataSize - ���ݳ���
*
*	��������: ����д�����ݵ���Ļ
*	
****************************************************************************************************************************************/

void  LCD_WriteBuff(uint16_t *DataBuff, uint16_t DataSize)
{
	LCD_CS_LOW;
	LCD_DC_Data;     // ����ָ��ѡ�� ��������ߵ�ƽ���������δ��� ����	

// �޸�Ϊ16λ���ݿ��ȣ�д�����ݸ���Ч�ʣ�����Ҫ���	
   LCD_SPI.Init.DataSize 	= SPI_DATASIZE_16BIT;   //	16λ���ݿ���
   HAL_SPI_Init(&LCD_SPI);		
	
	HAL_SPI_Transmit(&LCD_SPI, (uint8_t *)DataBuff, DataSize, 1000) ; // ����SPI����
	
// �Ļ�8λ���ݿ��ȣ���Ϊָ��Ͳ������ݶ��ǰ���8λ�����
	LCD_SPI.Init.DataSize 	= SPI_DATASIZE_8BIT;    //	8λ���ݿ���
   HAL_SPI_Init(&LCD_SPI);	
	LCD_CS_HIGH;
}

/****************************************************************************************************************************************
*	�� �� ��: SPI_LCD_Init
*
*	��������: ��ʼ��SPI�Լ���Ļ�������ĸ��ֲ���
*	
****************************************************************************************************************************************/

void SPI_LCD_Init(void)
{
	// Reuse CubeMX-generated SPI6 handle and switch to robust LCD settings.
	LCD_SPI.Init.NSS = SPI_NSS_SOFT;
	LCD_SPI.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
	LCD_SPI.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
	if (HAL_SPI_Init(&LCD_SPI) != HAL_OK)
	{
		Error_Handler();
	}

	GPIO_InitTypeDef GPIO_InitStruct = {0};
	__HAL_RCC_GPIOG_CLK_ENABLE();
	GPIO_InitStruct.Pin = LCD_CS_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LCD_CS_PORT, &GPIO_InitStruct);
	LCD_CS_HIGH;

	// Turn on backlight early to shorten perceived black screen time.
	LCD_Backlight_ON;
   
   HAL_Delay(10);               	// ��Ļ����ɸ�λʱ�������ϵ縴λ������Ҫ�ȴ�����5ms���ܷ���ָ��
 	LCD_WriteCommand(0x36);       // �Դ���ʿ��� ָ��������÷����Դ�ķ�ʽ
	LCD_WriteData_8bit(0x00);     // ���ó� ���ϵ��¡������ң�RGB���ظ�ʽ

	LCD_WriteCommand(0x3A);			// �ӿ����ظ�ʽ ָ���������ʹ�� 12λ��16λ����18λɫ
	LCD_WriteData_8bit(0x05);     // �˴����ó� 16λ ���ظ�ʽ

// �������ܶ඼�ǵ�ѹ����ָ�ֱ��ʹ�ó��Ҹ��趨ֵ
 	LCD_WriteCommand(0xB2);			
	LCD_WriteData_8bit(0x0C);
	LCD_WriteData_8bit(0x0C); 
	LCD_WriteData_8bit(0x00); 
	LCD_WriteData_8bit(0x33); 
	LCD_WriteData_8bit(0x33); 			

	LCD_WriteCommand(0xB7);		   // դ����ѹ����ָ��	
	LCD_WriteData_8bit(0x35);     // VGH = 13.26V��VGL = -10.43V

	LCD_WriteCommand(0xBB);			// ������ѹ����ָ��
	LCD_WriteData_8bit(0x19);     // VCOM = 1.35V

	LCD_WriteCommand(0xC0);
	LCD_WriteData_8bit(0x2C);

	LCD_WriteCommand(0xC2);       // VDV �� VRH ��Դ����
	LCD_WriteData_8bit(0x01);     // VDV �� VRH ���û���������

	LCD_WriteCommand(0xC3);			// VRH��ѹ ����ָ��  
	LCD_WriteData_8bit(0x12);     // VRH��ѹ = 4.6+( vcom+vcom offset+vdv)
				
	LCD_WriteCommand(0xC4);		   // VDV��ѹ ����ָ��	
	LCD_WriteData_8bit(0x20);     // VDV��ѹ = 0v

	LCD_WriteCommand(0xC6); 		// ����ģʽ��֡�ʿ���ָ��
	LCD_WriteData_8bit(0x0F);   	// ������Ļ��������ˢ��֡��Ϊ60֡    

	LCD_WriteCommand(0xD0);			// ��Դ����ָ��
	LCD_WriteData_8bit(0xA4);     // ��Ч���ݣ��̶�д��0xA4
	LCD_WriteData_8bit(0xA1);     // AVDD = 6.8V ��AVDD = -4.8V ��VDS = 2.3V

	LCD_WriteCommand(0xE0);       // ������ѹ٤��ֵ�趨
	LCD_WriteData_8bit(0xD0);
	LCD_WriteData_8bit(0x04);
	LCD_WriteData_8bit(0x0D);
	LCD_WriteData_8bit(0x11);
	LCD_WriteData_8bit(0x13);
	LCD_WriteData_8bit(0x2B);
	LCD_WriteData_8bit(0x3F);
	LCD_WriteData_8bit(0x54);
	LCD_WriteData_8bit(0x4C);
	LCD_WriteData_8bit(0x18);
	LCD_WriteData_8bit(0x0D);
	LCD_WriteData_8bit(0x0B);
	LCD_WriteData_8bit(0x1F);
	LCD_WriteData_8bit(0x23);

	LCD_WriteCommand(0xE1);      // ������ѹ٤��ֵ�趨
	LCD_WriteData_8bit(0xD0);
	LCD_WriteData_8bit(0x04);
	LCD_WriteData_8bit(0x0C);
	LCD_WriteData_8bit(0x11);
	LCD_WriteData_8bit(0x13);
	LCD_WriteData_8bit(0x2C);
	LCD_WriteData_8bit(0x3F);
	LCD_WriteData_8bit(0x44);
	LCD_WriteData_8bit(0x51);
	LCD_WriteData_8bit(0x2F);
	LCD_WriteData_8bit(0x1F);
	LCD_WriteData_8bit(0x1F);
	LCD_WriteData_8bit(0x20);
	LCD_WriteData_8bit(0x23);

	LCD_WriteCommand(0x21);       // �򿪷��ԣ���Ϊ����ǳ����ͣ�������Ҫ������

 // �˳�����ָ�LCD�������ڸ��ϵ硢��λʱ�����Զ���������ģʽ ����˲�����Ļ֮ǰ����Ҫ�˳�����  
	LCD_WriteCommand(0x11);       // �˳����� ָ��
   HAL_Delay(120);               // ��Ҫ�ȴ�120ms���õ�Դ��ѹ��ʱ�ӵ�·�ȶ�����

 // ����ʾָ�LCD�������ڸ��ϵ硢��λʱ�����Զ��ر���ʾ 
	LCD_WriteCommand(0x29);       // ����ʾ   	
	
// ���½���һЩ������Ĭ������
   LCD_SetDirection(Direction_H_Flip);  	      //	������ʾ����
	LCD_SetBackColor(LCD_RED);             // ���ñ���ɫ
 	LCD_SetColor(LCD_WHITE);               // ���û���ɫ  
	LCD_Clear();                           // ����

   LCD_SetAsciiFont(&ASCII_Font24);       // ����Ĭ������
   LCD_ShowNumMode(Fill_Zero);	      	// ���ñ�����ʾģʽ������λ���ո������0

// ȫ���������֮�󣬴򿪱���	
   LCD_Backlight_ON;  // ��������ߵ�ƽ��������
}

/****************************************************************************************************************************************
*	�� �� ��:	 LCD_SetAddress
*
*	��ڲ���:	 x1 - ��ʼˮƽ����   y1 - ��ʼ��ֱ����  
*              x2 - �յ�ˮƽ����   y2 - �յ㴹ֱ����	   
*	
*	��������:   ������Ҫ��ʾ����������		 			 
*****************************************************************************************************************************************/

void LCD_SetAddress(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2)		
{
	LCD_WriteCommand(0x2a);			//	�е�ַ���ã���X����
	LCD_WriteData_16bit(x1+LCD.X_Offset);
	LCD_WriteData_16bit(x2+LCD.X_Offset);

	LCD_WriteCommand(0x2b);			//	�е�ַ���ã���Y����
	LCD_WriteData_16bit(y1+LCD.Y_Offset);
	LCD_WriteData_16bit(y2+LCD.Y_Offset);

	LCD_WriteCommand(0x2c);			//	��ʼд���Դ棬��Ҫ��ʾ����ɫ����
}

/****************************************************************************************************************************************
*	�� �� ��:	LCD_SetColor
*
*	��ڲ���:	Color - Ҫ��ʾ����ɫ��ʾ����0x0000FF ��ʾ��ɫ
*
*	��������:	�˺����������û��ʵ���ɫ��������ʾ�ַ������㻭�ߡ���ͼ����ɫ
*
*	˵    ��:	1. Ϊ�˷����û�ʹ���Զ�����ɫ����ڲ��� Color ʹ��24λ RGB888����ɫ��ʽ���û����������ɫ��ʽ��ת��
*					2. 24λ����ɫ�У��Ӹ�λ����λ�ֱ��Ӧ R��G��B  3����ɫͨ��
*
*****************************************************************************************************************************************/

void LCD_SetColor(uint32_t Color)
{
	uint16_t Red_Value = 0, Green_Value = 0, Blue_Value = 0; //������ɫͨ����ֵ

	Red_Value   = (uint16_t)((Color&0x00F80000)>>8);   // ת���� 16λ ��RGB565��ɫ
	Green_Value = (uint16_t)((Color&0x0000FC00)>>5);
	Blue_Value  = (uint16_t)((Color&0x000000F8)>>3);

	LCD.Color = (uint16_t)(Red_Value | Green_Value | Blue_Value);  // ����ɫд��ȫ��LCD����		
}

/****************************************************************************************************************************************
*	�� �� ��:	LCD_SetBackColor
*
*	��ڲ���:	Color - Ҫ��ʾ����ɫ��ʾ����0x0000FF ��ʾ��ɫ
*
*	��������:	���ñ���ɫ,�˺������������Լ���ʾ�ַ��ı���ɫ
*
*	˵    ��:	1. Ϊ�˷����û�ʹ���Զ�����ɫ����ڲ��� Color ʹ��24λ RGB888����ɫ��ʽ���û����������ɫ��ʽ��ת��
*					2. 24λ����ɫ�У��Ӹ�λ����λ�ֱ��Ӧ R��G��B  3����ɫͨ��
*
*****************************************************************************************************************************************/

void LCD_SetBackColor(uint32_t Color)
{
	uint16_t Red_Value = 0, Green_Value = 0, Blue_Value = 0; //������ɫͨ����ֵ

	Red_Value   = (uint16_t)((Color&0x00F80000)>>8);   // ת���� 16λ ��RGB565��ɫ
	Green_Value = (uint16_t)((Color&0x0000FC00)>>5);
	Blue_Value  = (uint16_t)((Color&0x000000F8)>>3);

	LCD.BackColor = (uint16_t)(Red_Value | Green_Value | Blue_Value);	// ����ɫд��ȫ��LCD����			   	
}

/****************************************************************************************************************************************
*	�� �� ��:	LCD_SetDirection
*
*	��ڲ���:	direction - Ҫ��ʾ�ķ���
*
*	��������:	����Ҫ��ʾ�ķ���
*
*	˵    ��:   1. ��������� Direction_H ��Direction_V ��Direction_H_Flip ��Direction_V_Flip        
*              2. ʹ��ʾ�� LCD_DisplayDirection(Direction_H) ����������Ļ������ʾ
*
*****************************************************************************************************************************************/

void LCD_SetDirection(uint8_t direction)
{
	LCD.Direction = direction;    // д��ȫ��LCD����

   if( direction == Direction_H )   // ������ʾ
   {
      LCD_WriteCommand(0x36);    		// �Դ���ʿ��� ָ��������÷����Դ�ķ�ʽ
      LCD_WriteData_8bit(0x70);        // ������ʾ
      LCD.X_Offset   = 0;             // ���ÿ���������ƫ����
      LCD.Y_Offset   = 0;   
      LCD.Width      = LCD_Height;		// ���¸�ֵ������
      LCD.Height     = LCD_Width;		
   }
   else if( direction == Direction_V )
   {
      LCD_WriteCommand(0x36);    		// �Դ���ʿ��� ָ��������÷����Դ�ķ�ʽ
      LCD_WriteData_8bit(0x00);        // ��ֱ��ʾ
      LCD.X_Offset   = 0;              // ���ÿ���������ƫ����
      LCD.Y_Offset   = 0;     
      LCD.Width      = LCD_Width;		// ���¸�ֵ������
      LCD.Height     = LCD_Height;						
   }
   else if( direction == Direction_H_Flip )
   {
      LCD_WriteCommand(0x36);   			 // �Դ���ʿ��� ָ��������÷����Դ�ķ�ʽ
      LCD_WriteData_8bit(0xA0);         // ������ʾ�������·�ת��RGB���ظ�ʽ
      LCD.X_Offset   = 80;              // ���ÿ���������ƫ����
      LCD.Y_Offset   = 0;      
      LCD.Width      = LCD_Height;		 // ���¸�ֵ������
      LCD.Height     = LCD_Width;				
   }
   else if( direction == Direction_V_Flip )
   {
      LCD_WriteCommand(0x36);    		// �Դ���ʿ��� ָ��������÷����Դ�ķ�ʽ
      LCD_WriteData_8bit(0xC0);        // ��ֱ��ʾ �������·�ת��RGB���ظ�ʽ
      LCD.X_Offset   = 0;              // ���ÿ���������ƫ����
      LCD.Y_Offset   = 80;     
      LCD.Width      = LCD_Width;		// ���¸�ֵ������
      LCD.Height     = LCD_Height;				
   }   
}

/****************************************************************************************************************************************
*	�� �� ��:	LCD_SetAsciiFont
*
*	��ڲ���:	*fonts - Ҫ���õ�ASCII����
*
*	��������:	����ASCII���壬��ѡ��ʹ�� 3216/2412/2010/1608/1206 ���ִ�С������
*
*	˵    ��:	1. ʹ��ʾ�� LCD_SetAsciiFont(&ASCII_Font24) �������� 2412�� ASCII����
*					2. �����ģ����� lcd_fonts.c 			
*
*****************************************************************************************************************************************/

void LCD_SetAsciiFont(pFONT *Asciifonts)
{
  LCD_AsciiFonts = Asciifonts;
}

/****************************************************************************************************************************************
*	�� �� ��:	LCD_Clear
*
*	��������:	������������LCD���Ϊ LCD.BackColor ����ɫ
*
*	˵    ��:	���� LCD_SetBackColor() ����Ҫ����ı���ɫ���ٵ��øú�����������
*
*****************************************************************************************************************************************/

void LCD_Clear(void)
{
   LCD_SetAddress(0,0,LCD.Width-1,LCD.Height-1);	// ��������
	
	LCD_CS_LOW;
	LCD_DC_Data;     // ����ָ��ѡ�� ��������ߵ�ƽ���������δ��� ����	

// �޸�Ϊ16λ���ݿ��ȣ�д�����ݸ���Ч�ʣ�����Ҫ���	
   LCD_SPI.Init.DataSize 	= SPI_DATASIZE_16BIT;   //	16λ���ݿ���
   HAL_SPI_Init(&LCD_SPI);		
	
   LCD_SPI_Transmit(&LCD_SPI, LCD.BackColor, LCD.Width * LCD.Height) ;   // ��������

// �Ļ�8λ���ݿ��ȣ���Ϊָ��Ͳ������ݶ��ǰ���8λ�����
	LCD_SPI.Init.DataSize 	= SPI_DATASIZE_8BIT;    //	8λ���ݿ���
   HAL_SPI_Init(&LCD_SPI);
	LCD_CS_HIGH;
}

/****************************************************************************************************************************************
*	�� �� ��:	LCD_ClearRect
*
*	��ڲ���:	x - ��ʼˮƽ����
*					y - ��ʼ��ֱ����
*					width  - Ҫ�������ĺ��򳤶�
*					height - Ҫ���������������
*
*	��������:	�ֲ�������������ָ��λ�ö�Ӧ���������Ϊ LCD.BackColor ����ɫ
*
*	˵    ��:	1. ���� LCD_SetBackColor() ����Ҫ����ı���ɫ���ٵ��øú�����������
*				   2. ʹ��ʾ�� LCD_ClearRect( 10, 10, 100, 50) ���������(10,10)��ʼ�ĳ�100��50������
*
*****************************************************************************************************************************************/

void LCD_ClearRect(uint16_t x, uint16_t y, uint16_t width, uint16_t height)
{
   LCD_SetAddress( x, y, x+width-1, y+height-1);	// ��������	
	
	LCD_CS_LOW;
	LCD_DC_Data;     // ����ָ��ѡ�� ��������ߵ�ƽ���������δ��� ����	

// �޸�Ϊ16λ���ݿ��ȣ�д�����ݸ���Ч�ʣ�����Ҫ���	
   LCD_SPI.Init.DataSize 	= SPI_DATASIZE_16BIT;   //	16λ���ݿ���
   HAL_SPI_Init(&LCD_SPI);		
	
   LCD_SPI_Transmit(&LCD_SPI, LCD.BackColor, width*height) ;  // ��������

// �Ļ�8λ���ݿ��ȣ���Ϊָ��Ͳ������ݶ��ǰ���8λ�����
	LCD_SPI.Init.DataSize 	= SPI_DATASIZE_8BIT;    //	8λ���ݿ���
   HAL_SPI_Init(&LCD_SPI);
	LCD_CS_HIGH;

}

/****************************************************************************************************************************************
*	�� �� ��:	LCD_DrawPoint
*
*	��ڲ���:	x - ��ʼˮƽ����
*					y - ��ʼ��ֱ����
*					color  - Ҫ���Ƶ���ɫ��ʹ�� 24λ RGB888 ����ɫ��ʽ���û����������ɫ��ʽ��ת��
*
*	��������:	��ָ���������ָ����ɫ�ĵ�
*
*	˵    ��:	ʹ��ʾ�� LCD_DrawPoint( 10, 10, 0x0000FF) ��������(10,10)������ɫ�ĵ�
*
*****************************************************************************************************************************************/

void LCD_DrawPoint(uint16_t x,uint16_t y,uint32_t color)
{
	LCD_SetAddress(x,y,x,y);	//	�������� 

	LCD_WriteData_16bit(color)	;
}

/****************************************************************************************************************************************
*	�� �� ��:	LCD_DisplayChar
*
*	��ڲ���:	x - ��ʼˮƽ����
*					y - ��ʼ��ֱ����
*					c  - ASCII�ַ�
*
*	��������:	��ָ��������ʾָ�����ַ�
*
*	˵    ��:	1. ������Ҫ��ʾ�����壬����ʹ�� LCD_SetAsciiFont(&ASCII_Font24) ����Ϊ 2412��ASCII����
*					2.	������Ҫ��ʾ����ɫ������ʹ�� LCD_SetColor(0xff0000FF) ����Ϊ��ɫ
*					3. �����ö�Ӧ�ı���ɫ������ʹ�� LCD_SetBackColor(0x000000) ����Ϊ��ɫ�ı���ɫ
*					4. ʹ��ʾ�� LCD_DisplayChar( 10, 10, 'a') ��������(10,10)��ʾ�ַ� 'a'
*
*****************************************************************************************************************************************/

void LCD_DisplayChar(uint16_t x, uint16_t y,uint8_t c)
{
	uint16_t  index = 0, counter = 0 ,i = 0, w = 0;		// ��������
   uint8_t   disChar;		//�洢�ַ��ĵ�ַ

	c = c - 32; 	// ����ASCII�ַ���ƫ��

	for(index = 0; index < LCD_AsciiFonts->Sizes; index++)	
	{
		disChar = LCD_AsciiFonts->pTable[c*LCD_AsciiFonts->Sizes + index]; //��ȡ�ַ���ģֵ
		for(counter = 0; counter < 8; counter++)
		{ 
			if(disChar & 0x01)	
			{		
            LCD_Buff[i] =  LCD.Color;			// ��ǰģֵ��Ϊ0ʱ��ʹ�û���ɫ���
			}
			else		
			{		
            LCD_Buff[i] = LCD.BackColor;		//����ʹ�ñ���ɫ���Ƶ�
			}
			disChar >>= 1;
			i++;
         w++;
 			if( w == LCD_AsciiFonts->Width ) // ���д������ݴﵽ���ַ����ȣ����˳���ǰѭ��
			{								   // ������һ�ַ���д��Ļ���
				w = 0;
				break;
			}        
		}	
	}		
   LCD_SetAddress( x, y, x+LCD_AsciiFonts->Width-1, y+LCD_AsciiFonts->Height-1);	   // ��������	
   LCD_WriteBuff(LCD_Buff,LCD_AsciiFonts->Width*LCD_AsciiFonts->Height);          // д���Դ�
}

/****************************************************************************************************************************************
*	�� �� ��:	LCD_DisplayString
*
*	��ڲ���:	x - ��ʼˮƽ����
*					y - ��ʼ��ֱ����
*					p - ASCII�ַ������׵�ַ
*
*	��������:	��ָ��������ʾָ�����ַ���
*
*	˵    ��:	1. ������Ҫ��ʾ�����壬����ʹ�� LCD_SetAsciiFont(&ASCII_Font24) ����Ϊ 2412��ASCII����
*					2.	������Ҫ��ʾ����ɫ������ʹ�� LCD_SetColor(0x0000FF) ����Ϊ��ɫ
*					3. �����ö�Ӧ�ı���ɫ������ʹ�� LCD_SetBackColor(0x000000) ����Ϊ��ɫ�ı���ɫ
*					4. ʹ��ʾ�� LCD_DisplayString( 10, 10, "") ������ʼ����Ϊ(10,10)�ĵط���ʾ�ַ���""
*
*****************************************************************************************************************************************/

void LCD_DisplayString( uint16_t x, uint16_t y, char *p) 
{  
	while ((x < LCD.Width) && (*p != 0))	//�ж���ʾ�����Ƿ񳬳���ʾ�������ַ��Ƿ�Ϊ���ַ�
	{
		 LCD_DisplayChar( x,y,*p);
		 x += LCD_AsciiFonts->Width; //��ʾ��һ���ַ�
		 p++;	//ȡ��һ���ַ���ַ
	}
}

/****************************************************************************************************************************************
*	�� �� ��:	LCD_SetTextFont
*
*	��ڲ���:	*fonts - Ҫ���õ��ı�����
*
*	��������:	�����ı����壬�������ĺ�ASCII�ַ���
*
*	˵    ��:	1. ��ѡ��ʹ�� 3232/2424/2020/1616/1212 ���ִ�С���������壬
*						���Ҷ�Ӧ������ASCII����Ϊ 3216/2412/2010/1608/1206
*					2. �����ģ����� lcd_fonts.c 
*					3. �����ֿ�ʹ�õ���С�ֿ⣬���õ��˶�Ӧ�ĺ�����ȥȡģ
*					4. ʹ��ʾ�� LCD_SetTextFont(&CH_Font24) �������� 2424�����������Լ�2412��ASCII�ַ�����
*
*****************************************************************************************************************************************/

void LCD_SetTextFont(pFONT *fonts)
{
	LCD_CHFonts = fonts;		// ������������
	switch(fonts->Width )
	{
		case 12:	LCD_AsciiFonts = &ASCII_Font12;	break;	// ����ASCII�ַ�������Ϊ 1206
		case 16:	LCD_AsciiFonts = &ASCII_Font16;	break;	// ����ASCII�ַ�������Ϊ 1608
		case 20:	LCD_AsciiFonts = &ASCII_Font20;	break;	// ����ASCII�ַ�������Ϊ 2010	
		case 24:	LCD_AsciiFonts = &ASCII_Font24;	break;	// ����ASCII�ַ�������Ϊ 2412
		case 32:	LCD_AsciiFonts = &ASCII_Font32;	break;	// ����ASCII�ַ�������Ϊ 3216		
		default: break;
	}
}
/******************************************************************************************************************************************
*	�� �� ��:	LCD_DisplayChinese
*
*	��ڲ���:	x - ��ʼˮƽ����
*					y - ��ʼ��ֱ����
*					pText - �����ַ�
*
*	��������:	��ָ��������ʾָ���ĵ��������ַ�
*
*	˵    ��:	1. ������Ҫ��ʾ�����壬����ʹ�� LCD_SetTextFont(&CH_Font24) ����Ϊ 2424�����������Լ�2412��ASCII�ַ�����
*					2.	������Ҫ��ʾ����ɫ������ʹ�� LCD_SetColor(0xff0000FF) ����Ϊ��ɫ
*					3. �����ö�Ӧ�ı���ɫ������ʹ�� LCD_SetBackColor(0xff000000) ����Ϊ��ɫ�ı���ɫ
*					4. ʹ��ʾ�� LCD_DisplayChinese( 10, 10, "��") ��������(10,10)��ʾ�����ַ�"��"
*
*****************************************************************************************************************************************/

void LCD_DisplayChinese(uint16_t x, uint16_t y, char *pText) 
{
	uint16_t  i=0,index = 0, counter = 0;	// ��������
	uint16_t  addr;	// ��ģ��ַ
   uint8_t   disChar;	//��ģ��ֵ
	uint16_t  Xaddress = 0; //ˮƽ����

	while(1)
	{		
		// �Ա������еĺ��ֱ��룬���Զ�λ�ú�����ģ�ĵ�ַ		
		if ( *(LCD_CHFonts->pTable + (i+1)*LCD_CHFonts->Sizes + 0)==*pText && *(LCD_CHFonts->pTable + (i+1)*LCD_CHFonts->Sizes + 1)==*(pText+1) )	
		{   
			addr=i;	// ��ģ��ַƫ��
			break;
		}				
		i+=2;	// ÿ�������ַ�����ռ���ֽ�

		if(i >= LCD_CHFonts->Table_Rows)	break;	// ��ģ�б�������Ӧ�ĺ���	
	}	
	i=0;
	for(index = 0; index <LCD_CHFonts->Sizes; index++)
	{	
		disChar = *(LCD_CHFonts->pTable + (addr)*LCD_CHFonts->Sizes + index);	// ��ȡ��Ӧ����ģ��ַ
		
		for(counter = 0; counter < 8; counter++)
		{ 
			if(disChar & 0x01)	
			{		
            LCD_Buff[i] =  LCD.Color;			// ��ǰģֵ��Ϊ0ʱ��ʹ�û���ɫ���
			}
			else		
			{		
            LCD_Buff[i] = LCD.BackColor;		// ����ʹ�ñ���ɫ���Ƶ�
			}
         i++;
			disChar >>= 1;
			Xaddress++;  //ˮƽ�����Լ�
			
			if( Xaddress == LCD_CHFonts->Width ) 	//	���ˮƽ����ﵽ���ַ����ȣ����˳���ǰѭ��
			{														//	������һ�еĻ���
				Xaddress = 0;
				break;
			}
		}	
	}	
   LCD_SetAddress( x, y, x+LCD_CHFonts->Width-1, y+LCD_CHFonts->Height-1);	   // ��������	
   LCD_WriteBuff(LCD_Buff,LCD_CHFonts->Width*LCD_CHFonts->Height);            // д���Դ�
}

/*****************************************************************************************************************************************
*	�� �� ��:	LCD_DisplayText
*
*	��ڲ���:	x - ��ʼˮƽ����
*					y - ��ʼ��ֱ����
*					pText - �ַ�����������ʾ���Ļ���ASCII�ַ�
*
*	��������:	��ָ��������ʾָ�����ַ���
*
*	˵    ��:	1. ������Ҫ��ʾ�����壬����ʹ�� LCD_SetTextFont(&CH_Font24) ����Ϊ 2424�����������Լ�2412��ASCII�ַ�����
*					2.	������Ҫ��ʾ����ɫ������ʹ�� LCD_SetColor(0xff0000FF) ����Ϊ��ɫ
*					3. �����ö�Ӧ�ı���ɫ������ʹ�� LCD_SetBackColor(0xff000000) ����Ϊ��ɫ�ı���ɫ
*					4. ʹ��ʾ�� LCD_DisplayChinese( 10, 10, "�Ƽ�STM32") ��������(10,10)��ʾ�ַ���"�Ƽ�STM32"
*
*****************************************************************************************************************************************/

void LCD_DisplayText(uint16_t x, uint16_t y, char *pText) 
{  
 	
	while(*pText != 0)	// �ж��Ƿ�Ϊ���ַ�
	{
		if(*pText<=0x7F)	// �ж��Ƿ�ΪASCII��
		{
			LCD_DisplayChar(x,y,*pText);	// ��ʾASCII
			x+=LCD_AsciiFonts->Width;				// ˮƽ���������һ���ַ���
			pText++;								// �ַ�����ַ+1
		}
		else					// ���ַ�Ϊ����
		{			
			LCD_DisplayChinese(x,y,pText);	// ��ʾ����
			x+=LCD_CHFonts->Width;				// ˮƽ���������һ���ַ���
			pText+=2;								// �ַ�����ַ+2�����ֵı���Ҫ2�ֽ�
		}
	}	
}

/*****************************************************************************************************************************************
*	�� �� ��:	LCD_ShowNumMode
*
*	��ڲ���:	mode - ���ñ�������ʾģʽ
*
*	��������:	���ñ�����ʾʱ����λ��0���ǲ��ո񣬿�������� Fill_Space ���ո�Fill_Zero �����
*
*	˵    ��:   1. ֻ�� LCD_DisplayNumber() ��ʾ���� �� LCD_DisplayDecimals()��ʾС�� �����������õ�
*					2. ʹ��ʾ�� LCD_ShowNumMode(Fill_Zero) ���ö���λ���0������ 123 ������ʾΪ 000123
*
*****************************************************************************************************************************************/

void LCD_ShowNumMode(uint8_t mode)
{
	LCD.ShowNum_Mode = mode;
}

/*****************************************************************************************************************************************
*	�� �� ��:	LCD_DisplayNumber
*
*	��ڲ���:	x - ��ʼˮƽ����
*					y - ��ʼ��ֱ����
*					number - Ҫ��ʾ������,��Χ�� -2147483648~2147483647 ֮��
*					len - ���ֵ�λ�������λ������len��������ʵ�ʳ�������������Ҫ��ʾ��������Ԥ��һ��λ�ķ�����ʾ�ռ�
*
*	��������:	��ָ��������ʾָ������������
*
*	˵    ��:	1. ������Ҫ��ʾ�����壬����ʹ�� LCD_SetAsciiFont(&ASCII_Font24) ����Ϊ��ASCII�ַ�����
*					2.	������Ҫ��ʾ����ɫ������ʹ�� LCD_SetColor(0x0000FF) ����Ϊ��ɫ
*					3. �����ö�Ӧ�ı���ɫ������ʹ�� LCD_SetBackColor(0x000000) ����Ϊ��ɫ�ı���ɫ
*					4. ʹ��ʾ�� LCD_DisplayNumber( 10, 10, a, 5) ��������(10,10)��ʾָ������a,�ܹ�5λ������λ��0��ո�
*						���� a=123 ʱ������� LCD_ShowNumMode()����������ʾ  123(ǰ�������ո�λ) ����00123
*						
*****************************************************************************************************************************************/

void  LCD_DisplayNumber( uint16_t x, uint16_t y, int32_t number, uint8_t len) 
{  
	char   Number_Buffer[15];				// ���ڴ洢ת������ַ���

	if( LCD.ShowNum_Mode == Fill_Zero)	// ����λ��0
	{
		sprintf( Number_Buffer , "%0*ld",len, (long)number );	// �� number ת�����ַ�����������ʾ		
	}
	else			// ����λ���ո�
	{	
		sprintf( Number_Buffer , "%*ld",len, (long)number );	// �� number ת�����ַ�����������ʾ		
	}
	
	LCD_DisplayString( x, y,(char *)Number_Buffer) ;  // ��ת���õ����ַ�����ʾ����
	
}

/***************************************************************************************************************************************
*	�� �� ��:	LCD_DisplayDecimals
*
*	��ڲ���:	x - ��ʼˮƽ����
*					y - ��ʼ��ֱ����
*					decimals - Ҫ��ʾ������, double��ȡֵ1.7 x 10^��-308��~ 1.7 x 10^��+308����������ȷ��׼ȷ����Чλ��Ϊ15~16λ
*
*       			len - ������������λ��������С����͸��ţ�����ʵ�ʵ���λ��������ָ������λ��������ʵ�ʵ��ܳ���λ�����
*							ʾ��1��С�� -123.123 ��ָ�� len <=8 �Ļ�����ʵ���ճ���� -123.123
*							ʾ��2��С�� -123.123 ��ָ�� len =10 �Ļ�����ʵ�����   -123.123(����ǰ����������ո�λ) 
*							ʾ��3��С�� -123.123 ��ָ�� len =10 �Ļ��������ú��� LCD_ShowNumMode() ����Ϊ���0ģʽʱ��ʵ����� -00123.123 
*
*					decs - Ҫ������С��λ������С����ʵ��λ��������ָ����С��λ����ָ���Ŀ��������������
*							 ʾ����1.12345 ��ָ�� decs Ϊ4λ�Ļ�����������Ϊ1.1235
*
*	��������:	��ָ��������ʾָ���ı���������С��
*
*	˵    ��:	1. ������Ҫ��ʾ�����壬����ʹ�� LCD_SetAsciiFont(&ASCII_Font24) ����Ϊ��ASCII�ַ�����
*					2.	������Ҫ��ʾ����ɫ������ʹ�� LCD_SetColor(0x0000FF) ����Ϊ��ɫ
*					3. �����ö�Ӧ�ı���ɫ������ʹ�� LCD_SetBackColor(0x000000) ����Ϊ��ɫ�ı���ɫ
*					4. ʹ��ʾ�� LCD_DisplayDecimals( 10, 10, a, 5, 3) ��������(10,10)��ʾ�ֱ���a,�ܳ���Ϊ5λ�����б���3λС��
*						
*****************************************************************************************************************************************/

void  LCD_DisplayDecimals( uint16_t x, uint16_t y, double decimals, uint8_t len, uint8_t decs) 
{  
	char  Number_Buffer[20];				// ���ڴ洢ת������ַ���
	
	if( LCD.ShowNum_Mode == Fill_Zero)	// ����λ���0ģʽ
	{
		sprintf( Number_Buffer , "%0*.*lf",len,decs, decimals );	// �� number ת�����ַ�����������ʾ		
	}
	else		// ����λ���ո�
	{
		sprintf( Number_Buffer , "%*.*lf",len,decs, decimals );	// �� number ת�����ַ�����������ʾ		
	}
	
	LCD_DisplayString( x, y,(char *)Number_Buffer) ;	// ��ת���õ����ַ�����ʾ����
}


/***************************************************************************************************************************************
*	�� �� ��: LCD_DrawLine
*
*	��ڲ���: x1 - ��� ˮƽ����
*			 	 y1 - ��� ��ֱ����
*
*				 x2 - �յ� ˮƽ����
*            y2 - �յ� ��ֱ����
*
*	��������: ������֮�仭��
*
*	˵    ��: �ú�����ֲ��ST�ٷ������������
*						 
*****************************************************************************************************************************************/

#define ABS(X)  ((X) > 0 ? (X) : -(X))    

void LCD_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
	int16_t deltax = 0, deltay = 0, x = 0, y = 0, xinc1 = 0, xinc2 = 0, 
	yinc1 = 0, yinc2 = 0, den = 0, num = 0, numadd = 0, numpixels = 0, 
	curpixel = 0;

	deltax = ABS(x2 - x1);        /* The difference between the x's */
	deltay = ABS(y2 - y1);        /* The difference between the y's */
	x = x1;                       /* Start x off at the first pixel */
	y = y1;                       /* Start y off at the first pixel */

	if (x2 >= x1)                 /* The x-values are increasing */
	{
	 xinc1 = 1;
	 xinc2 = 1;
	}
	else                          /* The x-values are decreasing */
	{
	 xinc1 = -1;
	 xinc2 = -1;
	}

	if (y2 >= y1)                 /* The y-values are increasing */
	{
	 yinc1 = 1;
	 yinc2 = 1;
	}
	else                          /* The y-values are decreasing */
	{
	 yinc1 = -1;
	 yinc2 = -1;
	}

	if (deltax >= deltay)         /* There is at least one x-value for every y-value */
	{
	 xinc1 = 0;                  /* Don't change the x when numerator >= denominator */
	 yinc2 = 0;                  /* Don't change the y for every iteration */
	 den = deltax;
	 num = deltax / 2;
	 numadd = deltay;
	 numpixels = deltax;         /* There are more x-values than y-values */
	}
	else                          /* There is at least one y-value for every x-value */
	{
	 xinc2 = 0;                  /* Don't change the x for every iteration */
	 yinc1 = 0;                  /* Don't change the y when numerator >= denominator */
	 den = deltay;
	 num = deltay / 2;
	 numadd = deltax;
	 numpixels = deltay;         /* There are more y-values than x-values */
	}
	for (curpixel = 0; curpixel <= numpixels; curpixel++)
	{
	 LCD_DrawPoint(x,y,LCD.Color);             /* Draw the current pixel */
	 num += numadd;              /* Increase the numerator by the top of the fraction */
	 if (num >= den)             /* Check if numerator >= denominator */
	 {
		num -= den;               /* Calculate the new numerator value */
		x += xinc1;               /* Change the x as appropriate */
		y += yinc1;               /* Change the y as appropriate */
	 }
	 x += xinc2;                 /* Change the x as appropriate */
	 y += yinc2;                 /* Change the y as appropriate */
	}  
}

/***************************************************************************************************************************************
*	�� �� ��: LCD_DrawLine_V
*
*	��ڲ���: x - ˮƽ����
*			 	 y - ��ֱ����
*				 height - ��ֱ����
*
*	��������: ��ָ��λ�û���ָ�������� ��ֱ ��
*
*	˵    ��: 1. �ú�����ֲ��ST�ٷ������������
*				 2. Ҫ���Ƶ������ܳ�����Ļ����ʾ����		
*            3. ���ֻ�ǻ���ֱ���ߣ�����ʹ�ô˺������ٶȱ� LCD_DrawLine ��ܶ�
*  ���ܲ��ԣ�
*****************************************************************************************************************************************/

void LCD_DrawLine_V(uint16_t x, uint16_t y, uint16_t height)
{
   uint16_t i ; // ��������

	for (i = 0; i < height; i++)
	{
       LCD_Buff[i] =  LCD.Color;  // д�뻺����
   }   
   LCD_SetAddress( x, y, x, y+height-1);	     // ��������	

   LCD_WriteBuff(LCD_Buff,height);          // д���Դ�
}

/***************************************************************************************************************************************
*	�� �� ��: LCD_DrawLine_H
*
*	��ڲ���: x - ˮƽ����
*			 	 y - ��ֱ����
*				 width  - ˮƽ����
*
*	��������: ��ָ��λ�û���ָ�������� ˮƽ ��
*
*	˵    ��: 1. �ú�����ֲ��ST�ٷ������������
*				 2. Ҫ���Ƶ������ܳ�����Ļ����ʾ����		
*            3. ���ֻ�ǻ� ˮƽ ���ߣ�����ʹ�ô˺������ٶȱ� LCD_DrawLine ��ܶ�
*  ���ܲ��ԣ�
*****************************************************************************************************************************************/

void LCD_DrawLine_H(uint16_t x, uint16_t y, uint16_t width)
{
   uint16_t i ; // ��������

	for (i = 0; i < width; i++)
	{
       LCD_Buff[i] =  LCD.Color;  // д�뻺����
   }   
   LCD_SetAddress( x, y, x+width-1, y);	     // ��������	

   LCD_WriteBuff(LCD_Buff,width);          // д���Դ�
}
/***************************************************************************************************************************************
*	�� �� ��: LCD_DrawRect
*
*	��ڲ���: x - ˮƽ����
*			 	 y - ��ֱ����
*			 	 width  - ˮƽ����
*				 height - ��ֱ����
*
*	��������: ��ָ��λ�û���ָ�������ľ�������
*
*	˵    ��: 1. �ú�����ֲ��ST�ٷ������������
*				 2. Ҫ���Ƶ������ܳ�����Ļ����ʾ����
*						 
*****************************************************************************************************************************************/

void LCD_DrawRect(uint16_t x, uint16_t y, uint16_t width, uint16_t height)
{
   // ����ˮƽ��
   LCD_DrawLine_H( x,  y,  width);           
   LCD_DrawLine_H( x,  y+height-1,  width);

   // ���ƴ�ֱ��
   LCD_DrawLine_V( x,  y,  height);
   LCD_DrawLine_V( x+width-1,  y,  height);
}


/***************************************************************************************************************************************
*	�� �� ��: LCD_DrawCircle
*
*	��ڲ���: x - Բ�� ˮƽ����
*			 	 y - Բ�� ��ֱ����
*			 	 r  - �뾶
*
*	��������: ������ (x,y) ���ư뾶Ϊ r ��Բ������
*
*	˵    ��: 1. �ú�����ֲ��ST�ٷ������������
*				 2. Ҫ���Ƶ������ܳ�����Ļ����ʾ����
*
*****************************************************************************************************************************************/

void LCD_DrawCircle(uint16_t x, uint16_t y, uint16_t r)
{
	int Xadd = -r, Yadd = 0, err = 2-2*r, e2;
	do {   

		LCD_DrawPoint(x-Xadd,y+Yadd,LCD.Color);
		LCD_DrawPoint(x+Xadd,y+Yadd,LCD.Color);
		LCD_DrawPoint(x+Xadd,y-Yadd,LCD.Color);
		LCD_DrawPoint(x-Xadd,y-Yadd,LCD.Color);
		
		e2 = err;
		if (e2 <= Yadd) {
			err += ++Yadd*2+1;
			if (-Xadd == Yadd && e2 <= Xadd) e2 = 0;
		}
		if (e2 > Xadd) err += ++Xadd*2+1;
    }
    while (Xadd <= 0);   
}


/***************************************************************************************************************************************
*	�� �� ��: LCD_DrawEllipse
*
*	��ڲ���: x - Բ�� ˮƽ����
*			 	 y - Բ�� ��ֱ����
*			 	 r1  - ˮƽ����ĳ���
*				 r2  - ��ֱ����ĳ���
*
*	��������: ������ (x,y) ����ˮƽ����Ϊ r1 ��ֱ����Ϊ r2 ����Բ����
*
*	˵    ��: 1. �ú�����ֲ��ST�ٷ������������
*				 2. Ҫ���Ƶ������ܳ�����Ļ����ʾ����
*
*****************************************************************************************************************************************/

void LCD_DrawEllipse(int x, int y, int r1, int r2)
{
  int Xadd = -r1, Yadd = 0, err = 2-2*r1, e2;
  float K = 0, rad1 = 0, rad2 = 0;
   
  rad1 = r1;
  rad2 = r2;
  
  if (r1 > r2)
  { 
    do {
      K = (float)(rad1/rad2);
		 
		LCD_DrawPoint(x-Xadd,y+(uint16_t)(Yadd/K),LCD.Color);
		LCD_DrawPoint(x+Xadd,y+(uint16_t)(Yadd/K),LCD.Color);
		LCD_DrawPoint(x+Xadd,y-(uint16_t)(Yadd/K),LCD.Color);
		LCD_DrawPoint(x-Xadd,y-(uint16_t)(Yadd/K),LCD.Color);     
		 
      e2 = err;
      if (e2 <= Yadd) {
        err += ++Yadd*2+1;
        if (-Xadd == Yadd && e2 <= Xadd) e2 = 0;
      }
      if (e2 > Xadd) err += ++Xadd*2+1;
    }
    while (Xadd <= 0);
  }
  else
  {
    Yadd = -r2; 
    Xadd = 0;
    do { 
      K = (float)(rad2/rad1);

		LCD_DrawPoint(x-(uint16_t)(Xadd/K),y+Yadd,LCD.Color);
		LCD_DrawPoint(x+(uint16_t)(Xadd/K),y+Yadd,LCD.Color);
		LCD_DrawPoint(x+(uint16_t)(Xadd/K),y-Yadd,LCD.Color);
		LCD_DrawPoint(x-(uint16_t)(Xadd/K),y-Yadd,LCD.Color);  
		 
      e2 = err;
      if (e2 <= Xadd) {
        err += ++Xadd*3+1;
        if (-Yadd == Xadd && e2 <= Yadd) e2 = 0;
      }
      if (e2 > Yadd) err += ++Yadd*3+1;     
    }
    while (Yadd <= 0);
  }
}

/***************************************************************************************************************************************
*	�� �� ��: LCD_FillCircle
*
*	��ڲ���: x - Բ�� ˮƽ����
*			 	 y - Բ�� ��ֱ����
*			 	 r  - �뾶
*
*	��������: ������ (x,y) ���뾶Ϊ r ��Բ������
*
*	˵    ��: 1. �ú�����ֲ��ST�ٷ������������
*				 2. Ҫ���Ƶ������ܳ�����Ļ����ʾ����
*
*****************************************************************************************************************************************/

void LCD_FillCircle(uint16_t x, uint16_t y, uint16_t r)
{
  int32_t  D;    /* Decision Variable */ 
  uint32_t  CurX;/* Current X Value */
  uint32_t  CurY;/* Current Y Value */ 
  
  D = 3 - (r << 1);
  
  CurX = 0;
  CurY = r;
  
  while (CurX <= CurY)
  {
    if(CurY > 0) 
    { 
      LCD_DrawLine_V(x - CurX, y - CurY,2*CurY);
      LCD_DrawLine_V(x + CurX, y - CurY,2*CurY);
    }
    
    if(CurX > 0) 
    {
		// LCD_DrawLine(x - CurY, y - CurX,x - CurY,y - CurX + 2*CurX);
		// LCD_DrawLine(x + CurY, y - CurX,x + CurY,y - CurX + 2*CurX); 	

      LCD_DrawLine_V(x - CurY, y - CurX,2*CurX);
      LCD_DrawLine_V(x + CurY, y - CurX,2*CurX);
    }
    if (D < 0)
    { 
      D += (CurX << 2) + 6;
    }
    else
    {
      D += ((CurX - CurY) << 2) + 10;
      CurY--;
    }
    CurX++;
  }
  LCD_DrawCircle(x, y, r);  
}

/***************************************************************************************************************************************
*	�� �� ��: LCD_FillRect
*
*	��ڲ���: x - ˮƽ����
*			 	 y - ��ֱ����
*			 	 width  - ˮƽ����
*				 height -��ֱ����
*
*	��������: ������ (x,y) ���ָ��������ʵ�ľ���
*
*	˵    ��: Ҫ���Ƶ������ܳ�����Ļ����ʾ����
*						 
*****************************************************************************************************************************************/

void LCD_FillRect(uint16_t x, uint16_t y, uint16_t width, uint16_t height)
{
   LCD_SetAddress( x, y, x+width-1, y+height-1);	// ��������	
	
	LCD_DC_Data;     // ����ָ��ѡ�� ��������ߵ�ƽ���������δ��� ����	

// �޸�Ϊ16λ���ݿ��ȣ�д�����ݸ���Ч�ʣ�����Ҫ���	
   LCD_SPI.Init.DataSize 	= SPI_DATASIZE_16BIT;   //	16λ���ݿ���
   HAL_SPI_Init(&LCD_SPI);		
	
   LCD_SPI_Transmit(&LCD_SPI, LCD.Color, width*height) ;

// �Ļ�8λ���ݿ��ȣ���Ϊָ��Ͳ������ݶ��ǰ���8λ�����
	LCD_SPI.Init.DataSize 	= SPI_DATASIZE_8BIT;    //	8λ���ݿ���
   HAL_SPI_Init(&LCD_SPI);
}


/***************************************************************************************************************************************
*	�� �� ��: LCD_DrawImage
*
*	��ڲ���: x - ��ʼˮƽ����
*				 y - ��ʼ��ֱ����
*			 	 width  - ͼƬ��ˮƽ����
*				 height - ͼƬ�Ĵ�ֱ����
*				*pImage - ͼƬ���ݴ洢�����׵�ַ
*
*	��������: ��ָ�����괦��ʾͼƬ
*
*	˵    ��: 1.Ҫ��ʾ��ͼƬ��Ҫ���Ƚ���ȡģ����ϤͼƬ�ĳ��ȺͿ���
*            2.ʹ�� LCD_SetColor() �������û���ɫ��LCD_SetBackColor() ���ñ���ɫ
*						 
*****************************************************************************************************************************************/

void 	LCD_DrawImage(uint16_t x,uint16_t y,uint16_t width,uint16_t height,const uint8_t *pImage) 
{  
   uint8_t   disChar;	         // ��ģ��ֵ
	uint16_t  Xaddress = x;       // ˮƽ����
 	uint16_t  Yaddress = y;       // ��ֱ����  
	uint16_t  i=0,j=0,m=0;        // ��������
	uint16_t  BuffCount = 0;      // ����������
   uint16_t  Buff_Height = 0;    // ������������

// ��Ϊ��������С���ޣ���Ҫ�ֶ��д��
   Buff_Height = (sizeof(LCD_Buff)/2) / height;    // ���㻺�����ܹ�д��ͼƬ�Ķ�����

	for(i = 0; i <height; i++)             // ѭ������д��
	{
		for(j = 0; j <(float)width/8; j++)  
		{
			disChar = *pImage;

			for(m = 0; m < 8; m++)
			{ 
				if(disChar & 0x01)	
				{		
               LCD_Buff[BuffCount] =  LCD.Color;			// ��ǰģֵ��Ϊ0ʱ��ʹ�û���ɫ���
				}
				else		
				{		
				   LCD_Buff[BuffCount] = LCD.BackColor;		//����ʹ�ñ���ɫ���Ƶ�
				}
				disChar >>= 1;     // ģֵ��λ
				Xaddress++;        // ˮƽ�����Լ�
				BuffCount++;       // ����������       
				if( (Xaddress - x)==width ) // ���ˮƽ����ﵽ���ַ����ȣ����˳���ǰѭ��,������һ�еĻ���		
				{											 
					Xaddress = x;				                 
					break;
				}
			}	
			pImage++;			
		}
      if( BuffCount == Buff_Height*width  )  // �ﵽ�������������ɵ��������ʱ
      {
         BuffCount = 0; // ������������0

         LCD_SetAddress( x, Yaddress , x+width-1, Yaddress+Buff_Height-1);	// ��������	
         LCD_WriteBuff(LCD_Buff,width*Buff_Height);          // д���Դ�     

         Yaddress = Yaddress+Buff_Height;    // ������ƫ�ƣ���ʼд����һ��������
      }     
      if( (i+1)== height ) // �������һ��ʱ
      {
         LCD_SetAddress( x, Yaddress , x+width-1,i+y);	   // ��������	
         LCD_WriteBuff(LCD_Buff,width*(i+1+y-Yaddress));    // д���Դ�     
      }
	}	
}


/***************************************************************************************************************************************
*	�� �� ��: LCD_CopyBuffer
*
*	��ڲ���: x - ��ʼˮƽ����
*				 y - ��ʼ��ֱ����
*			 	 width  - Ŀ�������ˮƽ����
*				 height - Ŀ������Ĵ�ֱ����
*				*pImage - ���ݴ洢�����׵�ַ
*
*	��������: ��ָ�����괦��ֱ�ӽ����ݸ��Ƶ���Ļ���Դ�
*
*	˵    ��: �������ƺ�������������ֲ LVGL ���߽�����ͷ�ɼ���ͼ����ʾ����
*						 
*****************************************************************************************************************************************/

void	LCD_CopyBuffer(uint16_t x, uint16_t y,uint16_t width,uint16_t height,const uint16_t *DataBuff)
{
	
	LCD_SetAddress(x,y,x+width-1,y+height-1);

	LCD_CS_LOW;
	LCD_DC_Data;     // ����ָ��ѡ�� ��������ߵ�ƽ���������δ��� ����	

// �޸�Ϊ16λ���ݿ��ȣ�д�����ݸ���Ч�ʣ�����Ҫ���	
   LCD_SPI.Init.DataSize 	= SPI_DATASIZE_16BIT;   //	16λ���ݿ���
   HAL_SPI_Init(&LCD_SPI);		
	
	LCD_SPI_TransmitBuffer(&LCD_SPI, (uint16_t *)DataBuff,width * height) ;
	
//	HAL_SPI_Transmit(&hspi5, (uint8_t *)DataBuff, (x2-x1+1) * (y2-y1+1), 1000) ;
	
// �Ļ�8λ���ݿ��ȣ���Ϊָ��Ͳ������ݶ��ǰ���8λ�����
	LCD_SPI.Init.DataSize 	= SPI_DATASIZE_8BIT;    //	8λ���ݿ���
   HAL_SPI_Init(&LCD_SPI);		
	LCD_CS_HIGH;
	
}

HAL_StatusTypeDef LCD_CopyBufferAsync(uint16_t x, uint16_t y, uint16_t width, uint16_t height, const uint16_t *DataBuff)
{
	return LCD_CopyBufferAsyncCallback(x, y, width, height, DataBuff, NULL, NULL);
}

HAL_StatusTypeDef LCD_CopyBufferAsyncCallback(uint16_t x, uint16_t y, uint16_t width, uint16_t height,
											 const uint16_t *DataBuff,
											 LCD_TransferCallback_t callback,
											 void *context)
{
	HAL_StatusTypeDef status;
	uint32_t primask;

	if ((DataBuff == NULL) || (width == 0U) || (height == 0U))
	{
		return HAL_ERROR;
	}

	if (s_lcd_spi_tx_busy != 0U)
	{
		return HAL_BUSY;
	}

	LCD_SPI_FinalizeTransfer();

	LCD_SetAddress(x, y, x + width - 1U, y + height - 1U);
	LCD_CS_LOW;
	LCD_DC_Data;

	status = LCD_SPI_SetDataSize(SPI_DATASIZE_16BIT);
	if (status != HAL_OK)
	{
		LCD_CS_HIGH;
		return status;
	}

	primask = __get_PRIMASK();
	__disable_irq();
	s_lcd_spi_tx_callback = callback;
	s_lcd_spi_tx_callback_context = context;
	if (primask == 0U)
	{
		__enable_irq();
	}

	status = LCD_SPI_TransmitBufferAsync(&LCD_SPI, DataBuff, (uint32_t)width * (uint32_t)height);
	if (status != HAL_OK)
	{
		primask = __get_PRIMASK();
		__disable_irq();
		s_lcd_spi_tx_callback = NULL;
		s_lcd_spi_tx_callback_context = NULL;
		if (primask == 0U)
		{
			__enable_irq();
		}
		(void)LCD_SPI_SetDataSize(SPI_DATASIZE_8BIT);
		LCD_CS_HIGH;
	}

	return status;
}

HAL_StatusTypeDef LCD_WaitTransmitDone(uint32_t timeout_ms)
{
	uint32_t tickstart;
	HAL_StatusTypeDef status;

	tickstart = HAL_GetTick();
	while (s_lcd_spi_tx_busy != 0U)
	{
		LCD_TransferService();
		if ((timeout_ms != HAL_MAX_DELAY) && ((HAL_GetTick() - tickstart) >= timeout_ms))
		{
			s_lcd_spi_tx_status = HAL_TIMEOUT;
			LCD_ResetTransferState();
			break;
		}
	}

	status = s_lcd_spi_tx_status;
	LCD_SPI_FinalizeTransfer();

	return status;
}

uint8_t LCD_IsTransmitBusy(void)
{
	return s_lcd_spi_tx_busy;
}

void LCD_ResetTransferState(void)
{
	uint32_t primask;
	LCD_TransferCallback_t callback;
	void *callback_context;

	if (s_lcd_bdma_hw_busy != 0U)
	{
		(void)HAL_SPI_Abort(&LCD_SPI);
	}

	primask = __get_PRIMASK();
	__disable_irq();
	callback = s_lcd_spi_tx_callback;
	callback_context = s_lcd_spi_tx_callback_context;
	s_lcd_spi_tx_callback = NULL;
	s_lcd_spi_tx_callback_context = NULL;
	s_lcd_spi_tx_busy = 0U;
	s_lcd_bdma_hw_busy = 0U;
	s_lcd_bdma_active_slot = LCD_BDMA_NO_ACTIVE_SLOT;
	s_lcd_bdma_slot_state[0] = LCD_BDMA_SLOT_FREE;
	s_lcd_bdma_slot_state[1] = LCD_BDMA_SLOT_FREE;
	s_lcd_spi_tx_source = NULL;
	s_lcd_spi_tx_total = 0U;
	s_lcd_spi_tx_source_pos = 0U;
	if (s_lcd_spi_tx_status == HAL_BUSY)
	{
		s_lcd_spi_tx_status = HAL_OK;
	}
	s_lcd_spi_needs_finalize = 1U;
	if (primask == 0U)
	{
		__enable_irq();
	}

	LCD_SPI_FinalizeTransfer();
	if (callback != NULL)
	{
		callback(HAL_ERROR, callback_context);
	}
}

/**********************************************************************************************************************************
*
* ���¼��������޸���HAL�Ŀ⺯����Ŀ����Ϊ��SPI�������ݲ������ݳ��ȵ�д�룬��������������ٶ�
*
*****************************************************************************************************************************/


/**
  * @brief Handle SPI Communication Timeout.
  * @param hspi: pointer to a SPI_HandleTypeDef structure that contains
  *              the configuration information for SPI module.
  * @param Flag: SPI flag to check
  * @param Status: flag state to check
  * @param Timeout: Timeout duration
  * @param Tickstart: Tick start value
  * @retval HAL status
  */
HAL_StatusTypeDef LCD_SPI_WaitOnFlagUntilTimeout(SPI_HandleTypeDef *hspi, uint32_t Flag, FlagStatus Status,
                                                    uint32_t Tickstart, uint32_t Timeout)
{
   /* Wait until flag is set */
   while ((__HAL_SPI_GET_FLAG(hspi, Flag) ? SET : RESET) == Status)
   {
      /* Check for the Timeout */
      if ((((HAL_GetTick() - Tickstart) >=  Timeout) && (Timeout != HAL_MAX_DELAY)) || (Timeout == 0U))
      {
         return HAL_TIMEOUT;
      }
   }
   return HAL_OK;
}


/**
 * @brief  Close Transfer and clear flags.
 * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
 *               the configuration information for SPI module.
 * @retval HAL_ERROR: if any error detected
 *         HAL_OK: if nothing detected
 */
 void LCD_SPI_CloseTransfer(SPI_HandleTypeDef *hspi)
{
  uint32_t itflag = hspi->Instance->SR;

  __HAL_SPI_CLEAR_EOTFLAG(hspi);
  __HAL_SPI_CLEAR_TXTFFLAG(hspi);

  /* Disable SPI peripheral */
  __HAL_SPI_DISABLE(hspi);

  /* Disable ITs */
  __HAL_SPI_DISABLE_IT(hspi, (SPI_IT_EOT | SPI_IT_TXP | SPI_IT_RXP | SPI_IT_DXP | SPI_IT_UDR | SPI_IT_OVR | SPI_IT_FRE | SPI_IT_MODF));

  /* Disable Tx DMA Request */
  CLEAR_BIT(hspi->Instance->CFG1, SPI_CFG1_TXDMAEN | SPI_CFG1_RXDMAEN);

  /* Report UnderRun error for non RX Only communication */
  if (hspi->State != HAL_SPI_STATE_BUSY_RX)
  {
    if ((itflag & SPI_FLAG_UDR) != 0UL)
    {
      SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_UDR);
      __HAL_SPI_CLEAR_UDRFLAG(hspi);
    }
  }

  /* Report OverRun error for non TX Only communication */
  if (hspi->State != HAL_SPI_STATE_BUSY_TX)
  {
    if ((itflag & SPI_FLAG_OVR) != 0UL)
    {
      SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_OVR);
      __HAL_SPI_CLEAR_OVRFLAG(hspi);
    }
  }

  /* SPI Mode Fault error interrupt occurred -------------------------------*/
  if ((itflag & SPI_FLAG_MODF) != 0UL)
  {
    SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_MODF);
    __HAL_SPI_CLEAR_MODFFLAG(hspi);
  }

  /* SPI Frame error interrupt occurred ------------------------------------*/
  if ((itflag & SPI_FLAG_FRE) != 0UL)
  {
    SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_FRE);
    __HAL_SPI_CLEAR_FREFLAG(hspi);
  }

  hspi->TxXferCount = (uint16_t)0UL;
  hspi->RxXferCount = (uint16_t)0UL;
}


/**
  * @brief  רΪ��Ļ�������޸ģ�����Ҫ��������ɫ��������
  * @param  hspi   : spi�ľ��
  * @param  pData  : Ҫд�������
  * @param  Size   : ���ݴ�С
  * @retval HAL status
  */

HAL_StatusTypeDef LCD_SPI_Transmit(SPI_HandleTypeDef *hspi,uint16_t pData, uint32_t Size)
{
   uint16_t chunk = 0;
   uint16_t color_buf[64];
   uint16_t i = 0;

   if (Size == 0UL)
   {
      return HAL_OK;
   }

   for (i = 0; i < (uint16_t)(sizeof(color_buf) / sizeof(color_buf[0])); i++)
   {
      color_buf[i] = pData;
   }

   while (Size > 0UL)
   {
      chunk = (Size > (uint32_t)(sizeof(color_buf) / sizeof(color_buf[0])))
                  ? (uint16_t)(sizeof(color_buf) / sizeof(color_buf[0]))
                  : (uint16_t)Size;

      if (HAL_SPI_Transmit(hspi, (uint8_t *)color_buf, chunk, 1000) != HAL_OK)
      {
         return HAL_ERROR;
      }

      Size -= chunk;
   }

   return HAL_OK;
}

/**
  * @brief  רΪ����д�������޸ģ�ʹ֮���޳��ȵĴ�������
  * @param  hspi   : spi�ľ��
  * @param  pData  : Ҫд�������
  * @param  Size   : ���ݴ�С
  * @retval HAL status
  */
HAL_StatusTypeDef LCD_SPI_TransmitBuffer (SPI_HandleTypeDef *hspi, uint16_t *pData, uint32_t Size)
{
   uint16_t chunk = 0;

   if ((pData == NULL) || (Size == 0UL))
   {
      return HAL_OK;
   }

   while (Size > 0UL)
   {
      chunk = (Size > 1024UL) ? 1024U : (uint16_t)Size;
      if (HAL_SPI_Transmit(hspi, (uint8_t *)pData, chunk, 1000) != HAL_OK)
      {
         return HAL_ERROR;
      }
      pData += chunk;
      Size -= chunk;
   }

   return HAL_OK;
}

static HAL_StatusTypeDef LCD_SPI_TransmitBufferAsync(SPI_HandleTypeDef *hspi, const uint16_t *pData, uint32_t Size)
{
	uint32_t primask;

	if ((pData == NULL) || (Size == 0UL))
	{
		return HAL_OK;
	}

	if (s_lcd_spi_tx_busy != 0U)
	{
		return HAL_BUSY;
	}

	if (hspi != &LCD_SPI)
	{
		return HAL_ERROR;
	}

	primask = __get_PRIMASK();
	__disable_irq();
	s_lcd_spi_tx_source = pData;
	s_lcd_spi_tx_total = Size;
	s_lcd_spi_tx_source_pos = 0U;
	s_lcd_bdma_slot_state[0] = LCD_BDMA_SLOT_FREE;
	s_lcd_bdma_slot_state[1] = LCD_BDMA_SLOT_FREE;
	s_lcd_bdma_slot_pixels[0] = 0U;
	s_lcd_bdma_slot_pixels[1] = 0U;
	s_lcd_bdma_active_slot = LCD_BDMA_NO_ACTIVE_SLOT;
	s_lcd_bdma_hw_busy = 0U;
	s_lcd_spi_tx_busy = 1U;
	s_lcd_spi_tx_status = HAL_BUSY;
	s_lcd_spi_needs_finalize = 1U;
	if (primask == 0U)
	{
		__enable_irq();
	}

	LCD_TransferService();
	if (s_lcd_spi_tx_status == HAL_ERROR)
	{
		return HAL_ERROR;
	}

	return HAL_OK;
}

static HAL_StatusTypeDef LCD_SPI_StartReadyChunk(void)
{
	uint8_t slot;

	if (s_lcd_bdma_hw_busy != 0U)
	{
		return HAL_BUSY;
	}

	for (slot = 0U; slot < LCD_BDMA_STAGE_COUNT; ++slot)
	{
		if (s_lcd_bdma_slot_state[slot] == LCD_BDMA_SLOT_READY)
		{
			s_lcd_bdma_active_slot = slot;
			s_lcd_bdma_hw_busy = 1U;
			if (HAL_SPI_Transmit_DMA(
					&LCD_SPI,
					(const uint8_t *)s_lcd_bdma_stage[slot],
					s_lcd_bdma_slot_pixels[slot]
				) != HAL_OK)
			{
				s_lcd_bdma_active_slot = LCD_BDMA_NO_ACTIVE_SLOT;
				s_lcd_bdma_hw_busy = 0U;
				s_lcd_spi_tx_status = HAL_ERROR;
				s_lcd_spi_tx_busy = 0U;
				return HAL_ERROR;
			}

			return HAL_OK;
		}
	}

	return HAL_BUSY;
}

void LCD_TransferService(void)
{
	uint8_t slot;

	if (s_lcd_spi_tx_busy == 0U)
	{
		return;
	}

	for (slot = 0U; slot < LCD_BDMA_STAGE_COUNT; ++slot)
	{
		const uint16_t *source;
		uint32_t source_pos;
		uint32_t chunk_pixels;
		uint32_t primask;

		primask = __get_PRIMASK();
		__disable_irq();
		if ((s_lcd_bdma_slot_state[slot] != LCD_BDMA_SLOT_FREE) ||
			(slot == s_lcd_bdma_active_slot) ||
			(s_lcd_spi_tx_source_pos >= s_lcd_spi_tx_total))
		{
			if (primask == 0U)
			{
				__enable_irq();
			}
			continue;
		}

		s_lcd_bdma_slot_state[slot] = LCD_BDMA_SLOT_FILLING;
		source_pos = s_lcd_spi_tx_source_pos;
		chunk_pixels = s_lcd_spi_tx_total - source_pos;
		if (chunk_pixels > LCD_BDMA_STAGE_PIXELS)
		{
			chunk_pixels = LCD_BDMA_STAGE_PIXELS;
		}
		s_lcd_spi_tx_source_pos += chunk_pixels;
		source = &s_lcd_spi_tx_source[source_pos];
		if (primask == 0U)
		{
			__enable_irq();
		}

		memcpy(s_lcd_bdma_stage[slot], source, chunk_pixels * sizeof(uint16_t));
		LCD_SPI_CleanDCacheForBuffer(s_lcd_bdma_stage[slot], chunk_pixels * sizeof(uint16_t));
		__DMB();

		primask = __get_PRIMASK();
		__disable_irq();
		s_lcd_bdma_slot_pixels[slot] = (uint16_t)chunk_pixels;
		s_lcd_bdma_slot_state[slot] = LCD_BDMA_SLOT_READY;
		if (primask == 0U)
		{
			__enable_irq();
		}
	}

	{
		uint32_t primask = __get_PRIMASK();
		__disable_irq();
		(void)LCD_SPI_StartReadyChunk();
		if (primask == 0U)
		{
			__enable_irq();
		}
	}
}

static void LCD_SPI_FinalizeTransfer(void)
{
	if ((s_lcd_spi_tx_busy == 0U) && (s_lcd_spi_needs_finalize != 0U))
	{
		(void)LCD_SPI_SetDataSize(SPI_DATASIZE_8BIT);
		LCD_CS_HIGH;
		s_lcd_spi_needs_finalize = 0U;
	}
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi->Instance == LCD_SPI.Instance)
	{
		uint8_t slot = s_lcd_bdma_active_slot;
		uint8_t any_pending = 0U;
		uint8_t index;

		if (slot < LCD_BDMA_STAGE_COUNT)
		{
			s_lcd_bdma_slot_state[slot] = LCD_BDMA_SLOT_FREE;
			s_lcd_bdma_slot_pixels[slot] = 0U;
		}
		s_lcd_bdma_active_slot = LCD_BDMA_NO_ACTIVE_SLOT;
		s_lcd_bdma_hw_busy = 0U;

		{
			HAL_StatusTypeDef start_status = LCD_SPI_StartReadyChunk();

			if (start_status == HAL_OK)
			{
				return;
			}

			if (start_status == HAL_ERROR)
			{
				LCD_TransferCallback_t callback = s_lcd_spi_tx_callback;
				void *callback_context = s_lcd_spi_tx_callback_context;

				s_lcd_spi_tx_callback = NULL;
				s_lcd_spi_tx_callback_context = NULL;
				if (callback != NULL)
				{
					callback(HAL_ERROR, callback_context);
				}
				return;
			}
		}

		for (index = 0U; index < LCD_BDMA_STAGE_COUNT; ++index)
		{
			if (s_lcd_bdma_slot_state[index] != LCD_BDMA_SLOT_FREE)
			{
				any_pending = 1U;
				break;
			}
		}

		if ((s_lcd_spi_tx_source_pos >= s_lcd_spi_tx_total) && (any_pending == 0U))
		{
			LCD_TransferCallback_t callback = s_lcd_spi_tx_callback;
			void *callback_context = s_lcd_spi_tx_callback_context;

			s_lcd_spi_tx_callback = NULL;
			s_lcd_spi_tx_callback_context = NULL;
			s_lcd_spi_tx_status = HAL_OK;
			s_lcd_spi_tx_busy = 0U;
			if (callback != NULL)
			{
				callback(HAL_OK, callback_context);
			}
		}
	}
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi->Instance == LCD_SPI.Instance)
	{
		LCD_TransferCallback_t callback = s_lcd_spi_tx_callback;
		void *callback_context = s_lcd_spi_tx_callback_context;

		s_lcd_spi_tx_callback = NULL;
		s_lcd_spi_tx_callback_context = NULL;
		s_lcd_bdma_hw_busy = 0U;
		s_lcd_bdma_active_slot = LCD_BDMA_NO_ACTIVE_SLOT;
		s_lcd_spi_tx_status = HAL_ERROR;
		s_lcd_spi_tx_busy = 0U;
		if (callback != NULL)
		{
			callback(HAL_ERROR, callback_context);
		}
	}
}

/*****************************************************************************************************************************************************************************************************************************************************************************/
// ʵ��ƽ̨�� STM32H743���İ�
//

