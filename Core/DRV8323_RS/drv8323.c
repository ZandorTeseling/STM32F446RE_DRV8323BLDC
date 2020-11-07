/*
 * drv8323.c
 *
 *  Created on: 26 Oct 2020
 *      Author: zandor
 */


#include <../DRV8323_RS/drv8323.h>

volatile SPIStruct s_drv832xSPI;

void DRV832x_initSPIInterface(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, SPI_HandleTypeDef* pspi){
	s_drv832xSPI.PORT_GPIOx = GPIOx;
	s_drv832xSPI.CS_GPIO_Pin = GPIO_Pin;
	s_drv832xSPI.pSPI_Handle = pspi;
	s_drv832xSPI.SPI_TX_Flag = 0;
	s_drv832xSPI.SPI_RX_Flag = 0;
}

void DRV832x_trigger_spi_write(uint16_t val)
{
	s_drv832xSPI.SPI_TX_Data[0] = val;
	HAL_GPIO_WritePin(s_drv832xSPI.PORT_GPIOx, s_drv832xSPI.CS_GPIO_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive_IT(s_drv832xSPI.pSPI_Handle, (uint8_t*)s_drv832xSPI.SPI_TX_Data, (uint8_t*)s_drv832xSPI.SPI_RX_Data, 1);
}

void DRV832x_read_FSR1(void)
{
    uint16_t val = (1<<15) | FSR1;
    DRV832x_trigger_spi_write(val);
}

void DRV832x_read_FSR2(void)
{
    uint16_t val = (1<<15) | FSR2;
    DRV832x_trigger_spi_write(val);
}

void DRV832x_trigger_read_register(int reg)
{
    DRV832x_trigger_spi_write((1<<15) | (reg<<11));
}

void DRV832x_write_register(int reg, int val)
{
	DRV832x_trigger_spi_write((reg<<11) | val);
}
void DRV832x_write_DCR(int DIS_CPUV, int DIS_GDF, int OTW_REP, int PWM_MODE, int PWM_COM, int PWM_DIR, int COAST, int BRAKE, int CLR_FLT)
{
    uint16_t val = (DCR<<11) | (DIS_CPUV<<9) | (DIS_GDF<<8) | (OTW_REP<<7) | (PWM_MODE<<5) | (PWM_COM<<4) | (PWM_DIR<<3) | (COAST<<2) | (BRAKE<<1) | CLR_FLT;
    DRV832x_trigger_spi_write(val);
    }
void DRV832x_write_HSR(int LOCK, int IDRIVEP_HS, int IDRIVEN_HS)
{
    uint16_t val = (HSR<<11) | (LOCK<<8) | (IDRIVEP_HS<<4) | IDRIVEN_HS;
    DRV832x_trigger_spi_write(val);
    }
void DRV832x_write_LSR(int CBC, int TDRIVE, int IDRIVEP_LS, int IDRIVEN_LS)
{
    uint16_t val = (LSR<<11) | (CBC<<10) | (TDRIVE<<8) | (IDRIVEP_LS<<4) | IDRIVEN_LS;
    DRV832x_trigger_spi_write(val);
    }
void DRV832x_write_OCPCR(int TRETRY, int DEAD_TIME, int OCP_MODE, int OCP_DEG, int VDS_LVL)
{
    uint16_t val = (OCPCR<<11) | (TRETRY<<10) | (DEAD_TIME<<8) | (OCP_MODE<<6) | (OCP_DEG<<4) | VDS_LVL;
    DRV832x_trigger_spi_write(val);
    }
void DRV832x_write_CSACR(int CSA_FET, int VREF_DIV, int LS_REF, int CSA_GAIN, int DIS_SEN, int CSA_CAL_A, int CSA_CAL_B, int CSA_CAL_C, int SEN_LVL)
{
    uint16_t val = (CSACR<<11) | (CSA_FET<<10) | (VREF_DIV<<9) | (LS_REF<<8) | (CSA_GAIN<<6) | (DIS_SEN<<5) | (CSA_CAL_A<<4) | (CSA_CAL_B<<3) | (CSA_CAL_C<<2) | SEN_LVL;
    DRV832x_trigger_spi_write(val);
}

void DRV832x_print_faults(void)
{
    DRV832x_read_FSR1();
    wait_us(10);
    uint16_t val1 = s_drv832xSPI.SPI_RX_Data[0];
    DRV832x_read_FSR2();
    wait_us(10);
    uint16_t val2 = s_drv832xSPI.SPI_RX_Data[0];

    if(val1 & (1<<10)){printf("\n\rFAULT\n\r");}

    if(val1 & (1<<9)){printf("VDS_OCP\n\r");}
    if(val1 & (1<<8)){printf("GDF\n\r");}
    if(val1 & (1<<7)){printf("UVLO\n\r");}
    if(val1 & (1<<6)){printf("OTSD\n\r");}
    if(val1 & (1<<5)){printf("VDS_HA\n\r");}
    if(val1 & (1<<4)){printf("VDS_LA\n\r");}
    if(val1 & (1<<3)){printf("VDS_HB\n\r");}
    if(val1 & (1<<2)){printf("VDS_LB\n\r");}
    if(val1 & (1<<1)){printf("VDS_HC\n\r");}
    if(val1 & (1)){printf("VDS_LC\n\r");}

    if(val2 & (1<<10)){printf("SA_OC\n\r");}
    if(val2 & (1<<9)){printf("SB_OC\n\r");}
    if(val2 & (1<<8)){printf("SC_OC\n\r");}
    if(val2 & (1<<7)){printf("OTW\n\r");}
    if(val2 & (1<<6)){printf("CPUV\n\r");}
    if(val2 & (1<<5)){printf("VGS_HA\n\r");}
    if(val2 & (1<<4)){printf("VGS_LA\n\r");}
    if(val2 & (1<<3)){printf("VGS_HB\n\r");}
    if(val2 & (1<<2)){printf("VGS_LB\n\r");}
    if(val2 & (1<<1)){printf("VGS_HC\n\r");}
    if(val2 & (1)){printf("VGS_LC\n\r");}
    }

void DRV832x_enable_gd(void)
{
	DRV832x_trigger_read_register(DCR);

    uint16_t val = (s_drv832xSPI.SPI_RX_Data[0]) & (~(0x1<<2));
    DRV832x_write_register(DCR, val);
}

void DRV832x_disable_gd(void)
{
	DRV832x_trigger_read_register(DCR);
    uint16_t val = (s_drv832xSPI.SPI_RX_Data[0]) | (0x1<<2);
    DRV832x_write_register(DCR, val);
}

void DRV832x_calibrate(void)
{
    uint16_t val = (0x1<<4) + (0x1<<3) + (0x1<<2);
    DRV832x_write_register(CSACR, val);
}
