// SPDX-License-Identifier: GPL-2.0
// 
// Shakti SPI Controller Driver (Master mode only)
//
// Copyright (C) 2021  IIT Madras. All rights reserved.
// Author(s): Sambhav Jain <sambhav.jv@gmail.com> , Venkatakrishnan Sutharsan <venkatakrishnan.sutharsan@gmail.com>

#include <linux/debugfs.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>
#include <linux/spi/spi.h>
#include <linux/types.h>
#include <linux/delay.h>


#define SHAKTI_SPI_DRIVER_NAME "shakti_spi"

/* Comment the next line to disable debug printk statements in 
 * the Shakti SPI Driver code.
 */
// #define SHAKTI_DEBUG

#define SHAKTI_SPI_DEFAULT_DEPTH         		 16

//SHAKTI SPI
#define SHAKTI_SPI1_BASE_ADDRESS            0x20100 /*! Standard Serial Peripheral Interface 0 Base address*/
#define SHAKTI_SPI_MAX_COUNT 			          1 /*! Number of Standard SHAKTI_SPI used in the SOC */
#define CLOCK_FREQUENCY 		           50000000
#define SHAKTI_SPI_DRIVER 		     	          1

#define SHAKTI_SPI_COMM_CTRL_REG	           0x00 // Standard SPI Communication Control Register (32-bit Read and Write register)
#define SHAKTI_SPI_CLK_CTRL_REG		           0x04 // Standard SPI Clock Control Register (32-bit Read and Write register)
#define SHAKTI_SPI_TX_DATA			           0x08 // Standard SPI Transmitter Data Register (32-bit Write only register)
#define SHAKTI_SPI_RX_DATA		   	           0x0C // Standard SPI Receiver Data Register (32-bit Read only register)
#define SHAKTI_SPI_INTR_EN    		           0x10 // Standard SPI Interrupt Enable Register (16-bit Read and Write register)
#define SHAKTI_SPI_FIFO_STATUS                 0x14 // Standard SPI FIFO Status Register (8-bit Read only register)
#define SHAKTI_SPI_COMM_STATUS                 0x18 // Standard SPI Communication Register (8-bit Read only register)
#define SHAKTI_SPI_QUAD                        0x1C // Standard SPI Input Qualification Control Register (8-bit Read and Write register)

/*! SPIx Communication Control Register */
#define SHAKTI_SPI_MASTER                   (1 << 0)
#define SHAKTI_SPI_ENABLE_BIT               (1 << 1)
#define SHAKTI_SPI_LSB_FIRST                (1 << 2)
#define SHAKTI_SPI_COMM_MODE(x)             (x << 4)
#define SHAKTI_SPI_TOTAL_BITS_TX(x)         (x << 6)
#define SHAKTI_SPI_TOTAL_BITS_RX(x)        (x << 14)
#define SHAKTI_SPI_OUT_EN_SCLK             (1 << 22)
#define SHAKTI_SPI_OUT_EN_NCS              (1 << 23)
#define SHAKTI_SPI_OUT_EN_MISO             (1 << 24)
#define SHAKTI_SPI_OUT_EN_MOSI             (1 << 25)

/*! SPIx Clock Control Register */
#define SHAKTI_SPI_CLK_POLARITY             (1 << 0)
#define SHAKTI_SPI_CLK_PHASE                (1 << 1)
#define SHAKTI_SPI_PRESCALE(x)              (x << 2)
#define SHAKTI_SPI_SS2TX_DELAY(x)          (x << 10)
#define SHAKTI_SPI_TX2SS_DELAY(x)          (x << 18)

/*! SPIx Interrupt Enable Register */
#define SHAKTI_SPI_TX_EMPTY_INTR_EN         (1 << 0)
#define SHAKTI_SPI_TX_QUAD_INTR_EN          (1 << 1)
#define SHAKTI_SPI_TX_HALF_INTR_EN          (1 << 2)
#define SHAKTI_SPI_TX_FULL_INTR_EN          (1 << 3)
#define SHAKTI_SPI_RX_EMPTY_INTR_EN         (1 << 4)
#define SHAKTI_SPI_RX_QUAD_INTR_EN          (1 << 5)
#define SHAKTI_SPI_RX_HALF_INTR_EN          (1 << 6)
#define SHAKTI_SPI_RX_FULL_INTR_EN          (1 << 7)
#define SHAKTI_SPI_RX_OVERRUN_INTR_EN       (1 << 8)

/*! SPIx Communication Status Register */
#define SHAKTI_SPI_BUSY                     (1 << 0)
#define SHAKTI_SPI_TXE                      (1 << 1)
#define SHAKTI_SPI_RXNE                     (1 << 2)
#define SHAKTI_SPI_TX_FIFO                  (3 << 3)
#define SHAKTI_SPI_RX_FIFO                  (3 << 5)
#define SHAKTI_SPI_OVR                      (1 << 7)

/*! SPIx FIFO Status Register */
#define SHAKTI_SPI_TX_EMPTY                 (1 << 0)
#define SHAKTI_SPI_TX_QUAD                  (1 << 1)
#define SHAKTI_SPI_TX_HALF                  (1 << 2)
#define SHAKTI_SPI_TX_FULL                  (1 << 3)
#define SHAKTI_SPI_RX_EMPTY                 (1 << 4)
#define SHAKTI_SPI_RX_QUAD                  (1 << 5)
#define SHAKTI_SPI_RX_HALF                  (1 << 6)
#define SHAKTI_SPI_RX_FULL                  (1 << 7)


//SHAKTI MACROS
// #define SHAKTI_SPI_MASTER                         1
// #define SHAKTI_SPI_SLAVE                          0

#define SHAKTI_SPI_DISABLE                        0
#define SHAKTI_SPI_ENABLE                         1

// #define SHAKTI_SPI_LSB_FIRST                      1
// #define SHAKTI_SPI_MSB_FIRST                      0

#define SHAKTI_SPI_SIMPLEX_TX                     0
#define SHAKTI_SPI_SIMPLEX_RX                     1
#define SHAKTI_SPI_HALF_DUPLEX                    2
#define SHAKTI_SPI_FULL_DUPLEX                    3

#define SHAKTI_SPI_SUCCESS                        0
#define SHAKTI_SPI_FAILURE                       -1
#define SHAKTI_SPI_TIMEOUT                       -2

#define SHAKTI_SPI_MIN_BR                         49 // 50MHz/ 50  = 1MHz 
#define SHAKTI_SPI_MAX_BR                     	 165 // 50MHz/ 166 = 300 KHz

#define PINMUX_CONFIGURE_REG 				0x41510
#define SHAKTI_SPI_PINMUX_MASK				0xFFC03FFF  //Clearing the SPI concerned bits						
#define SHAKTI_SPI_PINMUX_SET				0x154000				

/**
 * shakti _spi_reg - shakti SPI register & bitfield desc
 * @reg:		register offset
 * @mask:		bitfield mask
 * @shift:		left shift
 */
struct shakti_spi_reg {
	int reg;
	int mask;
	int shift;
};

/**
 * shakti_spi_regspec - shakti registers definition, compatible dependent data
 * en: enable register and SPI enable bit
 * cpol: clock polarity register and polarity bit
 * cpha: clock phase register and phase bit
 * lsb_first: LSB transmitted first register and bit
 * br: baud rate register and bitfields
 * rx: SPI RX data register
 * tx: SPI TX data register
 */
struct shakti_spi_regspec {
	const struct shakti_spi_reg en;
	const struct shakti_spi_reg master;
	// const struct shakti_spi_reg dma_rx_en;
	// const struct shakti_spi_reg dma_tx_en;
	const struct shakti_spi_reg cpol;
	const struct shakti_spi_reg cpha;
	const struct shakti_spi_reg lsb_first;
	const struct shakti_spi_reg br;
	const struct shakti_spi_reg rx;
	const struct shakti_spi_reg tx;
};


/**
 * shakti_spi_cfg - shakti compatible configuration data
 * @regs: registers descriptions
 * @get_fifo_size: routine to get fifo size
 * @get_bpw_mask: routine to get bits per word mask
 * @disable: routine to disable controller
 * @config: routine to configure controller as SPI Master
 * @set_bpw: routine to configure registers to for bits per word
 * @set_mode: routine to configure registers to desired mode
 * @set_data_idleness: optional routine to configure registers to desired idle
 * time between frames (if driver has this functionality)
 * set_number_of_data: optional routine to configure registers to desired
 * number of data (if driver has this functionality)
 * @can_dma: routine to determine if the transfer is eligible for DMA use
 * @transfer_one_dma_start: routine to start transfer a single spi_transfer
 * using DMA
 * @dma_rx cb: routine to call after DMA RX channel operation is complete
 * @dma_tx cb: routine to call after DMA TX channel operation is complete
 * @transfer_one_irq: routine to configure interrupts for driver
 * @irq_handler_event: Interrupt handler for SPI controller events
 * @irq_handler_thread: thread of interrupt handler for SPI controller
 * @baud_rate_div_min: minimum baud rate divisor
 * @baud_rate_div_max: maximum baud rate divisor
 * @has_fifo: boolean to know if fifo is used for driver
 * @has_startbit: boolean to know if start bit is used to start transfer
 */
struct shakti_spi_cfg {
	const struct shakti_spi_regspec *regs;
	int (*get_fifo_size)(struct shakti_spi *spi);
	int (*get_bpw_mask)(struct shakti_spi *spi);
	void (*disable)(struct shakti_spi *spi);
	int (*config)(struct shakti_spi *spi);
	void (*set_bpw)(struct shakti_spi *spi);
	int (*set_mode)(struct shakti_spi *spi, unsigned int comm_type);
	void (*set_data_idleness)(struct shakti_spi *spi, u32 length);
	int (*set_number_of_data)(struct shakti_spi *spi, u32 length);
	void (*transfer_one_dma_start)(struct shakti_spi *spi);
	// void (*dma_rx_cb)(void *data);
	// void (*dma_tx_cb)(void *data);
	// int (*transfer_one_irq)(struct shakti_spi *spi);
	// irqreturn_t (*irq_handler_event)(int irq, void *dev_id);
	// irqreturn_t (*irq_handler_thread)(int irq, void *dev_id);
	unsigned int baud_rate_div_min;
	unsigned int baud_rate_div_max;
	bool has_fifo;
};

struct shakti_spi {
	struct device *dev;
	struct spi_master *master;
	const struct shakti_spi_cfg *cfg;
	void __iomem *base;  //Base address for SPI 
	struct clk *clk;
	u32 clk_rate;
	struct reset_control *rst;
	spinlock_t lock; /* prevent I/O concurrent access */
	int irq;
	unsigned int fifo_size;
	int mode_configuration; // To set master or slave...
	int comm_mode;  // Communication mode 0-3 to be given in dts file...
	int lsb_first; // lsb_first to be given in dts file...
	u32 spi_rate; //SPI Clock rate used to calculcate the pre-scalar..

	unsigned int cur_midi;
	unsigned int cur_speed;
	unsigned int cur_bpw;
	unsigned int cur_fthlv; 
	unsigned int cur_comm;
	unsigned int cur_xferlen;
	bool cur_usedma;

	const void *tx_buf;
	void *rx_buf;
	int tx_len;
	int rx_len;
	// struct dma_chan *dma_tx;
	// struct dma_chan *dma_rx;
	// dma_addr_t phys_addr;  //DMA's Physical addresss
};

static const struct shakti_spi_regspec shakti_spi1_regspec = {
	/* SPI data transfer is enabled but spi_ker_ck is idle.
	 * CFG1 and CFG2 registers are write protected when SPE is enabled.
	 */
	.en = { SHAKTI_SPI_COMM_CTRL_REG, SHAKTI_SPI_ENABLE_BIT },

	// .dma_rx_en = { },
	// .dma_tx_en = { },

	.cpol = { SHAKTI_SPI_CLK_CTRL_REG, SHAKTI_SPI_CLK_POLARITY },
	.cpha = { SHAKTI_SPI_CLK_CTRL_REG, SHAKTI_SPI_CLK_PHASE },
	.lsb_first = { SHAKTI_SPI_COMM_CTRL_REG, SHAKTI_SPI_LSB_FIRST },
	// .br = { SHAKTI_SPI_CFG1, SHAKTI_SPI_CFG1_MBR,
	// SHAKTI_SPI_CFG1_MBR_SHIFT },

	.rx = { SHAKTI_SPI_RX_DATA },
	.tx = { SHAKTI_SPI_TX_DATA },
};

u8 shakti_spi_read8(struct shakti_spi *spi, u32 offset)
{
	return readb_relaxed(spi->base + offset);
}

u16 shakti_spi_read16(struct shakti_spi *spi, u32 offset)
{
	return readw_relaxed(spi->base + offset);
}

u32 shakti_spi_read32(struct shakti_spi *spi, u32 offset)
{
	return readl_relaxed(spi->base + offset);
}

/* Setting Bits for Shakti Registers */
static inline void shakti_spi_set8(struct shakti_spi *spi,
				      u32 offset, u8 bits)
{
	writeb_relaxed(readb_relaxed(spi->base + offset) | bits,
		       spi->base + offset);
}

static inline void shakti_spi_set16(struct shakti_spi *spi,
				      u32 offset, u16 bits)
{
	writew_relaxed(readw_relaxed(spi->base + offset) | bits,
		       spi->base + offset);
}

static inline void shakti_spi_set32(struct shakti_spi *spi,
				      u32 offset, u32 bits)
{
	writel_relaxed(readl_relaxed(spi->base + offset) | bits,
		       spi->base + offset);
}

/* Clearing Bits for Shakti Registers */
static inline void shakti_spi_clr8(struct shakti_spi *spi,
				      u32 offset, u8 bits)
{
	writeb_relaxed(readb_relaxed(spi->base + offset) & ~bits,
		       spi->base + offset);
}

static inline void shakti_spi_clr16(struct shakti_spi *spi,
				      u32 offset, u16 bits)
{
	writew_relaxed(readw_relaxed(spi->base + offset) & ~bits,
		       spi->base + offset);
}

static inline void shakti_spi_clr32(struct shakti_spi *spi,
				      u32 offset, u32 bits)
{
	writel_relaxed(readl_relaxed(spi->base + offset) & ~bits,
		       spi->base + offset);
}



static void shakti_spi_tx(struct shakti_spi *spi,
				      u8 *tx_ptr)
{
	#ifdef SHAKTI_DEBUG
	printk(KERN_ALERT "[SHAKTI-INFO] The Current data sent to buffer is : %x\n",*tx_ptr);
	#endif
	writeb_relaxed(*tx_ptr, spi->base + SHAKTI_SPI_TX_DATA);	//Writing in bytes
}

static void shakti_spi_rx(struct shakti_spi *spi,
				      u8 *rx_ptr)
{
	#ifdef SHAKTI_DEBUG
	printk(KERN_ALERT "[SHAKTI-INFO] The data received from buffer is : %x\n",*rx_ptr);
	#endif
	*rx_ptr = readb_relaxed(spi->base + SHAKTI_SPI_RX_DATA);	//Reading as bytes
}


/*
 * shakti_configure_master - to configure the controller as master
 */
int shakti_spi_configure(struct shakti_spi *spi){
	unsigned long flags;
	spin_lock_irqsave(&spi->lock, flags);

	shakti_spi_set32(spi, SHAKTI_SPI_COMM_CTRL_REG, SHAKTI_SPI_MASTER);
	#ifdef SHAKTI_DEBUG
	printk(KERN_ALERT "The comm register is %x(Hex)\n", readl_relaxed(spi->base + SHAKTI_SPI_COMM_CTRL_REG));
	#endif

	spin_unlock_irqrestore(&spi->lock, flags);
	return 0;
}

/*
 * shakti_configure_slave - to configure the controller as slave
 */
void shakti_spi_configure_slave(struct shakti_spi *spi){
	shakti_spi_clr32(spi, SHAKTI_SPI_COMM_CTRL_REG, SHAKTI_SPI_MASTER);
}

/*
 * shakti_spi_configure_pins - Function to configure the MISO, MOSI, CS_PIN and CLK_PIN in the Hardware...
 */
void shakti_spi_configure_pins(struct shakti_spi *spi, uint8_t  mosi_pin_cntrl, uint8_t miso_pin_cntrl, uint8_t cs_pin_cntrl, uint8_t clk_pin_cntrl)
{
    uint32_t temp = 0;
    
    if(mosi_pin_cntrl)
        temp |= SHAKTI_SPI_OUT_EN_MOSI;  
    else
        temp &= ~SHAKTI_SPI_OUT_EN_MOSI;  

    if(miso_pin_cntrl)
        temp |= SHAKTI_SPI_OUT_EN_MISO;  
    else
        temp &= ~SHAKTI_SPI_OUT_EN_MISO;  

    if(cs_pin_cntrl)
        temp |= SHAKTI_SPI_OUT_EN_NCS;  
    else
        temp &= ~SHAKTI_SPI_OUT_EN_NCS;  
		
    if(clk_pin_cntrl)
        temp |= SHAKTI_SPI_OUT_EN_SCLK;  
    else
        temp &= ~SHAKTI_SPI_OUT_EN_SCLK;

	// Write to comm ctrl register
	shakti_spi_set32(spi, SHAKTI_SPI_COMM_CTRL_REG, temp);
}

/*
 * shakti_msb_first - Function to set the MSB as first bit to be transmitted.
 */
void shakti_lsb_first(struct shakti_spi *spi){
	#ifdef SHAKTI_DEBUG
	printk(KERN_ALERT "[SHAKTI-INFO] Configuring as LSB First\n");
	#endif
	if(spi->lsb_first)
		shakti_spi_set32(spi, SHAKTI_SPI_COMM_CTRL_REG, SHAKTI_SPI_LSB_FIRST);
	else
		shakti_spi_clr32(spi, SHAKTI_SPI_COMM_CTRL_REG, SHAKTI_SPI_LSB_FIRST);

}

/*
 * shakti_configure_comm_mode - Function to configure the communication mode to the Hardware
 */
void shakti_configure_comm_mode(struct shakti_spi *spi, uint8_t comm_mode)
{
	#ifdef SHAKTI_DEBUG
	printk(KERN_ALERT "[SHAKTI-INFO] Communication Mode is %x(Hex)\n",comm_mode);
	#endif
	u8 shakti_mode =0; 
	if(comm_mode & (SPI_CONTROLLER_MUST_RX | SPI_CONTROLLER_MUST_TX) )
	{
		//full duplex
		shakti_mode = 0x3;
	}
	else if((comm_mode & SPI_CONTROLLER_NO_TX) | (comm_mode & SPI_CONTROLLER_MUST_RX))
	{
		//Simplex RX 
		shakti_mode = 0x1;
	}
	else if((comm_mode & SPI_CONTROLLER_NO_RX) | (comm_mode & SPI_CONTROLLER_MUST_TX))
	{
		//Simplex TX
		shakti_mode = 0x0;
	}
	else
	{
		//Half duplex
		shakti_mode = 0x2;
	}

	uint32_t temp = readl_relaxed(spi->base + SHAKTI_SPI_COMM_CTRL_REG) & ~(SHAKTI_SPI_COMM_MODE(3));
    writel_relaxed( (temp | (SHAKTI_SPI_COMM_MODE(shakti_mode))), spi->base + SHAKTI_SPI_COMM_CTRL_REG);
}

void shakti_configure_clock_mode_br(struct shakti_spi *spi, uint8_t clock_mode, uint32_t clock_freq)
{
	u8 shakti_mode = 0;
	if(clock_mode & SPI_MODE_1 )
	{
		shakti_mode = SHAKTI_SPI_CLK_PHASE;
	}
	else if(clock_mode & SPI_MODE_2)
	{
		shakti_mode = SHAKTI_SPI_CLK_POLARITY;
	}
	else if(clock_mode & SPI_MODE_3)
	{
		shakti_mode = SHAKTI_SPI_CLK_POLARITY | SHAKTI_SPI_CLK_PHASE;
	}
	else if(clock_mode & SPI_MODE_0)
	{
		shakti_mode = 0x0;
	}
	
	uint32_t clock_prescaler = ((clk_get_rate(spi->clk) / clock_freq) - 1);
	#ifdef SHAKTI_DEBUG
	printk(KERN_ALERT "[SHAKTI-INFO] The value of clock prescalar is %x (Hex) and mode is %x (Hex)\n",SHAKTI_SPI_PRESCALE(clock_prescaler), shakti_mode);
	#endif
	uint32_t temp = readl_relaxed(spi->base + SHAKTI_SPI_CLK_CTRL_REG) & ~(SHAKTI_SPI_CLK_POLARITY | SHAKTI_SPI_CLK_PHASE);
    writel_relaxed( (temp | shakti_mode | SHAKTI_SPI_PRESCALE(clock_prescaler)), spi->base + SHAKTI_SPI_CLK_CTRL_REG);
}

/*
 * shakti_spi_setup - Initialize SPI Master device 
 */
static int shakti_spi_setup(struct spi_device *spi_dev)
{
	#ifdef SHAKTI_DEBUG
	printk(KERN_ALERT "[SHAKTI-INFO] Inside shakti_spi_setup function...\n");
	#endif
	int ret = 0;
	struct shakti_spi *spi = spi_master_get_devdata(spi_dev->master);

	if(spi->mode_configuration)
	{
		#ifdef SHAKTI_DEBUG
		printk(KERN_ALERT "[SHAKTI-INFO] Setting up SPI as MASTER...\n");
		#endif
		// Configuring SPI Controller as Master...
		shakti_spi_configure(spi);

		// Configuring pins for transfer
		shakti_spi_configure_pins(spi, SHAKTI_SPI_ENABLE,SHAKTI_SPI_DISABLE,SHAKTI_SPI_ENABLE,SHAKTI_SPI_ENABLE);
	}
	else
	{
		#ifdef SHAKTI_DEBUG
		printk(KERN_ALERT "[SHAKTI-INFO] Setting up SPI as SLAVE...\n");
		#endif

		// Configuring SPI Controller as Slave...
		shakti_spi_configure_slave(spi);

		// Configuring pins for transfer
		shakti_spi_configure_pins(spi, SHAKTI_SPI_DISABLE,SHAKTI_SPI_ENABLE,SHAKTI_SPI_DISABLE,SHAKTI_SPI_DISABLE);
	}

	if (spi_dev->master->mode_bits & SPI_LSB_FIRST)
		shakti_lsb_first(spi);

	// if (spi_dev->master->mode_bits & SPI_MODE_3)
	// 	spi_dev->mode = SPI_MODE_3;
	// else if (spi_dev->master->mode_bits & SPI_MODE_2)
	// 	spi_dev->mode = SPI_MODE_2;
	// else if (spi_dev->master->mode_bits & SPI_MODE_1)
	// 	spi_dev->mode = SPI_MODE_1;
	// else if (spi_dev->master->mode_bits & SPI_MODE_0)
	// 	spi_dev->mode = SPI_MODE_0;

	//Communication mode - simplex, duplex; 
	shakti_configure_comm_mode(spi, spi_dev->master->flags);

	// Clock CPOL, CPHA and prescalar setting.
	shakti_configure_clock_mode_br(spi,spi_dev->mode,spi->spi_rate);
	// #ifdef SHAKTI_DEBUG
	// printk(KERN_ALERT "The comm reg value is %x\n", shakti_spi_read32(spi, SHAKTI_SPI_COMM_CTRL_REG));
	// #endif

	#ifdef SHAKTI_DEBUG
	printk(KERN_ALERT "[SHAKTI-INFO] Exiting shakti_spi_setup function...\n");
	#endif
	return ret;
}

/**
 * shakti_show_registers - Show the main register values at the calling point...
 */
static void shakti_show_registers(struct shakti_spi *spi)
{
	#ifdef SHAKTI_DEBUG
	printk(KERN_ALERT "[SHAKTI-INFO] The Communication Control Register has the following value : %x\n",shakti_spi_read32(spi, SHAKTI_SPI_COMM_CTRL_REG));
	printk(KERN_ALERT "[SHAKTI-INFO] The Clock Control Register has the following value : %x\n",shakti_spi_read32(spi, SHAKTI_SPI_CLK_CTRL_REG));
	printk(KERN_ALERT "[SHAKTI-INFO] The FIFO Status Register has the following value : %x\n",shakti_spi_read8(spi, SHAKTI_SPI_FIFO_STATUS));
	printk(KERN_ALERT "[SHAKTI-INFO] The Communication Status Register has the following value : %x\n",shakti_spi_read8(spi, SHAKTI_SPI_COMM_STATUS));
	#endif
}

/**
 * shakti_spi_prepare_msg - set up the controller to transfer a single message
 */
static int shakti_spi_prepare_msg(struct spi_master *master, struct spi_message *msg)
{
	#ifdef SHAKTI_DEBUG
	printk(KERN_ALERT "[SHAKTI-INFO] Entering Shakti SPI Prepare Message Function...\n");
	#endif
	struct spi_device *spi_dev = msg->spi;
	struct shakti_spi *spi = spi_master_get_devdata(master);

	shakti_configure_clock_mode_br(spi, spi_dev->mode, spi->spi_rate);  // CPOL, CPHA and PRESCALAR are set here...
	
	if (master->mode_bits & SPI_LSB_FIRST)
		shakti_lsb_first(spi);
	// TODO
	// shakti_spi_enable_interrupts(spi);

	#ifdef SHAKTI_DEBUG
	printk(KERN_ALERT "[SHAKTI-INFO] Exiting Shakti SPI Prepare Message Function...\n");
	#endif
	return 0;
}

/**
 * shakti_spi_transfer_one - transfer a single spi_transfer
 *
 * It must return 0 if the transfer is finished or 1 if the transfer is still
 * in progress.
 */
static int shakti_spi_transfer_one(struct spi_master *master, struct spi_device *spi_dev, struct spi_transfer *transfer)
{
	#ifdef SHAKTI_DEBUG
	printk(KERN_ALERT "[SHAKTI-INFO] Entering Shakti Transfer One Function...\n");
	#endif
	int ret = 0;
	struct shakti_spi *spi = spi_master_get_devdata(master);

	shakti_show_registers(spi);
	// //TO Check 
	spi->tx_buf = transfer->tx_buf;
	spi->rx_buf = transfer->rx_buf;
	spi->tx_len = spi->tx_buf ? transfer->len : 0;
	spi->rx_len = spi->rx_buf ? transfer->len : 0;
	const u8 *tx_ptr = transfer->tx_buf;
	u8 *rx_ptr = transfer->rx_buf;
	u8 rx_dummy[32];
	u8 *rx_ptr_dummy = rx_dummy;
	unsigned int r_words = transfer->len;
	// Clearing and Setting the total bits to be transferred...

	#ifdef SHAKTI_DEBUG
	printk(KERN_ALERT "[SHAKTI-INFO] Total bits to be transmitted is %d (int/bytes)\n", spi->tx_len);
	printk(KERN_ALERT "[SHAKTI-INFO] Total bits to be received is    %d (int/bytes)\n", spi->rx_len);
	#endif
	//Todo Enable before writing to fifo or after ? 
	//Setting total bits RX and TX here. 
	//TODO: To be shifted to prepare messages and supports only FD,Simplex TX and RX
	// if(rx_ptr & tx_ptr )
	// {
	// 	shakti_spi_set32(spi,SHAKTI_SPI_COMM_CTRL_REG,)
	// }
	#ifdef SHAKTI_DEBUG
	printk(KERN_ALERT "[SHAKTI-INFO] The spi_master flag has the value %x (Hex)\n",master->flags);
	printk(KERN_ALERT "[SHAKTI-INFO] The value of comm ctrl register before enable %x (Hex)\n",shakti_spi_read8(spi,SHAKTI_SPI_COMM_STATUS));
	#endif
	uint8_t number = 0;
	//Enabling the SPI 
	// shakti_spi_set32(spi, SHAKTI_SPI_COMM_CTRL_REG, SHAKTI_SPI_ENABLE_BIT);
	// shakti_show_registers(spi);
	// while(shakti_spi_read8(spi,SHAKTI_SPI_COMM_STATUS) & SHAKTI_SPI_BUSY);
	#ifdef SHAKTI_DEBUG
	printk(KERN_ALERT "[SHAKTI-INFO] Busy bit %d\n",(shakti_spi_read8(spi,SHAKTI_SPI_COMM_STATUS) & SHAKTI_SPI_BUSY));
	#endif
	
	// Dummy read to empty yhe RX FIFO in the SPI Controller
	// while(shakti_spi_read8(spi,SHAKTI_SPI_COMM_STATUS) & SHAKTI_SPI_RX_FIFO)
	// {
	// 	shakti_spi_rx(spi, rx_ptr_dummy++);
	// 	#ifdef SHAKTI_DEBUG
	// 	printk(KERN_ALERT "[SHAKTI-INFO] Shakti transfer_one dummy read %d\n", number);
	// 	#endif
	// 	number++;
	// 	// comm_status = shakti_spi_read8(spi,SHAKTI_SPI_COMM_STATUS);
	// }
	// // volatile uint8_t comm_status = shakti_spi_read8(spi,SHAKTI_SPI_COMM_STATUS);
	// printk(KERN_ALERT "after dummy read\n");
	shakti_show_registers(spi);
	while(r_words)
	{
		unsigned int n_words = min(r_words, spi->fifo_size);
		// printk("[TO]r_words %d n_words %d",r_words, n_words);
		shakti_spi_clr32(spi, SHAKTI_SPI_COMM_CTRL_REG, SHAKTI_SPI_TOTAL_BITS_RX(255));
		shakti_spi_clr32(spi, SHAKTI_SPI_COMM_CTRL_REG, SHAKTI_SPI_TOTAL_BITS_TX(255));
		//Assuming we have no Simplex Commnuication (tx or rx also )
		shakti_spi_set32(spi, SHAKTI_SPI_COMM_CTRL_REG, SHAKTI_SPI_TOTAL_BITS_RX((n_words) * 8));
		shakti_spi_set32(spi, SHAKTI_SPI_COMM_CTRL_REG, SHAKTI_SPI_TOTAL_BITS_TX((n_words) * 8));

		//32 and the rest r_words = r_words - n_words 
		unsigned int i;
		for(i=0; i< n_words;i++)
		{
			#ifdef SHAKTI_DEBUG
			printk(KERN_ALERT "[SHAKTI-INFO] Shakti TX condition inside...\n");
			#endif
			//Adding Tx data to the fifo 
			shakti_spi_tx(spi, tx_ptr++);
		}
		// udelay(5);
		shakti_spi_set32(spi, SHAKTI_SPI_COMM_CTRL_REG, SHAKTI_SPI_ENABLE_BIT);
		// udelay(1);
		//Wait for Tx to complete
		// shakti_show_registers(spi);
		while(!shakti_spi_read8(spi, SHAKTI_SPI_FIFO_STATUS) & SHAKTI_SPI_TX_EMPTY);
		while(shakti_spi_read8(spi,SHAKTI_SPI_COMM_STATUS) & SHAKTI_SPI_BUSY );
		u8 rx_counter = n_words;
		if(rx_ptr)
		{
			#ifdef SHAKTI_DEBUG
			printk(KERN_ALERT "[SHAKTI-INFO] Shakti RX condition inside...\n");
			#endif
			//Read till the RXE is set and store in the rx_ptrs 
			//Check if the len of r_words is not greater than num of
			// recieved words 
			while( (shakti_spi_read8(spi,SHAKTI_SPI_FIFO_STATUS) & (0x10)) == 0 && (rx_counter--))
			{
				#ifdef SHAKTI_DEBUG
				printk(KERN_ALERT "[SHAKTI-INFO] Shakti RX loop inside...%d\n",rx_counter);
				#endif
				shakti_spi_rx(spi, rx_ptr++);
			}
		}
		r_words -= n_words;
		#ifdef SHAKTI_DEBUG
		printk(KERN_ALERT "[SHAKTI-INFO] While loop r_word = %x (Hex), n_word = %x (Hex)\n",r_words,n_words);
		#endif
	}
	// shakti_spi_clr32(spi, SHAKTI_SPI_COMM_CTRL_REG, SHAKTI_SPI_ENABLE_BIT)
	#ifdef SHAKTI_DEBUG
	printk(KERN_ALERT "[SHAKTI-INFO] Exit Shakti Transfer One Function\n");
	#endif
	spi_finalize_current_transfer(master);
	return ret;

}

/**
 * shakti_spi_unprepare_msg - relax the hardware
 */
static int shakti_spi_unprepare_msg(struct spi_master *master, struct spi_message *msg)
{
	#ifdef SHAKTI_DEBUG
	printk(KERN_ALERT "[SHAKTI-INFO] Entering Shakti SPI Unprepare Message Function...\n");
	#endif
	struct shakti_spi *spi = spi_master_get_devdata(master);
	// TODO
	// shakti_spi_disable_interrupts(spi);
	#ifdef SHAKTI_DEBUG
	printk(KERN_ALERT "[SHAKTI-INFO] Exiting Shakti SPI Unprepare Message Function...\n");
	#endif
	return 0;
}

static const struct shakti_spi_cfg shakti_spi1_cfg = {
	.regs = &shakti_spi1_regspec,
	// .get_fifo_size = shakti_spi_get_fifo_size,
	// .get_bpw_mask = shakti_spi_get_bpw_mask,
	// .disable = shakti_spi_disable,
	// .config = shakti_spi_configure,
	// .set_bpw = shakti_spi_set_bpw,
	// .set_mode = shakti_spi_set_mode,
	// .set_data_idleness = shakti_spi_data_idleness,
	// .set_number_of_data = shakti_spi_number_of_data,
	// .transfer_one_dma_start = shakti_spi_transfer_one_dma_start,
	// .dma_rx_cb = shakti_spi_dma_cb,
	// .dma_tx_cb = shakti_spi_dma_cb,
	// .transfer_one_irq = shakti_spi_transfer_one_irq,
	// .irq_handler_thread = shakti_spi_irq_thread,
	.baud_rate_div_min = SHAKTI_SPI_MIN_BR,
	.baud_rate_div_max = SHAKTI_SPI_MAX_BR,
	.has_fifo = true,
};


static int shakti_spi_probe(struct platform_device *pdev)
{
	struct spi_master *master;
	struct shakti_spi *spi;
	struct resource *res;
	u32 cs_bits, max_bits_per_word, master_mode_bits = 0, master_flags = 0;
	int i, ret, temp = 0;

	#ifdef SHAKTI_DEBUG
	printk(KERN_ALERT "[SHAKTI-INFO] Inside shakti_spi_probe\n");
	#endif

	// Creating a virtual address for pinmux address...
	void __iomem *pinmux_register = ioremap(PINMUX_CONFIGURE_REG, 4);

	#ifdef SHAKTI_DEBUG
	printk(KERN_ALERT "[SHAKTI-INFO] The value of pinmux_register before setting the pinmux value : %x (Hex)", readl_relaxed(pinmux_register));
	#endif

	// Writing the pinmux register value for SPI...
	u32 pinmux_reg_read = ( readl_relaxed(pinmux_register) | SHAKTI_SPI_PINMUX_MASK );
	writel_relaxed((SHAKTI_SPI_PINMUX_SET | pinmux_reg_read) ,pinmux_register);
	
	#ifdef SHAKTI_DEBUG
	printk(KERN_ALERT "[SHAKTI-INFO] The value of pinmux_register after  setting the pinmux value : %x (Hex)", readl_relaxed(pinmux_register));
	#endif

	master = spi_alloc_master(&pdev->dev, sizeof(struct shakti_spi));
	if (!master) {
		dev_err(&pdev->dev, "spi master allocation failed\n");
		return -ENOMEM;
	}
	platform_set_drvdata(pdev, master);

	spi = spi_master_get_devdata(master);
	// init_completion(&spi->done);
	spi->dev = &pdev->dev;
	spi->master = master;
	spin_lock_init(&spi->lock);

	spi->cfg = (const struct shakti_spi_cfg *)
		of_match_device(pdev->dev.driver->of_match_table,
				&pdev->dev)->data;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	spi->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(spi->base)) {
		ret = PTR_ERR(spi->base);
		goto put_master;
		return -1;
	}
	#ifdef SHAKTI_DEBUG
	printk(KERN_ALERT "[SHAKTI-INFO] The basic resource allocated successfully...\n");
	#endif

	ret =
	  of_property_read_u32(pdev->dev.of_node, "shakti,controller",
			       &spi->mode_configuration);
	
	if(ret){
		dev_err(&pdev->dev, "Master/Slave not set in dts file\n");
		#ifdef SHAKTI_DEBUG
		printk(KERN_ALERT "Master/Slave not set in dts file\n");
		#endif
		ret = -EINVAL;
		goto put_master;
	}

	// /* Optional parameters */
	if (spi->cfg->has_fifo){
		ret =
		of_property_read_u32(pdev->dev.of_node, "shakti,fifo-depth",
					&spi->fifo_size);

		if (ret < 0){
			spi->fifo_size = SHAKTI_SPI_DEFAULT_DEPTH;
		}
		//SPI max size is 255 bits, hence max size of fifo 31 bytes
		if(spi->fifo_size > 31)
		{
			dev_err(&pdev->dev, "SPI FIFO size must be below 32 bytes\n");
			goto put_master;
		}
	}

	ret =
	  of_property_read_u32(pdev->dev.of_node, "shakti,spi-frequency",
			       &spi->spi_rate);
	
	if(ret){
		dev_err(&pdev->dev, "SPI Frequency not defined\n");
		#ifdef SHAKTI_DEBUG
		printk(KERN_ALERT "SPI Frequency not defined...\n");
		#endif
		ret = -EINVAL;
		goto put_master;
	}

	ret =
	  of_property_read_u32(pdev->dev.of_node, "shakti,lsb-first",
			       &spi->lsb_first);
	
	if(ret){
		dev_err(&pdev->dev, "LSB First/MSB First not defined\n");
		#ifdef SHAKTI_DEBUG
		printk(KERN_ALERT "LSB First/MSB First not defined...\n");
		#endif
		ret = -EINVAL;
		goto put_master;
	}
	if (spi->lsb_first){
		master_mode_bits |= SPI_LSB_FIRST;
	}

	ret =
	  of_property_read_u32(pdev->dev.of_node, "shakti,cpha",
			       &temp);
	
	if(ret){
		dev_err(&pdev->dev, "CPHA not defined\n");
		#ifdef SHAKTI_DEBUG
		printk(KERN_ALERT "CPHA not defined...\n");
		#endif
		ret = -EINVAL;
		goto put_master;
	}
	if (temp){
		master_mode_bits |= SPI_CPHA;
	}

	ret =
	  of_property_read_u32(pdev->dev.of_node, "shakti,cpol",
			       &temp);
	
	if(ret){
		dev_err(&pdev->dev, "CPOL not defined\n");
		#ifdef SHAKTI_DEBUG
		printk(KERN_ALERT "CPOL not defined...\n");
		#endif
		ret = -EINVAL;
		goto put_master;
	}
	if (temp){
		master_mode_bits |= SPI_CPOL;
	}

	ret =
	  of_property_read_u32(pdev->dev.of_node, "shakti,cs-high",
			       &temp);
	
	if(ret){
		dev_err(&pdev->dev, "CS High not defined\n");
		#ifdef DEBUG
		printk(KERN_ALERT "CS High not defined...\n");
		#endif
		ret = -EINVAL;
		goto put_master;
	}
	if (temp){
		master_mode_bits |= SPI_CS_HIGH;
	}

	ret =
	  of_property_read_u32(pdev->dev.of_node, "shakti,communication-mode",
			       &spi->comm_mode);
	
	if(ret | (spi->comm_mode < 0 && spi->comm_mode > 4)){
		dev_err(&pdev->dev, "Communication Mode not defined or not properly defined\n");
		#ifdef SHAKTI_DEBUG
		printk(KERN_ALERT "Communication Mode not defined or not properly defined...\n");
		#endif
		ret = -EINVAL;
		goto put_master;
	}

	if (spi->comm_mode == 0)
		master_flags = SPI_CONTROLLER_MUST_TX | SPI_CONTROLLER_NO_RX;
	else if (spi->comm_mode == 1)
		master_flags = SPI_CONTROLLER_MUST_RX | SPI_CONTROLLER_NO_TX;
	else if (spi->comm_mode == 2)
		master_flags = SPI_CONTROLLER_HALF_DUPLEX;
	else
		master_flags = SPI_CONTROLLER_MUST_RX | SPI_CONTROLLER_MUST_TX;


	// ret =
	//   of_property_read_u32(pdev->dev.of_node, "shakti,max-bits-per-word",
	// 		       &max_bits_per_word);

	// if (!ret && max_bits_per_word < 8) {
	// 	dev_err(&pdev->dev, "Only 8bit SPI words supported by the driver\n");
	// 	printk(KERN_ALERT "Only 8bit SPI words supported by the driver\n");
	// 	ret = -EINVAL;
	// 	goto put_master;
	// }

	// Find a clock bus ...
	spi->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(spi->clk)) {
		dev_err(&pdev->dev, "Unable to find bus clock\n");
		ret = PTR_ERR(spi->clk);
		goto put_master;
	}

	/* Spin up the bus clock before hitting registers */
	ret = clk_prepare_enable(spi->clk);
	if (ret) {
		dev_err(&pdev->dev, "Unable to enable bus clock\n");
		goto put_master;
	}
	#ifdef SHAKTI_DEBUG
	printk(KERN_ALERT "printing the value clk_get_rate : %ld\n",clk_get_rate(spi->clk));
	#endif

// 	spi->irq = platform_get_irq(pdev, 0);
// 	if (spi->irq <= 0) {
// 		ret = spi->irq;
// 		if (ret != -EPROBE_DEFER)
// 			dev_err(&pdev->dev, "failed to get irq: %d\n", ret);
// 		goto err_master_put;
// 	}
// 	ret = devm_request_threaded_irq(&pdev->dstm32_spiev, spi->irq,
// 					spi->cfg->irq_handler_event,
// 					spi->cfg->irq_handler_thread,
// 					IRQF_ONESHOT, pdev->name, master);
// 	if (ret) {
// 		dev_err(&pdev->dev, "irq%d request failed: %d\n", spi->irq,
// 			ret);
// 		goto err_master_put;
// 	}


	spi->clk_rate = clk_get_rate(spi->clk);
	if (!spi->clk_rate) {
		dev_err(&pdev->dev, "clk rate = 0\n");
		ret = -EINVAL;
		goto disable_clk;
	}
	// if (spi->cfg->has_fifo)  Already added using dts or default value..
	// 	spi->fifo_size = 16; //32;  // TO-DO: Verify the value.
		//spi->cfg->get_fifo_size(spi);
	pdev->dev.dma_mask = NULL;

	ret = shakti_spi_configure(spi);
	if (ret) {
		dev_err(&pdev->dev, "controller configuration failed: %d\n", ret);
		goto disable_clk;
	}

	master->dev.of_node = pdev->dev.of_node;
	master->bus_num = pdev->id;
	master->mode_bits =  master_mode_bits;
	master->bits_per_word_mask = SPI_BPW_MASK(8); // TO-DO: To configure for 16 bits...//spi->cfg->get_bpw_mask(spi);
	master->max_speed_hz = spi->clk_rate / (spi->cfg->baud_rate_div_min + 1);
	master->min_speed_hz = spi->clk_rate / (spi->cfg->baud_rate_div_max + 1);
	master->setup = shakti_spi_setup;
	master->prepare_message = shakti_spi_prepare_msg;
	master->transfer_one = shakti_spi_transfer_one;
	master->unprepare_message = shakti_spi_unprepare_msg;
	master->flags = master_flags;

	ret = devm_spi_register_master(&pdev->dev, master);
	if (ret) {
		dev_err(&pdev->dev, "spi master registration failed: %d\n", ret);
		goto disable_clk;
	}

	dev_info(&pdev->dev, "Shakti SPI Driver initialized\n");
	#ifdef SHAKTI_DEBUG
	printk(KERN_ALERT "[SHAKTI-INFO] Shakti SPI Driver initialized\n");
	#endif

	return 0;

disable_clk:
	clk_disable_unprepare(spi->clk);
put_master:
	spi_master_put(master);

	return ret;
}

/*
 * shakti_spi_remove - Remove function to remove the module. This function is
 * useful only when using this as a module.
 */
static int shakti_spi_remove(struct platform_device *pdev)
{
	struct spi_master *master = platform_get_drvdata(pdev);
	struct shakti_spi *spi = spi_master_get_devdata(master);

	// /* Disable all the interrupts just in case */
	clk_disable_unprepare(spi->clk);
	printk(KERN_ALERT "[SHAKTI-INFO} Shakti SPI Remove...\n");
	return 0;
}

static const struct of_device_id shakti_spi_of_match[] = {
	{ .compatible = "shakti,spi1", .data = (void *)&shakti_spi1_cfg },
	{}
};

MODULE_DEVICE_TABLE(of, shakti_spi_of_match);

static struct platform_driver shakti_spi_driver = {
	.probe = shakti_spi_probe,
	.remove = shakti_spi_remove,
	.driver = {
		.name = SHAKTI_SPI_DRIVER_NAME,
		.of_match_table = shakti_spi_of_match,
	},
};

module_platform_driver(shakti_spi_driver);

MODULE_ALIAS("platform:" SHAKTI_SPI_DRIVER_NAME);
MODULE_AUTHOR("Sambhav Jain, Venkatakrishnan Sutharsan");
MODULE_DESCRIPTION("Shakti SPI driver");
MODULE_LICENSE("GPL v2");
