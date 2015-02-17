/*
 *   serial-pl011.c
 *   Copyright (c) by Jaroslav Kysela <perex@perex.cz>,
 *                    Isaku Yamahata <yamahata@private.email.ne.jp>,
 *		      George Hansper <ghansper@apana.org.au>,
 *		      Hannu Savolainen
 *		      Kim Taylor
 *
 *   This code is based on the code from serial-u16550.
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 * Sat Mar 31 17:27:57 PST 2001 tim.mann@compaq.com 
 *      Added support for the Midiator MS-124T and for the MS-124W in
 *      Single Addressed (S/A) or Multiple Burst (M/B) mode, with
 *      power derived either parasitically from the serial port or
 *      from a separate power supply.
 *
 *      More documentation can be found in serial-u16550.txt.
 */

#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/ioport.h>
#include <linux/io.h>
#include <linux/module.h>
#include <sound/core.h>
#include <sound/rawmidi.h>
#include <sound/initval.h>

#include <linux/amba/bus.h>
#include <linux/amba/serial.h>
#include <linux/pinctrl/consumer.h>
#include <linux/clk.h>
#include <linux/jiffies.h>

#include <asm/io.h>

MODULE_DESCRIPTION("MIDI serial pl011");
MODULE_LICENSE("GPL");
MODULE_SUPPORTED_DEVICE("{{ALSA, MIDI serial pl011}}");

#define SNDRV_SERIAL_SOUNDCANVAS 0 /* Roland Soundcanvas; F5 NN selects part */
#define SNDRV_SERIAL_MS124T 1      /* Midiator MS-124T */
#define SNDRV_SERIAL_MS124W_SA 2   /* Midiator MS-124W in S/A mode */
#define SNDRV_SERIAL_MS124W_MB 3   /* Midiator MS-124W in M/B mode */
#define SNDRV_SERIAL_GENERIC 4     /* Generic Interface */
#define SNDRV_SERIAL_MAX_ADAPTOR SNDRV_SERIAL_GENERIC
static char *adaptor_names[] = {
	"Soundcanvas",
        "MS-124T",
	"MS-124W S/A",
	"MS-124W M/B",
	"Generic"
};

#define SNDRV_SERIAL_NORMALBUFF 0 /* Normal blocking buffer operation */
#define SNDRV_SERIAL_DROPBUFF   1 /* Non-blocking discard operation */
#define SNDRV_SERIAL_NOTHROTTLE 0
#define SNDRV_SERIAL_NORTSCTS 0
#define SNDRV_SERIAL_DEFAULT_FIFO 16
#define TIMER_ATTEMPTS_LIMIT 255

static int speed = 115200; /* 9600,19200,38400,57600,115200 */
static int outs = 1;	 /* 1 to 16 */
static int ins = 1;	/* 1 to 16 */
static int adaptor = SNDRV_SERIAL_GENERIC;
static bool droponfull = SNDRV_SERIAL_NORMALBUFF;
static bool throttle_tx = SNDRV_SERIAL_NOTHROTTLE;
static int throttle_delay = 1000;
static int dynamic_throttle = 0;
static int fifo_limit = SNDRV_SERIAL_DEFAULT_FIFO;
static bool flow_control = SNDRV_SERIAL_NORTSCTS;

module_param(speed, int, 0444);
MODULE_PARM_DESC(speed, "Speed in bauds.");
module_param(outs, int, 0444);
MODULE_PARM_DESC(outs, "Number of MIDI outputs.");
module_param(ins, int, 0444);
MODULE_PARM_DESC(ins, "Number of MIDI inputs.");
module_param(droponfull, bool, 0444);
MODULE_PARM_DESC(droponfull, "Flag to enable drop-on-full buffer mode");
module_param(throttle_tx, bool, 0444);
MODULE_PARM_DESC(throttle_tx, "Do not continuously fill TX FIFO");
module_param(throttle_delay, int, 0444);
MODULE_PARM_DESC(throttle_delay, "Time in us between each TX write");
module_param(dynamic_throttle, int, 0444);
MODULE_PARM_DESC(dynamic_throttle, "Dynamic delay between writes (percentage)");
module_param(flow_control, bool, 0444);
MODULE_PARM_DESC(flow_control, "Use RTS/CTS flow control");
module_param(fifo_limit, int, 0444);
MODULE_PARM_DESC(fifo_limit, "Maximum TX bytes per write");

module_param(adaptor, int, 0444);
MODULE_PARM_DESC(adaptor, "Type of adaptor.");

/*#define SNDRV_SERIAL_MS124W_MB_NOCOMBO 1*/  /* Address outs as 0-3 instead of bitmap */

#define SNDRV_SERIAL_MAX_OUTS	16		/* max 64, min 16 */
#define SNDRV_SERIAL_MAX_INS	16		/* max 64, min 16 */

#define TX_BUFF_SIZE		(1<<15)		/* Must be 2^n */
#define TX_BUFF_MASK		(TX_BUFF_SIZE - 1)

#define AMBA_ISR_PASS_LIMIT	256

#define SERIAL_MODE_NOT_OPENED 		(0)
#define SERIAL_MODE_INPUT_OPEN		(1 << 0)
#define SERIAL_MODE_OUTPUT_OPEN		(1 << 1)
#define SERIAL_MODE_INPUT_TRIGGERED	(1 << 2)
#define SERIAL_MODE_OUTPUT_TRIGGERED	(1 << 3)

struct snd_uart_pl011 {
	struct amba_device *dev;
	struct snd_card *card;
	struct snd_rawmidi *rmidi;
	struct snd_rawmidi_substream *midi_output[SNDRV_SERIAL_MAX_OUTS];
	struct snd_rawmidi_substream *midi_input[SNDRV_SERIAL_MAX_INS];

	int filemode;		/* open status of file */

	spinlock_t open_lock;
	wait_queue_head_t drain_wait;
	int draining;

	int irq;

	void __iomem *membase;
	unsigned long mapbase;
	struct resource *res_base;

	struct clk *clk;
	unsigned long clk_rate;
	unsigned int speed;

	/* parameter for using of write loop */
	short int fifo_limit;
        short int fifo_count;

	/* type of adaptor */
	int adaptor;

	/* inputs */
	int prev_in;
	unsigned char rstatus;

	/* outputs */
	int prev_out;
	unsigned char prev_status[SNDRV_SERIAL_MAX_OUTS];

	/* write buffer and its writing/reading position */
	unsigned char tx_buff[TX_BUFF_SIZE];
	int buff_in_count;
        int buff_in;
        int buff_out;
        int drop_on_full;
	int throttle_tx;
	int dynamic_throttle;
	int flow_control;
	ktime_t throttle_delay;

	int timer_running;
	u16 control_reg;
	enum {
		TX_IDLE,
		TX_BLOCK_RX,
		TX_BUSY,
	} tx_state;
	int tx_attempts;
	struct hrtimer buffer_timer;
	unsigned char bytes_pending[SNDRV_SERIAL_MAX_OUTS];
};

static inline void snd_uart_pl011_stop_rx(struct snd_uart_pl011 *uart)
{
	writew(uart->control_reg & ~UART011_CR_RTS, uart->membase + UART011_CR);
}

static inline void snd_uart_pl011_start_rx(struct snd_uart_pl011 *uart)
{
	writew(uart->control_reg | UART011_CR_RTS, uart->membase + UART011_CR);
}

static inline void snd_uart_pl011_reset_delay_times(struct snd_uart_pl011 *uart)
{
	if (uart->dynamic_throttle == 0) return;

	memset(uart->bytes_pending, 0, 
			sizeof(unsigned char) * SNDRV_SERIAL_MAX_OUTS);
}

static inline void snd_uart_pl011_update_delay_time(struct snd_uart_pl011 *uart)
{
	enum {	CUR_OUTPUT_INIT = -2,
		CUR_OUTPUT_CHANGE = -1,
		CUR_OUTPUT_VALID = 0
	};
	static int current_output = CUR_OUTPUT_INIT;

	if (uart->dynamic_throttle == 0) return;

	if (uart->tx_buff[uart->buff_out] == 0xf5) {
		current_output = CUR_OUTPUT_CHANGE;
		return;
	}

	if (current_output == CUR_OUTPUT_CHANGE) {
		current_output = uart->tx_buff[uart->buff_out] - 1;
	}

	if (current_output >= CUR_OUTPUT_VALID) {
		uart->bytes_pending[current_output]++;
	}
}

static inline void snd_uart_pl011_update_write_delay(
		struct snd_uart_pl011 *uart)
{
	/* Delay based on MIDI baud rate of 31250 and maximum number of bytes
	 * to send for any channel. A MIDI byte is 10 bits, so the delay time
	 * per byte is (10/31250) = 320 us */
	unsigned char i, max_bytes;

	if (uart->dynamic_throttle == 0) return;

	max_bytes = 0;
	for (i = 0; i < SNDRV_SERIAL_MAX_OUTS; i++) {
		if (uart->bytes_pending[i] > max_bytes)
			max_bytes = uart->bytes_pending[i];
	}
	uart->throttle_delay = ns_to_ktime(uart->dynamic_throttle * 
			(max_bytes * 320 * (1000/100)));
}

/* This macro is only used in snd_uart_pl011_io_loop */
static inline void snd_uart_pl011_buffer_output(struct snd_uart_pl011 *uart)
{
	unsigned short buff_out = uart->buff_out;
	writeb(uart->tx_buff[buff_out], uart->membase + UART01x_DR);
	uart->fifo_count++;
	buff_out++;
	buff_out &= TX_BUFF_MASK;
	uart->buff_out = buff_out;
	uart->buff_in_count--;
}

static inline int snd_uart_pl011_write_fifo_timer(struct snd_uart_pl011 * uart)
{
	/* Check CTS - FIFO is empty */
	if (!uart->flow_control ||
			readw(uart->membase + UART01x_FR) & UART01x_FR_CTS) {
		snd_uart_pl011_reset_delay_times(uart);
		while (uart->fifo_count < uart->fifo_limit
			&& uart->buff_in_count > 0) {
			snd_uart_pl011_buffer_output(uart);
			snd_uart_pl011_update_delay_time(uart);
		}
		snd_uart_pl011_update_write_delay(uart);
	
		uart->tx_state = TX_BUSY;
		if (unlikely(uart->draining))
			wake_up(&uart->drain_wait);
		return 1;
	}
	return 0;
}

static inline void snd_uart_pl011_start_timer(struct snd_uart_pl011 *uart)
{
        if (!uart->timer_running) {
		if (uart->flow_control) {
			snd_uart_pl011_stop_rx(uart);
			uart->tx_state = TX_BLOCK_RX;
			uart->tx_attempts = TIMER_ATTEMPTS_LIMIT;
		} else {
			snd_uart_pl011_write_fifo_timer(uart);
		}
		hrtimer_start(&uart->buffer_timer, uart->throttle_delay,
				HRTIMER_MODE_REL);
		uart->timer_running = 1;
        }
}

static inline void snd_uart_pl011_del_timer(struct snd_uart_pl011 *uart)
{
        hrtimer_cancel(&uart->buffer_timer);
        uart->timer_running = 0;
}

/* This loop should be called with interrupts disabled
 * We don't want to interrupt this, 
 * as we're already handling an interrupt 
 *
 * PL011 interrupts that must be serviced (and cleared):
 * UART011_RXIC	    (RX FIFO becoming full)
 * UART011_TXIC	    (TX FIFO becoming empty)
 * UART011_RTIC	    (RX timeout reached)
 */
static void snd_uart_pl011_io_loop(struct snd_uart_pl011 * uart)
{
	unsigned char c;
	int substream;
	int pass_counter = AMBA_ISR_PASS_LIMIT;

	/* recall previous stream */
	substream = uart->prev_in;
    
	/* Read Loop */
	while (!(readw(uart->membase + UART01x_FR) & UART01x_FR_RXFE)) {
		/* while receive data ready */
		c = readb(uart->membase + UART01x_DR);

		/* keep track of last status byte */
		if (c & 0x80)
			uart->rstatus = c;

		/* handle stream switch */
		if (uart->adaptor == SNDRV_SERIAL_GENERIC) {
			if (uart->rstatus == 0xf5) {
				if (c <= SNDRV_SERIAL_MAX_INS && c > 0)
					substream = c - 1;
				if (c != 0xf5)
					/* prevent future bytes from being
					   interpreted as streams */
					uart->rstatus = 0;
			} else if ((uart->filemode & SERIAL_MODE_INPUT_OPEN)
				   && uart->midi_input[substream])
				snd_rawmidi_receive(uart->midi_input[substream],
						    &c, 1);
		} else if ((uart->filemode & SERIAL_MODE_INPUT_OPEN) &&
			   uart->midi_input[substream])
			snd_rawmidi_receive(uart->midi_input[substream], &c, 1);

		if (readw(uart->membase + UART01x_FR) & UART011_FR_RXFF)
			snd_printk(KERN_WARNING
				   "%s: Overrun on device at 0x%lx\n",
			       uart->rmidi->name, uart->mapbase);

		if (pass_counter-- == 0) break;
	}

	/* remember the last stream */
	uart->prev_in = substream;

	if (uart->throttle_tx) return;

	/* Check write status, if we get a TX fifo interrupt,
	 * it's possible that there are still 2 bytes of data
	 * in the FIFO */
	if (readw(uart->membase + UART011_MIS) & UART011_TXIS) {
		writew(UART011_TXIC, uart->membase + UART011_ICR);
		uart->fifo_count = 2;
		if (unlikely(uart->draining)) wake_up(&uart->drain_wait);
	}

	if (readw(uart->membase + UART01x_FR) & UART011_FR_TXFE)
		uart->fifo_count = 0;

	/* Write loop */
	while (uart->fifo_count < uart->fifo_limit /* Can we write ? */
		&& uart->buff_in_count > 0)	/* Do we want to? */
		snd_uart_pl011_buffer_output(uart);
}

static irqreturn_t snd_uart_pl011_interrupt(int irq, void *dev_id)
{
	struct snd_uart_pl011 *uart;

	uart = dev_id;
	spin_lock(&uart->open_lock);
	if (uart->filemode == SERIAL_MODE_NOT_OPENED) {
		spin_unlock(&uart->open_lock);
		return IRQ_NONE;
	}

	snd_uart_pl011_io_loop(uart);
	spin_unlock(&uart->open_lock);
	return IRQ_HANDLED;
}

static enum hrtimer_restart snd_uart_pl011_buffer_timer(struct hrtimer *handle)
{
        struct snd_uart_pl011 *uart = 
		container_of(handle, struct snd_uart_pl011, buffer_timer);
	enum hrtimer_restart restart = HRTIMER_NORESTART;
	int double_delay = 0;

        spin_lock(&uart->open_lock);

	switch (uart->tx_state) {
	    case TX_IDLE:
		snd_uart_pl011_stop_rx(uart);
		uart->tx_attempts = TIMER_ATTEMPTS_LIMIT;
		uart->tx_state = TX_BLOCK_RX;
		restart = HRTIMER_RESTART;
		break;

	    case TX_BLOCK_RX:
		/* Write FIFO, reschedule if necessary */
		if (snd_uart_pl011_write_fifo_timer(uart)) {
			restart = HRTIMER_RESTART;
		} else {
			/* If the cable becomes disconnected, we may never get 
			 * CTS. Therefore limit the number of times we call the
			 * timer */
			if (uart->tx_attempts--) restart = HRTIMER_RESTART;
			else uart->tx_state = TX_IDLE;
		}

		break;

	    case TX_BUSY:
		if (readw(uart->membase + UART01x_FR) & UART01x_FR_BUSY) {
			/* Still writing, reschedule */
			restart = HRTIMER_RESTART;
		} else {
			/* Finished writing allow receive */
			uart->fifo_count = 0;
			if (uart->flow_control) {
				double_delay = 1;
				uart->tx_state = TX_IDLE;
				snd_uart_pl011_start_rx(uart);
			} else {
				uart->tx_state = TX_BLOCK_RX;
			}
			if (uart->buff_in_count > 0) {
				restart = HRTIMER_RESTART;
			}
		}
		break;
	}

	if (restart == HRTIMER_RESTART) {
		hrtimer_forward_now(&uart->buffer_timer,
			double_delay ? ktime_add(uart->throttle_delay, 
				uart->throttle_delay) : uart->throttle_delay);
	} else
		uart->timer_running = 0;

        spin_unlock(&uart->open_lock);

	return restart;
}

static int snd_uart_pl011_detect(struct snd_uart_pl011 *uart)
{
	int ok = 0;
	u16 status;
	int timeout = 1000;

	writew(0, uart->membase + UART011_IMSC);    /* Disable interrupts */

	writew(UART01x_CR_UARTEN
	     | UART011_CR_TXE
	     | UART011_CR_LBE
	     | UART011_CR_RXE
	     , uart->membase + UART011_CR);

	writew(0, uart->membase + UART011_FBRD);
	writew(1, uart->membase + UART011_IBRD);
	writew(0, uart->membase + UART011_LCRH);
	writew(0x55, uart->membase + UART01x_DR);
	while (timeout &&
		    (readw(uart->membase + UART01x_FR) & UART01x_FR_BUSY)) {
		timeout--;
		barrier();
	}
	status = readw(uart->membase + UART01x_DR) & 0xff;
	writew(0xffff, uart->membase + UART011_ICR);	/* Clear interrupts */
	
	/* Loopback with WLEN == 5 turns 0x55 into 0x15 */
	if (status == 0x15) ok = 1;

	return ok;
}

static void snd_uart_pl011_do_open(struct snd_uart_pl011 * uart)
{
	unsigned int quot;
	u16 reg;

	/* Initialize basic variables */
	uart->buff_in_count = 0;
	uart->buff_in = 0;
	uart->buff_out = 0;
	uart->fifo_count = 0;
	uart->tx_state = TX_IDLE;

	writew(UART01x_CR_UARTEN	/* Enable UART */
	     | UART011_CR_TXE		/* Enable UART TX */
	     | UART011_CR_RXE		/* Enable UART RX */
	     , uart->membase + UART011_CR);

	uart->clk_rate = clk_get_rate(uart->clk);
	quot = DIV_ROUND_CLOSEST(uart->clk_rate * 4, uart->speed);

	writew(quot & 0x3f, uart->membase + UART011_FBRD);
	writew(quot >> 6, uart->membase + UART011_IBRD);

	writew(UART01x_LCRH_FEN		/* Enable FIFOs */
	     | UART01x_LCRH_WLEN_8	/* 8 Bit words, 1 Stop, No Parity */
	     , uart->membase + UART011_LCRH);	/* FIFO Control Register */

	writew(UART011_IFLS_RX2_8 /* Set RX FIFO trigger at 4-bytes */
	     | UART011_IFLS_TX1_8 /* Set TX FIFO trigger at 2-bytes */
	     , uart->membase + UART011_IFLS);

	reg = readw(uart->membase + UART011_CR);
	switch (uart->adaptor) {
	default:
		reg |=	UART011_CR_RTS
			/* Hardware RTS unless throttling*/
		    | (uart->throttle_tx ? 0 : UART011_CR_RTSEN)	
			/* Hardware CTS unless throttling */
		    | (uart->throttle_tx ? 0 : UART011_CR_CTSEN);
		writew(reg, uart->membase + UART011_CR);
		break;
	case SNDRV_SERIAL_MS124W_SA:
	case SNDRV_SERIAL_MS124W_MB:
		/* FIXME: MS-124W can draw power from RTS and DTR if they
		   are in opposite states. */ 
		break;
	case SNDRV_SERIAL_MS124T:
		/* FIXME: MS-124T can draw power from RTS and/or DTR (preferably
		   both) if they are both asserted. */
		break;
	}

	uart->control_reg = reg;

	writew(UART011_RXIC /* Clear corresponting interrupts */
	     | UART011_TXIC
	     | UART011_RTIC
	     , uart->membase + UART011_ICR);

	if (uart->adaptor == SNDRV_SERIAL_MS124W_SA) {
		/* FIXME: Enable RX data and Modem Status */
	} else if (uart->adaptor == SNDRV_SERIAL_GENERIC) {
		writew(UART011_RXIM	/* Enable RX FIFO interrupt */
		     | UART011_RTIM	/* Enable RX timeout interrupt */
			/* Enable TX FIFO if not using throttling */
		     | (uart->throttle_tx ? 0 : UART011_TXIM)
		     , uart->membase + UART011_IMSC);
	} else {
		/* FIXME: Enable RX data and THRI */
	}
}

static void snd_uart_pl011_do_close(struct snd_uart_pl011 * uart)
{
	writew(0, uart->membase + UART011_IMSC); /* Interrupt enable Register */
	writew(0xffff, uart->membase + UART011_ICR);

	switch (uart->adaptor) {
	default:
		/* Disable everything */
		writew(0, uart->membase + UART011_CR); 
		break;
	case SNDRV_SERIAL_MS124W_SA:
	case SNDRV_SERIAL_MS124W_MB:
		/* FIXME: MS-124W can draw power from RTS and DTR if they
		   are in opposite states; leave it powered. */ 
		break;
	case SNDRV_SERIAL_MS124T:
		/* FIXME: MS-124T can draw power from RTS and/or DTR (preferably
		   both) if they are both asserted; leave it powered. */
		break;
	}

}

static int snd_uart_pl011_input_open(struct snd_rawmidi_substream *substream)
{
	unsigned long flags;
	struct snd_uart_pl011 *uart = substream->rmidi->private_data;

	spin_lock_irqsave(&uart->open_lock, flags);
	if (uart->filemode == SERIAL_MODE_NOT_OPENED)
		snd_uart_pl011_do_open(uart);
	uart->filemode |= SERIAL_MODE_INPUT_OPEN;
	uart->midi_input[substream->number] = substream;
	spin_unlock_irqrestore(&uart->open_lock, flags);
	return 0;
}

static int snd_uart_pl011_input_close(struct snd_rawmidi_substream *substream)
{
	unsigned long flags;
	struct snd_uart_pl011 *uart = substream->rmidi->private_data;

	spin_lock_irqsave(&uart->open_lock, flags);
	uart->filemode &= ~SERIAL_MODE_INPUT_OPEN;
	uart->midi_input[substream->number] = NULL;
	if (uart->filemode == SERIAL_MODE_NOT_OPENED)
		snd_uart_pl011_do_close(uart);
	spin_unlock_irqrestore(&uart->open_lock, flags);
	return 0;
}

static void snd_uart_pl011_input_trigger(struct snd_rawmidi_substream *substream,
					int up)
{
	unsigned long flags;
	struct snd_uart_pl011 *uart = substream->rmidi->private_data;

	spin_lock_irqsave(&uart->open_lock, flags);
	if (up)
		uart->filemode |= SERIAL_MODE_INPUT_TRIGGERED;
	else
		uart->filemode &= ~SERIAL_MODE_INPUT_TRIGGERED;
	spin_unlock_irqrestore(&uart->open_lock, flags);
}

static int snd_uart_pl011_output_open(struct snd_rawmidi_substream *substream)
{
	unsigned long flags;
	struct snd_uart_pl011 *uart = substream->rmidi->private_data;

	spin_lock_irqsave(&uart->open_lock, flags);
	if (uart->filemode == SERIAL_MODE_NOT_OPENED)
		snd_uart_pl011_do_open(uart);
	uart->filemode |= SERIAL_MODE_OUTPUT_OPEN;
	uart->midi_output[substream->number] = substream;
	spin_unlock_irqrestore(&uart->open_lock, flags);
	return 0;
};

static int snd_uart_pl011_output_close(struct snd_rawmidi_substream *substream)
{
	unsigned long flags;
	struct snd_uart_pl011 *uart = substream->rmidi->private_data;

	spin_lock_irqsave(&uart->open_lock, flags);
	uart->filemode &= ~SERIAL_MODE_OUTPUT_OPEN;
	uart->midi_output[substream->number] = NULL;
	if (uart->filemode == SERIAL_MODE_NOT_OPENED)
		snd_uart_pl011_do_close(uart);
	spin_unlock_irqrestore(&uart->open_lock, flags);
	return 0;
};

static inline int snd_uart_pl011_buffer_can_write(struct snd_uart_pl011 *uart,
						 int Num)
{
	if (uart->buff_in_count + Num < TX_BUFF_SIZE)
		return 1;
	else
		return 0;
}

static inline int snd_uart_pl011_write_buffer(struct snd_uart_pl011 *uart,
					     unsigned char byte)
{
	unsigned short buff_in = uart->buff_in;
	if (uart->buff_in_count < TX_BUFF_SIZE) {
		uart->tx_buff[buff_in] = byte;
		buff_in++;
		buff_in &= TX_BUFF_MASK;
		uart->buff_in = buff_in;
		uart->buff_in_count++;
		if (uart->throttle_tx)
			snd_uart_pl011_start_timer(uart);
		return 1;
	} else
		return 0;
}

static int snd_uart_pl011_output_byte(struct snd_uart_pl011 *uart,
				     struct snd_rawmidi_substream *substream,
				     unsigned char midi_byte)
{
	if ((uart->buff_in_count == 0) && !uart->throttle_tx) {
	        /* Tx Buffer Empty - try to write immediately */
		if (readw(uart->membase + UART01x_FR) & UART011_FR_TXFE) {
		        uart->fifo_count = 1;
			writeb(midi_byte, uart->membase + UART01x_DR);
		} else {
		        if (uart->fifo_count < uart->fifo_limit) {
			        uart->fifo_count++;
				writeb(midi_byte, uart->membase + UART01x_DR);
			} else {
			        /* Cannot write (buffer empty) -
				 * put char in buffer */
				snd_uart_pl011_write_buffer(uart, midi_byte);
			}
		}
	} else {
		if (!snd_uart_pl011_write_buffer(uart, midi_byte)) {
			snd_printk(KERN_WARNING
				   "%s: Buffer overrun on device at 0x%lx\n",
				   uart->rmidi->name, uart->mapbase);
			return 0;
		}
	}
	return 1;
}

static void snd_uart_pl011_output_write(struct snd_rawmidi_substream *substream)
{
	unsigned char midi_byte, addr_byte;
	struct snd_uart_pl011 *uart = substream->rmidi->private_data;
	char first;
	static unsigned long lasttime = 0;
	
	/* Interrupts are disabled during the updating of the tx_buff,
	 * since it is 'bad' to have two processes updating the same
	 * variables (ie buff_in & buff_out)
	 */

	if (uart->adaptor == SNDRV_SERIAL_MS124W_MB) {
		while (1) {
			/* buffer full? */
			/* in this mode we need two bytes of space */
			if (uart->buff_in_count > TX_BUFF_SIZE - 2)
				break;
			if (snd_rawmidi_transmit(substream, &midi_byte, 1) != 1)
				break;
#ifdef SNDRV_SERIAL_MS124W_MB_NOCOMBO
			/* select exactly one of the four ports */
			addr_byte = (1 << (substream->number + 4)) | 0x08;
#else
			/* select any combination of the four ports */
			addr_byte = (substream->number << 4) | 0x08;
			/* ...except none */
			if (addr_byte == 0x08)
				addr_byte = 0xf8;
#endif
			snd_uart_pl011_output_byte(uart, substream, addr_byte);
			/* send midi byte */
			snd_uart_pl011_output_byte(uart, substream, midi_byte);
		}
	} else {
		first = 0;
		while (snd_rawmidi_transmit_peek(substream, &midi_byte, 1) == 1) {
			/* Also send F5 after 3 seconds with no data
			 * to handle device disconnect */
			if (first == 0 &&
			    (uart->adaptor == SNDRV_SERIAL_SOUNDCANVAS ||
			     uart->adaptor == SNDRV_SERIAL_GENERIC) &&
			    (uart->prev_out != substream->number ||
			     time_after(jiffies, lasttime + 3*HZ))) {

				if (snd_uart_pl011_buffer_can_write(uart, 3)) {
					/* Roland Soundcanvas part selection */
					/* If this substream of the data is
					 * different previous substream
					 * in this uart, send the change part
					 * event
					 */
					uart->prev_out = substream->number;
					/* change part */
					snd_uart_pl011_output_byte(uart, substream,
								  0xf5);
					/* data */
					snd_uart_pl011_output_byte(uart, substream,
								  uart->prev_out + 1);
					/* If midi_byte is a data byte,
					 * send the previous status byte */
					if (midi_byte < 0x80 &&
					    uart->adaptor == SNDRV_SERIAL_SOUNDCANVAS)
						snd_uart_pl011_output_byte(uart, substream, uart->prev_status[uart->prev_out]);
				} else if (!uart->drop_on_full)
					break;

			}

			/* send midi byte */
			if (!snd_uart_pl011_output_byte(uart, substream, midi_byte) &&
			    !uart->drop_on_full )
				break;

			if (midi_byte >= 0x80 && midi_byte < 0xf0)
				uart->prev_status[uart->prev_out] = midi_byte;
			first = 1;

			snd_rawmidi_transmit_ack( substream, 1 );
		}
		lasttime = jiffies;
	}
}

static void snd_uart_pl011_output_trigger(struct snd_rawmidi_substream *substream,
					 int up)
{
	unsigned long flags;
	struct snd_uart_pl011 *uart = substream->rmidi->private_data;

	spin_lock_irqsave(&uart->open_lock, flags);
	if (up) {
		uart->filemode |= SERIAL_MODE_OUTPUT_TRIGGERED;
		snd_uart_pl011_output_write(substream);
	} else {
		uart->filemode &= ~SERIAL_MODE_OUTPUT_TRIGGERED;
		snd_uart_pl011_del_timer(uart);
	}
	spin_unlock_irqrestore(&uart->open_lock, flags);
}

static void snd_uart_pl011_output_drain(struct snd_rawmidi_substream *substream)
{
	unsigned long flags;
	struct snd_uart_pl011 *uart = substream->rmidi->private_data;
	DEFINE_WAIT(wait);
	long timeout;

	spin_lock_irqsave(&uart->open_lock, flags);

	if (uart->filemode == SERIAL_MODE_NOT_OPENED) {
		spin_unlock_irqrestore(&uart->open_lock, flags);
		return;
	}

	if (uart->buff_in_count || uart->fifo_count) {
		uart->draining++;
		do {
			timeout = msecs_to_jiffies(50);
			prepare_to_wait(&uart->drain_wait, &wait,
					TASK_UNINTERRUPTIBLE);
			spin_unlock_irqrestore(&uart->open_lock, flags);
			timeout = schedule_timeout(timeout);
			spin_lock_irqsave(&uart->open_lock, flags);
		} while ((uart->buff_in_count || uart->fifo_count) && timeout);
		uart->draining = 0;
		finish_wait(&uart->drain_wait, &wait);
	}
	spin_unlock_irqrestore(&uart->open_lock, flags);
}

static struct snd_rawmidi_ops snd_uart_pl011_output =
{
	.open =		snd_uart_pl011_output_open,
	.close =	snd_uart_pl011_output_close,
	.trigger =	snd_uart_pl011_output_trigger,
	.drain =	snd_uart_pl011_output_drain,
};

static struct snd_rawmidi_ops snd_uart_pl011_input =
{
	.open =		snd_uart_pl011_input_open,
	.close =	snd_uart_pl011_input_close,
	.trigger =	snd_uart_pl011_input_trigger,
};

static int snd_uart_pl011_free(struct snd_uart_pl011 *uart)
{
	if (uart->irq >= 0)
		free_irq(uart->irq, uart);
	if (!IS_ERR(uart->clk) && uart->clk) clk_disable_unprepare(uart->clk);
	if (uart->dev) pinctrl_pm_select_sleep_state(&uart->dev->dev);
	release_and_free_resource(uart->res_base);
	kfree(uart);
	return 0;
};

static int snd_uart_pl011_dev_free(struct snd_device *device)
{
	struct snd_uart_pl011 *uart = device->device_data;
	return snd_uart_pl011_free(uart);
}

static int snd_uart_pl011_create(struct snd_card *card,
				struct amba_device *devptr,
				unsigned int speed,
				int adaptor,
				int droponfull,
				int throttle_tx,
				int throttle_delay,
				int dynamic_throttle,
				int flow_control,
				int fifo_limit,
				struct snd_uart_pl011 **ruart)
{
	static struct snd_device_ops ops = {
		.dev_free =	snd_uart_pl011_dev_free,
	};
	struct snd_uart_pl011 *uart;
	int err;
	void __iomem *membase;


	if ((uart = kzalloc(sizeof(*uart), GFP_KERNEL)) == NULL)
		return -ENOMEM;

	uart->irq = -1;
	uart->res_base = request_mem_region(devptr->res.start, 
			resource_size(&devptr->res), "Serial MIDI");

	if (uart->res_base == NULL) {
		snd_printk(KERN_ERR "pl011: can't grab port\n");
		return -EBUSY;
	}
	
	membase = devm_ioremap(&devptr->dev, devptr->res.start,
				    resource_size(&devptr->res));

	if (!membase) {
		snd_printk(KERN_ERR "pl011: ioremap error\n");
		snd_uart_pl011_free(uart);
		return -ENOMEM;
	}

	uart->clk = devm_clk_get(&devptr->dev, NULL);
	if (IS_ERR(uart->clk)) {
		snd_printk(KERN_ERR "pl011: unable to get clock resource\n");
		snd_uart_pl011_free(uart);
		return -ENODEV;
	}

	clk_prepare_enable(uart->clk);

	uart->adaptor = adaptor;
	uart->card = card;
	spin_lock_init(&uart->open_lock);
	uart->membase = membase;
	uart->mapbase = devptr->res.start;
	uart->drop_on_full = droponfull;
	uart->throttle_tx = throttle_tx;
	uart->throttle_delay = ns_to_ktime(throttle_delay * 1000);
	uart->dynamic_throttle = dynamic_throttle;
	uart->flow_control = flow_control;
	uart->fifo_limit = fifo_limit;
	uart->speed = speed;
	uart->prev_out = -1;
	memset(uart->prev_status, 0x80,
			sizeof(unsigned char) * SNDRV_SERIAL_MAX_OUTS);
	hrtimer_init(&uart->buffer_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
        uart->buffer_timer.function = snd_uart_pl011_buffer_timer;

	if (snd_uart_pl011_detect(uart) == 0) {
		snd_printk(KERN_ERR "no UART detected\n");
		snd_uart_pl011_free(uart);
		return -ENODEV;
	}

	if (request_irq(devptr->irq[0], snd_uart_pl011_interrupt,
				0, "Serial MIDI", uart)) {
		snd_printk(KERN_ERR "unable to request IRQ\n");
		snd_uart_pl011_free(uart);
		return -ENODEV;
	}
	uart->irq = devptr->irq[0];

	init_waitqueue_head(&uart->drain_wait);

	snd_printk(KERN_INFO "Detected PL011 at 0x%lx using irq: %i\n",
			uart->mapbase, uart->irq);

	/* Register device */
	if ((err = snd_device_new(card, SNDRV_DEV_LOWLEVEL, uart, &ops)) < 0) {
		snd_uart_pl011_free(uart);
		return err;
	}

	/* FIXME: CTS/RTS pins */
	uart->dev = devptr;
	pinctrl_pm_select_default_state(&uart->dev->dev);

	switch (uart->adaptor) {
	case SNDRV_SERIAL_MS124W_SA:
	case SNDRV_SERIAL_MS124W_MB:
		/* FIXME: MS-124W can draw power from RTS and DTR if they
		   are in opposite states. */ 
		break;
	case SNDRV_SERIAL_MS124T:
		/* FIXME: MS-124T can draw power from RTS and/or DTR (preferably
		   both) if they are asserted. */
		break;
	default:
		break;
	}

	if (ruart)
		*ruart = uart;

	return 0;
}

static void snd_uart_pl011_substreams(struct snd_rawmidi_str *stream)
{
	struct snd_rawmidi_substream *substream;

	list_for_each_entry(substream, &stream->substreams, list) {
		sprintf(substream->name, "Serial MIDI %d", substream->number + 1);
	}
}

static int snd_uart_pl011_rmidi(struct snd_uart_pl011 *uart, int device,
			       int outs, int ins,
			       struct snd_rawmidi **rmidi)
{
	struct snd_rawmidi *rrawmidi;
	int err;

	err = snd_rawmidi_new(uart->card, "UART Serial MIDI", device,
			      outs, ins, &rrawmidi);
	if (err < 0)
		return err;
	snd_rawmidi_set_ops(rrawmidi, SNDRV_RAWMIDI_STREAM_INPUT,
			    &snd_uart_pl011_input);
	snd_rawmidi_set_ops(rrawmidi, SNDRV_RAWMIDI_STREAM_OUTPUT,
			    &snd_uart_pl011_output);
	strcpy(rrawmidi->name, "Serial MIDI");
	snd_uart_pl011_substreams(&rrawmidi->streams[SNDRV_RAWMIDI_STREAM_OUTPUT]);
	snd_uart_pl011_substreams(&rrawmidi->streams[SNDRV_RAWMIDI_STREAM_INPUT]);
	rrawmidi->info_flags = SNDRV_RAWMIDI_INFO_OUTPUT |
			       SNDRV_RAWMIDI_INFO_INPUT |
			       SNDRV_RAWMIDI_INFO_DUPLEX;
	rrawmidi->private_data = uart;
	if (rmidi)
		*rmidi = rrawmidi;
	return 0;
}

static int snd_serial_probe(struct amba_device *devptr,
		const struct amba_id *id)
{
	struct snd_card *card;
	struct snd_uart_pl011 *uart;
	int err;

	switch (adaptor) {
	case SNDRV_SERIAL_SOUNDCANVAS:
		ins = 1;
		break;
	case SNDRV_SERIAL_MS124T:
	case SNDRV_SERIAL_MS124W_SA:
		outs = 1;
		ins = 1;
		break;
	case SNDRV_SERIAL_MS124W_MB:
		outs = 16;
		ins = 1;
		break;
	case SNDRV_SERIAL_GENERIC:
		break;
	default:
		snd_printk(KERN_ERR
			   "Adaptor type is out of range 0-%d (%d)\n",
			   SNDRV_SERIAL_MAX_ADAPTOR, adaptor);
		return -ENODEV;
	}

	if (outs < 1 || outs > SNDRV_SERIAL_MAX_OUTS) {
		snd_printk(KERN_ERR
			   "Count of outputs is out of range 1-%d (%d)\n",
			   SNDRV_SERIAL_MAX_OUTS, outs);
		return -ENODEV;
	}

	if (ins < 1 || ins > SNDRV_SERIAL_MAX_INS) {
		snd_printk(KERN_ERR
			   "Count of inputs is out of range 1-%d (%d)\n",
			   SNDRV_SERIAL_MAX_INS, ins);
		return -ENODEV;
	}

	err  = snd_card_new(&devptr->dev, -1, NULL, THIS_MODULE,
			    0, &card);
	if (err < 0)
		return err;

	strcpy(card->driver, "Serial");
	strcpy(card->shortname, "Serial MIDI (PL011)");

	if ((err = snd_uart_pl011_create(card, devptr,
					speed,
					adaptor,
					droponfull,
					throttle_tx,
					throttle_delay,
					dynamic_throttle,
					flow_control,
					fifo_limit,
					&uart)) < 0)
		goto _err;

	err = snd_uart_pl011_rmidi(uart, 0, outs, ins, &uart->rmidi);
	if (err < 0)
		goto _err;

	sprintf(card->longname, "%s [%s] at %#lx, irq %d",
		card->shortname,
		adaptor_names[uart->adaptor],
		(unsigned long) uart->membase,
		uart->irq);

	snd_card_set_dev(card, &devptr->dev);

	if ((err = snd_card_register(card)) < 0)
		goto _err;

	amba_set_drvdata(devptr, card);
	return 0;

 _err:
	snd_card_free(card);
	return err;
}

static int snd_serial_remove(struct amba_device *devptr)
{
	snd_card_free(amba_get_drvdata(devptr));
	return 0;
}

#define SND_SERIAL_DRIVER	"snd_serial_pl011"

static struct amba_id snd_serial_ids[] = {
	{
		.id     = 0x00041011,
		.mask   = 0x000fffff,
	},
	{ 0, 0 },
};

static struct amba_driver snd_serial_driver = {
	.drv = {
		.name	= SND_SERIAL_DRIVER,
	},
	.id_table	= snd_serial_ids,
	.probe		= snd_serial_probe,
	.remove		= snd_serial_remove,
};

static int __init alsa_card_serial_init(void)
{
	snd_printk(KERN_INFO "snd-serial-pl011: PL011 based MIDI device\n");
	return amba_driver_register(&snd_serial_driver);
}

static void __exit alsa_card_serial_exit(void)
{
	amba_driver_unregister(&snd_serial_driver);
}

module_init(alsa_card_serial_init)
module_exit(alsa_card_serial_exit)
