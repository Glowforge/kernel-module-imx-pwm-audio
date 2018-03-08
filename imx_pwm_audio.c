/**
 * i.MX PWM audio driver
 * Copyright (C) 2016-2018 Glowforge, Inc. <opensource@glowforge.com>
 * Written by Matt Sarnoff.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * Much of this code is adapted from the i.MX6 PWM driver.
 * (drivers/pwm/pwm-imx.c)
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <asm/uaccess.h>

#include <linux/platform_data/epit-imx.h>
#include <linux/platform_data/dma-imx-sdma.h>

/** Module parameters */
static u32 sample_rate        = 22000;
static u32 bits_per_sample    = 8;
static int signed_samples     = 0;  /* 1 if samples are signed, 0 if unsigned */
module_param(sample_rate,     uint, 0);
module_param(bits_per_sample, uint, 0);
module_param(signed_samples,  int, 0);
static u32 sample_offset = 0;

#define AUDIO_DEVICE_NAME "pwm_audio"
#define AUDIO_DEVICE_PATH "/dev/" AUDIO_DEVICE_NAME

/* I've seen cases where the kernel hands out the same minor number */
/* to multiple modules that use MISC_DYNAMIC_MINOR. */
/* The Linux Allocated Devices list assigns 128 to "/dev/beep", */
/* a.k.a. "fancy beep device," a.k.a. the IBM PC speaker. */
/* So it's somewhat appropriate. :) */
#define AUDIO_DEVICE_MINOR_NUMBER 128

static const u32 sdma_script[] = {
  0x0a000901, 0x69c80400, 0x69c86a2b, 0x620002df, 0x7d090568, 0x7d0b6209, 0x02a602bd, 0x6a2b0400,
  0x01607df2, 0x03000400, 0x01607dee, 0x620a7df4
};

/* PWM Control Register */
#define PWMCR   0x00
/* PWM Status Register */
#define PWMSR   0x04
/* PWM Interrupt Register */
#define PWMIR   0x08
/* PWM Sample Register */
#define PWMSAR  0x0c
/* PWM Period Register */
#define PWMPR   0x10
/* PWM Counter Register */
#define PWMCNR  0x14

#define PWMCR_FWM(x)          ((((x)-1) & 0x3) << 26)
#define PWMCR_DOZEEN          (1 << 24)
#define PWMCR_WAITEN          (1 << 23)
#define PWMCR_DBGEN           (1 << 22)
#define PWMCR_CLKSRC_IPG_HIGH (2 << 16)
#define PWMCR_CLKSRC_IPG      (1 << 16)
#define PWMCR_PRESCALER(x)    ((((x) - 1) & 0xFFF) << 4)
#define PWMCR_SWR             (1 << 3)
#define PWMCR_REPEAT_1        (0 << 1)
#define PWMCR_REPEAT_2        (1 << 1)
#define PWMCR_REPEAT_4        (2 << 1)
#define PWMCR_REPEAT_8        (3 << 1)
#define PWMCR_EN              (1 << 0)

#define PWMSR_FWE             (1 << 6)
#define PWMSR_CMP             (1 << 5)
#define PWMSR_ROV             (1 << 4)
#define PWMSR_FE              (1 << 3)
#define PWMSR_FIFOAV_MASK     0x7

#define PWMIR_CIE             (1 << 2)
#define PWMIR_RIE             (1 << 1)
#define PWMIR_FIE             (1 << 0)

#define RES_TO_PWMPR(bits)    ((1<<(bits))-3)

#define BYTES_PER_SAMPLE      DIV_ROUND_UP(bits_per_sample, 8)
#define BYTES_PER_SECOND      ((sample_rate)*(BYTES_PER_SAMPLE))
#define SAMPLE_BUF_SIZE       (10*BYTES_PER_SECOND)

/** Playing begins when this many bytes of data have been enqueued. */
#define START_THRESHOLD       (1*BYTES_PER_SECOND)


struct imx_pwm_audio_data {
  /** Pointer to parent device */
  struct device *dev;
  /** Lock to ensure the device can only be opened by one process at a time */
  struct mutex lock;
  /** Peripheral clock */
  struct clk *clk_per;
  /** IPG clock */
  struct clk *clk_ipg;
  /** Base address of PWM registers */
  void __iomem *mmio_base;
  /** Physical address of PWM register base */
  u32 mmio_base_phys;
  /** Userspace interface */
  struct miscdevice audiodev;
  /** Number of the SPKR_EN GPIO */
  int enable_gpio;
  /** Pointer to the timer used to drive the DMA engine */
  struct epit *epit;
  /** SDMA script load address (offset in bytes) */
  u32 sdma_script_origin;
  /** SDMA channel number */
  u32 sdma_ch_num;
  /** Pointer to the SDMA engine */
  struct sdma_engine *sdma;
  /** Pointer to the SDMA channel used by the driver */
  struct sdma_channel *sdmac;
  /** Pointer to the beginning of the sample buffer */
  void *sample_buf;
  /** Physical address of the beginning of the sample buffer */
  dma_addr_t sample_buf_phys;
  /** Number of bytes of sample data enqueued */
  u32 sample_buf_len;
};



#pragma mark - DMA

static int load_sdma_script(struct imx_pwm_audio_data *self)
{
  int ret;
  const u32 *script = sdma_script;
  size_t script_len = sizeof(sdma_script);
  struct sdma_context_data initial_context = {{0}};

  /* write the script code to SDMA RAM */
  dev_dbg(self->dev, "loading SDMA script (%d bytes)...", script_len);
  ret = sdma_write_datamem(self->sdma, (void *)script, script_len,
    self->sdma_script_origin);
  if (ret) {
    dev_err(self->dev, "failed to load script");
    return ret;
  }

  /* load the initial context */
  ret = sdma_load_partial_context(self->sdmac, &initial_context, 0,
    sizeof(initial_context));

  dev_dbg(self->dev, "script loaded");
  return ret;
}


/**
 * Sets script program counter and arguments
 */
static int load_script_context(struct imx_pwm_audio_data *self)
{
  int ret;
  struct sdma_context_data context = {{0}};

  /* set the script arguments and initial PC */
  /* see the specific asm file for argument requirements */
  context.channel_state.pc = self->sdma_script_origin * 2; /* in program space addressing */
  context.gReg[5] = (1<<bits_per_sample)-1;
  context.gReg[6] = sample_offset;
  context.msa = self->sample_buf_phys;
  context.mda = self->mmio_base_phys+PWMSAR;
  context.ms = 0x00100000; /* destination address frozen; source address postincrement; start in read mode */
  context.pda = epit_status_register_address(self->epit);
  context.ps = 0x000c0400; /* destination address frozen; 32-bit write size; start in write mode */

  /* acquire the channel and load its context; */
  /* it's triggered externally by the EPIT */
  sdma_setup_channel(self->sdmac, true);

  ret = sdma_load_partial_context(self->sdmac, &context, 0, sizeof(context));
  if (ret) {
    dev_err(self->dev, "failed to set up channel");
    return ret;
  }

  return 0;
}


static int pwm_audio_set_sample_buf_len(struct imx_pwm_audio_data *self,
  size_t new_len)
{
  int context_len = 0;
  struct sdma_context_data context = {{0}};
  dev_dbg(self->dev, "%s: %u", __func__, new_len);

  if (new_len > SAMPLE_BUF_SIZE) {
    return -EINVAL;
  }

  self->sample_buf_len = new_len;
  context.gReg[7] = self->sample_buf_phys+self->sample_buf_len;
  context.mda = self->mmio_base_phys+PWMSAR;
  context.msa = self->sample_buf_phys;
  /* If there's data, only update the pointer to the end of the buffer (r7) */
  if (new_len > 0) {
    context_len = 4;
  }
  /* If clearing data, update r7 and MSA (MDA too since it's in the middle) */
  else {
    context_len = 12;
  }

  /**
   * This operation is run by channel 0, which is mutually exclusive with our
   * scripts's channel. Since only one channel is running at a time, and
   * channels cannot preempt each other, this ensures that the context is only
   * updated between iterations of our script. (i.e. registers won't get
   * suddenly updated while our script is running)
   */
  return sdma_load_partial_context(self->sdmac,
    (struct sdma_context_data *)&context.gReg[7],
    offsetof(struct sdma_context_data, gReg[7]),
    context_len);
}



#pragma mark - Audio functions

static int pwm_audio_play(struct imx_pwm_audio_data *self)
{
  /* Return an error if no data to play */
  if (self->sample_buf_len == 0) {
    return -ENODATA;
  }
  /* Do nothing if already playing */
  if (epit_is_running(self->epit)) {
    return 0;
  }
  dev_dbg(self->dev, "%s", __func__);
  /* Turn speaker on */
  gpio_set_value(self->enable_gpio, 1);
  /* Enable timer events */
  sdma_event_enable(self->sdmac, epit_sdma_event(self->epit));
  epit_start_hz(self->epit, sample_rate); /* start generating periodic events */
  /* Set a nonzero priority to start the script */
  sdma_set_channel_priority(self->sdmac, 5);
  return 0;
}


static int pwm_audio_stop(struct imx_pwm_audio_data *self)
{
  dev_dbg(self->dev, "%s", __func__);
  /* Stop the timer and disable the sdma channel */
  epit_stop(self->epit);
  sdma_event_disable(self->sdmac, epit_sdma_event(self->epit));
  /* Turn speaker off */
  gpio_set_value(self->enable_gpio, 0);
  return 0;
}


static int pwm_audio_stop_and_clear_data(struct imx_pwm_audio_data *self)
{
  pwm_audio_stop(self);
  return pwm_audio_set_sample_buf_len(self, 0);
}


static ssize_t pwm_audio_enqueue_data(struct imx_pwm_audio_data *self,
  const char __user *data, size_t count)
{
  size_t bytes_free = 0, nbytes = 0;
  int ret;
  dev_dbg(self->dev, "%s: %u", __func__, count);
  if (self->sample_buf_len >= SAMPLE_BUF_SIZE) {
    return -EPERM;
  }
  bytes_free = SAMPLE_BUF_SIZE - self->sample_buf_len;
  nbytes = min(count, bytes_free);
  if (copy_from_user(self->sample_buf+self->sample_buf_len, data, nbytes)) {
    dev_err(self->dev, "unable to copy data from userspace");
    return -EBADF;
  }
  ret = pwm_audio_set_sample_buf_len(self, self->sample_buf_len+nbytes);
  return (ret < 0) ? ret : nbytes;
}


/**
 * Called when the SDMA engine executes a "done 3" instruction, setting the
 * interrupt flag for our channel.
 * This callback executes in tasklet context.
 */
static void pwm_audio_sdma_interrupt(void *param)
{
  struct imx_pwm_audio_data *self = (struct imx_pwm_audio_data *)param;
  pwm_audio_stop(self);
}



#pragma mark - PWM setup

static int pwm_enable(struct imx_pwm_audio_data *self)
{
  u32 period = RES_TO_PWMPR(bits_per_sample);
  u32 cr;

  /* Enable clocks */
  int ret = 0;

  if ((ret = clk_prepare_enable(self->clk_per))) {
    return ret;
  }
  if ((ret = clk_prepare_enable(self->clk_ipg))) {
    return ret;
  }

  /* TODO: PWMCR_REPEAT needs to be based on sample rate */
  /* May need to be PWMCR_REPEAT_4, 2, or 1 for higher sample rates/sizes */
  cr = PWMCR_REPEAT_8 | PWMCR_DOZEEN | PWMCR_WAITEN | PWMCR_DBGEN | PWMCR_CLKSRC_IPG_HIGH |
    PWMCR_EN;

  /* Software reset */
  __raw_writel(PWMCR_SWR, self->mmio_base+PWMCR);
  udelay(10);
  /* Activate & de-activate PWM (seems to be necessary after a reset) */
  __raw_writel(PWMCR_EN, self->mmio_base+PWMCR);
  __raw_writel(0, self->mmio_base+PWMCR);

  /* Silence */
  __raw_writel(0, self->mmio_base+PWMSAR);

  /* Set period, add some headroom to prevent distortion at high amplitudes. */
  /* Increasing the period increases the maximum PWM counter value. Since the */
  /* signal values aren't changed, this has the effect of reducing the */
  /* amplitude of the output signal. */
  /* The factor of 25% was experimentally determined to be the best balance */
  /* between clarity and amplitude, while preserving full dynamic range. */
  period += period >> 2;
  dev_dbg(self->dev, "setting PWMPR: %08x (%lu Hz)", period,
    clk_get_rate(self->clk_ipg)/(period+2));

  __raw_writel(period, self->mmio_base+PWMPR);

  /* Configure and enable */
  dev_dbg(self->dev, "setting PWMCR: %08x", cr);
  __raw_writel(cr, self->mmio_base+PWMCR);
  return 0;
}


static int pwm_disable(struct imx_pwm_audio_data *self)
{
  u32 cr = 0;
  /* Disable PWM */
  __raw_writel(cr, self->mmio_base+PWMCR);
  /* Release clocks */
  clk_disable_unprepare(self->clk_per);
  clk_disable_unprepare(self->clk_ipg);
  return 0;
}



#pragma mark - Userspace API

/* miscdevice sets filp's private_data pointer to itself */
#define DEV_SELF_FROM_FILP(filp) \
  struct device *dev = ((struct miscdevice *)filp->private_data)->parent; \
  struct imx_pwm_audio_data *self = dev_get_drvdata(dev)


static int imx_pwm_audio_dev_open(struct inode *inode, struct file *filp)
{
  /* On open, silence and clear all data */
  /* Only one process may have /dev/pwm_audio open at a time. */
  /* No mixing is supported. */
  DEV_SELF_FROM_FILP(filp);
  if (!mutex_trylock(&self->lock)) {
    dev_warn(dev, AUDIO_DEVICE_PATH " is in use");
    return -EBUSY;
  } else {
    dev_dbg(dev, AUDIO_DEVICE_PATH " opened");
  }
  return pwm_audio_stop_and_clear_data(self);
}


static int imx_pwm_audio_dev_close(struct inode *inode, struct file *filp)
{
  DEV_SELF_FROM_FILP(filp);
  dev_dbg(dev, AUDIO_DEVICE_PATH " closed");
  /* Ensure we're playing any enqueued data after closing, in case the */
  /* process has written less than START_THRESHOLD's worth of data. */
  /* (e.g. `cat`ing a very short audio file) */
  if (self->sample_buf_len > 0) {
    pwm_audio_play(self);
  }
  mutex_unlock(&self->lock);
  return 0;
}


static ssize_t imx_pwm_audio_dev_read(struct file *filp, char __user *data,
  size_t count, loff_t *offp)
{
  /* Reads from /dev/pwm_audio are not supported */
  return 0;
}


static ssize_t imx_pwm_audio_dev_write(struct file *filp,
  const char __user *data, size_t count, loff_t *offp)
{
  DEV_SELF_FROM_FILP(filp);
  ssize_t ret = pwm_audio_enqueue_data(self, data, count);
  /* If we've got enough data enqueued, start playing */
  if (self->sample_buf_len >= START_THRESHOLD) {
    pwm_audio_play(self);
  }
  return ret;
}


static int imx_pwm_audio_dev_fsync(struct file *filp, loff_t start, loff_t end, int datasync)
{
  DEV_SELF_FROM_FILP(filp);
  /* Start playing if we have data enqueued */
  return pwm_audio_play(self);
}


static loff_t imx_pwm_audio_dev_llseek(struct file *filp, loff_t off, int whence)
{
  DEV_SELF_FROM_FILP(filp);
  /* An lseek() to 0 can be used to stop audio and clear the buffer. */
  if (off != 0 || whence != SEEK_SET) {
    return -EINVAL;
  }
  return pwm_audio_stop_and_clear_data(self);
}


const struct file_operations audio_dev_fops = {
  .owner    = THIS_MODULE,
  .open     = imx_pwm_audio_dev_open,
  .read     = imx_pwm_audio_dev_read,
  .write    = imx_pwm_audio_dev_write,
  .fsync    = imx_pwm_audio_dev_fsync,
  .llseek   = imx_pwm_audio_dev_llseek,
  .release  = imx_pwm_audio_dev_close,
};



#pragma mark - platform_device probe/remove

static const struct of_device_id imx_pwm_audio_dt_ids[] = {
  { .compatible = "glowforge,imx-pwm-audio" },
  { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx_pwm_audio_dt_ids);

static int imx_pwm_audio_probe(struct platform_device *pdev)
{
  const struct of_device_id *of_id = of_match_device(imx_pwm_audio_dt_ids,
    &pdev->dev);
  struct imx_pwm_audio_data *self;
  struct resource *r;
  int ret = 0;
  struct device_node *epit_np = NULL;
  u32 sdma_params[2];

  if (!of_id) {
    return -ENODEV;
  }

  self = devm_kzalloc(&pdev->dev, sizeof(*self), GFP_KERNEL);
  if (self == NULL) {
    dev_err(&pdev->dev, "failed to allocate memory");
    return -ENOMEM;
  }

  self->clk_per = devm_clk_get(&pdev->dev, "per");
  if (IS_ERR(self->clk_per)) {
    dev_err(&pdev->dev, "getting per clock failed with %ld",
      PTR_ERR(self->clk_per));
    return PTR_ERR(self->clk_per);
  }
  dev_dbg(&pdev->dev, "clk_per rate: %lu", clk_get_rate(self->clk_per));

  self->clk_ipg = devm_clk_get(&pdev->dev, "ipg");
  if (IS_ERR(self->clk_ipg)) {
    dev_err(&pdev->dev, "getting ipg clock failed with %ld",
        PTR_ERR(self->clk_ipg));
    return PTR_ERR(self->clk_ipg);
  }
  dev_dbg(&pdev->dev, "clk_ipg rate: %lu", clk_get_rate(self->clk_ipg));

  r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
  self->mmio_base = devm_ioremap_resource(&pdev->dev, r);
  if (IS_ERR(self->mmio_base)) {
    return PTR_ERR(self->mmio_base);
  }
  self->mmio_base_phys = r->start;

  self->enable_gpio = of_get_named_gpio(pdev->dev.of_node, "enable-gpio", 0);
  if (self->enable_gpio < 0) {
    dev_err(&pdev->dev, "no enable-gpio specified");
    return -EINVAL;
  }
  ret = devm_gpio_request_one(&pdev->dev, self->enable_gpio, 0, "spkr_en");
  if (ret) {
    dev_err(&pdev->dev, "cannot reserve spkr_en gpio");
    return ret;
  }

  self->dev = &pdev->dev;
  platform_set_drvdata(pdev, self);

  /* Allocate contiguous memory for sample buffer */
  self->sample_buf = dma_zalloc_coherent(&pdev->dev, SAMPLE_BUF_SIZE,
    &self->sample_buf_phys, GFP_KERNEL);
  if (!self->sample_buf) {
    dev_err(&pdev->dev, "failed to allocate sample buffer");
    return -ENOMEM;
  }
  dev_dbg(&pdev->dev, "allocated %d byte sample buffer at 0x%08x",
    SAMPLE_BUF_SIZE, self->sample_buf_phys);

  /* Set up timer */
  epit_np = of_parse_phandle(pdev->dev.of_node, "timer", 0);
  if (IS_ERR(epit_np)) {
    dev_err(&pdev->dev, "no timer specified");
    ret = -ENODEV;
    goto failed_epit_init;
  }
  self->epit = epit_get(epit_np);
  of_node_put(epit_np);
  if (!self->epit) {
    dev_err(&pdev->dev, "failed to get timer");
    ret = -ENODEV;
    goto failed_epit_init;
  }
  ret = epit_init_freerunning(self->epit, NULL, NULL);
  if (ret) {
    dev_err(&pdev->dev, "failed to initialize timer");
    goto failed_epit_init;
  }

  /* Read SDMA channel number and load address */
  if (of_property_read_u32_array(pdev->dev.of_node, "sdma-params",
    sdma_params, ARRAY_SIZE(sdma_params)) == 0) {
    self->sdma_ch_num = sdma_params[0];
    self->sdma_script_origin = sdma_params[1];
  } else {
    dev_err(&pdev->dev, "sdma-params property not specified");
    goto failed_sdma_init;
  }

  /* Set up SDMA and get a channel reference */
  self->sdma = sdma_engine_get();
  if (IS_ERR(self->sdma)) {
    dev_err(&pdev->dev, "failed to get sdma engine");
    ret = -ENODEV;
    goto failed_sdma_init;
  }
  self->sdmac = sdma_get_channel(self->sdma, self->sdma_ch_num);
  if (IS_ERR(self->sdmac)) {
    dev_err(&pdev->dev, "failed to get sdma channel");
    ret = -ENODEV;
    goto failed_sdma_init;
  }

  /* Load the SDMA script */
  ret = load_sdma_script(self);
  if (ret) {
    goto failed_load_sdma_script;
  }
  ret = load_script_context(self);
  if (ret) {
    goto failed_load_sdma_script;
  }
  sdma_set_channel_interrupt_callback(self->sdmac, pwm_audio_sdma_interrupt,
    self);

  mutex_init(&self->lock);

  self->audiodev.minor = AUDIO_DEVICE_MINOR_NUMBER;
  self->audiodev.name = AUDIO_DEVICE_NAME;
  self->audiodev.fops = &audio_dev_fops;
  self->audiodev.parent = &pdev->dev;
  ret = misc_register(&self->audiodev);
  if (ret) {
    dev_err(&pdev->dev, "unable to register " AUDIO_DEVICE_PATH);
    goto failed_dev_register;
  }

  return pwm_enable(self);

failed_dev_register:
  mutex_destroy(&self->lock);
failed_load_sdma_script:
failed_sdma_init:
  epit_stop(self->epit);
failed_epit_init:
  dma_free_coherent(&pdev->dev, SAMPLE_BUF_SIZE, self->sample_buf,
    self->sample_buf_phys);
  return ret;
}


static int imx_pwm_audio_remove(struct platform_device *pdev)
{
  struct imx_pwm_audio_data *self = platform_get_drvdata(pdev);
  misc_deregister(&self->audiodev);
  sdma_set_channel_interrupt_callback(self->sdmac, NULL, NULL);
  pwm_audio_stop(self);
  pwm_disable(self);
  dma_free_coherent(&pdev->dev, SAMPLE_BUF_SIZE, self->sample_buf,
    self->sample_buf_phys);
  mutex_destroy(&self->lock);
  return 0;
}


static struct platform_driver imx_pwm_audio_driver = {
  .driver = {
    .name = "imx-pwm-audio",
    .owner = THIS_MODULE,
    .of_match_table = imx_pwm_audio_dt_ids,
  },
  .probe = imx_pwm_audio_probe,
  .remove = imx_pwm_audio_remove,
};



#pragma mark - Module init/exit

static int __init imx_pwm_audio_init(void)
{
  int ret = 0;

  pr_info("%s: %u bits/sample%s, %u Hz\n", THIS_MODULE->name,
    bits_per_sample, (signed_samples) ? " (signed)" : "", sample_rate);

  /* Reject nonsense parameters */
  if (bits_per_sample < 4 || bits_per_sample > 16) {
    pr_err("value %u is invalid for bits_per_sample param\n", bits_per_sample);
    return -EINVAL;
  }
  if (sample_rate < 1000 || sample_rate > 48000) {
    pr_err("value %u is invalid for sample_rate param\n", sample_rate);
    return -EINVAL;
  }
  /* Compute sample offset */
  if (!signed_samples) {
    u32 bits_per_sample_rounded_up = BYTES_PER_SAMPLE*8;
    sample_offset = (1 << (bits_per_sample_rounded_up-1)) -
                    (1 << (bits_per_sample-1));
  } else {
    sample_offset = -(1 << (bits_per_sample-1));
  }
  /* TODO: alert if sample rate is higher than PWM frequency for given bps */

  ret = platform_driver_register(&imx_pwm_audio_driver);
  if (ret < 0) {
    pr_err("failed to initialize audio driver\n");
  }

  return ret;
}
module_init(imx_pwm_audio_init);


static void __exit imx_pwm_audio_exit(void)
{
  platform_driver_unregister(&imx_pwm_audio_driver);
  pr_info("%s: unloaded\n", THIS_MODULE->name);
}
module_exit(imx_pwm_audio_exit);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Glowforge, Inc. <opensource@glowforge.com>");
MODULE_DESCRIPTION("Freescale i.MX6 PWM audio interface");
