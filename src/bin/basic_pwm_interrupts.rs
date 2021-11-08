// $ cargo rb pwm
#![no_main]
#![no_std]

use f411_rtic as _; // global logger + panicking-behavior + memory layout

#[rtic::app(device = stm32f4xx_hal::stm32, peripherals = true)]
mod app {
    use pwm::C1;
    use dwt_systick_monotonic::DwtSystick;
    use rtic_monotonic::{Milliseconds, Seconds};
    use stm32f4xx_hal::{
        gpio::{gpioc::PC13, Edge, ExtiPin, Input, PullUp},
        prelude::*,
        pwm::{self, PwmChannels},
        stm32::TIM2,
        timer::Timer,
        adc::{
            config::{AdcConfig, Dma, SampleTime, Scan, Sequence},
            Adc,
        },
        dma::{
            config::DmaConfig, MemoryToPeripheral, PeripheralToMemory, Stream0, Stream7,
            StreamsTuple, Transfer
        },
        pac::{ADC1, DMA2},
    };

    const BUF_SIZE: usize = 4;
    const FREQ: u32 = 10;

    #[monotonic(binds = SysTick, default = true)]
    type SysMono = DwtSystick<FREQ>;

    #[shared]
    struct Shared {
        voltage: u16,
        #[lock_free]
        adc_transfer:
            Transfer<Stream0<DMA2>, Adc<ADC1>, PeripheralToMemory, &'static mut [u16; 1], 0>,
    }

    #[local]
    struct Local {
        pwm: PwmChannels<TIM2, C1>
    }

    // Code starts running here
    #[init(local = [adc_buf: [u16; 1] = [0u16; 1]])] 
    fn init(mut ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        // Enable SYSCFG.
        let mut sys_cfg = ctx.device.SYSCFG.constrain();

        // Set up the system clock.
        ctx.device.RCC.ahb1enr.modify(|_, w| w.dma1en().enabled());
        let rcc = ctx.device.RCC.constrain();
        let clocks = rcc
            .cfgr
            .require_pll48clk()
            .sysclk(FREQ.hz())
            .hclk(FREQ.hz())
            .pclk1((FREQ / 2).hz())
            .pclk2(FREQ.hz())
            .freeze();
        let mono = DwtSystick::new(&mut ctx.core.DCB, ctx.core.DWT, ctx.core.SYST, FREQ);


        // Define pins. On the Nucleo-F411RE the LED is connected to pin PA5.
        let gpioa = ctx.device.GPIOA.split();
        let gpiob = ctx.device.GPIOB.split();
        
        let led = gpioa.pa5.into_alternate_af1(); // uncommented this still flashed the LED
        let adc_pin = gpiob.pb1.into_analog();
        let pwm_pin = gpioa.pa0.into_alternate_af1(); //pa6 didn't work..
        
        // Set up PWM (Pulse Width Modulation)
        let pwm = Timer::new(ctx.device.TIM2, &clocks).pwm(led, FREQ.hz());

        // Set up DMA (Direct Memory Acces) for ADC
        let dma = StreamsTuple::new(ctx.device.DMA2);
        let dma_config = DmaConfig::default()
            .transfer_complete_interrupt(true)
            .memory_increment(true);

        // Set up ADC (Analog to Digital Converter)
        let adc_config = AdcConfig::default()
            .dma(Dma::Continuous)
            .scan(Scan::Enabled);
        let mut adc = Adc::adc1(ctx.device.ADC1, true, adc_config);
        adc.configure_channel(&adc_pin, Sequence::One, SampleTime::Cycles_480);
        adc.enable_temperature_and_vref();
        let adc_transfer =
            Transfer::init_peripheral_to_memory(dma.0, adc, ctx.local.adc_buf, None, dma_config);

        defmt::info!("Starting...");
        (
            Shared {
                adc_transfer,
                voltage: 0,
            },
            Local {pwm},
            init::Monotonics(mono),
        )
    }

    #[idle]  // Runs when no tasks are queued
    fn idle(_: idle::Context) -> ! {
        loop {}
    }

    #[task(binds = TIM2, shared=[adc_transfer], priority = 2)]
    fn start_conversion(ctx: start_conversion::Context) {
        ctx.shared.adc_transfer.start(|adc| {
            adc.start_conversion();
        });
    }

    // Triggers on ADC DMA transfer complete
    #[task(binds = DMA2_STREAM0, shared = [adc_transfer, voltage], local = [pwm, d: u32 = 0], priority = 2)]
    fn on_adc_dma(mut ctx: on_adc_dma::Context) {
        let transfer = ctx.shared.adc_transfer;
        let raw_voltage = unsafe {
            transfer
                .next_transfer_with(|buf, _| {
                    let voltage = buf[0];
                    (buf, voltage)
                })
                .unwrap()
        };
        ctx.shared.voltage.lock(|v| *v = raw_voltage);
        
        let (d, pwm) = (ctx.local.d, ctx.local.pwm);
        let voltage = (raw_voltage as f32) / ((2_i32.pow(12) - 1) as f32) * 3.3;
        let max_duty = pwm.get_max_duty();
        let duty = (voltage / 3.3 * (max_duty as f32)) as u16;
        pwm.set_duty(duty);
    }
}