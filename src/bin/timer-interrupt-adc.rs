// $ cargo rb timer-interrupt-adc
// 
#![no_main]
#![no_std]

use f411_rtic as _; // global logger + panicking-behavior + memory layout

#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true, dispatchers=[SPI1, SPI2])]
mod app {
    use cortex_m::peripheral::NVIC;
    use dwt_systick_monotonic::DwtSystick;
    use rtic_monotonic::{Milliseconds, Seconds};
    use embedded_hal::adc::Channel;
    use pwm::C1;
    use stm32f4xx_hal::{
        adc::{
            config::{AdcConfig, Dma, SampleTime, Scan, Sequence, Eoc, TriggerMode, ExternalTrigger},
            Adc,
        },
        dma::{
            config::DmaConfig, MemoryToPeripheral, PeripheralToMemory, Stream0, Stream7,
            StreamsTuple, Transfer,
        },
        pac::{ADC1, DMA2,USART1},
        prelude::*,
        serial::{config::*, Serial, Tx},
        gpio::{gpioc::PC13, Edge, ExtiPin, Input, PullUp},
        pwm::{self, PwmChannels},
        stm32::{interrupt, Interrupt, TIM2},
        timer::{Event, Timer, CountDownTimer},
    };
    const FREQ: u32 = 84_000; //84_000_000; // Larger is faster 84 Mhz results in ADC reading every 55s?

    #[monotonic(binds = SysTick, default = true)]
    type SysMono = DwtSystick<FREQ>;

    #[shared]
    struct Shared {

    }

    #[local]
    struct Local {
        adc: Adc<ADC1>,
        timer: TIM2,
        //pwm: PwmChannels<TIM2, C1>,
    }

    #[init]
    fn init(mut ctx: init::Context) -> (Shared, Local, init::Monotonics) {
       
        ctx.device.RCC.apb1enr.modify(|_, w| w.tim2en().set_bit()); // Sets the bit true

        let rcc = ctx.device.RCC.constrain();
        let clocks = rcc
            .cfgr
            .require_pll48clk()
            .sysclk(FREQ.khz())
            .hclk(FREQ.khz())
            .pclk1((FREQ / 2).khz())
            .pclk2(FREQ.khz())
            .freeze();
        let mono = DwtSystick::new(&mut ctx.core.DCB, ctx.core.DWT, ctx.core.SYST, FREQ);

        let timer = ctx.device.TIM2;

        // Set Timer Registers
        timer.cr1.modify(|_, w| unsafe { w.cen().clear_bit() });  // Counter disabled
        timer.cr1.modify(|_, w| unsafe { w.cms().bits(3) });  // Up-Down Counter w/ interrupts on up and down

        timer.cr1.modify(|_, w| unsafe { w.udis().bit(false) }); // Update event enabled
        timer.psc.modify(|_, w| unsafe { w.psc().bits(8399)});  // Sets pre-scale (8399 for 1 kHz)
        
        timer.egr.write(|w| w.ug().set_bit()); // Update generation, reinitialize te counter and generates an update of registers
        
        timer.arr.modify(|_, w| unsafe { w.arr().bits(9999)});  // Sets ARR register counter limit (9999 for 1 Hz)
        timer.dier.modify(|_, w| unsafe { w.uie().set_bit()});  // Enables update interrupt  // Seems to break the ADC timer interrupt
        //ctx.device.TIM2.dier.modify(|_, w| unsafe { w.cc2ie().set_bit()});  // Enables Capture/Compare channel 2 interrupt
        //ctx.device.TIM2.ccr2.modify(|_, w| unsafe { w.ccr().bits(5000)});  // Sets Capture/Compare channel 2 value
        //ctx.device.TIM2.ccmr1_output().modify(|_, w| unsafe { w.cc2s().bits(0)});
        
        timer.cr1.modify(|_, w| unsafe { w.arpe().bit(true) }); // Pre- Enabled
        timer.cr2.modify(|_, w| unsafe { w.mms().bits(2) }); // (2) Update event enabled
     
        // PWM registers?
        //ctx.device.TIM2.ccmr1_output().modify(|_, w| unsafe { w.oc1m().bits(6) }); // Set PWM mode 1

        // Enable TIM2 Interrupt, Not Needed with RTIC
        NVIC::unpend(Interrupt::TIM2);
        NVIC::unpend(Interrupt::TIM3);
        NVIC::unpend(Interrupt::ADC);
        unsafe {
            NVIC::unmask(Interrupt::TIM2);
            NVIC::unmask(Interrupt::TIM3);
            NVIC::unmask(Interrupt::ADC);
        }

        // Enable Timer Counter
        timer.cr1.modify(|_, w| unsafe { w.cen().enabled() });  // Counter enabled
        
        /*
        let mut pwm_timer = Timer::new(ctx.device.TIM2, &clocks);
        let gpioa = ctx.device.GPIOA.split();
        let led = gpioa.pa5.into_alternate();
        let mut pwm = pwm_timer.pwm(led, 1u32.hz());
        pwm.set_duty(pwm.get_max_duty() / 2);
        pwm.enable();
        */

        ctx.device.ADC1.sr.modify(|_, w| unsafe { w.ovr().bit(false)});

        /* ADC Setup */
        let gpiob = ctx.device.GPIOB.split();
        let adc_pin = gpiob.pb1.into_analog();
        let adc_config = AdcConfig::default()
            .end_of_conversion_interrupt(Eoc::Conversion)
            .external_trigger(TriggerMode::BothEdges, ExternalTrigger::Tim_2_trgo);
        let mut adc = Adc::adc1(ctx.device.ADC1, true, adc_config);
        adc.configure_channel(&adc_pin, Sequence::One, SampleTime::Cycles_3);
        adc.enable();


        adc.start_conversion();

        /* ADC Timer Interrupt */
        //let mut tim = Timer::new(ctx.device.TIM2, &clocks);


        
        defmt::info!("Starting...");

        (
            Shared {
                //tx,
                //adc_transfer,
                //voltage: 0,
            },
            Local {
                adc,
                timer,
                //pwm,
            },
            init::Monotonics(mono),
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {

        }
    }

    /*
    #[task(binds = TIM2, priority = 2)]
    fn test(mut ctx: test::Context) {
        defmt::info!("Test");
    }
    */

    ///*
    // Triggers on ADC DMA transfer complete
    //#[task(binds = ADC, local = [pwm, adc], priority = 2)]
    #[task(binds = ADC, local = [adc, timer], priority = 2)]
    fn on_adc_eoc(mut ctx: on_adc_eoc::Context) {
        //let (adc, pwm) = (ctx.local.adc, ctx.local.pwm);
        let adc = ctx.local.adc;
        let timer = ctx.local.timer;

        let raw_sample = adc.current_sample();
    
        let voltage = (raw_sample as f32) / ((2_i32.pow(12) - 1) as f32) * 3.3;
        defmt::info!("Voltage: {}", voltage);

        timer.sr.modify(|_, w| w.uif().clear_bit());
        timer.egr.write(|w| w.ug().set_bit()); // Setting this to 1 in the RTIC framework resamples
        //let max_duty = pwm.get_max_duty();
        //let duty = (voltage / 3.3 * (max_duty as f32)) as u16;
        //pwm.set_duty(duty);

        //adc.start_conversion();
    }
    //*/
}
