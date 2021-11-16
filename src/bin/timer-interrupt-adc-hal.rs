// $ cargo rb adc-serial-dma-tx
// Transmit serial data using DMA
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
    const FREQ: u32 = 84_000_000;

    #[monotonic(binds = SysTick, default = true)]
    type SysMono = DwtSystick<FREQ>;

    #[shared]
    struct Shared {

    }

    #[local]
    struct Local {
        adc: Adc<ADC1>,
        pwm: PwmChannels<TIM2, C1>,
    }

    #[init]
    fn init(mut ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        ctx.device.RCC.apb1enr.modify(|_, w| w.tim2en().clear_bit()); // Sets the bit false
        
        ctx.device.RCC.apb1enr.modify(|_, w| w.tim2en().set_bit()); // Sets the bit true
        
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
 
        let tim2 = &ctx.device.TIM2;

        /* Pin Setup */
        let gpioa = ctx.device.GPIOA.split();
        let gpiob = ctx.device.GPIOB.split();
        let adc_pin = gpiob.pb1.into_analog();
        let led = gpioa.pa5.into_alternate();

        /* ADC Setup */
        let adc_config = AdcConfig::default()
            .end_of_conversion_interrupt(Eoc::Conversion)
            .external_trigger(TriggerMode::BothEdges, ExternalTrigger::Tim_2_trgo);
        let mut adc = Adc::adc1(ctx.device.ADC1, true, adc_config);
        adc.configure_channel(&adc_pin, Sequence::One, SampleTime::Cycles_3);
        //adc.start_conversion();


        /* Timer Interrupt Setup */
        // Changes to registers get overwritten when creating a Timer object
        ctx.device.TIM2.cr1.modify(|_, w| unsafe { w.cms().bits(3) });
        ctx.device.TIM2.cr1.modify(|_, w| unsafe { w.udis().bit(false) });
        ctx.device.TIM2.cr2.modify(|_, w| unsafe { w.mms().bits(2) });

        let mut timer = Timer::new(ctx.device.TIM2, &clocks); // This enables and resets the timer...
        //timer.tim;  //Becomes a private field that cannot be accessed...

        // Instead need to access it somehow like so... 
        // Ref using older HAL version: https://github.com/stm32-rs/stm32f4xx-hal/issues/236
        unsafe {
            let tim2_regb = &(*(stm32f4xx_hal::stm32::TIM2::ptr()));
            
            // Enable Update Interrupt
            tim2_regb.cr2.modify(|_, w| unsafe { w.mms().bits(2) }); // (2) Update event enabled

            // Enable complementary outputs
            //tim2_regb.ccer.modify(|_, w| w.cc1ne().set_bit());
            //tim2_regb.ccer.modify(|_, w| w.cc2ne().set_bit());
            //tim2_regb.ccer.modify(|_, w| w.cc3ne().set_bit());
    
            // Set dead time
            //tim2_regb.bdtr.modify(|_, w| w.dtg().bits(10));
    
            // Center aligned
            //tim2_regb.cr1.modify(|_, w| w.cms().center_aligned1());
        }

        let mut pwm = timer.pwm(led, 1u32.khz());

        // Enable timer 2?
        //ctx.device.RCC.apb1enr.modify(|_, w| w.tim2en().set_bit());

        pwm.enable();
        pwm.set_duty(pwm.get_max_duty() / 2);
        adc.enable();
        //timer.start_count_down(FREQ);
        
        // Enable TIM2 Interrupt, Not Needed with RTIC
        /*
        NVIC::unpend(Interrupt::TIM2);
        NVIC::unpend(Interrupt::ADC);
        unsafe {
            NVIC::unmask(Interrupt::TIM2);
            NVIC::unmask(Interrupt::ADC);
        }
        */

        (
            Shared {
                //tx,
                //adc_transfer,
                //voltage: 0,
            },
            Local {
                adc,
                pwm,
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
    // Test timer interrupt
    #[task(binds = TIM2, priority = 2)]
    fn test(mut ctx: test::Context) {
        defmt::info!("Test");
    }
    */

    // Triggers on ADC DMA transfer complete
    #[task(binds = ADC, local = [pwm, adc], priority = 2)]
    //#[task(binds = ADC, local = [adc], priority = 2)]
    fn on_adc_eoc(mut ctx: on_adc_eoc::Context) {
        let (adc, pwm) = (ctx.local.adc, ctx.local.pwm);
        //adc.clear_end_of_conversion_flag(); // Not needed? Current sample clears?
        //let adc = ctx.local.adc;

        let raw_sample = adc.current_sample(); // Clears ADC interrupt
        //TODO: Why is the ADC SOC only being triggered once?
        // Check is passing the timer without pwm can have the interrupt cleared?

        let voltage = (raw_sample as f32) / ((2_i32.pow(12) - 1) as f32) * 3.3;
        //defmt::info!("Voltage: {}", voltage);
        let max_duty = pwm.get_max_duty();
        let duty = (voltage / 3.3 * (max_duty as f32)) as u16;
        pwm.set_duty(duty);

        //adc.start_conversion();
    }
}
