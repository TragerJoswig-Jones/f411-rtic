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
        timer::{Event, Timer},
    };
    const BUF_SIZE: usize = 4;
    const FREQ: u32 = 84_000_000;

    #[monotonic(binds = SysTick, default = true)]
    type SysMono = DwtSystick<FREQ>;

    #[shared]
    struct Shared {
        //voltage: u16,
        //#[lock_free]
        //adc_transfer:
        //    Transfer<Stream0<DMA2>, Adc<ADC1>, PeripheralToMemory, &'static mut [u16; 1], 0>,
        //#[lock_free]
        //tx: Transfer<Stream7<DMA2>, Tx<USART1>, MemoryToPeripheral, &'static mut [u8; BUF_SIZE], 4>,
    }

    #[local]
    struct Local {
        adc: Adc<ADC1>,
        pwm: PwmChannels<TIM2, C1>,
    }

    #[init(local = [tx_buf: [u8; BUF_SIZE] = [0; BUF_SIZE], adc_buf: [u16; 1] = [0u16; 1]])]
    fn init(mut ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        ctx.device.RCC.apb1enr.modify(|_, w| w.tim2en().set_bit());
        //ctx.device.RCC.ahb1enr.modify(|_, w| w.dma1en().enabled());
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
        tim2.cr2.write(|w| w.mms().update());
        tim2.cr1.modify(|_, w| w.cen().enabled());  // enable TIM2
        let mut timer = Timer::new(ctx.device.TIM2, &clocks);
        //let mut timer = Timer::tim2(ctx.device.TIM2, 5.hz(), clocks, &mut rcc.apb1);
        //timer.listen(Event::Update);

        //tim2.listen(Event::Update);
        //tim2.cr2 = MMS_A::UPDATE;
        

        // Enable TIM2 Interrupt
        //NVIC::unpend(Interrupt::TIM2);
        //NVIC::unpend(Interrupt::ADC);
        unsafe {
            NVIC::unmask(Interrupt::TIM2);
            NVIC::unmask(Interrupt::ADC);
        }

        let gpioa = ctx.device.GPIOA.split();
        let gpiob = ctx.device.GPIOB.split();
        let adc_pin = gpiob.pb1.into_analog();
        //let tx_pin = gpioa.pa9.into_alternate();
        let led = gpioa.pa5.into_alternate();
        
        let mut pwm = timer.pwm(led, 1u32.khz());
        pwm.enable();

        /* 
        let dma = StreamsTuple::new(ctx.device.DMA2);
        let dma_config = DmaConfig::default()
            .transfer_complete_interrupt(true)
            .memory_increment(true);
        */
        
        let adc_config = AdcConfig::default()
            .dma(Dma::Disabled)
            .end_of_conversion_interrupt(Eoc::Conversion)
            .external_trigger(TriggerMode::BothEdges, ExternalTrigger::Tim_2_trgo)
            .scan(Scan::Enabled);
        let mut adc = Adc::adc1(ctx.device.ADC1, true, adc_config);
        adc.configure_channel(&adc_pin, Sequence::One, SampleTime::Cycles_112);
        adc.enable_temperature_and_vref();
        /*
        let adc_transfer =
            Transfer::init_peripheral_to_memory(dma.0, adc, ctx.local.adc_buf, None, dma_config);

        let serial_config = Config {
            baudrate: 115_200.bps(),
            dma: stm32f4xx_hal::serial::config::DmaConfig::Tx,
            ..Config::default()
        };
        let serial = Serial::tx(ctx.device.USART1, tx_pin, serial_config, clocks).unwrap();
        let tx =
            Transfer::init_memory_to_peripheral(dma.7, serial, ctx.local.tx_buf, None, dma_config);
        */
        //start_conversion::spawn().ok();
        adc.start_conversion();
        //emit_status::spawn().ok();
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

    // TEST: Set register to trigger ADC from TIM2 trigger out
    //#[task(binds = TIM2,shared=[adc_transfer], priority = 2)]
    //fn start_conversion(ctx: start_conversion::Context) {
    //    ctx.shared.adc_transfer.start(|adc| {
    //        adc.start_conversion();
    //    });
    //}

    // Triggers on ADC DMA transfer complete
    #[task(binds = ADC, local = [pwm, adc], priority = 2)]
    fn on_adc_eoc(mut ctx: on_adc_eoc::Context) {
        let (adc, pwm) = (ctx.local.adc, ctx.local.pwm);

        let raw_sample = adc.current_sample();
    
        let voltage = (raw_sample as f32) / ((2_i32.pow(12) - 1) as f32) * 3.3;
        let max_duty = pwm.get_max_duty();
        let duty = (voltage / 3.3 * (max_duty as f32)) as u16;
        pwm.set_duty(duty);

        //adc.start_conversion();

        //start_conversion::spawn_after(Milliseconds(100_u32)).ok();
    }

    /* 
    #[task(shared = [tx, voltage])]
    fn emit_status(mut ctx: emit_status::Context) {
        let tx = ctx.shared.tx;
        let raw_voltage = ctx.shared.voltage.lock(|t| *t);
        let voltage = (raw_voltage as f32) / ((2_i32.pow(12) - 1) as f32) * 3.3;
        defmt::info!("Voltage: {}V", voltage);
        unsafe {
            tx.next_transfer_with(|buf, _| {
                buf.copy_from_slice(&voltage.to_be_bytes());
                (buf, ())
            })
            .ok();
        }
    }

    // Triggers on serial DMA transfer complete
    #[task(binds=DMA2_STREAM7, shared = [tx])]
    fn on_tx_dma(ctx: on_tx_dma::Context) {
        ctx.shared.tx.clear_transfer_complete_interrupt();
        emit_status::spawn_after(Seconds(1_u32)).ok();
    }
    */
}
