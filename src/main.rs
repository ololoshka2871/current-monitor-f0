#![no_std]
#![no_main]

mod config;
mod report;

use panic_halt as _;

use rtic::app;
use systick_monotonic::Systick;

use stm32f0xx_hal::{
    gpio::{
        gpiob::{PB10, PB11},
        Alternate, AF1,
    },
    i2c::I2c,
    prelude::*,
    stm32::Interrupt,
    stm32::{I2C1, TIM14},
    timers::Timer,
    usb::Peripheral,
};

use stm32_usbd::UsbBus;
use usb_device::class_prelude::UsbBusAllocator;
use usb_device::prelude::UsbDevice;

use usbd_hid::descriptor::SerializedDescriptor;
use usbd_hid::hid_class::HIDClass;

use ina219::{INA219, INA219_ADDR};

/// Calibration value
pub const CAL: u16 = (4096.0 / 100.0 / config::R_SHUNT) as u16;

#[app(device = stm32f0xx_hal::pac, peripherals = true)]
mod app {
    use super::*;

    #[shared]
    struct Shared {
        hid: HIDClass<'static, UsbBus<Peripheral>>,
    }

    #[local]
    struct Local {
        usb_dev: UsbDevice<'static, UsbBus<Peripheral>>,
        ina219: INA219<I2c<I2C1, PB10<Alternate<AF1>>, PB11<Alternate<AF1>>>>,
        _timer: Timer<TIM14>,
    }

    #[monotonic(binds = SysTick, default = true)]
    type MonoTimer = Systick<1000>;

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        static mut USB_BUS: Option<UsbBusAllocator<UsbBus<Peripheral>>> = None;

        let mut flash = ctx.device.FLASH;

        let mut rcc = ctx
            .device
            .RCC
            .configure()
            .hsi48()
            .enable_crs(ctx.device.CRS)
            .sysclk(48.mhz())
            .pclk(24.mhz())
            .freeze(&mut flash);

        let mono = Systick::new(ctx.core.SYST, rcc.clocks.sysclk().0);

        let gpioa = ctx.device.GPIOA.split(&mut rcc);
        let gpiob = ctx.device.GPIOB.split(&mut rcc);

        let usb = Peripheral {
            usb: ctx.device.USB,
            pin_dm: gpioa.pa11,
            pin_dp: gpioa.pa12,
        };

        unsafe {
            USB_BUS = Some(UsbBus::new(usb));
        }

        let mut timer = Timer::tim14(ctx.device.TIM14, 100.hz(), &mut rcc);
        timer.listen(stm32f0xx_hal::timers::Event::TimeOut);

        let i2c1 = ctx.device.I2C1;
        let mut i2c = cortex_m::interrupt::free(move |cs| {
            // Configure pins for I2C
            let scl = gpiob.pb10.into_alternate_af1(cs).internal_pull_up(cs, true);
            let sda = gpiob.pb11.into_alternate_af1(cs).internal_pull_up(cs, true);

            // Configure I2C with 400kHz rate
            I2c::i2c1(i2c1, (scl, sda), 400.khz(), &mut rcc)
        });

        if i2c
            .write(
                INA219_ADDR - 1,
                // INA219 config register setup
                &[
                    0u8,        // config register
                    0b00100001, // BRNG | PG = 0b00 | BADC = 0b0011 |
                    0b10011111, // SADC = 0b0011 | mode = 0b111
                ],
            )
            .is_err()
        {
            panic!("Ina219 config failed");
        }

        let mut ina = INA219::new(
            i2c,
            INA219_ADDR - 1, // A0, A1 == 0
        );

        if ina.calibrate(CAL).is_err() {
            panic!("Calibration failed");
        }

        let hid = HIDClass::new(
            unsafe { USB_BUS.as_ref().unwrap_unchecked() },
            report::Ina219Report::desc(),
            10,
        );

        let usb_device = usb_device::device::UsbDeviceBuilder::new(
            unsafe { USB_BUS.as_ref().unwrap_unchecked() },
            usb_device::device::UsbVidPid(0x0483, 0x5789),
        )
        .manufacturer("Shilo_XyZ_")
        .product("Fast Ampermeter")
        .serial_number("0000002")
        .build();

        (
            Shared { hid },
            Local {
                usb_dev: usb_device,
                ina219: ina,
                _timer: timer,
            },
            init::Monotonics(mono),
        )
    }

    #[idle]
    fn idle(_ctx: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(binds = USB, local = [usb_dev], shared = [hid], priority = 2)]
    fn usb(mut ctx: usb::Context) {
        let usb_dev = ctx.local.usb_dev;

        ctx.shared.hid.lock(|hid| {
            usb_dev.poll(&mut [hid]);
        });
    }

    #[task(binds = TIM14, local = [ina219], shared = [hid])]
    fn tim14(mut ctx: tim14::Context) {
        match report::Ina219Report::new(&mut ctx.local.ina219) {
            Ok(report) => {
                ctx.shared.hid.lock(|hid| {
                    if report.push_to_hid(hid).is_ok() {
                        rtic::pend(Interrupt::USB);
                    }
                });
            }
            Err(_) => {}
        }
    }
}
