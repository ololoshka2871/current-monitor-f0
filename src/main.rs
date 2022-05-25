#![no_std]
#![no_main]
#![feature(alloc_error_handler)]

extern crate alloc;

mod support;

use cortex_m_rt::entry;

// без этого будет ошибка отсутвия векторов прерываний
use stm32f0xx_hal as hal;

use hal::stm32f0::stm32f0x2::Interrupt;
use hal::{i2c::I2c, pac, pac::interrupt, prelude::*};

use ina219::{INA219, INA219_ADDR};

//-------------------------------------------------------

// replace with FreeRTOS-rust allocator if use freertos
#[global_allocator]
static ALLOCATOR: alloc_cortex_m::CortexMHeap = alloc_cortex_m::CortexMHeap::empty();

//-------------------------------------------------------

#[entry]
fn main() -> ! {
    defmt::info!("++ Startup ++");

    // Initialize the allocator BEFORE you use it
    init_heap();

    let (i2c, usb_bus, nvic) =
        if let (Some(p), Some(dp)) = (cortex_m::Peripherals::take(), pac::Peripherals::take()) {
            cortex_m::interrupt::free(move |cs| {
                let mut flash = dp.FLASH;

                let mut rcc = dp
                    .RCC
                    .configure()
                    .hsi48()
                    .enable_crs(dp.CRS)
                    .sysclk(48.mhz())
                    .pclk(24.mhz())
                    .freeze(&mut flash);

                let gpiob = dp.GPIOB.split(&mut rcc);

                // Configure pins for I2C
                let scl = gpiob.pb10.into_alternate_af1(cs).internal_pull_up(cs, true);
                let sda = gpiob.pb11.into_alternate_af1(cs).internal_pull_up(cs, true);

                // Configure I2C with 100kHz rate
                let i2c = I2c::i2c1(dp.I2C1, (scl, sda), 100.khz(), &mut rcc);

                let gpioa = dp.GPIOA.split(&mut rcc);

                let usb = hal::usb::Peripheral {
                    usb: dp.USB,
                    pin_dm: gpioa.pa11,
                    pin_dp: gpioa.pa12,
                };

                let usb_bus = hal::usb::UsbBus::new(usb);

                (i2c, usb_bus, p.NVIC)
            })
        } else {
            defmt::error!("Failed to take peripherials!");
            panic!();
        };

    let mut ina = INA219::new(
        i2c,
        INA219_ADDR - 1, // A0, A1 == 0
    );

    if let Err(e) = ina.calibrate(0x0000) {
        defmt::error!("Calibration failed: {}", defmt::Debug2Format(&e));
        panic!();
    }

    let mut serial = usbd_serial::SerialPort::new(&usb_bus);

    let mut usb_dev = usb_device::device::UsbDeviceBuilder::new(
        &usb_bus,
        usb_device::device::UsbVidPid(0x0483, 0x5789),
    )
    .manufacturer("Shilo_XyZ_")
    .product("Fast Ampermeter")
    .serial_number("0000001")
    .device_class(usbd_serial::USB_CLASS_CDC)
    .build();

    unsafe {
        let c: *mut cortex_m::peripheral::NVIC = &nvic as *const _ as *mut _;
        (*c).set_priority(Interrupt::USB, 1)
    }

    loop {
        /*
        let voltage = ina.voltage().unwrap();
        defmt::info!("bus voltage: {}", voltage);

        let shunt_voltage = ina.shunt_voltage().unwrap();
        defmt::info!("shunt voltage: {}", shunt_voltage);

        let current = ina.current().unwrap();
        defmt::info!("current: {}", current);

        let power = ina.power().unwrap();
        defmt::info!("power: {}", power);
        */

        // Важно! Список передаваемый сюда в том же порядке,
        // что были инициализированы интерфейсы
        let res = usb_dev.poll(&mut [&mut serial]);

        if res {
            serial_echo(&mut serial);
        } else {
            // block until usb interrupt
            // enable usb interrupt
            unsafe { cortex_m::peripheral::NVIC::unmask(Interrupt::USB) };

            // sleep (wfi() почему-то не работает.)
            cortex_m::asm::wfe();

            // disable usb interrupt
            cortex_m::peripheral::NVIC::mask(Interrupt::USB);
        }
    }
}

fn serial_echo<B: usb_device::bus::UsbBus>(serial: &mut usbd_serial::SerialPort<B>) {
    let mut buf = [0u8; 64];

    match serial.read(&mut buf) {
        Ok(count) if count > 0 => {
            // Echo back in upper case
            for c in buf[0..count].iter_mut() {
                if 0x61 <= *c && *c <= 0x7a {
                    *c &= !0x20;
                }
            }

            let mut write_offset = 0;
            while write_offset < count {
                match serial.write(&buf[write_offset..count]) {
                    Ok(len) if len > 0 => {
                        write_offset += len;
                    }
                    _ => {}
                }
            }
        }
        _ => {}
    }
}

#[interrupt]
unsafe fn USB() {
    // путое прерывание, просто чтобы проц проснулся

    cortex_m::peripheral::NVIC::mask(Interrupt::USB);
    cortex_m::peripheral::NVIC::unpend(Interrupt::USB);
}

fn init_heap() {
    use core::mem::MaybeUninit;
    const HEAP_SIZE: usize = 2048;
    static mut HEAP: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
    unsafe { ALLOCATOR.init(HEAP.as_ptr() as usize, HEAP_SIZE) }
    defmt::info!("Initialized Heap with size {} Bytes", HEAP_SIZE);
}
