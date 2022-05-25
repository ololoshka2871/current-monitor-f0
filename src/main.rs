#![no_std]
#![no_main]
#![feature(alloc_error_handler)]

extern crate alloc;

mod report;
mod support;

use cortex_m_rt::entry;

use hal::time::Hertz;
// без этого будет ошибка отсутвия векторов прерываний
use stm32f0xx_hal as hal;

use hal::timers::Timer;
use hal::{i2c::I2c, pac, pac::interrupt, prelude::*, usb::Peripheral};

use ina219::{INA219, INA219_ADDR};

use stm32_usbd::UsbBus;
use usb_device::{class_prelude::UsbBusAllocator, device::UsbDevice};
use usbd_hid::{descriptor::SerializedDescriptor, hid_class::HIDClass};

//-------------------------------------------------------

// replace with FreeRTOS-rust allocator if use freertos
#[global_allocator]
static ALLOCATOR: alloc_cortex_m::CortexMHeap = alloc_cortex_m::CortexMHeap::empty();

//-------------------------------------------------------

static mut USB_BUS: Option<UsbBusAllocator<UsbBus<Peripheral>>> = None;
static mut USB_DEV: Option<UsbDevice<UsbBus<Peripheral>>> = None;
static mut HID: Option<HIDClass<UsbBus<Peripheral>>> = None;

#[entry]
fn main() -> ! {
    defmt::info!("++ Startup ++");

    // Initialize the allocator BEFORE you use it
    init_heap();

    let (i2c, nvic, mut timer) =
        if let (Some(cp), Some(dp)) = (cortex_m::Peripherals::take(), pac::Peripherals::take()) {
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

                unsafe {
                    USB_BUS = Some(hal::usb::UsbBus::new(usb));
                }

                let timer = Timer::tim14(dp.TIM14, Hertz(10), &mut rcc);

                (i2c, cp.NVIC, timer)
            })
        } else {
            defmt::error!("Failed to take peripherials!");
            panic!();
        };

    let mut ina = INA219::new(
        i2c,
        INA219_ADDR - 1, // A0, A1 == 0
    );

    defmt::info!("Calibration...");
    if let Err(e) = ina.calibrate(0x0000) {
        defmt::error!("Calibration failed: {}", defmt::Debug2Format(&e));
        panic!();
    }

    defmt::info!("Register hid interface");
    unsafe {
        HID = Some(HIDClass::new(
            USB_BUS.as_ref().unwrap_unchecked(),
            report::Ina219Report::desc(),
            60,
        ));
    }

    defmt::info!("Register USB device");
    unsafe {
        USB_DEV = Some(
            usb_device::device::UsbDeviceBuilder::new(
                USB_BUS.as_ref().unwrap_unchecked(),
                usb_device::device::UsbVidPid(0x0483, 0x5789),
            )
            .manufacturer("Shilo_XyZ_")
            .product("Fast Ampermeter")
            .serial_number("0000001")
            .build(),
        );
    }

    unsafe {
        const SCB_SCR_SEVONPEND: u32 = 1 << 4;

        let c: *mut cortex_m::peripheral::NVIC = &nvic as *const _ as *mut _;
        (*c).set_priority(interrupt::USB, 1);

        // exit wfe() on eny HW event
        (*cortex_m::peripheral::SCB::ptr())
            .scr
            .modify(|scr| scr | SCB_SCR_SEVONPEND);
        //(*c).set_priority(interrupt::TIM14, 2);
    }

    loop {
        // block until usb interrupt
        // enable usb interrupt
        unsafe { cortex_m::peripheral::NVIC::unmask(interrupt::USB) };

        // sleep
        #[cfg(not(debug_assertions))]
        cortex_m::asm::wfe();

        // disable usb interrupt
        cortex_m::peripheral::NVIC::mask(interrupt::USB);

        if timer.wait().is_ok() {
            if let Some(_hid) = unsafe { HID.as_mut() } {
                match report::Ina219Report::new(&mut ina) {
                    Ok(report) => {
                        if let Err(e) = report.push(_hid) {
                            defmt::error!("USB error: {}", defmt::Debug2Format(&e))
                        }
                    }
                    Err(e) => defmt::error!("I2C error: {}", defmt::Debug2Format(&e)),
                }
            }
        }
    }
}

#[interrupt]
unsafe fn USB() {
    if let (Some(usb_dev), Some(hid)) = (USB_DEV.as_mut(), HID.as_mut()) {
        // Важно! Список передаваемый сюда в том же порядке,
        // что были инициализированы интерфейсы
        usb_dev.poll(&mut [hid]);
    }

    cortex_m::peripheral::NVIC::mask(interrupt::USB);
    cortex_m::peripheral::NVIC::unpend(interrupt::USB);
}

#[interrupt]
unsafe fn TIM14() {
    // just disable interrupt, dont clear flag in timer
    cortex_m::peripheral::NVIC::mask(interrupt::TIM14);
    cortex_m::peripheral::NVIC::unpend(interrupt::TIM14);
}

fn init_heap() {
    use core::mem::MaybeUninit;
    const HEAP_SIZE: usize = 2048;
    static mut HEAP: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
    unsafe { ALLOCATOR.init(HEAP.as_ptr() as usize, HEAP_SIZE) }
    defmt::info!("Initialized Heap with size {} Bytes", HEAP_SIZE);
}
