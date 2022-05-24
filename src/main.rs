#![no_std]
#![no_main]
#![feature(alloc_error_handler)]

extern crate alloc;

mod support;

use cortex_m_rt::entry;

use ina219::{INA219, INA219_ADDR};
// без этого будет ошибка отсутвия векторов прерываний
use stm32f0xx_hal as hal;

use hal::{pac, prelude::*};

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

    let i2c = if let Some(p) = pac::Peripherals::take() {
        cortex_m::interrupt::free(move |cs| {
            let mut flash = p.FLASH;
            let mut rcc = p.RCC.configure().freeze(&mut flash);

            let gpiob = p.GPIOB.split(&mut rcc);

            // Configure pins for I2C
            let scl = gpiob.pb10.into_alternate_af1(cs);
            let sda = gpiob.pb11.into_alternate_af1(cs);

            // Configure I2C with 100kHz rate
            //I2c(hal::i2c::I2c::i2c1(p.I2C1, (scl, sda), 100.khz(), &mut rcc))
            hal::i2c::I2c::i2c1(p.I2C1, (scl, sda), 100.khz(), &mut rcc)
        })
    } else {
        defmt::error!("Failed to take peripherials!");
        panic!();
    };

    let mut ina = INA219::new(i2c, INA219_ADDR);

    ina.calibrate(0x0100).unwrap();

    loop {
        let voltage = ina.voltage().unwrap();
        defmt::info!("bus voltage: {}", voltage);

        let shunt_voltage = ina.shunt_voltage().unwrap();
        defmt::info!("shunt voltage: {}", shunt_voltage);

        let current = ina.current().unwrap();
        defmt::info!("current: {}", current);

        let power = ina.power().unwrap();
        defmt::info!("power: {}", power);
    }
}

fn init_heap() {
    use core::mem::MaybeUninit;
    const HEAP_SIZE: usize = 2048;
    static mut HEAP: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
    unsafe { ALLOCATOR.init(HEAP.as_ptr() as usize, HEAP_SIZE) }
    defmt::info!("Initialized Heap with size {} Bytes", HEAP_SIZE);
}
