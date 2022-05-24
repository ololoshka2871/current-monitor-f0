#![no_std]
#![no_main]
#![feature(alloc_error_handler)]

extern crate alloc;

mod support;

use cortex_m_rt::entry;

// без этого будет ошибка отсутвия векторов прерываний
use stm32f0xx_hal as hal;

//-------------------------------------------------------

// replace with FreeRTOS-rust allocator if use freertos
#[global_allocator]
static ALLOCATOR: alloc_cortex_m::CortexMHeap = alloc_cortex_m::CortexMHeap::empty();

//-------------------------------------------------------

#[entry]
fn main() -> ! {
    defmt::info!("++ Startup ++");

    // Initialize the allocator BEFORE you use it
    {
        use core::mem::MaybeUninit;
        const HEAP_SIZE: usize = 2048;
        static mut HEAP: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe { ALLOCATOR.init(HEAP.as_ptr() as usize, HEAP_SIZE) }
        defmt::info!("Initialized Heap with size {} Bytes", HEAP_SIZE);
    }

    loop {}
}
