#![allow(non_camel_case_types)]
#![allow(non_snake_case)]
#![allow(improper_ctypes_definitions)]

use cortex_m_rt::{exception, ExceptionFrame};

#[exception]
unsafe fn DefaultHandler(irqn: i16) {
    // custom default handler
    // irqn is negative for Cortex-M exceptions
    // irqn is positive for device specific (line IRQ)
    defmt::error!("Unregistred irq: {}", irqn);
    panic!();
}

#[exception]
unsafe fn HardFault(_ef: &ExceptionFrame) -> ! {
    defmt::error!("HardFault");
    panic!()
}

// libcore panic -> this function
// need if lto = false
#[allow(unused_variables)]
#[no_mangle]
pub extern "C" fn rust_begin_unwind(
    _fmt: ::core::fmt::Arguments,
    file: &'static str,
    line: u32,
) -> ! {
    cortex_m::asm::udf();
}

#[alloc_error_handler]
fn oom(layout: core::alloc::Layout) -> ! {
    defmt::error!(
        "oom(): Requested {} bytes with allignment {}",
        layout.size(),
        layout.align()
    );
    panic!();
}
