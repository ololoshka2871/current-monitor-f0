use embedded_hal::blocking::i2c;
use ina219::INA219;
use serde::ser::{Serialize, SerializeTuple, Serializer};
use usb_device::UsbError;
use usbd_hid::{
    descriptor::{AsInputReport, SerializedDescriptor},
    hid_class::HIDClass,
};
use usbd_hid_macros::gen_hid_descriptor;

/// Ina219 input report (RAW values)
#[gen_hid_descriptor(
    (collection = APPLICATION, usage_page = VENDOR_DEFINED_START, usage = 0x01) = {
        voltage=input;
        shunt_voltage=input;
        current=input;
        power=input;
     }
)]
#[allow(dead_code)]
pub struct Ina219Report {
    pub voltage: u16,
    pub shunt_voltage: i16,
    pub current: i16,
    pub power: i16,
}

impl Ina219Report {
    pub fn new<I2C, E>(ina: &mut INA219<I2C>) -> Result<Self, E>
    where
        I2C: i2c::Write<Error = E> + i2c::Read<Error = E>,
    {
        Ok(Self {
            voltage: ina.voltage()?, // mv
            shunt_voltage: ina.shunt_voltage()?,
            current: ina.current()?,
            power: ina.power()?,
        })
    }

    pub fn push<B>(&self, hid: &mut HIDClass<B>) -> usb_device::Result<usize>
    where
        B: usb_device::bus::UsbBus,
    {
        match cortex_m::interrupt::free(move |_| hid.push_input(self)) {
            Ok(r) => Ok(r),
            Err(UsbError::WouldBlock) => Ok(0),
            Err(e) => Err(e),
        }
    }
}
