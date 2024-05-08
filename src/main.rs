#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

extern crate alloc;
use core::result;

use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use embassy_executor::Spawner;
use esp_println::println;
use hal::{
    clock::ClockControl, delay::Delay, gpio::IO, peripherals::Peripherals, prelude::*, rtc_cntl::Rtc, timer::TimerGroup
};

#[global_allocator]
static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();
#[main]
async fn main(_spawner: Spawner) {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let mut clocks = ClockControl::boot_defaults(system.clock_control).freeze();
    let _delay = Delay::new(&clocks);
    // Disable the RTC and TIMG watchdog timers
    let mut rtc = Rtc::new(peripherals.LPWR, None);
    let mut timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks, None);
    let mut wdt0 = timer_group0.wdt;
    let timer_group1 = TimerGroup::new(peripherals.TIMG1, &clocks, None);
    let mut wdt1 = timer_group1.wdt;
    rtc.rwdt.disable();
    wdt0.disable();
    wdt1.disable();
    timer_group0.timer0.start(1u64.micros());

    //Initialize GPIO Pins
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let sda = io.pins.gpio8;
    let scl = io.pins.gpio9;

    let mut i2c = hal::i2c::I2C::new(
        peripherals.I2C0, 
        sda, 
        scl, 
        100u32.kHz(), 
        &mut clocks,
        None
    );

    let mut version = [0u8; 1];
    println!("version = {:02x}", version[0]);
    let addr = 0x68;
    let resulta =i2c.write_read(addr, &[0x75], &mut version);
    println!("{:02X} = {:?}", addr, resulta);
    loop {
        Timer::after(Duration::from_millis(5)).await;
    }
}