#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

//Global allocator
extern crate alloc;
//use core::time::Duration;
use alloc::boxed::Box;
use alloc::rc::Rc;
use esp_backtrace as _;
//Embedded hal and graphics
use embedded_graphics::{pixelcolor::raw::RawU16, prelude::*, primitives::Rectangle};
use embedded_hal::PwmPin;
use hal::embassy;
use hal::gpio::{Analog, GpioPin, Input, PullUp};
use hal::peripherals::SENS;
// Slint ui
use slint::platform::software_renderer::MinimalSoftwareWindow;
use slint::platform::Platform;
// Display and HAL
use display_interface_spi::SPIInterfaceNoCS;
use esp_println::println;
use hal::{
    adc::{AdcConfig, Attenuation, ADC},
    clock::ClockControl,
    peripherals::Peripherals,
    prelude::*,
    spi::SpiMode,
    timer::TimerGroup,
    Delay, Rtc, IO,
};
//Embassy
use embassy_executor::Executor;
use embassy_futures::join::join;
use embassy_time::{Duration, Ticker, Timer};
use static_cell::StaticCell;

slint::include_modules!();
static EXECUTOR: StaticCell<Executor> = StaticCell::new();

struct Channel;
impl PwmPin for Channel {
    type Duty = ();
    fn disable(&mut self) {}
    fn enable(&mut self) {}
    fn get_duty(&self) -> Self::Duty {}
    fn get_max_duty(&self) -> Self::Duty {}
    fn set_duty(&mut self, _duty: Self::Duty) {}
}

struct MyPlatform<'d> {
    window: alloc::rc::Rc<MinimalSoftwareWindow>,
    // optional: some timer device from your device's HAL crate
    rtc: Rtc<'d>,
    // ... maybe more devices
}
impl<'d> Platform for MyPlatform<'d> {
    fn create_window_adapter(
        &self,
    ) -> Result<Rc<dyn slint::platform::WindowAdapter>, slint::PlatformError> {
        // Since on MCUs, there can be only one window, just return a clone of self.window.
        // We'll also use the same window in the event loop.
        Ok(self.window.clone())
    }
    // optional: You can put the event loop there, or in the main function, see later
    fn run_event_loop(&self) -> Result<(), slint::PlatformError> {
        todo!();
    }
    fn duration_since_start(&self) -> core::time::Duration {
        core::time::Duration::from_millis(self.rtc.get_time_ms())
    }
}

#[global_allocator]
static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();

struct DisplayWrapper<'a, T> {
    display: &'a mut T,
    line_buffer: &'a mut [slint::platform::software_renderer::Rgb565Pixel],
}

impl<T: DrawTarget<Color = embedded_graphics::pixelcolor::Rgb565>>
    slint::platform::software_renderer::LineBufferProvider for DisplayWrapper<'_, T>
{
    type TargetPixel = slint::platform::software_renderer::Rgb565Pixel;
    fn process_line(
        &mut self,
        line: usize,
        range: core::ops::Range<usize>,
        render_fn: impl FnOnce(&mut [Self::TargetPixel]),
    ) {
        // Render into the line
        render_fn(&mut self.line_buffer[range.clone()]);

        // Send the line to the screen using DrawTarget::fill_contiguous
        self.display
            .fill_contiguous(
                &Rectangle::new(
                    Point::new(range.start as _, line as _),
                    Size::new(range.len() as _, 1),
                ),
                self.line_buffer[range.clone()]
                    .iter()
                    .map(|p| RawU16::new(p.0).into()),
            )
            .map_err(drop)
            .unwrap();
    }
}

type RgbDisplay = impl DrawTarget<Color = embedded_graphics::pixelcolor::Rgb565> + 'static;

#[embassy_executor::task]
async fn ui_scene_loop(
    mut next_btn: GpioPin<Input<PullUp>, 5>,
    mut back_btn: GpioPin<Input<PullUp>, 15>,
) {
    loop {
        join(next_btn.wait_for_low(), back_btn.wait_for_low())
            .await
            .0
            .ok();
        if next_btn.is_low().unwrap() && back_btn.is_low().unwrap() {
            println!("clicked");
        }
        Timer::after(Duration::from_millis(300)).await;
    }
}

#[embassy_executor::task]
async fn adc(sens: SENS, adc_pin: GpioPin<Analog, 35>) {
    let analog = sens.split();
    let mut adc_config = AdcConfig::new();
    let mut pin = adc_config.enable_pin(adc_pin, Attenuation::Attenuation11dB);
    let mut adc = ADC::adc(analog.adc1, adc_config).unwrap();
    loop {
        let adc_value: u32 = nb::block!(adc.read(&mut pin)).unwrap();
        let battery_percentage = (adc_value - 150) * 100 / (3350 - 150);
        println!("Battery percentage: {}", battery_percentage);

        Timer::after(Duration::from_secs(60)).await;
    }
}

#[embassy_executor::task]
async fn ui_update_loop(window: alloc::rc::Rc<MinimalSoftwareWindow>, mut display: RgbDisplay) {
    loop {
        // Let Slint run the timer hooks and update animations.
        slint::platform::update_timers_and_animations();

        //Draw the scene if something needs to be drawn.
        window.draw_if_needed(|renderer| {
            let mut line_buffer = [slint::platform::software_renderer::Rgb565Pixel(0); 240];
            renderer.render_by_line(DisplayWrapper {
                display: &mut display,
                line_buffer: &mut line_buffer,
            });
        });

        Timer::after(Duration::from_millis(500)).await;
    }
}

#[embassy_executor::task]
async fn clock_ticker(ui: MainWindow) {
    let mut ticker = Ticker::every(Duration::from_secs(1));
    loop {
        ui.set_clock(ui.get_clock() + 1);
        //println!("Secs:{}", clock);
        ticker.next().await;
    }
}

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let mut system = peripherals.DPORT.split();
    let mut clocks = ClockControl::boot_defaults(system.clock_control).freeze();
    let mut delay = Delay::new(&clocks);
    // Disable the RTC and TIMG watchdog timers
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);
    let mut timer_group0 = TimerGroup::new(
        peripherals.TIMG0,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    let mut wdt0 = timer_group0.wdt;
    let timer_group1 = TimerGroup::new(
        peripherals.TIMG1,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    let mut wdt1 = timer_group1.wdt;
    rtc.rwdt.disable();
    wdt0.disable();
    wdt1.disable();
    timer_group0.timer0.start(1u64.micros());
    init_heap();

    //Initialize GPIO Pins
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let sclk = io.pins.gpio18;
    let mosi = io.pins.gpio23;
    let dc_pin = io.pins.gpio16.into_push_pull_output();
    let rst_pin = io.pins.gpio4.into_push_pull_output();
    let next_btn = io.pins.gpio5.into_pull_up_input();
    let back_btn = io.pins.gpio15.into_pull_up_input();
    let adc_pin = io.pins.gpio35.into_analog();
    //let accept_btn = io.pins.gpio22.into_pull_up_input();

    //Initialize SPI
    let spi = hal::spi::Spi::new_no_cs_no_miso(
        peripherals.SPI2,
        sclk,
        mosi,
        8000u32.kHz(),
        SpiMode::Mode0,
        &mut system.peripheral_clock_control,
        &mut clocks,
    );

    let spi_interface = SPIInterfaceNoCS::new(spi, dc_pin);
    // Create display driver
    let mut display = gc9a01a::GC9A01A::new(spi_interface, rst_pin, Channel);
    // Bring out of reset
    display.reset(&mut delay).unwrap();
    // Initialize registers
    display.initialize(&mut delay).unwrap();

    let window = MinimalSoftwareWindow::new(Default::default());
    slint::platform::set_platform(Box::new(MyPlatform {
        window: window.clone(),
        rtc: rtc,
    }))
    .unwrap();

    // Setup the UI.
    let ui = MainWindow::new().unwrap();
    window.set_size(slint::PhysicalSize::new(240, 240));

    #[cfg(feature = "embassy-time-timg0")]
    embassy::init(&clocks, timer_group0.timer0);

    // Async requires the GPIO interrupt to wake futures
    hal::interrupt::enable(
        hal::peripherals::Interrupt::GPIO,
        hal::interrupt::Priority::Priority1,
    )
    .unwrap();

    let executor = EXECUTOR.init(Executor::new());
    executor.run(|spawner| {
        spawner.spawn(ui_scene_loop(next_btn, back_btn)).ok();
        spawner.spawn(ui_update_loop(window, display)).ok();
        spawner.spawn(clock_ticker(ui)).ok();
        spawner.spawn(adc(peripherals.SENS, adc_pin)).ok();
    });
}

fn init_heap() {
    const HEAP_SIZE: usize = 32 * 1024;

    extern "C" {
        static mut _heap_start: u32;
        static mut _heap_end: u32;
    }

    unsafe {
        let heap_start = &_heap_start as *const _ as usize;
        let heap_end = &_heap_end as *const _ as usize;
        assert!(
            heap_end - heap_start > HEAP_SIZE,
            "Not enough available heap memory."
        );
        ALLOCATOR.init(heap_start as *mut u8, HEAP_SIZE);
    }
}
