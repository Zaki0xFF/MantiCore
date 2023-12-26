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
use embedded_hal_async::digital::Wait;
use gc9a01a::GC9A01A;
use hal::embassy;
use hal::gpio::{Analog, GpioPin, Input, PullUp};
use hal::peripherals::SENS;
// Slint ui
use slint::platform::software_renderer::MinimalSoftwareWindow;
use slint::platform::{Platform, Key};
// Display and HAL
use display_interface_spi::SPIInterfaceNoCS;
use embassy_executor::Spawner;
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
use embassy_futures::select::select;
use embassy_time::{Duration, Ticker, Timer};

slint::include_modules!();

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

type RgbDisplay = GC9A01A<
    SPIInterfaceNoCS<
        hal::spi::master::Spi<'static, hal::peripherals::SPI2, hal::spi::FullDuplexMode>,
        GpioPin<hal::gpio::Output<hal::gpio::PushPull>, 16>
    >,
    GpioPin<hal::gpio::Output<hal::gpio::PushPull>, 4>, Channel>;

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
async fn ui_scene_loop(
    mut next_btn: GpioPin<Input<PullUp>, 5>,
    mut back_btn: GpioPin<Input<PullUp>, 15>,
    window: alloc::rc::Rc<MinimalSoftwareWindow>,
    ui: Rc<MainWindow>
) {
    let mut is_open = false;
    loop {
        select(next_btn.wait_for_low(), back_btn.wait_for_low()).await;
        let next_low = next_btn.is_low().unwrap();
        let back_low = back_btn.is_low().unwrap();

        if next_low&&back_low{
            is_open = !is_open;
            ui.invoke_set_menu(is_open);
            println!("clicked both");
        }else if next_low{
            println!("clicked nexts");
            window.dispatch_event(slint::platform::WindowEvent::KeyPressed { text: Key::RightArrow.into() });
        }else if back_low{
            println!("clicked back");
            window.dispatch_event(slint::platform::WindowEvent::KeyPressed { text: Key::LeftArrow.into() });
        }

        Timer::after(Duration::from_millis(300)).await;
    }
}

#[embassy_executor::task]
async fn adc(sens: SENS, adc_pin: GpioPin<Analog, 35>, ui: Rc<MainWindow>) {
    let analog = sens.split();
    let mut adc_config = AdcConfig::new();
    let mut pin = adc_config.enable_pin(adc_pin, Attenuation::Attenuation11dB);
    let mut adc = ADC::adc(analog.adc1, adc_config).unwrap();
    loop {
        let adc_value: u32 = nb::block!(adc.read(&mut pin)).unwrap() as u32;
        let mut battery_percentage: u32 = (adc_value - 150) * 100 / (3350 - 150);
        if battery_percentage > 100 {
            println!("Battery not connected");
            battery_percentage = 0;
        }
        ui.set_battery(battery_percentage.try_into().unwrap());
        Timer::after(Duration::from_secs(60)).await;
    }
}

#[embassy_executor::task]
async fn clock_ticker(ui: Rc<MainWindow>) {
    let mut ticker = Ticker::every(Duration::from_secs(1));
    loop {
        ui.set_clock(ui.get_clock() + 1);
        //println!("Secs:{}", clock);
        ticker.next().await;
    }
}

#[main]
async fn main(spawner: Spawner) {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let mut clocks = ClockControl::boot_defaults(system.clock_control).freeze();
    let mut delay = Delay::new(&clocks);
    // Disable the RTC and TIMG watchdog timers
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);
    let mut timer_group0 = TimerGroup::new(
        peripherals.TIMG0,
        &clocks
    );
    let mut wdt0 = timer_group0.wdt;
    let timer_group1 = TimerGroup::new(
        peripherals.TIMG1,
        &clocks
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
    let spi = hal::spi::master::Spi::new(
        peripherals.SPI2,
        8000u32.kHz(),
        SpiMode::Mode0,
        &mut clocks,
    ).with_sck(sclk).with_mosi(mosi);

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
    let ui = Rc::new(MainWindow::new().unwrap());
    window.set_size(slint::PhysicalSize::new(240, 240));
    ui.invoke_set_menu(false);

    #[cfg(feature = "embassy-time-timg0")]
    embassy::init(&clocks, timer_group0.timer0);

    // Async requires the GPIO interrupt to wake futures
    hal::interrupt::enable(
        hal::peripherals::Interrupt::GPIO,
        hal::interrupt::Priority::Priority1,
    )
    .unwrap();

    _ = spawner.spawn(ui_scene_loop(next_btn, back_btn, window.clone(), ui.clone()));
    _ = spawner.spawn(ui_update_loop(window, display));
    _ = spawner.spawn(adc(peripherals.SENS, adc_pin, ui.clone()));
    _ = spawner.spawn(clock_ticker(ui));

    loop {
        // Tick...
        Timer::after(Duration::from_millis(1_000)).await;
    }
}

fn init_heap() {
    const HEAP_SIZE: usize = 32 * 2048;
    static mut HEAP: core::mem::MaybeUninit<[u8; HEAP_SIZE]> = core::mem::MaybeUninit::uninit();

    unsafe {
        ALLOCATOR.init(HEAP.as_mut_ptr().cast(), HEAP_SIZE);
    }
}
