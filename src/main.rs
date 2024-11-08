#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]

mod mocks;

//Global allocator
extern crate alloc;
//Touch
use cst816s::{self, CST816S};
use alloc::boxed::Box;
use alloc::rc::Rc;
// Display and HAL
use embassy_executor::Spawner;
//Embassy
use embassy_futures::select::select;
use embassy_time::{Duration, Ticker, Timer};
//Embedded hal and graphics
use embedded_graphics::{pixelcolor::raw::RawU16, prelude::*, primitives::Rectangle};
use embedded_hal_bus::spi::{ExclusiveDevice, NoDelay};
use esp_backtrace as _;
use esp_println::println;
use esp_hal::gpio::{AnyInput, AnyOutput, GpioPin, Input, Io, Level, Output, Pull};
use esp_hal::peripherals::ADC1;
use esp_hal::{
    analog::adc::{Adc, AdcConfig, Attenuation},
    clock::ClockControl,
    peripherals::Peripherals,
    prelude::*,
    rtc_cntl::Rtc,
    spi::SpiMode,
};
use esp_hal::delay::Delay;
use esp_hal::spi::FullDuplexMode;
use esp_hal::system::{SystemClockControl};
use esp_hal::timer::timg::TimerGroup;
use gc9a01a_driver::GC9A01A;
// Slint ui
use slint::platform::software_renderer::MinimalSoftwareWindow;
use slint::platform::{Key, Platform};
use crate::mocks::NoOutputPin;

slint::include_modules!();

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

impl<T: DrawTarget<Color=embedded_graphics::pixelcolor::Rgb565>>
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
    ExclusiveDevice<esp_hal::spi::master::Spi<'static, esp_hal::peripherals::SPI2, FullDuplexMode>, NoOutputPin, NoDelay>,
    AnyOutput<'static>,
    NoOutputPin,
    AnyOutput<'static>
>;

#[embassy_executor::task]
async fn ui_update_loop(window: Rc<MinimalSoftwareWindow>, mut display: RgbDisplay) {
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
    mut next_btn: AnyInput<'static>,
    mut back_btn: AnyInput<'static>,
    window: Rc<MinimalSoftwareWindow>,
    ui: Rc<MainWindow>,
) {
    let mut is_open = false;
    loop {
        select(next_btn.wait_for_low(), back_btn.wait_for_low()).await;
        let next_low = next_btn.is_low();
        let back_low = back_btn.is_low();

        if next_low && back_low {
            is_open = !is_open;
            ui.invoke_set_menu(is_open);
            println!("clicked both");
        } else if next_low {
            println!("clicked nexts");
            window.dispatch_event(slint::platform::WindowEvent::KeyPressed { text: Key::RightArrow.into() });
        } else if back_low {
            println!("clicked back");
            window.dispatch_event(slint::platform::WindowEvent::KeyPressed { text: Key::LeftArrow.into() });
        }

        Timer::after(Duration::from_millis(300)).await;
    }
}

#[embassy_executor::task]
async fn adc(adc1: ADC1, adc_pin: GpioPin<35>, ui: Rc<MainWindow>) {
    let mut adc_config = AdcConfig::new();
    let mut pin = adc_config.enable_pin(adc_pin, Attenuation::Attenuation11dB);
    let mut adc = Adc::new(adc1, adc_config);
    loop {
        let adc_value: u32 = nb::block!(adc.read_oneshot(&mut pin)).unwrap() as u32;
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
    let system = SystemClockControl::new();
    let mut clocks = ClockControl::boot_defaults(system).freeze();
    let mut delay = Delay::new(&clocks);
    // Disable the RTC and TIMG watchdog timers
    let mut rtc = Rtc::new(peripherals.LPWR);
    let timer_group0 = TimerGroup::new(
        peripherals.TIMG0,
        &clocks,
    );
    let mut wdt0 = timer_group0.wdt;
    let timer_group1 = TimerGroup::new(
        peripherals.TIMG1,
        &clocks,
    );
    let mut wdt1 = timer_group1.wdt;
    rtc.rwdt.disable();
    wdt0.disable();
    wdt1.disable();
    timer_group0.timer0.start();
    init_heap();

    //Initialize GPIO Pins
    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    let sclk = io.pins.gpio18;
    let mosi = io.pins.gpio23;
    let dc_pin = io.pins.gpio16;
    let rst_pin = io.pins.gpio4;
    let next_btn = io.pins.gpio5;
    let back_btn = io.pins.gpio15;
    let adc_pin = io.pins.gpio35;

    //Touch GPIO
    let touch_int = Input::new(io.pins.gpio26, Pull::Up);
    let touch_rst = Output::new(io.pins.gpio13, Level::Low);
    let sda = io.pins.gpio12;
    let scl = io.pins.gpio25;

    //Initialize SPI
    let spi = esp_hal::spi::master::Spi::new(
        peripherals.SPI2,
        8000u32.kHz(),
        SpiMode::Mode0,
        &mut clocks,
    ).with_sck(sclk).with_mosi(mosi);
    let spi = embedded_hal_bus::spi::ExclusiveDevice::new_no_delay(spi, NoOutputPin).unwrap();

    let i2c = esp_hal::i2c::I2C::new(
        peripherals.I2C0,
        sda,
        scl,
        400u32.kHz(),
        &mut clocks,
    );

    // Create touch driver
    let mut touchpad = CST816S::new(i2c, touch_int, touch_rst);
    touchpad.setup(&mut delay).unwrap();

    // Create display driver
    let mut display = GC9A01A::new(
        spi,
        AnyOutput::new(dc_pin, Level::Low),
        NoOutputPin,
        AnyOutput::new(rst_pin, Level::Low),
        true,
        240,
        240,
    );
    // Initialize registers
    display.init(&mut delay).unwrap();

    let window = MinimalSoftwareWindow::new(Default::default());
    slint::platform::set_platform(Box::new(MyPlatform { window: window.clone(), rtc })).unwrap();

    // Setup the UI.
    let ui = Rc::new(MainWindow::new().unwrap());
    window.set_size(slint::PhysicalSize::new(240, 240));
    ui.invoke_set_menu(false);

    esp_hal_embassy::init(&clocks, timer_group0.timer0);

    // Async requires the GPIO interrupt to wake futures
    esp_hal::interrupt::enable(
        esp_hal::peripherals::Interrupt::GPIO,
        esp_hal::interrupt::Priority::Priority1,
    ).unwrap();

    _ = spawner.spawn(ui_scene_loop(AnyInput::new(next_btn, Pull::None), AnyInput::new(back_btn, Pull::None), window.clone(), ui.clone()));
    _ = spawner.spawn(ui_update_loop(window, display));
    _ = spawner.spawn(adc(peripherals.ADC1, adc_pin, ui.clone()));
    _ = spawner.spawn(clock_ticker(ui));

    loop {
        // Tick...
        if let Some(evt) = touchpad.read_one_touch_event(true) {
            println!("{:?}", evt);
        }
        Timer::after(Duration::from_millis(5)).await;
    }
}

fn init_heap() {
    const HEAP_SIZE: usize = 32 * 2048;
    static mut HEAP: core::mem::MaybeUninit<[u8; HEAP_SIZE]> = core::mem::MaybeUninit::uninit();

    unsafe {
        ALLOCATOR.init(HEAP.as_mut_ptr().cast(), HEAP_SIZE);
    }
}
