//using the st7735-lcd-example code
// and expanding it to play pong simulation on pico

#![no_std]
#![no_main]


/// The hal is the hardware abstraction layer and its the high level
/// abstraction of the drivers for multiple microcontrollers.
/// The PAC is the peripheral access crate (PAC) which providers the lower-level access to hardware
/// peripherals such as sensors, actuators, and communication interfaces
/// pacs are closer to the register level.
/// Basically The HAL functions will ask for PAC parameters.
/// This adds modularity to the code to swap interfaces at the HAL level.

use panic_semihosting as _;

// the macro for our start-up function
use cortex_m_rt::entry;

//ensure we halt the program on panic.
use defmt_rtt as _;

use core::fmt::Write;
// Alias for our HAL crate
use rp2040_hal as hal;

// Some traits we need
use embedded_graphics::primitives::{Rectangle, PrimitiveStyle};
use embedded_graphics::prelude::*;
use embedded_graphics::pixelcolor::{Rgb565};
//use embedded_time::rate::*;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::adc::OneShot;
use fugit::RateExtU32;
use rp2040_hal::clocks::Clock;
//use rp2040_hal::entry;
use st7735_lcd;
use st7735_lcd::Orientation;
// UART related types
use hal::uart::{DataBits, StopBits, UartConfig};

// A shorter alias for the Peripheral Access Crate, which provides low level
// register access
use hal::pac;

// the linker will place this boot block at the start of our program image.
// we need this to help the ROM bootloader get our code up and running.
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

//external high-speed crystal on the raspb pi pico board is 12 MHz.
// Adjust if your board hsa a different freq.
const XTAL_FREQ_HZ: u32 = 12_000_000u32;


/// entry point to our bare metal application.
/// 
/// The "#[entry]" macro ensures the cortex-M start up code calls this function
/// as soon as all global variables are intialized
/// 
/// the function configures the RP2040 peripherals, then performs some example
/// SPI transactions, then goes to sleep.
#[entry]
fn main() -> !
{

   
    
    //grab our single ton objects.
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    // set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    //configure the clocks phase-locked loop (ppls basically your oscilatting crystal)
    let clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ, // XOSC crystal freq.
        pac.XOSC, //XOSC devce
        pac.CLOCKS, //clock device
        pac.PLL_SYS, //ppl sys device
        pac.PLL_USB, //ppl usb dev
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    //configure the system timer as a delay provider
    //Delay new takes in two parameters the system timer , and the ahb freq as a u32 integer.
    // the ahb bus is widely used in ARM7 and ARM9 cortex M based designs
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());
    let mut timer = rp2040_hal::Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);


    // the single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // set the pins to their default state
    //creating a new pin bsnk based on the sio.
    let pins = hal::gpio::Pins::new
    (
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    //these are implicitly used by the spi driver if they are in the correct mode
    let _spi_sclk = pins.gpio6.into_function::<hal::gpio::FunctionSpi>();
    let _spi_mosi = pins.gpio7.into_function::<hal::gpio::FunctionSpi>();
    let _spi_miso = pins.gpio4.into_function::<hal::gpio::FunctionSpi>();
    let spi = hal::Spi::<_, _,_,8>::new(pac.SPI0,(_spi_mosi, _spi_miso, _spi_sclk));

    //push pull output pins are pins that can be actually on or off
    let mut lcd_led = pins.gpio12.into_push_pull_output();
    let dc = pins.gpio13.into_push_pull_output();
    let rst = pins.gpio14.into_push_pull_output();

    //Exchange the uninitialized SPI driver for an initialzed one.
    let spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        16_000_000u32.Hz(),
        &embedded_hal::spi::MODE_0,
    );

    let uart_pins =
    (
       pins.gpio0.into_function::<hal::gpio::FunctionUart>(),
       pins.gpio1.into_function::<hal::gpio::FunctionUart>(),
    );

        // Create a UART driver
        let mut uart = hal::uart::UartPeripheral::new(pac.UART0, uart_pins, &mut pac.RESETS)
        .enable(
            UartConfig::new(115200.Hz(), DataBits::Eight, None, StopBits::One),
            clocks.peripheral_clock.freq(),
        )
        .unwrap();

    // Write to the UART
    uart.write_full_blocking(b"ADC example\r\n");

    // Enable ADC
    let mut adc = hal::Adc::new(pac.ADC, &mut pac.RESETS);

    // Configure GPIO26 as an ADC input
    let mut adc_pin_0 = hal::adc::AdcPin::new(pins.gpio26).unwrap();

    //setup the ST7735 display
    let mut disp = st7735_lcd::ST7735::new(spi, dc, rst, true, false, 160, 128);

    disp.init(&mut timer).unwrap();
    disp.set_orientation(&Orientation::Landscape).unwrap();
    disp.clear(Rgb565::BLACK).unwrap();
    disp.set_offset(0, 25);

    // Draw a filled square
    Rectangle::with_corners(Point::new(36, 16), Point::new(36 + 16, 16 + 16))
        .into_styled(PrimitiveStyle::with_stroke(Rgb565::GREEN, 1))
        .draw(&mut disp)
        .unwrap();

    lcd_led.set_high().unwrap();

    // Draw a filled square
    let mut paddle1 =  Rectangle::with_corners(Point::new(2, 50+ 50), Point::new(2+16, 100+60))
    .into_styled(PrimitiveStyle::with_stroke(Rgb565::GREEN, 1))
    .draw(&mut disp)
    .unwrap();

    let mut paddle2 =  Rectangle::with_corners(Point::new(142, 50+ 50 ), Point::new(142 + 16, 100+60))
      .into_styled(PrimitiveStyle::with_stroke(Rgb565::GREEN, 1))
      .draw(&mut disp)
      .unwrap();
    
    loop {
      // Read the raw ADC counts from the temperature sensor channel.
      let pin_adc_counts: u16 = adc.read(&mut adc_pin_0).unwrap();
      if pin_adc_counts == 4095
      {
        writeln!(
          uart,
          "ADC readings: is 4095"
      )
      .unwrap();
      }
      else 
      {
        writeln!(
          uart,
          "no input yet \n"    
      )
      .unwrap();
      }
      delay.delay_ms(1000);     
    }

}
