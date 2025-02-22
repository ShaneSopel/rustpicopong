//using the st7735-lcd-example code
// and expanding it to play pong simulation on pico

#![no_std]
#![no_main]


use core::error;
use core::fmt::{Display, Error};

use defmt::Str;
use embedded_graphics::text::Text;
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

use rp2040_hal::pac::spi0;
//use core::fmt::Write;
// Alias for our HAL crate
use rp2040_hal as hal;

// Some traits we need
use embedded_graphics::primitives::{Circle, PrimitiveStyle, Rectangle};
use embedded_graphics::prelude::*;
use embedded_graphics::pixelcolor::{Rgb565};
use embedded_graphics::mono_font::{ascii::FONT_6X10, MonoTextStyle};
//use embedded_time::rate::*;
use embedded_hal::blocking::spi;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::adc::OneShot;
use fugit::RateExtU32;
use rp2040_hal::clocks::Clock;
//use rp2040_hal::entry;
use st7735_lcd::{self, ST7735};
use st7735_lcd::Orientation;
// UART related types
//use hal::uart::{DataBits, StopBits, UartConfig};

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

type st7735_lcd_t =  ST7735<rp2040_hal::Spi<rp2040_hal::spi::Enabled, pac::SPI0, (rp2040_hal::gpio::Pin<rp2040_hal::gpio::bank0::Gpio7, rp2040_hal::gpio::FunctionSpi, 
rp2040_hal::gpio::PullDown>, rp2040_hal::gpio::Pin<rp2040_hal::gpio::bank0::Gpio4, rp2040_hal::gpio::FunctionSpi, rp2040_hal::gpio::PullDown>, rp2040_hal::gpio::Pin<rp2040_hal::gpio::bank0::Gpio6, rp2040_hal::gpio::FunctionSpi, rp2040_hal::gpio::PullDown>)>, rp2040_hal::gpio::Pin<rp2040_hal::gpio::bank0::Gpio13, rp2040_hal::gpio::FunctionSio<rp2040_hal::gpio::SioOutput>, rp2040_hal::gpio::PullDown>, rp2040_hal::gpio::Pin<rp2040_hal::gpio::bank0::Gpio14, rp2040_hal::gpio::FunctionSio<rp2040_hal::gpio::SioOutput>, rp2040_hal::gpio::PullDown>>;

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

    let mut pong: Pongvals = Pongvals
    {
      // values for the paddle 1 location
      paddle1_p1 : 61,
      paddle1_p2 : 20,

      //values for the paddle 2 location
      paddle2_p1 : 61,
      paddle2_p2 : 20,

      //values for the ball location
      ball_x : 80,
      ball_y : 80,
      ball_diameter: 12,

      //values for game height and width (LCD is 128 x 160)
      game_height : 128,
      game_width : 160,

      state_move_paddle1 : false,
      state_move_paddle2 : false,
      state_move_ball : true,

      player1val : 0,
      player2val : 0,
      player1text : "Player 1: ",
      player2text : "Player 2: ",
      player1_text_location : Point::new(50,0),
      player2_text_location : Point::new(50,0),

    };

    //text defaults
    let character_style = MonoTextStyle::new(&FONT_6X10, Rgb565::GREEN);
    
    //grab our single ton objects.
    let mut pac = pac::Peripherals::take().unwrap();
    //let core = pac::CorePeripherals::take().unwrap();

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
    //let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());
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

   // let uart_pins =
   // (
   //    pins.gpio0.into_function::<hal::gpio::FunctionUart>(),
   //    pins.gpio1.into_function::<hal::gpio::FunctionUart>(),
   // );

        // Create a UART driver
      //  let mut uart = hal::uart::UartPeripheral::new(pac.UART0, uart_pins, &mut pac.RESETS)
      //  .enable(
      //      UartConfig::new(115200.Hz(), DataBits::Eight, None, StopBits::One),
      //      clocks.peripheral_clock.freq(),
      //  )
      //  .unwrap();

    // Write to the UART
    //uart.write_full_blocking(b"ADC example\r\n");

    // Enable ADC
    let mut adc = hal::Adc::new(pac.ADC, &mut pac.RESETS);

    // Configure GPIO22 GPIO26 GPIO27 and GPIO28 as an ADC input
    let mut adc_pin_1 = hal::adc::AdcPin::new(pins.gpio26).unwrap();
    let mut adc_pin_2 = hal::adc::AdcPin::new(pins.gpio27).unwrap();
    let mut adc_pin_3 = hal::adc::AdcPin::new(pins.gpio28).unwrap();
    let mut adc_pin_4 = hal::adc::AdcPin::new(pins.gpio29).unwrap();
    //let mut adc_pin_5 = 2;


    //setup the ST7735 display
    let mut disp = st7735_lcd::ST7735::new(spi, dc, rst, true, false, 160, 128);    

    disp.init(&mut timer).unwrap();
    disp.set_orientation(&Orientation::Landscape).unwrap();
    disp.clear(Rgb565::BLACK).unwrap();
    disp.set_offset(0, 25);

    // Draw a filled square
    Rectangle::with_corners(Point::new(2, 60 + 1), Point::new(2 + 16, 20))
        .into_styled(PrimitiveStyle::with_stroke(Rgb565::GREEN, 1))
        .draw(&mut disp)
        .unwrap();
   
   Rectangle::with_corners(Point::new(142, 60 + 1 ), Point::new(142 + 16, 20 ))
      .into_styled(PrimitiveStyle::with_stroke(Rgb565::GREEN, 1))
      .draw(&mut disp)
      .unwrap();  

    Rectangle::with_center(Point::new(80, 100), Size::new(155, 5))
    .into_styled(PrimitiveStyle::with_stroke(Rgb565::RED, 1))
    .draw(&mut disp)
    .unwrap(); 

     Rectangle::with_center(Point::new(80, 110), Size::new(155, 5))
    .into_styled(PrimitiveStyle::with_stroke(Rgb565::RED, 1))
    .draw(&mut disp)
    .unwrap();   
 
    lcd_led.set_high().unwrap();

    
    loop {

         // Read the raw ADC counts from button presses
    let pin_adc_26: u16 = adc.read(&mut adc_pin_1).unwrap();
    let pin_adc_27: u16 = adc.read(&mut adc_pin_2).unwrap();
    let pin_adc_28: u16 = adc.read(&mut adc_pin_3).unwrap();
    let pin_adc_29: u16 = adc.read(&mut adc_pin_4).unwrap();

    //let pin_adc_x:  u16 = adc.read(&mut adc_pin_4).unwrap();
    //let pin_adc_y:  u16 = adc.read(&mut adc_pin_5).unwrap();

    //move the players using functions
    pong.player1_moveup(pin_adc_26,   &mut disp);
    pong.player1_movedown(pin_adc_27, &mut disp);

    pong.player2_moveup(pin_adc_28, &mut disp);
    pong.player2_movedown(pin_adc_29, &mut disp);

    //text for scores
    //let player1_score = Text::with_baseline(pong.player1text, pong.player1_text_location, character_style, embedded_graphics::text::Baseline::Middle)
    //.draw(&mut disp)
    //.unwrap();

    //let player2_score = Text::with_baseline(pong.player2text,  pong.player2_text_location, character_style, embedded_graphics::text::Baseline::Middle)
    //.draw(&mut disp)
    //.unwrap();

    // net (I might add a net down the center... just a bunch of dashes)
   // pong.player2_score(disp);
    
    //ball
    let mut ball = Circle::with_center(Point::new(pong.ball_x, pong.ball_y), pong.ball_diameter)
    .into_styled(PrimitiveStyle::with_stroke(Rgb565::BLUE, 1))
    .draw(&mut disp)
    .unwrap();

    //paddle 2
    let mut paddle2 =  Rectangle::with_corners(Point::new(142, pong.paddle2_p1 ), Point::new(142 + 16, pong.paddle2_p2))
    .into_styled(PrimitiveStyle::with_stroke(Rgb565::GREEN, 1))
    .draw(&mut disp)
    .unwrap();

    //paddle 1
    let mut paddle1 =  Rectangle::with_corners(Point::new(2, pong.paddle1_p1), Point::new(2+16, pong.paddle1_p2))
    .into_styled(PrimitiveStyle::with_stroke(Rgb565::GREEN, 1))
    .draw(&mut disp)
    .unwrap();
    
    }

}

pub struct Pongvals
{
      // values for the paddle 1 location
      paddle1_p1 : i32,
      paddle1_p2 : i32,

      //values for the paddle 2 location
      paddle2_p1 : i32,
      paddle2_p2 : i32,
  
      //values for the ball location
      ball_x : i32,
      ball_y : i32,
      ball_diameter: u32,
  
      //values for game height and width (LCD is 128 x 160)
      game_height : i32,
      game_width  : i32,

      //Game States
      state_move_paddle1 : bool,
      state_move_paddle2 : bool,
      state_move_ball : bool,
  
      //Text Defaults
      player1val : u16,
      player2val : u16,
      player2text : &'static str,
      player1text : &'static str,
      player2_text_location : Point,
      player1_text_location : Point,
}

impl Pongvals
{
  fn reset_game(&mut self) 
  {
   
  }

  fn player1_score(&mut self,  disp: &st7735_lcd_t) 
  {
     //score player 1?
    if self.ball_x > self.game_width
    {
      self.player1val = self.player1val + 1;
    }
      
  }

  fn player2_score(&mut self, disp: &st7735_lcd_t) 
  {
     // score player 2?
     if self.ball_x <= 0
     {
       self.player2val = self.player2val + 1;
     }
      
  }

  fn clear_screen(&mut self, disp: &mut st7735_lcd_t)
  {
    disp.clear(Rgb565::BLACK).unwrap();
  }

  fn player1_moveup(&mut self, pin_adc_26: u16, disp: &mut st7735_lcd_t) -> bool 
  {
    if pin_adc_26 == 4095
      {
        self.paddle1_p1 = self.paddle1_p1 + 5;
        self.paddle1_p2 = self.paddle1_p2 + 5;
 
        self.clear_screen(disp);

        return self.state_move_paddle1 == true;
      }
      else {
          return self.state_move_paddle1 == false;
      }
  }

  fn player1_movedown(&mut self, pin_adc_27: u16, disp: &mut st7735_lcd_t) -> bool
  {
    if pin_adc_27 == 4095
    {
      self.paddle1_p1 = self.paddle1_p1 - 5;
      self.paddle1_p2 = self.paddle1_p2 - 5;

      self.clear_screen(disp);

      return self.state_move_paddle1 == true;
    }
    else {
        return self.state_move_paddle1 == false;
    }

  }

  fn player2_moveup(& mut self, pin_adc_28 : u16, disp: &mut st7735_lcd_t) -> bool 
  {
    if pin_adc_28 == 4095
    {
      self.paddle2_p1 = self.paddle2_p1 + 5;
      self.paddle2_p2 = self.paddle2_p2 + 5;

      self.clear_screen(disp);

      return self.state_move_paddle2 == true;
    }
    else {

      return self.state_move_paddle2 == false;
    }
  }

  fn player2_movedown(& mut self, pin_adc_29 : u16, disp: &mut st7735_lcd_t) -> bool 
  {
    if pin_adc_29 == 4095
    {
      self.paddle2_p1 = self.paddle2_p1 - 5;
      self.paddle2_p2 = self.paddle2_p2 - 5;

      self.clear_screen(disp);

      return self.state_move_paddle2 == true;
    }
    else {

      return self.state_move_paddle2 == false;
    }
      
  }
  
}

