#![warn(clippy::all)]
#![no_std]
#![no_main]
#![feature(alloc)]
#![feature(alloc_error_handler)]

use alloc_cortex_m::CortexMHeap;
use core::alloc::Layout as AllocLayout;
use core::panic::PanicInfo;
use cortex_m::asm;
use cortex_m_rt::{entry, exception};
use stm32f7::stm32f7x6::{CorePeripherals, Peripherals};
use stm32f7_discovery::{
    gpio::{GpioPort, OutputPin},
    init,
    lcd::{self, Color, Layer},
    print, println,
    system_clock::{self, Hz},
};

// Enables us to put stuff on the heap
#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();

// Called when that heap's out of memory
#[alloc_error_handler]
fn oom_handler(_: AllocLayout) -> ! {
    println!("Out of memory!");

    asm::bkpt();
    loop {}
}

// systick handler, increments the system clock
#[exception]
fn SysTick() {
    system_clock::tick();
}

// Panic handler
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    println!("[!] feck");
    println!("{}", info);

    asm::bkpt();
    loop {}
}

#[entry]
fn blub() -> ! {
    // Acquire systick
    let core_peripherals = CorePeripherals::take().unwrap();
    let mut systick = core_peripherals.SYST;

    // Initialize 216 MHz processor and enable GPIO
    let peripherals = Peripherals::take().unwrap();
    let mut rcc = peripherals.RCC;
    let mut pwr = peripherals.PWR;
    let mut flash = peripherals.FLASH;
    let mut fmc = peripherals.FMC;
    let mut ltdc = peripherals.LTDC;
    init::init_system_clock_216mhz(&mut rcc, &mut pwr, &mut flash);
    init::enable_gpio_ports(&mut rcc);

    // Initialize GPIO
    let mut pins = init::pins(
        GpioPort::new(peripherals.GPIOA),
        GpioPort::new(peripherals.GPIOB),
        GpioPort::new(peripherals.GPIOC),
        GpioPort::new(peripherals.GPIOD),
        GpioPort::new(peripherals.GPIOE),
        GpioPort::new(peripherals.GPIOF),
        GpioPort::new(peripherals.GPIOG),
        GpioPort::new(peripherals.GPIOH),
        GpioPort::new(peripherals.GPIOI),
        GpioPort::new(peripherals.GPIOJ),
        GpioPort::new(peripherals.GPIOK),
    );

    // Initialize systick
    init::init_systick(Hz(50), &mut systick, &rcc);
    systick.enable_interrupt();

    // Enable SDRAM (prereq for LCD)
    init::init_sdram(&mut rcc, &mut fmc);
    // Initialize LCD
    let mut lcd = init::init_lcd(&mut ltdc, &mut rcc);
    // Enable display and backlight
    pins.display_enable.set(true);
    pins.backlight.set(true);

    let mut layer_1 = lcd.layer_1().unwrap();
    let mut layer_2 = lcd.layer_2().unwrap();
    layer_1.clear();
    layer_2.clear();
    lcd::init_stdout(layer_2);

    println!("[*] initialized");

    let mut counter = 0;
    let mut color_counter = 0;
    let colors = [
        Color::from_hex(0xff0000),
        Color::from_hex(0x00ff00),
        Color::from_hex(0x0000ff),
    ];

    let mut old_ticks = system_clock::ticks();
    loop {
        let new_ticks = system_clock::ticks();
        if new_ticks - old_ticks >= 5 {
            if lcd::HEIGHT <= 2 * counter {
                counter = 0;
                color_counter += 1;
            }
            stroke_rect(
                &mut layer_1,
                counter,
                counter,
                lcd::WIDTH - 2 * counter - 1,
                lcd::HEIGHT - 2 * counter - 1,
                colors[color_counter % colors.len()],
            );
            counter += 1;

            old_ticks = new_ticks;
        }
    }
}

fn stroke_rect<T: lcd::Framebuffer>(
    layer: &mut Layer<T>,
    x: usize,
    y: usize,
    w: usize,
    h: usize,
    color: Color,
) {
    for i in 1..w {
        layer.print_point_color_at(x + i, y, color);
        layer.print_point_color_at(x + i, y + h, color);
    }
    for j in 0..=h {
        layer.print_point_color_at(x, y + j, color);
        layer.print_point_color_at(x + w, y + j, color);
    }
}
