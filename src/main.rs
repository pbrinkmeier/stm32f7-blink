#![warn(clippy::all)]
#![no_std]
#![no_main]
#![feature(alloc_error_handler)]

use stm32f7_discovery::{
    gpio::{GpioPort, OutputPin},
    init,
    system_clock::{self, Hz}
};
use stm32f7::stm32f7x6::{CorePeripherals, Peripherals};
use alloc_cortex_m::CortexMHeap;
use core::alloc::Layout as AllocLayout;
use core::panic::PanicInfo;
use cortex_m_rt::{entry, exception};

// Enables us to put stuff on the heap
#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();

// Called when that heap's out of memory
#[alloc_error_handler]
fn oom_handler(_: AllocLayout) -> ! {
    loop {}
}

// systick handler, increments the system clock
#[exception]
fn SysTick() {
    system_clock::tick();
}

// Panic handler
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
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
        GpioPort::new(peripherals.GPIOK)
    );

    // Initialize systick
    init::init_systick(Hz(20), &mut systick, &rcc);
    systick.enable_interrupt();

    pins.led.set(true);
    loop {
        if system_clock::ticks() % 10 == 0 {
            pins.led.toggle();
        }
    }
}
