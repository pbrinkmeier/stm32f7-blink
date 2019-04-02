#![warn(clippy::all)]
#![no_std]
#![no_main]
#![feature(alloc_error_handler)]
#![feature(alloc)]

#[macro_use]
extern crate alloc;
extern crate alloc_cortex_m;
extern crate cortex_m;
extern crate cortex_m_rt as rt;
extern crate cortex_m_semihosting as sh;
extern crate stm32f7;
extern crate stm32f7_discovery;
extern crate smoltcp;

use alloc::vec::Vec;
use alloc_cortex_m::CortexMHeap;
use core::alloc::Layout as AllocLayout;
use core::panic::PanicInfo;
use cortex_m_rt::{entry, exception};
use smoltcp::{
    socket::{Socket, SocketSet, TcpSocket, TcpSocketBuffer},
    time::Instant,
    wire::{EthernetAddress, IpEndpoint, Ipv4Address},
};
use stm32f7::stm32f7x6::{
    CorePeripherals, Peripherals,
};
use stm32f7_discovery::{
    ethernet,
    gpio::{GpioPort, OutputPin},
    init,
    system_clock::{self, Hz},
    lcd,
    print,
    println
};

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

const ETH_ADDR: EthernetAddress = EthernetAddress([0x00, 0x08, 0xdc, 0xab, 0xcd, 0xef]);

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
    let mut syscfg = peripherals.SYSCFG;
    let mut ethernet_mac = peripherals.ETHERNET_MAC;
    let mut ethernet_dma = peripherals.ETHERNET_DMA;

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
    init::init_systick(Hz(20), &mut systick, &rcc);
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

    let mut ethernet_interface = ethernet::EthernetDevice::new(
        Default::default(),
        Default::default(),
        &mut rcc,
        &mut syscfg,
        &mut ethernet_mac,
        &mut ethernet_dma,
        ETH_ADDR,
    )
    .map(|device| {
        let iface = device.into_interface();
        let ip_addr = iface.ipv4_addr().unwrap();
        (iface, ip_addr)
    });
    if let Err(e) = ethernet_interface {
        println!("Ethernet init failed: {:?}", e);
    }

    let mut sockets = SocketSet::new(Vec::new());
    let endpoint = IpEndpoint::new(smoltcp::wire::IpAddress::Ipv4(<Ipv4Address>::new(10, 0, 0, 2)), 15);

    let tcp_rx_buffer = TcpSocketBuffer::new(vec![0; ethernet::MTU]);
    let tcp_tx_buffer = TcpSocketBuffer::new(vec![0; ethernet::MTU]);
    let mut tcp_socket = TcpSocket::new(tcp_rx_buffer, tcp_tx_buffer);
    tcp_socket.listen(endpoint).unwrap();
    sockets.add(tcp_socket);

    loop {
        if let Ok((ref mut iface, ref mut _ip_addr)) = ethernet_interface {
            let timestamp = Instant::from_millis(system_clock::ms() as i64);

            match iface.poll(&mut sockets, timestamp) {
                Err(::smoltcp::Error::Exhausted) => {
                    println!("Exhausted");
                    continue;
                }
                Err(::smoltcp::Error::Unrecognized) => println!("Unrecognized"),
                Err(e) => println!("Network error: {:?}", e),
                Ok(socket_changed) => {
                    println!("check 3");
                    if socket_changed {
                        for mut socket in sockets.iter_mut() {
                            println!("polling socket...");
                            poll_socket(&mut socket).expect("socket poll failed");
                        }
                    }
                }
            }
        }
    }
}

fn poll_socket(socket: &mut Socket) -> Result<(), smoltcp::Error> {
    match socket {
        &mut Socket::Tcp(ref mut socket) => match socket.local_endpoint().port {
            15 => {
                if !socket.may_recv() {
                    return Ok(());
                }
                let reply = socket.recv(|data| {
                    if data.len() > 0 {
                        let mut reply = Vec::from("tcp: ");
                        let start_index = reply.len();
                        reply.extend_from_slice(data);
                        reply[start_index..(start_index + data.len() - 1)].reverse();
                        (data.len(), Some(reply))
                    } else {
                        (data.len(), None)
                    }
                })?;
                if let Some(reply) = reply {
                    assert_eq!(socket.send_slice(&reply)?, reply.len());
                }
            }
            _ => {}
        },
        _ => {}
    }
    Ok(())
}
