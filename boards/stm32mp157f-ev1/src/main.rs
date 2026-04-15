// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright Tock Contributors 2026.

//! Board file for STM32WLE5JC Seeed Studio LoRa E5 HF mini development board.
//!
//! - <https://wiki.seeedstudio.com/LoRa_E5_mini/>

#![no_std]
// Disable this attribute when documenting, as a workaround for
// https://github.com/rust-lang/rust/issues/62184.
#![cfg_attr(not(doc), no_main)]
#![deny(missing_docs)]

use core::ptr::addr_of_mut;

use capsules_core::virtualizers::virtual_alarm::VirtualMuxAlarm;
use kernel::capabilities;
use kernel::component::Component;
use kernel::debug::PanicResources;
use kernel::hil::led::LedLow;
use kernel::hil::time::Counter;
use kernel::platform::{KernelResources, SyscallDriverLookup};
use kernel::utilities::single_thread_value::SingleThreadValue;
use kernel::{create_capability, debug, static_init};
use stm32mp157f::chip_specs::Stm32mp157fSpecs;
use stm32mp157f::interrupt_service::Stm32mp157fDefaultPeripherals;
// use stm32wle5jc::clocks::msi::MSI_FREQUENCY_MHZ;
// use stm32wle5jc::gpio::{PinId, PortId};
// use stm32wle5jc::interrupt_service::Stm32wle5jcDefaultPeripherals;
// use stm32wle5jc::subghz_radio::SubGhzRadioVirtualGpio;

/// Support routines for debugging I/O.
pub mod io;

///This platform's chip type:
pub type ChipHw = stm32mp157f::chip::Stm32mp157x<
    'static,
    stm32mp157f::interrupt_service::Stm32mp157fDefaultPeripherals<'static>,
>;

// Number of concurrent processes this platform supports.
const NUM_PROCS: usize = 4;

type ProcessPrinterInUse = capsules_system::process_printer::ProcessPrinterText;

type SchedulerInUse = components::sched::round_robin::RoundRobinComponentType;

/// Resources for when a board panics used by io.rs.
static PANIC_RESOURCES: SingleThreadValue<PanicResources<ChipHw, ProcessPrinterInUse>> =
    SingleThreadValue::new(PanicResources::new());

// How should the kernel respond when a process faults.
const FAULT_RESPONSE: capsules_system::process_policies::PanicFaultPolicy =
    capsules_system::process_policies::PanicFaultPolicy {};

const LORA_SPI_DRIVER_NUM: usize = capsules_core::driver::NUM::LoRaPhySPI as usize;
const LORA_GPIO_DRIVER_NUM: usize = capsules_core::driver::NUM::LoRaPhyGPIO as usize;

/// Dummy buffer that causes the linker to reserve enough space for the stack.
#[no_mangle]
#[link_section = ".stack_buffer"]
pub static mut STACK_MEMORY: [u8; 0x2000] = [0; 0x2000];

/// A structure representing this platform that holds references to all
/// capsules for this platform.
struct Stm32mp157fev1 {
    scheduler: &'static SchedulerInUse,
}

/// Mapping of integer syscalls to objects that implement syscalls.
impl SyscallDriverLookup for Stm32mp157fev1 {
    fn with_driver<F, R>(&self, driver_num: usize, f: F) -> R
    where
        F: FnOnce(Option<&dyn kernel::syscall::SyscallDriver>) -> R,
    {
        match driver_num {
            _ => f(None),
        }
    }
}

impl
    KernelResources<
        stm32mp157f::chip::Stm32mp157x<
            'static,
            stm32mp157f::interrupt_service::Stm32mp157fDefaultPeripherals<'static>,
        >,
    > for Stm32mp157fev1
{
    type SyscallDriverLookup = Self;
    type SyscallFilter = ();
    type ProcessFault = ();
    type Scheduler = SchedulerInUse;
    type SchedulerTimer = ();
    type WatchDog = ();
    type ContextSwitchCallback = ();

    fn syscall_driver_lookup(&self) -> &Self::SyscallDriverLookup {
        self
    }
    fn syscall_filter(&self) -> &Self::SyscallFilter {
        &()
    }
    fn process_fault(&self) -> &Self::ProcessFault {
        &()
    }
    fn scheduler(&self) -> &Self::Scheduler {
        self.scheduler
    }
    fn scheduler_timer(&self) -> &Self::SchedulerTimer {
        &()
    }
    fn watchdog(&self) -> &Self::WatchDog {
        &()
    }
    fn context_switch_callback(&self) -> &Self::ContextSwitchCallback {
        &()
    }
}

/*
/// Helper function for miscellaneous peripheral functions
unsafe fn setup_peripherals(tim2: &stm32wle5jc::tim2::Tim2, subghz_spi: &stm32wle5jc::spi::Spi) {
    cortexm4::nvic::Nvic::new(stm32wle5jc::nvic::USART1).enable();
    cortexm4::nvic::Nvic::new(stm32wle5jc::nvic::USART2).enable();

    cortexm4::nvic::Nvic::new(stm32wle5jc::nvic::TIM2).enable();
    tim2.enable_clock();
    tim2.start().expect("Failure starting stm32wle5jc TIM2.");
}
*/

/// Statically initialize the core peripherals for the chip.
///
/// This is in a separate, inline(never) function so that its stack frame is
/// removed when this function returns. Otherwise, the stack space used for
/// these static_inits is wasted.
#[inline(never)]
unsafe fn create_peripherals() -> &'static mut Stm32mp157fDefaultPeripherals<'static> {
    let rcc = static_init!(stm32mp157f::rcc::Rcc, stm32mp157f::rcc::Rcc::new());
    let clocks = static_init!(
        stm32mp157f::clocks::Clocks<Stm32mp157fSpecs>,
        stm32mp157f::clocks::Clocks::new(rcc)
    );

    let peripherals = static_init!(
        Stm32mp157fDefaultPeripherals,
        Stm32mp157fDefaultPeripherals::new(clocks)
    );

    peripherals
}

/// Main function
///
/// This is called after RAM initialization is complete.
#[no_mangle]
pub unsafe fn main() {
    // Initialize deferred calls very early.
    kernel::deferred_call::initialize_deferred_call_state::<
        <ChipHw as kernel::platform::chip::Chip>::ThreadIdProvider,
    >();

    stm32mp157f::init();

    let peripherals = create_peripherals();
    peripherals.init();
    let base_peripherals = &peripherals.stm32mp157x;

    // Create an array to hold process references.
    let processes = components::process_array::ProcessArrayComponent::new()
        .finalize(components::process_array_component_static!(NUM_PROCS));

    // Setup space to store the core kernel data structure.
    let board_kernel = static_init!(kernel::Kernel, kernel::Kernel::new(processes.as_slice()));

    let chip = static_init!(
        stm32mp157f::chip::Stm32mp157x<Stm32mp157fDefaultPeripherals>,
        stm32mp157f::chip::Stm32mp157x::new(peripherals)
    );

    // setup_peripherals(&base_peripherals.tim2, &base_peripherals.subghz_spi);

    // Create capabilities that the board needs to call certain protected kernel
    // functions.
    let main_loop_capability = create_capability!(capabilities::MainLoopCapability);
    let process_management_capability =
        create_capability!(capabilities::ProcessManagementCapability);

    // Clock to all GPIO Ports is enabled in `set_pin_primary_functions()`
    // let gpio_ports = &base_peripherals.gpio_ports;
    // gpio_ports.get_port_from_port_id(PortId::B).enable_clock();
    // gpio_ports.get_port_from_port_id(PortId::A).enable_clock();

    //--------------------------------------------------------------------
    // Usart
    //--------------------------------------------------------------------
    base_peripherals.usart4.enable_clock();

    // EV1 ST-LINK VCP is routed from UART4:
    // TX=PG11 (AF6), RX=PB2 (AF8)
    base_peripherals
        .gpio_ports
        .get_pin(stm32mp157f::gpio::PinId::PG11)
        .map(|pin| {
            pin.set_mode(stm32mp157f::gpio::Mode::AlternateFunctionMode);
            pin.set_alternate_function(stm32mp157f::gpio::AlternateFunction::AF6);
        });
    base_peripherals
        .gpio_ports
        .get_pin(stm32mp157f::gpio::PinId::PB02)
        .map(|pin| {
            pin.set_mode(stm32mp157f::gpio::Mode::AlternateFunctionMode);
            pin.set_alternate_function(stm32mp157f::gpio::AlternateFunction::AF8);
        });

    let uart_mux = components::console::UartMuxComponent::new(&base_peripherals.usart4, 115200)
        .finalize(components::uart_mux_component_static!());

    (*addr_of_mut!(io::WRITER)).set_initialized();

    //--------------------------------------------------------------------
    // Alarm
    //--------------------------------------------------------------------
    // let tim2 = &base_peripherals.tim2;
    // let mux_alarm = components::alarm::AlarmMuxComponent::new(tim2).finalize(
    //     components::alarm_mux_component_static!(stm32wle5jc::tim2::Tim2),
    // );

    // let alarm = components::alarm::AlarmDriverComponent::new(
    //     board_kernel,
    //     capsules_core::alarm::DRIVER_NUM,
    //     mux_alarm,
    // )
    // .finalize(components::alarm_component_static!(stm32wle5jc::tim2::Tim2));

    //--------------------------------------------------------------------
    // Console.
    //--------------------------------------------------------------------
    // let console = components::console::ConsoleComponent::new(
    //     board_kernel,
    //     capsules_core::console::DRIVER_NUM,
    //     uart_mux,
    // )
    // .finalize(components::console_component_static!());

    // Create the debugger object that handles calls to `debug!()`.
    components::debug_writer::DebugWriterComponent::new::<
        <ChipHw as kernel::platform::chip::Chip>::ThreadIdProvider,
    >(
        uart_mux,
        create_capability!(capabilities::SetDebugWriterCapability),
    )
    .finalize(components::debug_writer_component_static!());

    //let process_printer = components::process_printer::ProcessPrinterTextComponent::new()
    //    .finalize(components::process_printer_text_component_static!());

    //--------------------------------------------------------------------
    // LED
    //--------------------------------------------------------------------
    // let led = components::led::LedsComponent::new().finalize(components::led_component_static!(
    //     LedLow<'static, stm32wle5jc::gpio::Pin>,
    //     LedLow::new(gpio_ports.get_pin(stm32wle5jc::gpio::PinId::PB05).unwrap()),
    // ));

    // PROCESS CONSOLE
    //--------------------------------------------------------------------
    // let process_console = components::process_console::ProcessConsoleComponent::new(
    //     board_kernel,
    //     uart_mux,
    //     mux_alarm,
    //     process_printer,
    //     Some(cortexm4::support::reset),
    // )
    // .finalize(components::process_console_component_static!(
    //     stm32wle5jc::tim2::Tim2
    // ));
    // let _ = process_console.start();

    // let scheduler = components::sched::round_robin::RoundRobinComponent::new(processes)
    //    .finalize(components::round_robin_component_static!(NUM_PROCS));

    let stm32mp157f_ev1 = Stm32mp157fev1 {
        scheduler: static_init!(SchedulerInUse, SchedulerInUse::new()),
    };

    debug!("Initialization complete. Entering main loop...");
    // These symbols are defined in the linker script.
    extern "C" {
        /// Beginning of the ROM region containing app images.
        static _sapps: u8;
        /// End of the ROM region containing app images.
        static _eapps: u8;
        /// Beginning of the RAM region for app memory.
        static mut _sappmem: u8;
        /// End of the RAM region for app memory.
        static _eappmem: u8;
    }

    kernel::process::load_processes(
        board_kernel,
        chip,
        core::slice::from_raw_parts(
            core::ptr::addr_of!(_sapps),
            core::ptr::addr_of!(_eapps) as usize - core::ptr::addr_of!(_sapps) as usize,
        ),
        core::slice::from_raw_parts_mut(
            core::ptr::addr_of_mut!(_sappmem),
            core::ptr::addr_of!(_eappmem) as usize - core::ptr::addr_of!(_sappmem) as usize,
        ),
        &FAULT_RESPONSE,
        &process_management_capability,
    )
    .unwrap_or_else(|err| {
        debug!("Error loading processes!");
        debug!("{:?}", err);
    });

    //Uncomment to run multi alarm test
    /*components::test::multi_alarm_test::MultiAlarmTestComponent::new(mux_alarm)
    .finalize(components::multi_alarm_test_component_buf!(stm32f429zi::tim2::Tim2))
    .run();*/

    board_kernel.kernel_loop(
        &stm32mp157f_ev1,
        chip,
        None::<&kernel::ipc::IPC<2>>,
        &main_loop_capability,
    );
}
