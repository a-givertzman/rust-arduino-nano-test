#![no_std]
#![no_main]

extern crate alloc;

use core::{cell::UnsafeCell, alloc::{GlobalAlloc, Layout}};

use alloc::{vec::{Vec}, format};
use panic_halt as _;
// use ufmt_float::uFmt_f32;
// use rustfft::{FftPlanner, num_complex::Complex};

#[arduino_hal::entry]
fn main() -> ! {
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);

    /*
     * For examples (and inspiration), head to
     *
     *     https://github.com/Rahix/avr-hal/tree/main/examples
     *
     * NOTE: Not all examples were ported to all boards!  There is a good chance though, that code
     * for a different board can be adapted for yours.  The Arduino Uno currently has the most
     * examples available.
     */

    let mut led = pins.d13.into_output();


    let mut adc = arduino_hal::Adc::new(dp.ADC, Default::default());
    let ai0 = pins.a0.into_analog_input(&mut adc);
    let ai1 = pins.a1.into_analog_input(&mut adc);
    let ai2 = pins.a2.into_analog_input(&mut adc);
    let ai3 = pins.a3.into_analog_input(&mut adc);
    let ai4 = pins.a4.into_analog_input(&mut adc);
    let ai5 = pins.a5.into_analog_input(&mut adc);
    let mut serial = arduino_hal::default_serial!(dp, pins, 57600);
    // Vec<>


    loop {
        let mut voltage: Vec<u16> = Vec::new();
        for i in 0..100 {
            voltage[i] = ai0.analog_read(&mut adc);
        }
        
        let mut vStr = "";
        for v in voltage {
            vStr = &format!("{} | {}", vStr, v);
        }
        
        
        ufmt::uwriteln!(&mut serial, "{}", vStr).unwrap();
        // let voltage = adc.read_blocking(&a0);
        
        // alternatively, a non-blocking interface exists
        // let voltage = nb::block!(adc.read_nonblocking(&a0)).void_unwrap();
    
        // let test_data_f = f32::from(voltage);
        // ufmt::uwriteln!(&mut serial, "A0: {:?}", voltage).unwrap();
    

        // let test_data_f_str = uFmt_f32::Three(test_data_f);
        // ufmt::uwriteln!(&mut serial, "value as f32: {} ", test_data_f_str).unwrap();
        // ufmt::uwriteln!(&mut serial, "f32 back to u16: {} ", test_data_f as u16).unwrap();
        // arduino_hal::delay_ms(10);

        // led.toggle();
        // arduino_hal::delay_ms(100);
        // led.toggle();
        // arduino_hal::delay_ms(100);
        // led.toggle();
        // arduino_hal::delay_ms(100);
        // led.toggle();
        // arduino_hal::delay_ms(1000);
        // let mut planner = FftPlanner::new();
        // let fft = planner.plan_fft_forward(1234);
        
        // let mut buffer = vec![Complex{ re: 0.0f32, im: 0.0f32 }; 1234];
        // fft.process(&mut buffer);        
    }
}

// Bump pointer allocator for *single* core systems
struct BumpPointerAlloc {
    head: UnsafeCell<usize>,
    end: usize,
}

unsafe impl Sync for BumpPointerAlloc {}

#[avr_device::interrupt(atmega328p)]
fn INTO() {
    let current = REV
}

unsafe impl GlobalAlloc for BumpPointerAlloc {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        // `interrupt::free` is a critical section that makes our allocator safe
        // to use from within interrupts
        avr_device::interrupt::free(|_| {
            let head = self.head.get();
            let size = layout.size();
            let align = layout.align();
            let align_mask = !(align - 1);

            // move start up to the next alignment boundary
            let start = (*head + align - 1) & align_mask;

            if start + size > self.end {
                // a null pointer signal an Out Of Memory condition
                ptr::null_mut()
            } else {
                *head = start + size;
                start as *mut u8
            }
        })
    }

    unsafe fn dealloc(&self, _: *mut u8, _: Layout) {
        // this allocator never deallocates memory
    }
}

#[global_allocator]
static HEAP: BumpPointerAlloc = BumpPointerAlloc {
    head: UnsafeCell::new(0x2000_0100),
    end: 0x2000_0200,
};