use core::slice;
use vcell::VolatileCell;
use crate::atomic_mutex::AtomicMutex;
use crate::bus::UsbRegs;
use cortex_m::interrupt;
use usb_device::endpoint::{EndpointType, EndpointAddress};
use usb_device::{Result, UsbDirection};
use stm32f4xx_hal::stm32f429_public::OTG_HS_DEVICE;
use fiprintln::fiprintln;
use core::ptr;

type EndpointBuffer = &'static mut [VolatileCell<u32>];

// fixme: this should be 6
pub const NUM_ENDPOINTS: usize = 5;
//const DFIFO_BASE: isize = 0x4004_1000 as *mut VolatileCell<u32>;
const DFIFO_BASE: u32 = 0x4004_1000;

pub trait Endpoint {
    fn reset(&self, otg_hs_device: &OTG_HS_DEVICE);
    fn enable(&self, otg_hs_device: &OTG_HS_DEVICE);
    fn address(&self) -> EndpointAddress;
}

pub struct EndpointConfiguration {
    pub ep_type: EndpointType,
    pub max_packet_size: u16,
}

pub struct OutEndpoint {
    index: u8,
    //    rx_buf: AtomicMutex<EndpointBuffer>,
    pub configuration: Option<EndpointConfiguration>,
}

pub struct InEndpoint {
    index: u8,
    //    tx_buf: AtomicMutex<EndpointBuffer>,
    pub configuration: Option<EndpointConfiguration>,
}

impl InEndpoint {
    pub fn new(index: u8) -> InEndpoint {
        InEndpoint {
            index,
//            tx_buf: AtomicMutex::new(unsafe { slice::from_raw_parts_mut::u32(DFIFO_BASE.offset((index as isize) * 1024isize), 1) }),
            configuration: None,
        }
    }

    pub fn index(&self) -> u8 { self.index }

    pub fn write(&self, buf: &[u8]) -> Result<usize> {
        let mut written: usize = 0;
        let mut data: u32 = 0;

        interrupt::free(|cs| {
//            let tx_buf = self.tx_buf.lock(cs);
            fiprintln!("writing {} bytes of data", buf.len());

            for (i, byte) in buf.iter().enumerate() {
                if i != 0 && i & 3 == 0 {
//                    fiprintln!("writing data: {:#034b}", data);
                    unsafe { ptr::write_volatile((DFIFO_BASE + ((self.index as u32) << 12 as u32)) as *mut u32, data) };
                    data = 0;
                }

//                fiprintln!("copying byte: {:#010b}", byte);
                data |= ((byte.clone() as u32) << ((i & 3) << 3)) as u32;
                written += 1;
                // fiprintln!("data written, written: {}", written);
            }

            unsafe { ptr::write_volatile((DFIFO_BASE + ((self.index as u32) << 12 as u32)) as *mut u32, data) };
//            fiprintln!("writing data: {:#034b}", data);
            fiprintln!("written: {}", written);

            Ok(written)
        })
    }
}

impl Endpoint for InEndpoint {
    fn reset(&self, otg_hs_device: &OTG_HS_DEVICE) {
        match self.index {
            0 => {
                otg_hs_device.otg_hs_diepctl0.modify(|_, w| w.snak().set_bit());
                otg_hs_device.otg_hs_dieptsiz0.reset();
                otg_hs_device.otg_hs_diepint0.write(|w| unsafe { w.bits(0xFF) });
            }
            1 => {
                if otg_hs_device.otg_hs_diepctl1.read().epena().bit_is_set() {
                    otg_hs_device.otg_hs_diepctl1.write(|w| w.epdis().set_bit().snak().set_bit());
                }

                otg_hs_device.otg_hs_dieptsiz1.reset();
                otg_hs_device.otg_hs_diepint1.write(|w| unsafe { w.bits(0xFF) });
            }
            2 => {
                if otg_hs_device.otg_hs_diepctl2.read().epena().bit_is_set() {
                    otg_hs_device.otg_hs_diepctl2.write(|w| w.epdis().set_bit().snak().set_bit());
                }

                otg_hs_device.otg_hs_dieptsiz2.reset();
                otg_hs_device.otg_hs_diepint2.write(|w| unsafe { w.bits(0xFF) });
            }
            3 => {
                if otg_hs_device.otg_hs_diepctl3.read().epena().bit_is_set() {
                    otg_hs_device.otg_hs_diepctl3.write(|w| w.epdis().set_bit().snak().set_bit());
                }

                otg_hs_device.otg_hs_dieptsiz3.reset();
                otg_hs_device.otg_hs_diepint3.write(|w| unsafe { w.bits(0xFF) });
            }
            4 => {
                if otg_hs_device.otg_hs_diepctl4.read().epena().bit_is_set() {
                    otg_hs_device.otg_hs_diepctl4.write(|w| w.epdis().set_bit().snak().set_bit());
                }

                otg_hs_device.otg_hs_dieptsiz4.reset();
                otg_hs_device.otg_hs_diepint4.write(|w| unsafe { w.bits(0xFF) });
            }
            5 => {
                if otg_hs_device.otg_hs_diepctl5.read().epena().bit_is_set() {
                    otg_hs_device.otg_hs_diepctl5.write(|w| w.epdis().set_bit().snak().set_bit());
                }

                otg_hs_device.otg_hs_dieptsiz5.reset();
                otg_hs_device.otg_hs_diepint5.write(|w| unsafe { w.bits(0xFF) });
            }

            _ => unreachable!()
        }
    }

    fn enable(&self, otg_hs_device: &OTG_HS_DEVICE) {
        let configuration = self.configuration.as_ref().unwrap();

        let ep_type_bits = match configuration.ep_type {
            EndpointType::Control => 0,
            EndpointType::Isochronous => 1,
            EndpointType::Bulk => 2,
            EndpointType::Interrupt => 3
        };

        otg_hs_device.otg_hs_daintmsk.modify(|r, w| unsafe { w.iepm().bits(r.iepm().bits() | (1 << self.index)) });

        match self.index {
            0 => {
                if configuration.max_packet_size & (configuration.max_packet_size - 1) != 0 || configuration.max_packet_size < 8 || configuration.max_packet_size > 64 {
                    panic!("invalid max packet size for IN EP 0");
                }

                otg_hs_device.otg_hs_diepctl0.modify(|_, w| unsafe {
                    w
                        .mpsiz().bits(configuration.max_packet_size as u16)
                        .txfnum().bits(self.index)
                });
            }
            1 => {
                otg_hs_device.otg_hs_diepctl1.modify(|_, w| unsafe {
                    w
                        .usbaep().set_bit()
                        .mpsiz().bits(configuration.max_packet_size)
                        .txfnum().bits(self.index)
                        .sd0pid_sevnfrm().set_bit()
                        .eptyp().bits(ep_type_bits)
                });
            }
            2 => {
                otg_hs_device.otg_hs_diepctl2.modify(|_, w| unsafe {
                    w
                        .usbaep().set_bit()
                        .mpsiz().bits(configuration.max_packet_size)
                        .txfnum().bits(self.index)
                        .sd0pid_sevnfrm().set_bit()
                        .eptyp().bits(ep_type_bits)
                });
            }
            3 => {
                otg_hs_device.otg_hs_diepctl3.modify(|_, w| unsafe {
                    w
                        .usbaep().set_bit()
                        .mpsiz().bits(configuration.max_packet_size)
                        .txfnum().bits(self.index)
                        .sd0pid_sevnfrm().set_bit()
                        .eptyp().bits(ep_type_bits)
                });
            }
            4 => {
                otg_hs_device.otg_hs_diepctl4.modify(|_, w| unsafe {
                    w
                        .usbaep().set_bit()
                        .mpsiz().bits(configuration.max_packet_size)
                        .txfnum().bits(self.index)
                        .sd0pid_sevnfrm().set_bit()
                        .eptyp().bits(ep_type_bits)
                });
            }
            5 => {
                otg_hs_device.otg_hs_diepctl5.modify(|_, w| unsafe {
                    w
                        .usbaep().set_bit()
                        .mpsiz().bits(configuration.max_packet_size)
                        .txfnum().bits(self.index)
                        .sd0pid_sevnfrm().set_bit()
                        .eptyp().bits(ep_type_bits)
                });
            }
            _ => unreachable!()
        };
    }

    fn address(&self) -> EndpointAddress {
        EndpointAddress::from_parts(self.index as usize, UsbDirection::In)
    }
}

impl OutEndpoint {
    pub fn new(index: u8) -> OutEndpoint {
        OutEndpoint {
            index,
//            rx_buf: AtomicMutex::new(unsafe { slice::from_raw_parts_mut(DFIFO_BASE.offset((index as isize * 1024isize) as isize), 1) }),
            configuration: None,
        }
    }

    pub fn index(&self) -> u8 {
        self.index
    }

    pub fn read(&self, buf: &mut [u8], packet_length: usize) -> Result<usize> {
//        if packet_length > buf.write_u32() {
//            Err(UsbError::BufferOverflow)
//        }

        interrupt::free(|cs| {
            let mut data: u32 = 0;
//            let rx_buf = self.rx_buf.lock(cs);

            // packet length is in bytes
            for i in 0..packet_length {
                if i & 3 == 0 {
                    data = unsafe { ptr::read_volatile((DFIFO_BASE + ((self.index as u32) << 12)) as *const u32) };
//                    fiprintln!("read data: {:#034b}", data);
                }

                buf[i] = (data >> ((i & 3) << 3)) as u8;
//                fiprintln!("read byte {}: {:#010b}", i, buf[i]);
            }

            Ok(packet_length)
        })
    }
}

impl Endpoint for OutEndpoint {
    fn reset(&self, otg_hs_device: &OTG_HS_DEVICE) {
        otg_hs_device.otg_hs_daintmsk.modify(|r, w| unsafe { w.oepm().bits(r.oepm().bits() & !(1 << self.index)) });
//        otg_fs_device.diepempmsk.modify(|r, w| unsafe { w.ineptxfem().bits(r.ineptxfem().bits() & (1 << self.index)) }); // tx fifo empty interrupt mask

        match self.index {
            0 => {
                otg_hs_device.otg_hs_doepctl0.modify(|_, w| w.snak().set_bit());
                otg_hs_device.otg_hs_doeptsiz0.reset();
                otg_hs_device.otg_hs_doepint0.write(|w| unsafe { w.bits(0xFF) });
            }
            1 => {
                if otg_hs_device.otg_hs_doepctl1.read().epena().bit_is_set() {
                    otg_hs_device.otg_hs_doepctl1.write(|w| w.epdis().set_bit().snak().set_bit());
                }

                otg_hs_device.otg_hs_doeptsiz1.reset();
                otg_hs_device.otg_hs_doepint1.write(|w| unsafe { w.bits(0xFF) });
            }
            2 => {
                if otg_hs_device.otg_hs_doepctl2.read().epena().bit_is_set() {
                    otg_hs_device.otg_hs_doepctl2.write(|w| w.epdis().set_bit().snak().set_bit());
                }

                otg_hs_device.otg_hs_doeptsiz2.reset();
                otg_hs_device.otg_hs_doepint2.write(|w| unsafe { w.bits(0xFF) });
            }
            3 => {
                if otg_hs_device.otg_hs_doepctl3.read().epena().bit_is_set() {
                    otg_hs_device.otg_hs_doepctl3.write(|w| w.epdis().set_bit().snak().set_bit());
                }

                otg_hs_device.otg_hs_doeptsiz3.reset();
                otg_hs_device.otg_hs_doepint3.write(|w| unsafe { w.bits(0xFF) });
            }
            4 => {
                if otg_hs_device.otg_hs_doepctl4.read().epena().bit_is_set() {
                    otg_hs_device.otg_hs_doepctl4.write(|w| w.epdis().set_bit().snak().set_bit());
                }

                otg_hs_device.otg_hs_doeptsiz4.reset();
                otg_hs_device.otg_hs_doepint4.write(|w| unsafe { w.bits(0xFF) });
            }
            5 => {
                if otg_hs_device.otg_hs_doepctl5.read().epena().bit_is_set() {
                    otg_hs_device.otg_hs_doepctl5.write(|w| w.epdis().set_bit().snak().set_bit());
                }

                otg_hs_device.otg_hs_doeptsiz5.reset();
                otg_hs_device.otg_hs_doepint5.write(|w| unsafe { w.bits(0xFF) });
            }
            _ => unreachable!()
        }
    }

    fn enable(&self, otg_hs_device: &OTG_HS_DEVICE) {
        let configuration = self.configuration.as_ref().unwrap();

        let ep_type_bits = match configuration.ep_type {
            EndpointType::Control => 0,
            EndpointType::Isochronous => 1,
            EndpointType::Bulk => 2,
            EndpointType::Interrupt => 3
        };

        otg_hs_device.otg_hs_daintmsk.modify(|r, w| unsafe { w.oepm().bits(r.oepm().bits() | (1 << self.index)) });
        //otg_hs_device.otg_hs_diepempmsk.modify(|r, w| unsafe { w.ineptxfem().bits(r.ineptxfem().bits() | (1 << self.index)) });

        match self.index {
            0 => {}
            1 => {
                otg_hs_device.otg_hs_doepctl1.modify(|_, w| unsafe {
                    w
                        .usbaep().set_bit()
                        .mpsiz().bits(configuration.max_packet_size)
                        .sd0pid_sevnfrm().set_bit()
                        .eptyp().bits(ep_type_bits)
                });
            }
            2 => {
                otg_hs_device.otg_hs_doepctl2.modify(|_, w| unsafe {
                    w
                        .usbaep().set_bit()
                        .mpsiz().bits(configuration.max_packet_size)
                        .sd0pid_sevnfrm().set_bit()
                        .eptyp().bits(ep_type_bits)
                });
            }
            3 => {
                otg_hs_device.otg_hs_doepctl3.modify(|_, w| unsafe {
                    w
                        .usbaep().set_bit()
                        .mpsiz().bits(configuration.max_packet_size)
                        .sd0pid_sevnfrm().set_bit()
                        .eptyp().bits(ep_type_bits)
                });
            }
            4 => {
                otg_hs_device.otg_hs_doepctl4.modify(|_, w| unsafe {
                    w
                        .usbaep().set_bit()
                        .mpsiz().bits(configuration.max_packet_size)
                        .sd0pid_sevnfrm().set_bit()
                        .eptyp().bits(ep_type_bits)
                });
            }
            5 => {
                otg_hs_device.otg_hs_doepctl5.modify(|_, w| unsafe {
                    w
                        .usbaep().set_bit()
                        .mpsiz().bits(configuration.max_packet_size)
                        .sd0pid_sevnfrm().set_bit()
                        .eptyp().bits(ep_type_bits)
                });
            }
            _ => unreachable!()
        };
    }

    fn address(&self) -> EndpointAddress {
        EndpointAddress::from_parts(self.index as usize, UsbDirection::Out)
    }
}