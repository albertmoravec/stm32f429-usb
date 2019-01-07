use core::slice;
use vcell::VolatileCell;
use crate::atomic_mutex::AtomicMutex;
use crate::bus::UsbRegs;
use cortex_m::interrupt;
use usb_device::endpoint::{EndpointType, EndpointAddress};
use usb_device::{Result, UsbDirection};
use stm32f4xx_hal::stm32f4::stm32f429::OTG_FS_DEVICE;

type EndpointBuffer = &'static mut [VolatileCell<u32>];

pub const NUM_ENDPOINTS: usize = 4;
const DFIFO_BASE: *mut VolatileCell<u32> = 0x5000_1000 as *mut VolatileCell<u32>;

pub trait Endpoint {
    fn reset(&self, otg_fs_device: &OTG_FS_DEVICE);
    fn enable(&self, otg_fs_device: &OTG_FS_DEVICE);
    fn address(&self) -> EndpointAddress;
}

pub struct EndpointConfiguration {
    pub ep_type: EndpointType,
    pub max_packet_size: u16,
}

pub struct OutEndpoint {
    index: u8,
    rx_buf: AtomicMutex<EndpointBuffer>,
    pub configuration: Option<EndpointConfiguration>,
}

pub struct InEndpoint {
    index: u8,
    tx_buf: AtomicMutex<EndpointBuffer>,
    pub configuration: Option<EndpointConfiguration>,
}

impl InEndpoint {
    pub fn new(index: u8) -> InEndpoint {
        InEndpoint {
            index,
            tx_buf: AtomicMutex::new(unsafe { slice::from_raw_parts_mut(DFIFO_BASE.offset((index as isize) * 1024isize), 1) }),
            configuration: None,
        }
    }

    pub fn index(&self) -> u8 { self.index }

    pub fn write(&self, buf: &[u8]) -> Result<usize> {
        let mut written: usize = 0;
        let mut data: u32 = 0;

        interrupt::free(|cs| {
            let tx_buf = self.tx_buf.lock(cs);

            for (i, byte) in buf.iter().enumerate() {
                if i != 0 && i & 3 == 0 {
                    tx_buf[0].set(data);
                    data = 0;
                }

                data |= (byte << ((i & 3) << 3)) as u32;
                written += 1;
            }

            tx_buf[0].set(data);

            Ok(written)
        })
    }
}

impl Endpoint for InEndpoint {
    fn reset(&self, otg_fs_device: &OTG_FS_DEVICE) {
        match self.index {
            0 => {
                otg_fs_device.fs_diepctl0.modify(|_, w| w.snak().set_bit());
                otg_fs_device.dieptsiz0.reset();
                otg_fs_device.diepint0.write(|w| unsafe { w.bits(0xFF) });
            }
            1 => {
                if otg_fs_device.diepctl1.read().epena().bit_is_set() {
                    otg_fs_device.diepctl1.write(|w| w.epdis().set_bit().snak().set_bit());
                }

                otg_fs_device.dieptsiz1.reset();
                otg_fs_device.diepint1.write(|w| unsafe { w.bits(0xFF) });
            }
            2 => {
                if otg_fs_device.diepctl2.read().epena().bit_is_set() {
                    otg_fs_device.diepctl2.write(|w| w.epdis().set_bit().snak().set_bit());
                }

                otg_fs_device.dieptsiz2.reset();
                otg_fs_device.diepint2.write(|w| unsafe { w.bits(0xFF) });
            }
            3 => {
                if otg_fs_device.diepctl3.read().epena().bit_is_set() {
                    otg_fs_device.diepctl3.write(|w| w.epdis().set_bit().snak().set_bit());
                }

                otg_fs_device.dieptsiz3.reset();
                otg_fs_device.diepint3.write(|w| unsafe { w.bits(0xFF) });
            }
            _ => unreachable!()
        }
    }

    fn enable(&self, otg_fs_device: &OTG_FS_DEVICE) {
        let configuration = self.configuration.as_ref().unwrap();

        let ep_type_bits = match configuration.ep_type {
            EndpointType::Control => 0,
            EndpointType::Isochronous => 1,
            EndpointType::Bulk => 2,
            EndpointType::Interrupt => 3
        };

        otg_fs_device.fs_daintmsk.modify(|r, w| unsafe { w.iepm().bits(r.iepm().bits() | (1 << self.index)) });

        match self.index {
            0 => {
                if configuration.max_packet_size & (configuration.max_packet_size - 1) != 0 || configuration.max_packet_size < 8 || configuration.max_packet_size > 64 {
                    panic!("invalid max packet size for IN EP 0");
                }

                otg_fs_device.fs_diepctl0.modify(|_, w| unsafe {
                    w
                        .mpsiz().bits(configuration.max_packet_size as u8)
                        .txfnum().bits(self.index)
                });
            }
            1 => {
                otg_fs_device.diepctl1.modify(|_, w| unsafe {
                    w
                        .usbaep().set_bit()
                        .mpsiz().bits(configuration.max_packet_size)
                        .txfnum().bits(self.index)
                        .sd0pid_sevnfrm().set_bit()
                        .eptyp().bits(ep_type_bits)
                });
            }
            2 => {
                otg_fs_device.diepctl2.modify(|_, w| unsafe {
                    w
                        .usbaep().set_bit()
                        .mpsiz().bits(configuration.max_packet_size)
                        .txfnum().bits(self.index)
                        .sd0pid_sevnfrm().set_bit()
                        .eptyp().bits(ep_type_bits)
                });
            }
            3 => {
                otg_fs_device.diepctl3.modify(|_, w| unsafe {
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
            rx_buf: AtomicMutex::new(unsafe { slice::from_raw_parts_mut(DFIFO_BASE.offset((index as isize * 1024isize) as isize), 1) }),
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
            let rx_buf = self.rx_buf.lock(cs);

            // packet length is in bytes
            for i in 0..packet_length {
                if i & 3 == 0 {
                    data = rx_buf[0].get();
                }

                buf[i] = (data >> ((i & 3) << 3)) as u8;
            }

            Ok(packet_length)
        })
    }
}

impl Endpoint for OutEndpoint {
    fn reset(&self, otg_fs_device: &OTG_FS_DEVICE) {
        otg_fs_device.fs_daintmsk.modify(|r, w| unsafe { w.oepint().bits(r.oepint().bits() & !(1 << self.index)) });
//        otg_fs_device.diepempmsk.modify(|r, w| unsafe { w.ineptxfem().bits(r.ineptxfem().bits() & (1 << self.index)) }); // tx fifo empty interrupt mask

        match self.index {
            0 => {
                otg_fs_device.doepctl0.modify(|_, w| w.snak().set_bit());
                otg_fs_device.doeptsiz0.reset();
                otg_fs_device.doepint0.write(|w| unsafe { w.bits(0xFF) });
            }
            1 => {
                if otg_fs_device.doepctl1.read().epena().bit_is_set() {
                    otg_fs_device.doepctl1.write(|w| w.epdis().set_bit().snak().set_bit());
                }

                otg_fs_device.doeptsiz1.reset();
                otg_fs_device.doepint1.write(|w| unsafe { w.bits(0xFF) });
            }
            2 => {
                if otg_fs_device.doepctl2.read().epena().bit_is_set() {
                    otg_fs_device.doepctl2.write(|w| w.epdis().set_bit().snak().set_bit());
                }

                otg_fs_device.doeptsiz2.reset();
                otg_fs_device.doepint2.write(|w| unsafe { w.bits(0xFF) });
            }
            3 => {
                if otg_fs_device.doepctl3.read().epena().bit_is_set() {
                    otg_fs_device.doepctl3.write(|w| w.epdis().set_bit().snak().set_bit());
                }

                otg_fs_device.doeptsiz3.reset();
                otg_fs_device.doepint3.write(|w| unsafe { w.bits(0xFF) });
            }
            _ => unreachable!()
        }
    }

    fn enable(&self, otg_fs_device: &OTG_FS_DEVICE) {
        let configuration = self.configuration.as_ref().unwrap();

        let ep_type_bits = match configuration.ep_type {
            EndpointType::Control => 0,
            EndpointType::Isochronous => 1,
            EndpointType::Bulk => 2,
            EndpointType::Interrupt => 3
        };

        otg_fs_device.fs_daintmsk.modify(|r, w| unsafe { w.oepint().bits(r.oepint().bits() | (1 << self.index)) });
        otg_fs_device.diepempmsk.modify(|r, w| unsafe { w.ineptxfem().bits(r.ineptxfem().bits() | (1 << self.index)) });

        match self.index {
            0 => {}
            1 => {
                otg_fs_device.doepctl1.modify(|_, w| unsafe {
                    w
                        .usbaep().set_bit()
                        .mpsiz().bits(configuration.max_packet_size)
                        .sd0pid_sevnfrm().set_bit()
                        .eptyp().bits(ep_type_bits)
                });
            }
            2 => {
                otg_fs_device.doepctl2.modify(|_, w| unsafe {
                    w
                        .usbaep().set_bit()
                        .mpsiz().bits(configuration.max_packet_size)
                        .sd0pid_sevnfrm().set_bit()
                        .eptyp().bits(ep_type_bits)
                });
            }
            3 => {
                otg_fs_device.doepctl3.modify(|_, w| unsafe {
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