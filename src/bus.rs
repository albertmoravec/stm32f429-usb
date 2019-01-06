use core::mem;
use crate::helpers::*;
use crate::atomic_mutex::AtomicMutex;
use bare_metal::CriticalSection;
use cortex_m::interrupt;
use cortex_m::asm::delay;
use crate::endpoint::{Endpoint, InEndpoint, OutEndpoint, EndpointConfiguration, NUM_ENDPOINTS};
use usb_device::bus::UsbBus as UsbBusTrait;
use usb_device::bus::{PollResult, UsbBusAllocator};
use usb_device::endpoint::{EndpointAddress, EndpointType};
use usb_device::{UsbDirection, UsbError, Result};
use stm32f4xx_hal::stm32f4::stm32f429::{OTG_FS_DEVICE, OTG_FS_GLOBAL, OTG_FS_PWRCLK, otg_fs_global::FS_GRXSTSR_DEVICE, otg_fs_global::fs_grxstsr_device::{DPIDR, PKTSTSR}};
use stm32f4xx_hal::stm32f4::stm32f429::otg_fs_global::fs_grxstsr_device::R as GrxstsrReg;
use stm32f4xx_hal::gpio::{gpioa, gpiob, Alternate, AF12, AF10};
use cortex_m_semihosting::hprintln;

pub struct UsbBus {
    regs: AtomicMutex<UsbRegs>,
    gpio_pins: AtomicMutex<GpioPins>,
    in_endpoints: [InEndpoint; NUM_ENDPOINTS],
    out_endpoints: [OutEndpoint; NUM_ENDPOINTS],
    max_rx_packet_size: u16,
    waiting_read: AtomicMutex<WaitingPacketInfoWrapper>,
}

struct WaitingPacketInfoWrapper {
    //    waiting_packet_info: Option<WaitingPacketInfo>
    waiting_packet_info: Option<GrxstsrReg>
}

pub struct UsbRegs {
    otg_fs_global: OTG_FS_GLOBAL,
    otg_fs_device: OTG_FS_DEVICE,
    otg_fs_pwrclk: OTG_FS_PWRCLK,
}

pub struct GpioPins {
    dm: gpioa::PA11<Alternate<AF10>>,
    dp: gpioa::PA12<Alternate<AF10>>,
}

//struct WaitingPacketInfo {
//    frame_number: u8,
//    packet_status: PacketStatus,
//    data_pid: DataPID,
//    byte_count: usize,
//    endpoint_number: u8,
//}
//
//impl WaitingPacketInfo {
//    fn from_reg(reg: GrxstsrReg) -> WaitingPacketInfo {
//        WaitingPacketInfo {
//            frame_number: reg.frmnum().bits(),
//            packet_status: PacketStatus::from_pktsts_register(reg.pktsts()),
//            data_pid: DataPID::from_dpid_register(reg.dpid()),
//            byte_count: reg.bcnt().bits() as usize,
//            endpoint_number: reg.epnum().bits(),
//        }
//    }
//}
//
//enum DataPID {
//    DATA0 = 0,
//    DATA1 = 2,
//    DATA2 = 1,
//    MDATA = 3,
//}
//
//impl DataPID {
//    fn from_dpid_register(reg: DPIDR) -> DataPID {
//        match reg.bits() {
//            0 => DataPID::DATA0,
//            2 => DataPID::DATA1,
//            1 => DataPID::DATA2,
//            3 => DataPID::MDATA,
//            _ => unreachable!()
//        }
//    }
//}
//
//enum PacketStatus {
//    GlobalOutNAK = 1,
//    OutDataPacketReceived = 2,
//    OutTransferCompleted = 3,
//    SetupTransactionCompleted = 4,
//    SetupDataPacketReceived = 6,
//}
//
//impl PacketStatus {
//    fn from_pktsts_register(reg: PKTSTSR) -> PacketStatus {
//        match reg.bits() {
//            1 => PacketStatus::GlobalOutNAK,
//            2 => PacketStatus::OutDataPacketReceived,
//            3 => PacketStatus::OutTransferCompleted,
//            4 => PacketStatus::SetupTransactionCompleted,
//            6 => PacketStatus::SetupDataPacketReceived,
//            _ => unreachable!()
//        }
//    }
//}

impl UsbBus {
    pub fn new(otg_fs_global: OTG_FS_GLOBAL,
               otg_fs_device: OTG_FS_DEVICE,
               otg_fs_pwrclk: OTG_FS_PWRCLK,
               dm_pin: gpioa::PA11<Alternate<AF10>>,
               dp_pin: gpioa::PA12<Alternate<AF10>>,
    ) -> UsbBusAllocator<Self> {
        hprintln!("initializing usb").unwrap();
        let bus = UsbBus {
            regs: AtomicMutex::new(UsbRegs {
                otg_fs_global,
                otg_fs_device,
                otg_fs_pwrclk,
            }),
            gpio_pins: AtomicMutex::new(GpioPins {
                dm: dm_pin,
                dp: dp_pin,
            }),
            in_endpoints: unsafe {
                let mut in_ep: [InEndpoint; NUM_ENDPOINTS] = mem::uninitialized();

                for i in 0..NUM_ENDPOINTS {
                    in_ep[i] = InEndpoint::new(i as u8)
                }

                in_ep
            },
            out_endpoints: unsafe {
                let mut out_ep: [OutEndpoint; NUM_ENDPOINTS] = mem::uninitialized();

                for i in 0..NUM_ENDPOINTS {
                    out_ep[i] = OutEndpoint::new(i as u8);
                }

                out_ep
            },
            max_rx_packet_size: 0,
            waiting_read: AtomicMutex::new(WaitingPacketInfoWrapper { waiting_packet_info: None }),
        };

        hprintln!("calling usb bus allocator").unwrap();

        let usbbus = UsbBusAllocator::new(bus);
        hprintln!("usb bus allocator called").unwrap();

        usbbus
    }

    // count of 32-bit blocks in fifo (1280 B)
    const FIFO_SIZE: u16 = 320;

    fn disable_global_interrupt(&self, regs: &UsbRegs) {
        regs.otg_fs_global.fs_gahbcfg.modify(|_, w| w.gint().clear_bit());
    }

    fn enable_global_interrupt(&self, regs: &UsbRegs) {
        regs.otg_fs_global.fs_gahbcfg.modify(|_, w| w.gint().set_bit());
    }

    fn init_rx_buffer(&self, regs: &UsbRegs) {
        let enabled_out_ep_count = self.out_endpoints.iter().filter(|ep| ep.configuration.is_some()).count();

        let buf_size_raw = 10 + 1 + (2 * (self.max_rx_packet_size / 4)) + 1 + (enabled_out_ep_count as u16);
        let buf_size = clamp(16u16, buf_size_raw, 512u16);

        regs.otg_fs_global.fs_grxfsiz.write(|w| unsafe { w.rxfd().bits(buf_size) });

        self.flush_rx_fifo(regs);
    }

    fn init_tx_buffers(&self, regs: &UsbRegs) {
        self.in_endpoints.iter().filter(|ep| ep.configuration.is_some()).for_each(|ep| {
            let buf_size = clamp(16, (ep.configuration.as_ref().unwrap().max_packet_size + 3) / 4, 256);
        });
        for ep in self.in_endpoints.iter().filter(|ep| ep.configuration.is_some()) {
            // fixme: 256 should be maxuimum only for EP0, leaving that for other EPs as well for now
            let buf_size = clamp(16, (ep.configuration.as_ref().unwrap().max_packet_size + 3) / 4, 256);

            let fifo_head = self.calculate_fifo_head(regs, ep.index());

            unsafe {
                match ep.index() {
                    0 => {
                        regs.otg_fs_global.fs_gnptxfsiz_device.write(|w| w.tx0fd().bits(buf_size).tx0fsa().bits(fifo_head));
                        regs.otg_fs_device.fs_diepctl0.modify(|_, w| w.txfnum().bits(ep.index()))
                    }
                    1 => {
                        regs.otg_fs_global.fs_dieptxf1.write(|w| w.ineptxfd().bits(buf_size).ineptxsa().bits(fifo_head));
                        regs.otg_fs_device.diepctl1.modify(|_, w| w.txfnum().bits(ep.index()))
                    }
                    2 => {
                        regs.otg_fs_global.fs_dieptxf2.write(|w| w.ineptxfd().bits(buf_size).ineptxsa().bits(fifo_head));
                        regs.otg_fs_device.diepctl2.modify(|_, w| w.txfnum().bits(ep.index()))
                    }
                    3 => {
                        regs.otg_fs_global.fs_dieptxf3.write(|w| w.ineptxfd().bits(buf_size).ineptxsa().bits(fifo_head));
                        regs.otg_fs_device.diepctl3.modify(|_, w| w.txfnum().bits(ep.index()))
                    }
                    _ => unreachable!()
                }
            }
        }

        self.flush_tx_fifos(regs);
    }

    fn calculate_fifo_head(&self, regs: &UsbRegs, next_index: u8) -> u16 {
        let mut size = regs.otg_fs_global.fs_grxfsiz.read().rxfd().bits();

        // this expects that fifos are ordered and span continuous block of memory
        for i in 0..next_index {
            size += match i {
                0 => regs.otg_fs_global.fs_gnptxfsiz_device.read().tx0fd().bits(),
                1 => regs.otg_fs_global.fs_dieptxf1.read().ineptxfd().bits(),
                2 => regs.otg_fs_global.fs_dieptxf2.read().ineptxfd().bits(),
                3 => regs.otg_fs_global.fs_dieptxf3.read().ineptxfd().bits(),
                _ => unreachable!()
            };
        }

        size
    }

    // todo: check if core is not using the fifo before flushing
    fn flush_tx_fifos(&self, regs: &UsbRegs) {
        regs.otg_fs_global.fs_grstctl.modify(|_, w| unsafe { w.txfnum().bits(0b10000).txfflsh().set_bit() });

        hprintln!("waiting for tx fifos to flush").unwrap();

        while regs.otg_fs_global.fs_grstctl.read().txfflsh().bit_is_set() {}

        delay(60);
    }

    fn flush_tx_fifo(&self, regs: &UsbRegs, ep_num: u8) {
        regs.otg_fs_global.fs_grstctl.modify(|_, w| unsafe { w.txfnum().bits(ep_num).txfflsh().set_bit() });

        hprintln!("waiting for tx fifo to flush").unwrap();

        while regs.otg_fs_global.fs_grstctl.read().txfflsh().bit_is_set() {}

        delay(60);
    }

    fn flush_rx_fifo(&self, regs: &UsbRegs) {
        regs.otg_fs_global.fs_grstctl.modify(|_, w| w.rxfflsh().set_bit());

        hprintln!("waiting for rx fifo to flush").unwrap();

        while regs.otg_fs_global.fs_grstctl.read().rxfflsh().bit_is_set() {}

        delay(60);
    }

    fn reset_core(&self, regs: &UsbRegs) {
        hprintln!("waiting for ahb to become idle").unwrap();
        // wait for AHB to become idle
        while regs.otg_fs_global.fs_grstctl.read().ahbidl().bit_is_clear() {}

        hprintln!("resetting core").unwrap();
        regs.otg_fs_global.fs_grstctl.modify(|_, w| w.csrst().set_bit());

        hprintln!("waiting for core to reset").unwrap();
        while regs.otg_fs_global.fs_grstctl.read().csrst().bit_is_set() {}
    }

    fn init_phy(&self, regs: &UsbRegs) {
        // this probably has no effect
        regs.otg_fs_global.fs_gusbcfg.write(|w| w.physel().set_bit());

        self.reset_core(regs);

        // activate transceiver
        regs.otg_fs_global.fs_gccfg.modify(|_, w| w.pwrdwn().set_bit());

        delay(60);
    }

    fn init_core(&self, regs: &UsbRegs) {
        self.init_phy(regs);

        // DMA and OTG init goes there once possible
    }

    fn init_device_core(&self, regs: &UsbRegs) {
        // Restart the Phy Clock
        // fixme: that's how ST does it... but what?!
        regs.otg_fs_pwrclk.fs_pcgcctl.modify(|_, w| unsafe { w.bits(0) });

        // periodic frame interval = 80% & device speed = FS
        regs.otg_fs_device.fs_dcfg.modify(|_, w| unsafe { w.pfivl().bits(0b0).dspd().bits(3) });

        self.init_rx_buffer(regs);
        self.init_tx_buffers(regs);

        hprintln!("clearing pending device interrupts").unwrap();
        self.clear_pending_device_interrupts(regs);

        hprintln!("resetting in").unwrap();
        self.in_endpoints.iter().for_each(|ep| ep.reset(&regs.otg_fs_device));
        hprintln!("resetting out").unwrap();
        self.out_endpoints.iter().for_each(|ep| ep.reset(&regs.otg_fs_device));

        hprintln!("enabling in endpoints").unwrap();
        self.in_endpoints.iter()
            .filter(|ep| ep.configuration.is_some())
            .for_each(|ep| ep.enable(&regs.otg_fs_device));

        hprintln!("enabling out endpoints").unwrap();
        self.out_endpoints.iter()
            .filter(|ep| ep.configuration.is_some())
            .for_each(|ep| ep.enable(&regs.otg_fs_device));

        hprintln!("enabling device interrupts").unwrap();
        self.enable_device_interrupts(regs);
    }

    fn set_device_mode(&self, regs: &UsbRegs) {
        regs.otg_fs_global.fs_gusbcfg.modify(|_, w| w.fdmod().set_bit());

        // 50ms delay
        delay(50 * 12 * 1000);
    }

    fn clear_pending_device_interrupts(&self, regs: &UsbRegs) {
        regs.otg_fs_device.fs_diepmsk.reset();
        regs.otg_fs_device.fs_doepmsk.reset();
//        regs.otg_fs_device.fs_daint.write(|w| unsafe { w.bits(0xFFFFFFFF) });
        regs.otg_fs_device.fs_daintmsk.reset();
    }

    fn enable_device_interrupts(&self, regs: &UsbRegs) {
        regs.otg_fs_global.fs_gintmsk.write(|w| unsafe { w.bits(0) });
        regs.otg_fs_global.fs_gintsts.write(|w| unsafe { w.bits(0xBFFFFFFF) });

        regs.otg_fs_global.fs_gintmsk.write(|w| w
            .wuim().set_bit()
            .rxflvlm().set_bit()
            .usbsuspm().set_bit()
            .usbrst().set_bit()
//            .enumdnem().set_bit()
            .iepint().set_bit()
//            .oepint().set_bit()
        );

        regs.otg_fs_device.fs_diepmsk.write(|w| w.xfrcm().set_bit());
//        regs.otg_fs_device.fs_doepmsk.write(|w| w.xfrcm().set_bit());
    }

    fn read_packet_status(&self, _cs: &CriticalSection) -> Option<GrxstsrReg> {
        let grxstsr = &self.regs.lock(_cs).otg_fs_global.fs_grxstsr_device;
        let grxstsp = unsafe {
            let grxstsr_ptr = grxstsr as *const FS_GRXSTSR_DEVICE;

            &*grxstsr_ptr.offset(1)
        };

        loop {
            let packet_status = grxstsp.read();

            // fifo is empty (can also be checked using RXFLVL interrupt bit)
            if packet_status.bits() == 0 {
                break None;
            }

            // setup or data packet received
            if packet_status.pktsts().bits() == 2 || packet_status.pktsts().bits() == 6 {
                break Some(packet_status);
            }
        }
    }

    fn update_packet_status(&self, _cs: &CriticalSection) {
        let mut container = self.waiting_read.lock(_cs);

        if container.waiting_packet_info.is_none() {
            container.waiting_packet_info = self.read_packet_status(_cs);
        }
    }

    fn enable_stall(&self, regs: &UsbRegs, ep_addr: EndpointAddress) {
        // todo: do it properly, should wait for epdis to take effect and flush tx fifo

        match (ep_addr.direction(), ep_addr.index()) {
            (UsbDirection::In, 0) => regs.otg_fs_device.fs_diepctl0.modify(|_, w| w.stall().set_bit()),
            (UsbDirection::In, 1) => regs.otg_fs_device.diepctl1.modify(|_, w| w.epdis().set_bit().stall().set_bit()),
            (UsbDirection::In, 2) => regs.otg_fs_device.diepctl1.modify(|_, w| w.epdis().set_bit().stall().set_bit()),
            (UsbDirection::In, 3) => regs.otg_fs_device.diepctl1.modify(|_, w| w.epdis().set_bit().stall().set_bit()),
            (UsbDirection::Out, 0) => regs.otg_fs_device.doepctl0.modify(|_, w| w.stall().set_bit()),
            (UsbDirection::Out, 1) => regs.otg_fs_device.doepctl1.modify(|_, w| w.epdis().set_bit().stall().set_bit()),
            (UsbDirection::Out, 2) => regs.otg_fs_device.doepctl1.modify(|_, w| w.epdis().set_bit().stall().set_bit()),
            (UsbDirection::Out, 3) => regs.otg_fs_device.doepctl1.modify(|_, w| w.epdis().set_bit().stall().set_bit()),
            (_, _) => unreachable!()
        };

        if ep_addr.direction() == UsbDirection::In {
            self.flush_tx_fifo(regs, ep_addr.index() as u8);
        }
    }

    fn disable_stall(&self, regs: &UsbRegs, ep_addr: EndpointAddress) {
        match (ep_addr.direction(), ep_addr.index()) {
            (UsbDirection::In, 0) => regs.otg_fs_device.fs_diepctl0.modify(|_, w| w.stall().clear_bit()),
            (UsbDirection::In, 1) => regs.otg_fs_device.diepctl1.modify(|_, w| w.stall().clear_bit().sd0pid_sevnfrm().set_bit()),
            (UsbDirection::In, 2) => regs.otg_fs_device.diepctl1.modify(|_, w| w.stall().clear_bit().sd0pid_sevnfrm().set_bit()),
            (UsbDirection::In, 3) => regs.otg_fs_device.diepctl1.modify(|_, w| w.stall().clear_bit().sd0pid_sevnfrm().set_bit()),
            (UsbDirection::Out, 0) => regs.otg_fs_device.doepctl0.modify(|_, w| w.stall().clear_bit()),
            (UsbDirection::Out, 1) => regs.otg_fs_device.doepctl1.modify(|_, w| w.stall().clear_bit().sd0pid_sevnfrm().set_bit()),
            (UsbDirection::Out, 2) => regs.otg_fs_device.doepctl1.modify(|_, w| w.stall().clear_bit().sd0pid_sevnfrm().set_bit()),
            (UsbDirection::Out, 3) => regs.otg_fs_device.doepctl1.modify(|_, w| w.stall().clear_bit().sd0pid_sevnfrm().set_bit()),
            (_, _) => unreachable!()
        };
    }

    fn next_free_endpoint(&self, ep_dir: UsbDirection) -> Option<EndpointAddress> {
        match ep_dir {
            UsbDirection::In =>
                self.in_endpoints.iter()
                    .find(|ep| ep.configuration.is_none())
                    .and_then(|ep| Some(EndpointAddress::from_parts(ep.index() as usize, ep_dir))),
            UsbDirection::Out =>
                self.out_endpoints.iter()
                    .find(|ep| ep.configuration.is_none())
                    .and_then(|ep| Some(EndpointAddress::from_parts(ep.index() as usize, ep_dir)))
        }
    }
}

impl UsbBusTrait for UsbBus {
    /// Allocates an endpoint and specified endpoint parameters. This method is called by the device
    /// and class implementations to allocate endpoints, and can only be called before
    /// [`enable`](UsbBus::enable) is called.
    ///
    /// # Arguments
    ///
    /// * `ep_dir` - The endpoint direction.
    /// * `ep_addr` - A static endpoint address to allocate. If Some, the implementation should
    ///   attempt to return an endpoint with the specified address. If None, the implementation
    ///   should return the next available one.
    /// * `max_packet_size` - Maximum packet size in bytes.
    /// * `interval` - Polling interval parameter for interrupt endpoints.
    ///
    /// # Errors
    ///
    /// * [`EndpointOverflow`](crate::UsbError::EndpointOverflow) - Available total number of
    ///   endpoints, endpoints of the specified type, or endpoind packet memory has been exhausted.
    ///   This is generally caused when a user tries to add too many classes to a composite device.
    /// * [`InvalidEndpoint`](crate::UsbError::InvalidEndpoint) - A specific `ep_addr` was specified
    ///   but the endpoint in question has already been allocated.
    fn alloc_ep(
        &mut self,
        ep_dir: UsbDirection,
        ep_addr: Option<EndpointAddress>,
        ep_type: EndpointType,
        max_packet_size: u16,
        interval: u8) -> Result<EndpointAddress> {
        if ep_addr.is_some() && ep_addr.unwrap().index() > 3 {
            return Err(UsbError::EndpointOverflow);
        }

        interrupt::free(|cs| {
            let regs = self.regs.lock(cs);

            let result = match (ep_addr, ep_dir) {
                (None, _) => self.next_free_endpoint(ep_dir).ok_or(UsbError::EndpointOverflow),
                (Some(addr), UsbDirection::In) => {
                    let ep = &mut self.in_endpoints[addr.index()];

                    match ep.configuration {
                        Some(_) => Err(UsbError::InvalidEndpoint),
                        None => {
                            ep.configuration.replace(EndpointConfiguration { ep_type, max_packet_size });
                            Ok(addr)
                        }
                    }
                }
                (Some(addr), UsbDirection::Out) => {
                    let ep = &mut self.out_endpoints[addr.index()];

                    match ep.configuration {
                        Some(_) => Err(UsbError::InvalidEndpoint),
                        None => {
                            ep.configuration.replace(EndpointConfiguration { ep_type, max_packet_size });
                            Ok(addr)
                        }
                    }
                }
            };

            if result.is_ok() && ep_dir == UsbDirection::Out && max_packet_size > self.max_rx_packet_size {
                self.max_rx_packet_size = max_packet_size;
            }

            result
        })
    }

    /// Enables and initializes the USB peripheral. Soon after enabling the device will be reset, so
    /// there is no need to perform a USB reset in this method.
    fn enable(&mut self) {
        interrupt::free(|cs| {
            let regs = self.regs.lock(cs);

            hprintln!("disabling global interrupt").unwrap();
            self.disable_global_interrupt(&regs);
            hprintln!("initializing core").unwrap();

            self.init_core(&regs);
            hprintln!("setting device mode").unwrap();

            self.set_device_mode(&regs);
            hprintln!("initializing device core").unwrap();

            self.init_device_core(&regs);
            hprintln!("enabling global interrupt").unwrap();

            self.enable_global_interrupt(&regs);
        });
    }

    /// Performs a USB reset. This method should reset the platform-specific peripheral as well as
    /// ensure that all endpoints previously allocate with alloc_ep are initialized as specified.
    fn reset(&self) {}

    /// Sets the device USB address to `addr`.
    fn set_device_address(&self, addr: u8) {}

    /// Writes a single packet of data to the specified endpoint and returns number of bytes
    /// actually written.
    ///
    /// The only reason for a short write is if the caller passes a slice larger than the amount of
    /// memory allocated earlier, and this is generally an error in the class implementation.
    ///
    /// # Errors
    ///
    /// * [`InvalidEndpoint`](crate::UsbError::InvalidEndpoint) - The `ep_addr` does not point to a
    ///   valid endpoint that was previously allocated with [`UsbBus::alloc_ep`].
    /// * [`WouldBlock`](crate::UsbError::WouldBlock) - A previously written packet is still pending
    ///   to be sent.
    /// * [`BufferOverflow`](crate::UsbError::BufferOverflow) - The packet is too long to fit in the
    ///   transmission buffer. This is generally an error in the class implementation, because the
    ///   class shouldn't provide more data than the `max_packet_size` it specified when allocating
    ///   the endpoint.
    ///
    /// Implementations may also return other errors if applicable.
    fn write(&self, ep_addr: EndpointAddress, buf: &[u8]) -> Result<usize> {
        if ep_addr.direction() == UsbDirection::Out {
            return Err(UsbError::InvalidEndpoint);
        }

        let ep = &self.in_endpoints[ep_addr.index()];

        if ep.configuration.is_none() {
            return Err(UsbError::InvalidEndpoint);
        }

        if buf.len() > ep.configuration.as_ref().unwrap().max_packet_size as usize {
            return Err(UsbError::BufferOverflow);
        }

        // this needs a critical section(?)
        if buf.len() != 0 {
            let mut buffer_available: usize = 0;

            interrupt::free(|cs| {
                let regs = self.regs.lock(cs);

                buffer_available = match ep.index() {
                    0 => regs.otg_fs_device.dtxfsts0.read().ineptfsav().bits() as usize,
                    1 => regs.otg_fs_device.dtxfsts1.read().ineptfsav().bits() as usize,
                    2 => regs.otg_fs_device.dtxfsts2.read().ineptfsav().bits() as usize,
                    3 => regs.otg_fs_device.dtxfsts3.read().ineptfsav().bits() as usize,
                    _ => unreachable!()
                };
            });


            if (buf.len() + 3) / 4 > buffer_available {
                return Err(UsbError::WouldBlock);
            }
        }

        interrupt::free(|cs| {
            let regs = self.regs.lock(cs);

            unsafe {
                match ep.index() {
                    0 => {
                        regs.otg_fs_device.dieptsiz0.write(|w| w.xfrsiz().bits(buf.len() as u8));
                        // fixme EP0 EPENA read-only for some reason, hope this works
                        regs.otg_fs_device.fs_diepctl0.modify(|r, w| w.cnak().set_bit().bits(r.bits() | (1 << 31)))
                    }
                    1 => {
                        regs.otg_fs_device.dieptsiz1.write(|w| w.pktcnt().bits(1).xfrsiz().bits(buf.len() as u32));
                        regs.otg_fs_device.diepctl1.modify(|_, w| w.cnak().set_bit().epena().set_bit());
                    }
                    2 => {
                        regs.otg_fs_device.dieptsiz2.write(|w| w.pktcnt().bits(1).xfrsiz().bits(buf.len() as u32));
                        regs.otg_fs_device.diepctl2.modify(|_, w| w.cnak().set_bit().epena().set_bit());
                    }
                    3 => {
                        regs.otg_fs_device.dieptsiz3.write(|w| w.pktcnt().bits(1).xfrsiz().bits(buf.len() as u32));
                        regs.otg_fs_device.diepctl3.modify(|_, w| w.cnak().set_bit().epena().set_bit());
                    }
                    _ => unreachable!()
                };
            }
        });

        ep.write(buf)
    }

    /// Reads a single packet of data from the specified endpoint and returns the actual length of
    /// the packet.
    ///
    /// This should also clear any NAK flags and prepare the endpoint to receive the next packet.
    ///
    /// # Errors
    ///
    /// * [`InvalidEndpoint`](crate::UsbError::InvalidEndpoint) - The `ep_addr` does not point to a
    ///   valid endpoint that was previously allocated with [`UsbBus::alloc_ep`].
    /// * [`WouldBlock`](crate::UsbError::WouldBlock) - There is no packet to be read. Note that
    ///   this is different from a received zero-length packet, which is valid in USB. A zero-length
    ///   packet will return `Ok(0)`.
    /// * [`BufferOverflow`](crate::UsbError::BufferOverflow) - The received packet is too long to
    ///   fit in `buf`. This is generally an error in the class implementation, because the class
    ///   should use a buffer that is large enough for the `max_packet_size` it specified when
    ///   allocating the endpoint.
    ///
    /// Implementations may also return other errors if applicable.
    fn read(&self, ep_addr: EndpointAddress, buf: &mut [u8]) -> Result<usize> {
        if ep_addr.direction() == UsbDirection::In {
            return Err(UsbError::InvalidEndpoint);
        }

        let rx_not_empty = interrupt::free(|cs| self.regs.lock(cs).otg_fs_global.fs_gintsts.read().rxflvl().bit_is_clear());

        if !rx_not_empty {
            return Err(UsbError::WouldBlock);
        }

        let ep = &self.out_endpoints[ep_addr.index()];

        let byte_count = interrupt::free(|cs| {
            self.update_packet_status(cs);

            self.waiting_read.lock(cs).waiting_packet_info.as_ref()
                .ok_or(UsbError::WouldBlock)
                .and_then(|pi| {
                    if pi.epnum().bits() != ep_addr.index() as u8 {
                        return Err(UsbError::WouldBlock);
                    }

                    Ok(pi.bcnt().bits().clone())
                })
        });

        let result = byte_count.and_then(|bc| ep.read(buf, bc as usize));

        interrupt::free(|cs| self.waiting_read.lock(cs).waiting_packet_info = None);

        result
    }

    /// Sets or clears the STALL condition for an endpoint. If the endpoint is an OUT endpoint, it
    /// should be prepared to receive data again.
    fn set_stalled(&self, ep_addr: EndpointAddress, stalled: bool) {
        // todo: allow stalling control EP

        interrupt::free(|cs| {
            let regs = self.regs.lock(cs);

            match stalled {
                true => self.enable_stall(&regs, ep_addr),
                false => self.disable_stall(&regs, ep_addr)
            }
        });
    }

    /// Gets whether the STALL condition is set for an endpoint.
    fn is_stalled(&self, ep_addr: EndpointAddress) -> bool {
        interrupt::free(|cs| {
            let regs = self.regs.lock(cs);

            match (ep_addr.direction(), ep_addr.index()) {
                (UsbDirection::In, 0) | (UsbDirection::Out, 0) => false,
                (UsbDirection::In, 1) => regs.otg_fs_device.diepctl1.read().stall().bit(),
                (UsbDirection::In, 2) => regs.otg_fs_device.diepctl1.read().stall().bit(),
                (UsbDirection::In, 3) => regs.otg_fs_device.diepctl1.read().stall().bit(),
                (UsbDirection::Out, 1) => regs.otg_fs_device.doepctl1.read().stall().bit(),
                (UsbDirection::Out, 2) => regs.otg_fs_device.doepctl1.read().stall().bit(),
                (UsbDirection::Out, 3) => regs.otg_fs_device.doepctl1.read().stall().bit(),
                (_, _) => unreachable!()
            }
        })
    }

    /// Causes the USB peripheral to enter USB suspend mode, lowering power consumption and
    /// preparing to detect a USB wakeup event. This will be called after
    /// [`poll`](crate::device::UsbDevice::poll) returns [`PollResult::Suspend`]. The device will
    /// continue be polled, and it shall return a value other than `Suspend` from `poll` when it no
    /// longer detects the suspend condition.
    fn suspend(&self) {}

    /// Resumes from suspend mode. This may only be called after the peripheral has been previously
    /// suspended.
    fn resume(&self) {}

    /// Gets information about events and incoming data. Usually called in a loop or from an
    /// interrupt handler. See the [`PollResult`] struct for more information.
    fn poll(&self) -> PollResult {
        hprintln!("poll called, reading interrupts").unwrap();

        interrupt::free(|cs| {
            let regs = self.regs.lock(cs);
            let interrupts = &regs.otg_fs_global.fs_gintsts;
            let current = &interrupts.read();

            if current.usbrst().bit_is_set() {
                interrupts.modify(|_, w| w.usbrst().clear_bit());
                hprintln!("reset received").unwrap();
                return PollResult::Reset;
            }

            if current.rxflvl().bit_is_set() {
                self.update_packet_status(cs);

                match &self.waiting_read.lock(cs).waiting_packet_info {
                    Some(info) => {
                        if info.pktsts().bits() == 6 {
                            hprintln!("setup packet received").unwrap();

                            return PollResult::Data {
                                ep_setup: (1 << info.epnum().bits()) as u16,
                                ep_in_complete: 0,
                                ep_out: 0,
                            };
                        }

                        if info.pktsts().bits() == 2 {
                            hprintln!("data packet received").unwrap();

                            return PollResult::Data {
                                ep_out: (1 << info.epnum().bits()) as u16,
                                ep_in_complete: 0,
                                ep_setup: 0,
                            };
                        }
                    }
                    None => ()
                }

                if current.iepint().bit_is_set() {
                    // transfer complete interrupt hit
                    // fixme: this should check if it actually did

                    let ep_bits = regs.otg_fs_device.fs_daint.read().iepint().bits();

                    for i in 0..4 {
                        if (ep_bits & (1 << i)) != 0 {
                            match i {
                                0 => regs.otg_fs_device.diepint0.reset(),
                                1 => regs.otg_fs_device.diepint1.reset(),
                                2 => regs.otg_fs_device.diepint2.reset(),
                                3 => regs.otg_fs_device.diepint3.reset(),
                                _ => unreachable!()
                            }
                        }
                    }

                    hprintln!("transfer complete").unwrap();

                    return PollResult::Data {
                        ep_in_complete: ep_bits,
                        ep_out: 0,
                        ep_setup: 0,
                    };
                }
            }

            if current.usbsusp().bit_is_set() {
                interrupts.modify(|_, w| w.usbsusp().clear_bit());
                hprintln!("suspend received").unwrap();
                return PollResult::Suspend;
            }

            if current.wkupint().bit_is_set() {
                interrupts.modify(|_, w| w.wkupint().clear_bit());
                hprintln!("wakeup received").unwrap();
                return PollResult::Resume;
            }

            hprintln!("no event").unwrap();

            PollResult::None
        })
    }

    /// Simulates a disconnect from the USB bus, causing the host to reset and re-enumerate the
    /// device.
    ///
    /// Mostly used for development. By calling this at the start of your program ensures that the
    /// host re-enumerates your device after a new program has been flashed.
    ///
    /// The default implementation just returns `Unsupported`.
    ///
    /// # Errors
    ///
    /// * [`Unsupported`](crate::UsbError::Unsupported) - This UsbBus implementation doesn't support
    ///   simulating a disconnect or it has not been enabled at creation time.
    fn force_reset(&self) -> Result<()> {
        Err(UsbError::Unsupported)
    }
}