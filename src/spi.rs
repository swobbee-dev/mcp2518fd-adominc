use core::fmt::Debug;

use bitfield::bitfield;
use embedded_can::{Id, StandardId};

use embedded_hal::delay::DelayNs;
use embedded_hal::spi::Operation;
use embedded_hal::spi::SpiDevice;

use crate::memory::chip::{IoControlRegister, OscillatorControlRegister};
use crate::memory::controller::configuration::{
    CanControlRegister, DataBitTimeConfigurationRegister, NominalBitTimeConfigurationRegister,
    OperationMode, TimeStampControlRegister, TransmitterDelayCompensationMode,
    TransmitterDelayCompensationRegister,
};
use crate::memory::controller::fifo::{
    FifoControlRegister, FifoNumber, FifoStatusRegister, TxEventFifoControlRegister,
    TxEventFifoStatusRegister, TxQueueControlRegister, TxQueueStatusRegister, UserAddressKind,
    UserAddressRegister,
};
use crate::memory::controller::filter::{
    FilterControlRegister, FilterNumber, FilterObjectRegister, MaskRegister,
};
use crate::memory::controller::interrupt::{
    InterruptCodeRegister, InterruptRegister, RxInterruptStatusRegister,
    RxOverflowInterruptStatusRegister, TxAttemptInterruptStatusRegister, TxInterruptStatusRegister,
};
use crate::memory::{is_valid_ram_address, Register, RepeatedRegister, SFRAddress, RAM_BASE_ADDRESS};
use crate::message::rx::{RxHeader, RxMessage};
use crate::message::tx::{TxEventObject, TxHeader, TxMessage};
use crate::message::{len_for_dlc, MAX_FD_BUFFER_SIZE};
use crate::settings::{
    self, BitTimeConfiguration, FilterConfiguration, FilterMatchMode, RxFifoConfiguration,
    TxFifoConfiguration,
};
use crate::settings::{
    FifoConfiguration, IoConfiguration, OscillatorConfiguration, Pll, SysClkDivider,
    TxEventFifoConfiguration, TxQueueConfiguration,
};

/// Helper to round up SPI transfer sizes to align with 4-byte read/writes.
/// Only rounds up when needed (e.g., 1-4 rounds up to 4, 5-8 rounds up to 8).
fn round_up_spi_transfer_size(data_length: usize) -> usize {
    if data_length % 4 == 0 {
        data_length
    } else {
        // Add the number of bytes needed to round up to the next multiple of 4
        data_length + (4 - data_length % 4)
    }
}

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// Failed to read from the chip over SPI
    SPIRead,
    /// Failed to write to the chip over SPI
    SPIWrite,
    /// Attempted to access an invalid RAM address
    InvalidRamAddress(u16),
    /// Tried to read data from ram that was not a multiple of 4 bytes
    InvalidReadLength(usize),
    /// Tried to write data to ram that was not a multiple of 4 bytes
    InvalidWriteLength(usize),
    /// Tried to transmit a message through the TXQ, but the TXQ is not enabled
    TxQueueDisabled,
    /// Tried to transmit a message with a FIFO not configured for transmission
    FifoNotTx,
    /// Tried to send a message that was too big for the FIFO
    FifoTooSmall,
    /// FIFO is already full and can not take any more messages
    FifoFull,
    /// Tried to read a message from a FIFO not configured for reception
    FifoNotRx,
    Other,
}

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ConfigError {
    ChangeOpModeTimeout,
    ConfigurationModeTimeout,
    SPIFailedRAMEcho,
    PLLNotReady,
    Other(Error),
}

impl From<Error> for ConfigError {
    fn from(error: Error) -> Self {
        match error {
            Error::SPIRead | Error::SPIWrite => ConfigError::ConfigurationModeTimeout,
            _ => ConfigError::Other(error),
        }
    }
}

pub struct MCP2518FD<SPI> {
    spi: SPI,
}

impl<SPI, SPIE> MCP2518FD<SPI>
where
    SPI: SpiDevice<u8, Error = SPIE>,
    SPIE: Debug,
{
    /// Constructs a new MCP2518FD controller from an SPI bus and CS GPIO pin
    pub fn new(spi: SPI) -> MCP2518FD<SPI> {
        Self { spi }
    }

    /// Releases ownership of the SPI resources
    pub fn free(self) -> SPI {
        self.spi
    }

    /// Performs a software reset of the MCP2518FD chip over SPI (this puts it
    /// in configuration mode)
    pub fn reset(&mut self) -> Result<(), Error> {
        let instruction = Instruction(OpCode::RESET);

        self.spi
            .write(&instruction.0.to_be_bytes())
                        .map_err(|_| Error::SPIWrite)?;

        Ok(())
    }

    /// Does a full configuration sequence of the chip using the provided
    /// settings. This function puts the chip into configuration mode if it
    /// isn't already, verifies that SPI communication with the chip is
    /// working, and writes to all the necessary configuration registers.
    ///
    /// You may want to reset the chip before calling this method. See
    /// [`MCP2518FD::reset`] for more information.
    ///
    /// The data_bits_to_match field must be within 1..=18 if it is `Some`. A value of Some(0) will be interpreted the same as None, and
    pub fn configure(
        &mut self,
        settings: settings::Settings,
        delay: &mut impl DelayNs,
    ) -> Result<(), ConfigError> {
        self.set_op_mode(OperationMode::Configuration, delay)
                        .map_err(|_| ConfigError::ConfigurationModeTimeout)?;

        self.verify_spi_communications()?;

        self.configure_osc(settings.oscillator, delay)?;
        self.configure_io(settings.io_configuration)?;
        self.configure_bit_timing(settings.bit_time_configuration)
            ?;
        self.configure_tx_event_fifo(settings.tx_event_fifo)?;
        self.configure_tx_queue(settings.tx_queue)?;

        if settings.enable_time_based_counter {
            self.modify_register(|mut tscon: TimeStampControlRegister| {
                tscon.set_tbcen(true);
                tscon
            })
            ?;
        }

        if let Some(dncnt) = settings.data_bits_to_match {
            self.modify_register(|mut cicon: CanControlRegister| {
                cicon.set_dncnt(dncnt);
                cicon
            })
            ?;
        }

        self.modify_register(|mut cicon: CanControlRegister| {
            cicon.set_rtxat(true);
            cicon
        })
        ?;

        self.modify_register(|mut ciint: InterruptRegister| {
            ciint.set_rxie(true);
            ciint.set_txie(true);

            if settings.enable_can_error_interrupts {
                ciint.set_ivmie(true);
                ciint.set_cerrie(true);
                ciint.set_serrie(true);
            }

            if settings.enable_spi_error_interrupt {
                ciint.set_spicrcie(true);
            }

            if settings.enable_ecc_error_interrupt {
                ciint.set_eccie(true);
            }

            // FIXME: expose a dedicated setting for this
            if settings.enable_time_based_counter {
                ciint.set_tbcie(true);
            }

            ciint
        })
        ?;

        Ok(())
    }

    pub fn get_op_mode(&mut self) -> Result<OperationMode, Error> {
        let c1con = self.read_register::<CanControlRegister>()?;
        Ok(c1con.opmode())
    }

    /// Changes the operating mode of the chip. Will time out after 5 attempts.
    pub fn set_op_mode(
        &mut self,
        op_mode: OperationMode,
        delay: &mut impl DelayNs,
    ) -> Result<(), ConfigError> {
        self.modify_register(|mut c1con: CanControlRegister| {
            c1con.set_opmode(op_mode);
            c1con
        })
        ?;

        /* Delay 2ms checking every 500us for op mode change */

        const MAX_ATTEMPTS: usize = 5;

        for i in 0..MAX_ATTEMPTS {
            let c1con = self.read_register::<CanControlRegister>()?;

            if c1con.opmode() == op_mode {
                break;
            } else if i == MAX_ATTEMPTS - 1 {
                return Err(ConfigError::ChangeOpModeTimeout);
            }

            delay.delay_us(500u32);
        }

        Ok(())
    }

    pub fn configure_osc(
        &mut self,
        oscillator_settings: OscillatorConfiguration,
        delay: &mut impl DelayNs,
    ) -> Result<(), ConfigError> {
        self.modify_register(|mut osc: OscillatorControlRegister| {
            // If enabled, system clock comes from 10x PLL, otherwise it comes directly from the XTAL
            osc.set_pllen(match oscillator_settings.pll {
                Pll::On => true,
                Pll::Off => false,
            });

            // Whether or not to divide the system clock by 2
            osc.set_slckdiv(match oscillator_settings.divider {
                SysClkDivider::DivByOne => false,
                SysClkDivider::DivByTwo => true,
            });

            // Enable the clock
            osc.set_oscdis(false);

            osc
        })
        ?;

        if let settings::Pll::On = oscillator_settings.pll {
            const MAX_ATTEMPTS: usize = 3;

            // Wait for PLL ready
            for i in 0..MAX_ATTEMPTS {
                let osc = self.read_register::<OscillatorControlRegister>()?;

                if osc.pllrdy() {
                    break;
                } else if i == MAX_ATTEMPTS - 1 {
                    return Err(ConfigError::PLLNotReady);
                }

                delay.delay_us(500u32);
            }
        }

        Ok(())
    }

    pub fn configure_io(&mut self, io_config: IoConfiguration) -> Result<(), ConfigError> {
        self.modify_register(|mut iocon: IoControlRegister| {
            iocon.set_xstbyen(io_config.enable_tx_standby_pin);
            iocon.set_txcanod(io_config.tx_can_open_drain);
            iocon.set_sof(io_config.start_of_frame_on_clko);
            iocon.set_intod(io_config.interrupt_pin_open_drain);
            iocon
        })
        ?;

        Ok(())
    }

    pub fn configure_bit_timing(
        &mut self,
        bit_time_config: BitTimeConfiguration,
    ) -> Result<(), ConfigError> {
        self.modify_register(|mut cinbtcfg: NominalBitTimeConfigurationRegister| {
            cinbtcfg.set_brp(bit_time_config.nominal.baud_rate_prescaler);
            cinbtcfg.set_tseg1(bit_time_config.nominal.time_segment_1);
            cinbtcfg.set_tseg2(bit_time_config.nominal.time_segment_2.value());
            cinbtcfg.set_sjw(bit_time_config.nominal.synchronization_jump_width.value());

            cinbtcfg
        })
        ?;

        self.modify_register(|mut cidbtcfg: DataBitTimeConfigurationRegister| {
            cidbtcfg.set_brp(bit_time_config.data.baud_rate_prescaler);
            cidbtcfg.set_tseg1(bit_time_config.data.time_segment_1.value());
            cidbtcfg.set_tseg2(bit_time_config.data.time_segment_2.value());
            cidbtcfg.set_sjw(bit_time_config.data.synchronization_jump_width.value());

            cidbtcfg
        })
        ?;

        self.modify_register(|mut citdc: TransmitterDelayCompensationRegister| {
            citdc.set_tdcmod(TransmitterDelayCompensationMode::Automatic);
            citdc.set_tdco(
                bit_time_config
                    .data
                    .transmitter_delay_compensation_offset
                    .value(),
            );
            citdc.set_tdcv(0);

            citdc
        })
        ?;

        Ok(())
    }

    /// Enables/Disables the transmit event FIFO by setting C1CON.STEF and appropriate C1TEFCON bits.
    /// Be aware that fifo_size MUST be <= 32 and > 0, any other values will be clamped to 32.
    ///
    /// Also please keep in mind that the total RAM size is 2K and this code does absolutely
    /// zero validation that your configuration is under this limit. The documentation recommends
    /// configuring the TEF first, then TEQ, then FIFOs as necessary.
    pub fn configure_tx_event_fifo(
        &mut self,
        tx_event_fifo_config: Option<TxEventFifoConfiguration>,
    ) -> Result<(), ConfigError> {
        self.modify_register(|mut c1con: CanControlRegister| {
            c1con.set_stef(tx_event_fifo_config.is_some());
            c1con
        })
        ?;

        if let Some(config) = tx_event_fifo_config {
            self.modify_register(|mut tef_control: TxEventFifoControlRegister| {
                tef_control.set_fifo_size(config.fifo_size);

                tef_control.set_teftsen(config.enable_timestamps);

                tef_control.set_tefovie(config.enable_fifo_overflow_interrupt);
                tef_control.set_teffie(config.enable_fifo_full_interrupt);
                tef_control.set_tefhie(config.enable_fifo_half_full_interrupt);
                tef_control.set_tefneie(config.enable_fifo_not_empty_interrupt);

                tef_control
            })
            ?;
        }

        Ok(())
    }

    /// Enables/Disables the transmit queue by setting C1CON.TXEN and appropriate C1TXQCON bits.
    /// Be aware that fifo_size MUST be <= 32 and > 0, any other values will be clamped to 32.
    ///
    /// Also please keep in mind that the total RAM size is 2K and this code does absolutely
    /// zero validation that your configuration is under this limit. The documentation recommends
    /// configuring the TEF first, then TEQ, then FIFOs as necessary.
    pub fn configure_tx_queue(
        &mut self,
        tx_queue_config: Option<TxQueueConfiguration>,
    ) -> Result<(), ConfigError> {
        self.modify_register(|mut c1con: CanControlRegister| {
            c1con.set_txqen(tx_queue_config.is_some());
            c1con
        })
        ?;

        if let Some(config) = tx_queue_config {
            self.modify_register(|mut tx_queue_control: TxQueueControlRegister| {
                tx_queue_control.set_retransmission_attempts(config.retransmission_attempts);
                tx_queue_control.set_txpri(config.message_priority);
                tx_queue_control.set_fifo_size(config.fifo_size);
                tx_queue_control.set_payload_size(config.payload_size);

                tx_queue_control.set_txatie(config.enable_transmit_attempts_exhausted_interrupt);
                tx_queue_control.set_txqeie(config.enable_queue_empty_interrupt);
                tx_queue_control.set_txqnie(config.enable_queue_not_full_interrupt);

                tx_queue_control
            })
            ?;
        }

        Ok(())
    }

    /// Configures a FIFO based on the settings provided. As per documentation, a single FIFO must
    /// be dedicated to RX or TX and all objects in that queue must have the same payload size.
    pub fn configure_fifo(
        &mut self,
        fifo_number: FifoNumber,
        fifo_config: FifoConfiguration,
    ) -> Result<(), Error> {
        self.modify_repeated_register(fifo_number, |mut fifo_control: FifoControlRegister| {
            fifo_control.set_fifo_size(fifo_config.fifo_size);
            fifo_control.set_payload_size(fifo_config.payload_size);

            match fifo_config.mode {
                settings::FifoMode::Transmit(TxFifoConfiguration {
                    priority,
                    retransmission_attempts,
                    enable_auto_rtr,
                    enable_transmit_attempts_exhausted_interrupt,
                    enable_fifo_empty_interrupt,
                    enable_fifo_half_empty_interrupt,
                    enable_fifo_not_full_interrupt,
                }) => {
                    fifo_control.set_txen(true);

                    fifo_control.set_txpri(priority);
                    fifo_control.set_retransmission_attempts(retransmission_attempts);
                    fifo_control.set_rtren(enable_auto_rtr);
                    fifo_control.set_txatie(enable_transmit_attempts_exhausted_interrupt);
                    fifo_control.set_tferffie(enable_fifo_empty_interrupt);
                    fifo_control.set_tfhrfhie(enable_fifo_half_empty_interrupt);
                    fifo_control.set_tfnrfnie(enable_fifo_not_full_interrupt);
                }
                settings::FifoMode::Receive(RxFifoConfiguration {
                    enable_message_timestamps,
                    enable_fifo_overflow_interrupt,
                    enable_fifo_full_interrupt,
                    enable_fifo_half_full_interrupt,
                    enable_fifo_not_empty_interrupt,
                }) => {
                    fifo_control.set_txen(false);

                    fifo_control.set_rxtsen(enable_message_timestamps);
                    fifo_control.set_rxovie(enable_fifo_overflow_interrupt);
                    fifo_control.set_tferffie(enable_fifo_full_interrupt);
                    fifo_control.set_tfhrfhie(enable_fifo_half_full_interrupt);
                    fifo_control.set_tfnrfnie(enable_fifo_not_empty_interrupt);
                }
            }

            fifo_control
        })
        ?;

        Ok(())
    }

    /// Configures one of the 32 acceptance filters. If the filter_config is
    /// None, the filter will be disabled instead.
    ///
    /// Filters can be configured to accept only standard frames, only
    /// extended frames, or both standard and extended frames. If either the
    /// filter_bits or mask_bits fields are set as MessageId::Standard, the
    /// corresponding EID bits will be set to 0.
    ///
    /// When receiving standard frames, the EID component of the filter can be
    /// used to match against (up to) the first 18 bits of the message's data
    /// segment. The number of bits used is configured by `CiCON.DNCNT`. See
    /// the family reference manual for a more detailed description of this
    /// mechanism.
    pub fn configure_filter(
        &mut self,
        filter_number: FilterNumber,
        filter_config: Option<FilterConfiguration>,
    ) -> Result<(), Error> {
        let (control_register_number, filter_index) = filter_number.get_control_register();

        // We need to disable the filter no matter what to configure it
        self.modify_repeated_register(
            control_register_number,
            |mut control: FilterControlRegister| {
                control.set_enabled(filter_index, false);
                control
            },
        )
        ?;

        // If we are just disabling it, then we are done here
        let Some(filter_config) = filter_config else {
            return Ok(());
        };

        // Set filter object bits and filter mode
        self.modify_repeated_register(
            filter_number,
            |mut object_register: FilterObjectRegister| {
                match filter_config.filter_bits {
                    Id::Standard(id) => {
                        object_register.set_sid(id.as_raw());
                        object_register.set_eid(0);
                    }
                    Id::Extended(id) => {
                        object_register.set_sid(id.standard_id().as_raw());
                        object_register
                            .set_eid(id.as_raw() & ((!(StandardId::MAX.as_raw() as u32)) >> 11));
                    }
                }

                // 0b000S_SSSS_SSSS_SSEE_EEEE_EEEE_EEEE_EEEE
                //                     0b0000_0111_1111_1111
                //                     0b1111_1000_0000_0000
                // 0b0000_0000_0000_0000_1111_1000_0000_0000
                // 0b0000_0000_0000_0000_0000_0000_0001_1111

                object_register.set_exide(match filter_config.mode {
                    FilterMatchMode::StandardOnly | FilterMatchMode::Both => false,
                    FilterMatchMode::ExtendedOnly => true,
                });

                object_register
            },
        )
        ?;

        // Set the mask bits and exclusion mode
        self.modify_repeated_register(filter_number, |mut mask_register: MaskRegister| {
            match filter_config.mask_bits {
                Id::Standard(id) => {
                    mask_register.set_msid(id.as_raw());
                    mask_register.set_meid(0);
                }
                Id::Extended(id) => {
                    mask_register.set_msid(id.standard_id().as_raw());
                    mask_register
                        .set_meid(id.as_raw() & ((!(StandardId::MAX.as_raw() as u32)) >> 11));
                }
            }

            mask_register.set_mide(match filter_config.mode {
                FilterMatchMode::Both => false,
                FilterMatchMode::StandardOnly | FilterMatchMode::ExtendedOnly => true,
            });

            mask_register
        })
        ?;

        // Set the BP and reenable the filter
        self.modify_repeated_register(
            control_register_number,
            |mut control: FilterControlRegister| {
                control.set_buffer_pointer(filter_index, filter_config.buffer_pointer);
                control.set_enabled(filter_index, true);
                control
            },
        )
        ?;

        Ok(())
    }

    /* Transmit and Receive Functions */

    /// Pushes a new message into the TXQ without setting the TXREQ bit to
    /// request transmission.
    ///
    /// Use this function only if you need to queue multiple messages before
    /// transmitting all at once. To push a single message and immediately
    /// request transmission, use [`MCP2518FD::tx_queue_transmit_message`].
    pub fn tx_queue_push_message(&mut self, message: &TxMessage) -> Result<(), Error> {
        /* Make sure TXQ is enabled */

        if !self.read_register::<CanControlRegister>()?.txqen() {
            return Err(Error::TxQueueDisabled);
        }

        let mut control_register = self.read_register::<TxQueueControlRegister>()?;

        /* Make sure FIFO is big enough */

        if control_register.payload_size().num_bytes() < message.data().len() {
            return Err(Error::FifoTooSmall);
        }

        /* Make sure FIFO is not full */

        let status_register = self.read_register::<TxQueueStatusRegister>()?;

        if !status_register.txqnif() {
            return Err(Error::FifoFull);
        }

        /* Write message to RAM */

        let ram_address = self
            .read_repeated_register::<UserAddressRegister>(UserAddressKind::TxQueue)
            ?
            .calculate_ram_address();

        let (length, bytes) = message.as_bytes();

        // The reading length has to be a multiple of 4 thus we round up the data_len
        let data = &bytes[..round_up_spi_transfer_size(length)];

        self.write_ram(ram_address as u16, data)?;

        /* Increment tail pointer but do NOT request transmission */

        control_register.set_uinc();

        self.write_register(control_register)?;

        Ok(())
    }

    /// Pushes to the TXQ without reading any status registers or performing
    /// size checks. Callers of this function should be careful to check that
    /// there is room in the TXQ. Otherwise, the oldest message will be
    /// overriden.
    pub fn tx_queue_push_message_unchecked(
        &mut self,
        message: &TxMessage,
    ) -> Result<(), Error> {
        /* Write message to RAM */

        let ram_address = self
            .read_repeated_register::<UserAddressRegister>(UserAddressKind::TxQueue)
            ?
            .calculate_ram_address();

        let (length, bytes) = message.as_bytes();

        // The reading length has to be a multiple of 4 thus we round up the data_len
        let data = &bytes[..round_up_spi_transfer_size(length)];

        self.write_ram(ram_address as u16, data)?;

        /* Increment tail pointer but do NOT request transmission */

        self.modify_register(|mut txqcon: TxQueueControlRegister| {
            txqcon.set_uinc();
            txqcon
        })
        ?;

        Ok(())
    }

    /// Requests transmission of all messages in the TXQ by setting the TXREQ
    /// bit.
    ///
    /// Use this function only if you already previously queued one or more
    /// messages with [`MCP2518FD::tx_queue_push_message`]. To push a single
    /// message and immediately request transmission, prefer
    /// [`MCP2518FD::tx_queue_transmit_message`].
    pub fn tx_queue_request_transmission(&mut self) -> Result<(), Error> {
        self.modify_register(|mut txqcon: TxQueueControlRegister| {
            txqcon.set_txreq(true);
            txqcon
        })
        ?;

        Ok(())
    }

    /// Pushes a message into the TXQ and immediately requests transmission by
    /// setting the TXREQ bit.
    ///
    /// To push multiple messages before requesting transmission, see
    /// [`MCP2518FD::tx_queue_push_message`] and
    /// [`MCP2518FD::tx_queue_request_transmission`].
    pub fn tx_queue_transmit_message(&mut self, message: &TxMessage) -> Result<(), Error> {
        self.tx_queue_push_message(message)?;
        self.tx_queue_request_transmission()?;

        Ok(())
    }

    /// Fetches the status of the TXQ to determine whether it is empty
    pub fn tx_queue_is_empty(&mut self) -> Result<bool, Error> {
        let status = self.read_register::<TxQueueStatusRegister>()?;

        Ok(status.txqeif())
    }

    /// Fetches the status of the TXQ to determine whether it is full
    pub fn tx_queue_is_full(&mut self) -> Result<bool, Error> {
        let status = self.read_register::<TxQueueStatusRegister>()?;

        Ok(!status.txqnif())
    }

    /// Pushes a new message into the given TX FIFO without setting the TXREQ
    /// bit to request transmission.
    ///
    /// Use this function only if you need to queue multiple messages before
    /// transmitting all at once. To push a single message and immediately
    /// request transmission, use [`MCP2518FD::tx_fifo_transmit_message`].
    pub fn tx_fifo_push_message(
        &mut self,
        fifo_number: FifoNumber,
        message: &TxMessage,
    ) -> Result<(), Error> {
        let mut control_register = self
            .read_repeated_register::<FifoControlRegister>(fifo_number)
            ?;

        /* Make sure it's a transmit FIFO */

        if !control_register.txen() {
            return Err(Error::FifoNotTx);
        }

        /* Make sure FIFO is big enough */

        if control_register.payload_size().num_bytes() < message.data().len() {
            return Err(Error::FifoTooSmall);
        }

        /* Make sure FIFO is not full */

        let status_register = self
            .read_repeated_register::<FifoStatusRegister>(fifo_number)
            ?;

        if !status_register.tfnrfnif() {
            return Err(Error::FifoFull);
        }

        /* Write message to RAM */

        let ram_address = self
            .read_repeated_register::<UserAddressRegister>(UserAddressKind::Fifo(fifo_number))
            ?
            .calculate_ram_address();

        let (length, bytes) = message.as_bytes();

        // The reading length has to be a multiple of 4 thus we round up the data_len
        let data = &bytes[..round_up_spi_transfer_size(length)];

        self.write_ram(ram_address as u16, data)?;

        /* Increment tail pointer but to NOT request transmission */

        control_register.set_uinc();

        self.write_repeated_register(fifo_number, control_register)
            ?;

        Ok(())
    }

    /// Requests transmission of all messages in the given TX FIFO by setting
    /// the TXREQ bit.
    ///
    /// Use this function only if you already previously queued one or more
    /// messages with [`MCP2518FD::tx_fifo_push_message`]. To push a single
    /// message and immediately request transmission, prefer
    /// [`MCP2518FD::tx_fifo_transmit_message`].
    pub fn tx_fifo_request_transmission(
        &mut self,
        fifo_number: FifoNumber,
    ) -> Result<(), Error> {
        self.modify_repeated_register(fifo_number, |mut fifocon: FifoControlRegister| {
            fifocon.set_txreq(true);
            fifocon
        })
        ?;

        Ok(())
    }

    /// Pushes a message into the given TX FIFO and immediately requests
    /// transmission by setting the TXREQ bit.
    ///
    /// To push multiple messages before requesting transmission, see
    /// [`MCP2518FD::tx_fifo_push_message`] and
    /// [`MCP2518FD::tx_fifo_request_transmission`].
    pub fn tx_fifo_transmit_message(
        &mut self,
        fifo_number: FifoNumber,
        message: &TxMessage,
    ) -> Result<(), Error> {
        self.tx_fifo_push_message(fifo_number, message)?;
        self.tx_fifo_request_transmission(fifo_number)?;

        Ok(())
    }

    /// Fetches the status of the TX FIFO to determine whether it is empty
    pub fn tx_fifo_is_empty(&mut self, fifo_number: FifoNumber) -> Result<bool, Error> {
        let status = self
            .read_repeated_register::<FifoStatusRegister>(fifo_number)
            ?;

        Ok(status.tferffif())
    }

    /// Fetches the status of the TX FIFO to determine whether it is full
    pub fn tx_fifo_is_full(&mut self, fifo_number: FifoNumber) -> Result<bool, Error> {
        let status = self
            .read_repeated_register::<FifoStatusRegister>(fifo_number)
            ?;

        Ok(!status.tfnrfnif())
    }

    /// Gets the value of TXAT for this FIFO and clears it
    pub fn tx_fifo_transmission_attempts_exhausted(
        &mut self,
        fifo_number: FifoNumber,
    ) -> Result<bool, Error> {
        let mut exhausted = false;

        self.modify_repeated_register(fifo_number, |mut cififostam: FifoStatusRegister| {
            exhausted = cififostam.txatif();
            cififostam.clear_txatif();
            cififostam
        })
        ?;

        Ok(exhausted)
    }

    /// Checks to see if there are any messages in the TEF
    pub fn tx_event_fifo_has_next(&mut self) -> Result<bool, Error> {
        let status_register = self.read_register::<TxEventFifoStatusRegister>()?;

        Ok(status_register.tefneif())
    }

    /// If there is a message available in the TEF it will be read but the FIFO
    /// tail pointer will **NOT** be incremented
    ///
    /// Unless you have a specific use case for this, you most likely want to
    /// use [`MCP2518FD::tx_event_fifo_get_next`]
    pub fn tx_event_fifo_peek_next(&mut self) -> Result<Option<TxEventObject>, Error> {
        /* Make sure there is data to read */

        if !self.tx_event_fifo_has_next()? {
            return Ok(None);
        }

        /* Get the address of the next object */

        let ram_address = self
            .read_repeated_register::<UserAddressRegister>(UserAddressKind::TxEventFifo)
            ?
            .calculate_ram_address();

        /* Check if timestamps are enabled and read accordingly */

        let control_register = self.read_register::<TxEventFifoControlRegister>()?;

        let obj = if control_register.teftsen() {
            let mut buf = [0u8; 12];

            self.read_ram(ram_address as u16, &mut buf)?;

            TxEventObject {
                header: TxHeader([
                    u32::from_le_bytes(buf[0..4].try_into().unwrap()),
                    u32::from_le_bytes(buf[4..8].try_into().unwrap()),
                ]),
                timestamp: Some(u32::from_le_bytes(buf[8..12].try_into().unwrap())),
            }
        } else {
            let mut buf = [0u8; 8];

            self.read_ram(ram_address as u16, &mut buf)?;

            TxEventObject {
                header: TxHeader([
                    u32::from_le_bytes(buf[0..4].try_into().unwrap()),
                    u32::from_le_bytes(buf[4..8].try_into().unwrap()),
                ]),
                timestamp: None,
            }
        };

        Ok(Some(obj))
    }

    /// If there is a message available in the TEF it will be read and the FIFO
    /// tail pointer will be incremented to allow for the next read operation
    ///
    /// To only check if a message is available without pulling it from the
    /// FIFO, see [`MCP2518FD::tx_event_fifo_has_next`] and
    /// [`MCP2518FD::tx_event_fifo_peek_next`]
    pub fn tx_event_fifo_get_next(&mut self) -> Result<Option<TxEventObject>, Error> {
        let obj = self.tx_event_fifo_peek_next()?;

        let Some(obj) = obj else {
            return Ok(None);
        };

        self.modify_register(|mut tefcon: TxEventFifoControlRegister| {
            tefcon.set_uinc();
            tefcon
        })
        ?;

        Ok(Some(obj))
    }

    /// Checks to see if there are any messages in the given receive FIFO
    pub fn rx_fifo_has_next(&mut self, fifo_number: FifoNumber) -> Result<bool, Error> {
        /* Make sure it's a receive FIFO */

        let control_register = self
            .read_repeated_register::<FifoControlRegister>(fifo_number)
            ?;

        if control_register.txen() {
            return Err(Error::FifoNotRx);
        }

        /* Check is the FIFO has any messages in it */

        let status_register = self
            .read_repeated_register::<FifoStatusRegister>(fifo_number)
            ?;

        Ok(status_register.tfnrfnif())
    }

    /// If there is a message available in the given RX FIFO it will be read,
    /// but the FIFO head pointer will **NOT** be incremented
    ///
    /// Unless you have a specific use case for this, you most likely want to
    /// use [`MCP2518FD::rx_fifo_get_next`]
    pub fn rx_fifo_peek_next(
        &mut self,
        fifo_number: FifoNumber,
    ) -> Result<Option<RxMessage>, Error> {
        /* Make sure there is data to read */

        if !self.rx_fifo_has_next(fifo_number)? {
            return Ok(None);
        }

        /* Get the address of the next object */

        let ram_address = self
            .read_repeated_register::<UserAddressRegister>(UserAddressKind::Fifo(fifo_number))
            ?
            .calculate_ram_address();

        /* Read the message header to see how much data we need to read */

        let mut buf = [0u8; 8];

        self.read_ram(ram_address as u16, &mut buf)?;

        let header = RxHeader([
            u32::from_le_bytes(buf[0..4].try_into().unwrap()),
            u32::from_le_bytes(buf[4..8].try_into().unwrap()),
        ]);

        /* Read timestamp (if applicable) */

        let control_register = self
            .read_repeated_register::<FifoControlRegister>(fifo_number)
            ?;

        let timestamp = control_register
            .rxtsen()
            .then(|| {
                let mut ts = [0u8; 4];
                self.read_ram((ram_address + 4 * 2) as u16, &mut ts)?;

                Ok(u32::from_le_bytes(ts[..].try_into().unwrap()))
            })
            .transpose()?;

        /* Read the content of the message */

        let mut data = [0u8; MAX_FD_BUFFER_SIZE];
        let data_len = len_for_dlc(header.dlc(), header.fdf()).unwrap();
        let data_offset = if timestamp.is_some() { 3 } else { 2 };

        // The reading length has to be a multiple of 4 thus we round up the data_len
        let read_len = round_up_spi_transfer_size(data_len);

        self.read_ram(
            (ram_address + 4 * data_offset) as u16,
            &mut data[..read_len],
        )
        ?;

        /* Assemble RxMessage */

        Ok(Some(
            RxMessage::new(header, timestamp, &data[..data_len]).unwrap(),
        ))
    }

    /// If there is a message available in the given RX FIFO it will be read,
    /// and the FIFO head pointer will be incremented to allow for the next
    /// read operation
    ///
    /// To only check if a message is available without pulling it from the
    /// FIFO, see [`MCP2518FD::rx_fifo_has_next`] and
    /// [`MCP2518FD::rx_fifo_peek_next`]
    pub fn rx_fifo_get_next(
        &mut self,
        fifo_number: FifoNumber,
    ) -> Result<Option<RxMessage>, Error> {
        let msg = self.rx_fifo_peek_next(fifo_number)?;

        let Some(msg) = msg else {
            return Ok(None);
        };

        self.rx_fifo_increment_head(fifo_number)?;

        Ok(Some(msg))
    }

    /// Increments the RX FIFO head pointer (UINC) using an optimized single-byte write.
    ///
    /// This is more efficient than a read-modify-write cycle since UINC is a
    /// set-only bit that doesn't require reading the current register value.
    #[inline]
    fn rx_fifo_increment_head(&mut self, fifo_number: FifoNumber) -> Result<(), Error> {
        // UINC is bit 8, which is bit 0 of byte 1 of the register.
        // We can write just byte 1 with value 0x01 to set UINC without
        // affecting other register bits.
        let base_address = FifoControlRegister::get_address_for(fifo_number) as u16;
        self.write_sfr_byte(base_address + 1, 0x01)
    }

    /// Gets the RX FIFO status without checking if it's an RX FIFO.
    ///
    /// Returns (has_messages, fifo_index) where:
    /// - `has_messages` is true if there's at least one message available
    /// - `fifo_index` is the current FIFO head pointer index (NOT the fill count)
    #[inline]
    pub fn rx_fifo_status(&mut self, fifo_number: FifoNumber) -> Result<(bool, u8), Error> {
        let status = self.read_repeated_register::<FifoStatusRegister>(fifo_number)?;
        Ok((status.tfnrfnif(), status.fifoci()))
    }

    /// Reads multiple messages from an RX FIFO into the provided buffer.
    ///
    /// This is the most efficient batch read method, using only 3 SPI transactions per message:
    /// - 1x Combined status + address read (FifoStatusRegister + UserAddressRegister)
    /// - 1x RAM read (header + timestamp + data)
    /// - 1x UINC byte write
    ///
    /// # Arguments
    /// * `fifo_number` - The FIFO to read from
    /// * `timestamps_enabled` - Whether timestamps are enabled for this FIFO
    /// * `max_data_bytes` - Maximum payload size configured for this FIFO
    /// * `buffer` - Mutable slice to store received messages
    ///
    /// # Returns
    /// The number of messages actually read into the buffer. Reading stops when:
    /// - The buffer is full
    /// - The FIFO is empty
    ///
    /// # Note
    /// Caller must ensure the FIFO is configured for RX and that `max_data_bytes`
    /// matches the FIFO's configured payload size.
    pub fn rx_fifo_read_batch(
        &mut self,
        fifo_number: FifoNumber,
        timestamps_enabled: bool,
        max_data_bytes: usize,
        buffer: &mut [RxMessage],
    ) -> Result<usize, Error> {
        let timestamp_size = if timestamps_enabled { 4 } else { 0 };
        let total_read_size = round_up_spi_transfer_size(8 + timestamp_size + max_data_bytes);

        let mut count = 0;
        let mut read_buf = [0u8; 76];

        while count < buffer.len() {
            // Check status and get RAM address in one read (1 SPI transaction)
            let (has_message, ram_address) = self.read_rx_fifo_status_and_address(fifo_number)?;
            if !has_message {
                break;
            }

            // Read message data (1 SPI transaction)
            self.read_ram(ram_address, &mut read_buf[..total_read_size])?;

            // Parse header
            let header = RxHeader([
                u32::from_le_bytes(read_buf[0..4].try_into().unwrap()),
                u32::from_le_bytes(read_buf[4..8].try_into().unwrap()),
            ]);

            // Parse timestamp if enabled
            let timestamp = if timestamps_enabled {
                Some(u32::from_le_bytes(read_buf[8..12].try_into().unwrap()))
            } else {
                None
            };

            // Parse data
            let data_offset = if timestamps_enabled { 12 } else { 8 };
            let data_len = len_for_dlc(header.dlc(), header.fdf()).unwrap_or(0);

            // Increment FIFO head pointer (1 SPI transaction)
            self.rx_fifo_increment_head(fifo_number)?;

            buffer[count] = RxMessage::new(header, timestamp, &read_buf[data_offset..data_offset + data_len])
                .expect("DLC is 4 bits so data_len <= 64");
            count += 1;
        }

        Ok(count)
    }

    /* Interrupt related operations */

    pub fn get_highest_interrupt_codes(&mut self) -> Result<InterruptCodeRegister, Error> {
        self.read_register::<InterruptCodeRegister>()    }

    pub fn get_top_level_interrupt_statuses(&mut self) -> Result<InterruptRegister, Error> {
        self.read_register::<InterruptRegister>()    }

    pub fn get_rx_interrupt_statuses(&mut self) -> Result<RxInterruptStatusRegister, Error> {
        self.read_register::<RxInterruptStatusRegister>()    }

    pub fn get_rx_overflow_interrupt_statuses(
        &mut self,
    ) -> Result<RxOverflowInterruptStatusRegister, Error> {
        self.read_register::<RxOverflowInterruptStatusRegister>()
                }

    pub fn get_tx_interrupt_statuses(&mut self) -> Result<TxInterruptStatusRegister, Error> {
        self.read_register::<TxInterruptStatusRegister>()    }

    pub fn get_tx_attempt_interrupt_statuses(
        &mut self,
    ) -> Result<TxAttemptInterruptStatusRegister, Error> {
        self.read_register::<TxAttemptInterruptStatusRegister>()
                }

    /* Generic register ops with mapping */

    pub fn modify_repeated_register<R, F>(
        &mut self,
        index: R::Index,
        transform: F,
    ) -> Result<(), Error>
    where
        R: RepeatedRegister + From<u32> + Into<u32>,
        F: FnOnce(R) -> R,
    {
        let register = self.read_repeated_register::<R>(index)?;

        self.write_repeated_register::<R>(index, transform(register))
                }

    pub fn read_repeated_register<R>(&mut self, index: R::Index) -> Result<R, Error>
    where
        R: RepeatedRegister + From<u32>,
    {
        let address = R::get_address_for(index);

        self.read_sfr(&address).map(R::from)
    }

    pub fn write_repeated_register<R>(
        &mut self,
        index: R::Index,
        value: R,
    ) -> Result<(), Error>
    where
        R: RepeatedRegister + Into<u32>,
    {
        let address = R::get_address_for(index);

        self.write_sfr(&address, value.into())    }

    pub fn modify_register<R, F>(&mut self, transform: F) -> Result<(), Error>
    where
        R: Register + From<u32> + Into<u32>,
        F: FnOnce(R) -> R,
    {
        let register = self.read_register::<R>()?;

        self.write_register::<R>(transform(register))    }

    pub fn read_register<R>(&mut self) -> Result<R, Error>
    where
        R: Register + From<u32>,
    {
        let address = R::get_address();

        self.read_sfr(&address).map(R::from)
    }

    pub fn write_register<R>(&mut self, value: R) -> Result<(), Error>
    where
        R: Register + Into<u32>,
    {
        let address = R::get_address();

        self.write_sfr(&address, value.into())    }

    /* Raw SFR Ops (Minimal type checking) */

    fn read_sfr(&mut self, address: &SFRAddress) -> Result<u32, Error> {
        let mut instruction = Instruction(OpCode::READ);
        instruction.set_address(*address as u16);

        let mut buf = [0u8; 4];

        self.spi
            .transaction(&mut [
                Operation::Write(&instruction.into_spi_data()),
                Operation::Read(&mut buf),
            ])
                        .map_err(|_| Error::SPIRead)?;

        Ok(u32::from_le_bytes(buf))
    }

    fn write_sfr(&mut self, address: &SFRAddress, value: u32) -> Result<(), Error> {
        let mut instruction = Instruction(OpCode::WRITE);
        instruction.set_address(*address as u16);

        self.spi
            .transaction(&mut [
                Operation::Write(&instruction.into_spi_data()),
                // The "instruction" needs to be converted to BE bytes but the actual SFR register
                // needs to be in LE format!!!
                Operation::Write(&value.to_le_bytes()),
            ])
                        .map_err(|_| Error::SPIRead)?;

        Ok(())
    }

    /// Writes a single byte to an SFR address.
    ///
    /// This is useful for setting individual bits without a read-modify-write cycle,
    /// such as the UINC bit which is a set-only bit at byte offset 1 of FIFOCONm.
    fn write_sfr_byte(&mut self, address: u16, value: u8) -> Result<(), Error> {
        let mut instruction = Instruction(OpCode::WRITE);
        instruction.set_address(address);

        self.spi
            .transaction(&mut [
                Operation::Write(&instruction.into_spi_data()),
                Operation::Write(&[value]),
            ])
            .map_err(|_| Error::SPIWrite)?;

        Ok(())
    }

    /// Reads FifoStatusRegister and UserAddressRegister in a single SPI transaction.
    ///
    /// Returns `(has_messages, ram_address)` where ram_address is the absolute RAM address.
    /// Returns `Ok((false, 0))` if the FIFO is empty.
    #[inline]
    fn read_rx_fifo_status_and_address(&mut self, fifo_number: FifoNumber) -> Result<(bool, u16), Error> {
        // STA and UA registers are adjacent (STA at offset 0, UA at offset 4)
        let sta_address = FifoStatusRegister::get_address_for(fifo_number) as u16;

        let mut instruction = Instruction(OpCode::READ);
        instruction.set_address(sta_address);

        let mut buf = [0u8; 8];
        self.spi
            .transaction(&mut [
                Operation::Write(&instruction.into_spi_data()),
                Operation::Read(&mut buf),
            ])
            .map_err(|_| Error::SPIRead)?;

        let status = FifoStatusRegister(u32::from_le_bytes(buf[0..4].try_into().unwrap()));
        if !status.tfnrfnif() {
            return Ok((false, 0));
        }

        let ua = u32::from_le_bytes(buf[4..8].try_into().unwrap());
        let ram_address = (ua + RAM_BASE_ADDRESS) as u16;

        Ok((true, ram_address))
    }

    /* RAM related functions */

    /// Verify SPI connection is working by writing to an available ram location.
    pub fn verify_spi_communications(&mut self) -> Result<(), ConfigError> {
        let address = 0x400;
        for i in 0..32 {
            let data: u32 = 1 << i;
            self.write_ram(address, &data.to_le_bytes())?;

            let mut read_back_buf = [0u8; 4];
            self.read_ram(address, &mut read_back_buf)?;
            let read_back_value = u32::from_le_bytes(read_back_buf);
            if read_back_value != data {
                return Err(ConfigError::SPIFailedRAMEcho);
            }
        }
        Ok(())
    }

    pub fn verify_spi_communications_long(&mut self) -> Result<(), ConfigError> {
        let address = 0x400;

        let mut dword_data = [0u32; 32];
        for (i, d) in dword_data.iter_mut().enumerate() {
            *d = 1 << i;
        }

        let mut data = [0u8; 32 * 4];
        for i in 0..32 {
            let bytes = dword_data[i].to_le_bytes();

            data[i * 4] = bytes[0];
            data[i * 4 + 1] = bytes[1];
            data[i * 4 + 2] = bytes[2];
            data[i * 4 + 3] = bytes[3];
        }

        self.write_ram(address, &data)?;

        let mut read_back_buf = [0u8; 32 * 4];
        self.read_ram(address, &mut read_back_buf)?;

        if read_back_buf != data {
            return Err(ConfigError::SPIFailedRAMEcho);
        }

        Ok(())
    }

    /// Reads a contiguous range from RAM into the provided buffer
    pub fn read_ram(&mut self, address: u16, data: &mut [u8]) -> Result<(), Error> {
        is_valid_ram_address(address as u32, data.len())
            .then_some(())
            .ok_or(Error::InvalidRamAddress(address))?;

        if data.len() % 4 != 0 {
            return Err(Error::InvalidReadLength(data.len()));
        }

        let mut instruction = Instruction(OpCode::READ);
        instruction.set_address(address);

        self.spi
            .transaction(&mut [
                Operation::Write(&instruction.into_spi_data()),
                Operation::Read(data),
            ])
                        .map_err(|_| Error::SPIRead)?;

        Ok(())
    }

    /// Writes to a contiguous range in RAM from the provided buffer
    pub fn write_ram(&mut self, address: u16, data: &[u8]) -> Result<(), Error> {
        is_valid_ram_address(address as u32, data.len())
            .then_some(())
            .ok_or(Error::InvalidRamAddress(address))?;

        if data.len() % 4 != 0 {
            return Err(Error::InvalidWriteLength(data.len()));
        }

        let mut instruction = Instruction(OpCode::WRITE);
        instruction.set_address(address);

        self.spi
            .transaction(&mut [
                Operation::Write(&instruction.into_spi_data()),
                Operation::Write(data),
            ])
                        .map_err(|_| Error::SPIWrite)?;

        Ok(())
    }
}

/* Low level SPI instruction encoding */

bitfield! {
 struct Instruction(u16);
    impl Debug;
    u16;
    pub op_code, set_op_code: 15, 12;
    pub address, set_address: 11, 0;
}

impl Instruction {
    pub fn into_spi_data(self) -> [u8; 2] {
        self.0.to_be_bytes()
    }
}

struct OpCode;

impl OpCode {
    pub const RESET: u16 = 0b0000 << 12;
    pub const READ: u16 = 0b0011 << 12;
    pub const WRITE: u16 = 0b0010 << 12;
}

#[cfg(test)]
mod test {
    use super::round_up_spi_transfer_size;

    #[test]
    fn test_round_up_spi_transfer_size() {
        assert_eq!(round_up_spi_transfer_size(0), 0);
        assert_eq!(round_up_spi_transfer_size(1), 4);
        assert_eq!(round_up_spi_transfer_size(2), 4);
        assert_eq!(round_up_spi_transfer_size(3), 4);
        assert_eq!(round_up_spi_transfer_size(4), 4);
        assert_eq!(round_up_spi_transfer_size(5), 8);
        assert_eq!(round_up_spi_transfer_size(6), 8);
        assert_eq!(round_up_spi_transfer_size(7), 8);
        assert_eq!(round_up_spi_transfer_size(8), 8);
        assert_eq!(round_up_spi_transfer_size(9), 12);
    }
}
