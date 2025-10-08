//! CFU message bridge
//! TODO: remove this once we have a more generic FW update implementation
use crate::wrapper::backing::ControllerState;
use embassy_futures::select::{select, Either};
use embedded_cfu_protocol::protocol_definitions::{
    CfuUpdateContentResponseStatus, ComponentId, FwUpdateContentCommand, FwUpdateContentHeader,
    FwUpdateContentResponse, FwUpdateOfferExtended, FwUpdateOfferInformation, FwVerComponentInfo,
    GetFwVerRespHeaderByte3, GetFwVersionResponse, GetFwVersionResponseHeader, HostToken, OfferRejectReason,
    OfferStatus, FW_UPDATE_FLAG_FIRST_BLOCK, FW_UPDATE_FLAG_LAST_BLOCK, MAX_CMPT_COUNT,
};
use embedded_services::{
    cfu::component::{InternalResponseData, RequestData},
    debug, error, power,
    type_c::{controller::Controller, ControllerId},
};

use super::message::EventCfu;
use super::*;

/// Current state of the firmware update process
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FwUpdateState {
    /// None in progress
    Idle,
    /// Firmware update in progress
    /// Contains number of ticks [`super::DEFAULT_FW_UPDATE_TICK_INTERVAL_MS`] that have passed
    InProgress(u8),
    /// Firmware update has failed and the device is in an unknown state
    Recovery,
}

impl FwUpdateState {
    /// Check if the firmware update is in progress
    pub fn in_progress(&self) -> bool {
        matches!(self, FwUpdateState::InProgress(_) | FwUpdateState::Recovery)
    }
}

/// A CFU device that can receive offers, apply updates, and provide version information.
pub trait Device {
    /// The component ID of the device.
    fn component_id(&self) -> ComponentId;

    async fn get_fw_version(&mut self) -> Result<GetFwVersionResponse, InternalResponseData>;
    async fn handle_offer(&mut self, offer: &FwUpdateOffer) -> Result<FwUpdateOfferResponse, InternalResponseData>;
    async fn handle_content(
        &mut self,
        content: &FwUpdateContentCommand,
    ) -> Result<FwUpdateContentResponse, InternalResponseData>;
    async fn abort_update(&mut self) -> Result<(), InternalResponseData>;
    async fn finalize_update(&mut self) -> Result<(), InternalResponseData>;
    async fn prepare_for_update(&mut self) -> Result<(), InternalResponseData>;
    async fn handle_extended_offer(
        &mut self,
        offer: &FwUpdateOfferExtended,
    ) -> Result<FwUpdateOfferResponse, InternalResponseData>;
    async fn handle_offer_information(
        &mut self,
        info: &FwUpdateOfferInformation,
    ) -> Result<FwUpdateOfferResponse, InternalResponseData>;
}

struct ControllerWithPortStateAndValidator<'a, C: Controller, V: FwOfferValidator> {
    component_id: ComponentId,
    controller_id: ControllerId,
    power_devices: &'a [power::policy::device::Device],
    controller: &'a mut C,
    controller_state: &'a mut ControllerState,
    fw_version_validator: &'a V,
    ticker: &'a mut embassy_time::Ticker,
}

trait FwUpdateContentHeaderExt {
    /// Is this the first block of the update?
    fn is_first_block(&self) -> bool;

    /// Is this the last block of the update?
    fn is_last_block(&self) -> bool;
}

impl FwUpdateContentHeaderExt for FwUpdateContentHeader {
    fn is_first_block(&self) -> bool {
        self.flags & FW_UPDATE_FLAG_FIRST_BLOCK != 0
    }

    fn is_last_block(&self) -> bool {
        self.flags & FW_UPDATE_FLAG_LAST_BLOCK != 0
    }
}

trait ResultExt<T, E> {
    /// Maps a `Result<T, E>` to `E` by applying a function to a contained [`Ok`] value, or returning the contained
    /// [`Err`] value.
    fn map_or_unwrap_err<F>(self, f: F) -> E
    where
        F: FnOnce(T) -> E;
}

impl<T, E> ResultExt<T, E> for Result<T, E> {
    fn map_or_unwrap_err<F>(self, f: F) -> E
    where
        F: FnOnce(T) -> E,
    {
        self.map_or_else(core::convert::identity, f)
    }
}

impl<'a, C: Controller, V: FwOfferValidator> ControllerWithPortStateAndValidator<'a, C, V> {
    /// Create a new invalid FW version response
    fn create_invalid_fw_version_response(&self) -> InternalResponseData {
        let dev_inf = FwVerComponentInfo::new(FwVersion::new(0xffffffff), self.component_id());
        let comp_info: [FwVerComponentInfo; MAX_CMPT_COUNT] = [dev_inf; MAX_CMPT_COUNT];
        InternalResponseData::FwVersionResponse(GetFwVersionResponse {
            header: GetFwVersionResponseHeader::new(1, GetFwVerRespHeaderByte3::NoSpecialFlags),
            component_info: comp_info,
        })
    }

    fn create_offer_rejection() -> InternalResponseData {
        InternalResponseData::OfferResponse(FwUpdateOfferResponse::new_with_failure(
            HostToken::Driver,
            OfferRejectReason::InvalidComponent,
            OfferStatus::Reject,
        ))
    }
}

impl<'a, C: Controller, V: FwOfferValidator> Device for ControllerWithPortStateAndValidator<'a, C, V> {
    fn component_id(&self) -> ComponentId {
        self.component_id
    }

    async fn get_fw_version(&mut self) -> Result<GetFwVersionResponse, InternalResponseData> {
        let version = self.controller.get_active_fw_version().await.map_err(|e| {
            match e {
                Error::Bus(_) => error!("Failed to get active firmware version, bus error"),
                Error::Pd(e) => error!("Failed to get active firmware version: {:?}", e),
            }

            self.create_invalid_fw_version_response()
        })?;

        let dev_inf = FwVerComponentInfo::new(FwVersion::new(version), self.component_id());
        let comp_info: [FwVerComponentInfo; MAX_CMPT_COUNT] = [dev_inf; MAX_CMPT_COUNT];
        Ok(GetFwVersionResponse {
            header: GetFwVersionResponseHeader::new(1, GetFwVerRespHeaderByte3::NoSpecialFlags),
            component_info: comp_info,
        })
    }

    async fn handle_offer(&mut self, offer: &FwUpdateOffer) -> Result<FwUpdateOfferResponse, InternalResponseData> {
        if offer.component_info.component_id != self.component_id() {
            return Err(Self::create_offer_rejection());
        }

        let version = self.controller.get_active_fw_version().await.map_err(|e| {
            match e {
                Error::Bus(_) => error!("Failed to get active firmware version, bus error"),
                Error::Pd(e) => error!("Failed to get active firmware version: {:?}", e),
            }

            Self::create_offer_rejection()
        })?;

        Ok(self.fw_version_validator.validate(FwVersion::new(version), offer))
    }

    async fn handle_content(
        &mut self,
        content: &FwUpdateContentCommand,
    ) -> Result<FwUpdateContentResponse, InternalResponseData> {
        let data = &content.data[0..content.header.data_length as usize];
        debug!("Got content {:#?}", content);
        if content.header.is_first_block() {
            debug!("Got first block");

            // Detach from the power policy so it doesn't attempt to do anything while we are updating
            let mut detached_all = true;
            for power in self.power_devices {
                info!("{:?}: detaching power device (if attached)", self.controller_id);
                if let Err(e) = power.detach().await {
                    error!("{:?}: Failed to detach power device: {:?}", self.controller_id, e);
                    // TODO: sync state?
                    detached_all = false;
                    break;
                }
            }

            if !detached_all {
                error!(
                    "{:?}: Failed to detach all power devices, rejecting offer",
                    self.controller_id
                );

                return Err(InternalResponseData::ContentResponse(FwUpdateContentResponse::new(
                    content.header.sequence_num,
                    CfuUpdateContentResponseStatus::ErrorPrepare,
                )));
            }

            // Need to start the update
            self.ticker.reset();
            match self.controller.start_fw_update().await {
                Ok(()) => {
                    debug!("FW update started successfully");
                    self.controller_state.fw_update_state = FwUpdateState::InProgress(0);
                }
                Err(e) => {
                    match e {
                        Error::Pd(e) => error!("Failed to start FW update: {:?}", e),
                        Error::Bus(_) => error!("Failed to start FW update, bus error"),
                    }

                    self.controller_state.fw_update_state = FwUpdateState::Recovery;
                    return Err(InternalResponseData::ContentResponse(FwUpdateContentResponse::new(
                        content.header.sequence_num,
                        CfuUpdateContentResponseStatus::ErrorPrepare,
                    )));
                }
            }
        }

        self.controller
            .write_fw_contents(content.header.firmware_address as usize, data)
            .await
            .map_err(|e| {
                match e {
                    Error::Pd(e) => error!("Failed to write block: {:?}", e),
                    Error::Bus(_) => error!("Failed to write block, bus error"),
                }

                InternalResponseData::ContentResponse(FwUpdateContentResponse::new(
                    content.header.sequence_num,
                    CfuUpdateContentResponseStatus::ErrorWrite,
                ))
            })?;

        debug!("Block written successfully");

        if content.header.is_last_block() {
            match self.controller.finalize_fw_update().await {
                Ok(()) => {
                    debug!("FW update finalized successfully");
                    self.controller_state.fw_update_state = FwUpdateState::Idle;
                }
                Err(e) => {
                    match e {
                        Error::Pd(e) => error!("Failed to finalize FW update: {:?}", e),
                        Error::Bus(_) => error!("Failed to finalize FW update, bus error"),
                    }

                    self.controller_state.fw_update_state = FwUpdateState::Recovery;
                    return Err(Self::create_offer_rejection());
                }
            }
        }

        Ok(FwUpdateContentResponse::new(
            content.header.sequence_num,
            CfuUpdateContentResponseStatus::Success,
        ))
    }

    async fn abort_update(&mut self) -> Result<(), InternalResponseData> {
        // abort the update process
        match self.controller.abort_fw_update().await {
            Ok(()) => {
                self.controller_state.fw_update_state = FwUpdateState::Idle;
                Ok(())
            }
            Err(e) => {
                match e {
                    Error::Pd(e) => error!("Failed to abort FW update: {:?}", e),
                    Error::Bus(_) => error!("Failed to abort FW update, bus error"),
                }

                self.controller_state.fw_update_state = FwUpdateState::Recovery;
                Err(InternalResponseData::ComponentPrepared) // TODO: better error?
            }
        }
    }

    async fn finalize_update(&mut self) -> Result<(), InternalResponseData> {
        // Something about how UEFI calls finalize isn't useful for us, so we finalize when we get the last content block and just no-op here
        Ok(())
    }

    async fn prepare_for_update(&mut self) -> Result<(), InternalResponseData> {
        // Something about how UEFI calls prepare isn't useful for us, so we prepare when we get the first content block and just no-op here
        Ok(())
    }

    async fn handle_extended_offer(
        &mut self,
        _offer: &FwUpdateOfferExtended,
    ) -> Result<FwUpdateOfferResponse, InternalResponseData> {
        Err(Self::create_offer_rejection())
    }

    async fn handle_offer_information(
        &mut self,
        _info: &FwUpdateOfferInformation,
    ) -> Result<FwUpdateOfferResponse, InternalResponseData> {
        Err(Self::create_offer_rejection())
    }
}

impl<'a, M: RawMutex, C: Controller, V: FwOfferValidator> ControllerWrapper<'a, M, C, V> {
    /// Process a CFU tick
    pub async fn process_cfu_tick(&self, controller: &mut C, state: &mut dyn DynPortState<'_>) {
        match state.controller_state_mut().fw_update_state {
            FwUpdateState::Idle => {
                // No FW update in progress, nothing to do
                return;
            }
            FwUpdateState::InProgress(ticks) => {
                if ticks + 1 < DEFAULT_FW_UPDATE_TIMEOUT_TICKS {
                    trace!("CFU tick: {}", ticks);
                    state.controller_state_mut().fw_update_state = FwUpdateState::InProgress(ticks + 1);
                    return;
                } else {
                    error!("FW update timed out after {} ticks", ticks);
                }
            }
            FwUpdateState::Recovery => {
                // Continue recovery process
            }
        };

        // Update timed out, attempt to exit the FW update
        state.controller_state_mut().fw_update_state = FwUpdateState::Recovery;
        match controller.abort_fw_update().await {
            Ok(_) => {
                debug!("FW update aborted successfully");
            }
            Err(e) => {
                match e {
                    Error::Pd(e) => error!("Failed to abort FW update: {:?}", e),
                    Error::Bus(_) => error!("Failed to abort FW update, bus error"),
                }

                return;
            }
        }

        state.controller_state_mut().fw_update_state = FwUpdateState::Idle;
    }

    /// Process a CFU command
    pub async fn process_cfu_command(
        &self,
        controller: &mut C,
        state: &mut dyn DynPortState<'_>,
        command: &RequestData,
    ) -> InternalResponseData {
        if state.controller_state().fw_update_state == FwUpdateState::Recovery {
            debug!("FW update in recovery state, rejecting command");
            return InternalResponseData::ComponentBusy;
        }

        let mut ticker = self.fw_update_ticker.lock().await;
        let mut device = ControllerWithPortStateAndValidator {
            component_id: self.registration.cfu_device.component_id(),
            controller_id: self.registration.pd_controller.id(),
            power_devices: self.registration.power_devices,
            controller,
            controller_state: state.controller_state_mut(),
            fw_version_validator: &self.fw_version_validator,
            ticker: &mut ticker,
        };

        match command {
            RequestData::FwVersionRequest => {
                debug!("Got FwVersionRequest");
                device
                    .get_fw_version()
                    .await
                    .map_or_unwrap_err(InternalResponseData::FwVersionResponse)
            }
            RequestData::GiveOffer(offer) => {
                debug!("Got GiveOffer");
                device
                    .handle_offer(offer)
                    .await
                    .map_or_unwrap_err(InternalResponseData::OfferResponse)
            }
            RequestData::GiveContent(content) => {
                debug!("Got GiveContent");
                device
                    .handle_content(content)
                    .await
                    .map_or_unwrap_err(InternalResponseData::ContentResponse)
            }
            RequestData::AbortUpdate => {
                debug!("Got AbortUpdate");
                device.abort_update().await.map_or_unwrap_err(|()| {
                    debug!("FW update aborted successfully");
                    InternalResponseData::ComponentPrepared
                })
            }
            RequestData::FinalizeUpdate => {
                debug!("Got FinalizeUpdate");
                device.finalize_update().await.map_or_unwrap_err(|()| {
                    debug!("FW update finalized successfully");
                    InternalResponseData::ComponentPrepared
                })
            }
            RequestData::PrepareComponentForUpdate => {
                debug!("Got PrepareComponentForUpdate");
                device.prepare_for_update().await.map_or_unwrap_err(|()| {
                    debug!("Component prepared for update successfully");
                    InternalResponseData::ComponentPrepared
                })
            }
            RequestData::GiveOfferExtended(offer) => {
                debug!("Got GiveExtendedOffer");
                device
                    .handle_extended_offer(offer)
                    .await
                    .map_or_unwrap_err(InternalResponseData::OfferResponse)
            }
            RequestData::GiveOfferInformation(offer_info) => {
                debug!("Got GiveOfferInformation");
                device
                    .handle_offer_information(offer_info)
                    .await
                    .map_or_unwrap_err(InternalResponseData::OfferResponse)
            }
        }
    }

    /// Sends a CFU response to the command
    pub async fn send_cfu_response(&self, response: InternalResponseData) {
        self.registration.cfu_device.send_response(response).await;
    }

    /// Wait for a CFU command
    ///
    /// Returns None if the FW update ticker has ticked
    /// DROP SAFETY: No state that needs to be restored
    pub async fn wait_cfu_command(&self) -> EventCfu {
        // Only lock long enough to grab our state
        let fw_update_state = self.state.lock().await.controller_state().fw_update_state;
        match fw_update_state {
            FwUpdateState::Idle => {
                // No FW update in progress, just wait for a command
                EventCfu::Request(self.registration.cfu_device.wait_request().await)
            }
            FwUpdateState::InProgress(_) => {
                match select(
                    self.registration.cfu_device.wait_request(),
                    self.fw_update_ticker.lock().await.next(),
                )
                .await
                {
                    Either::First(command) => EventCfu::Request(command),
                    Either::Second(_) => {
                        debug!("FW update ticker ticked");
                        EventCfu::RecoveryTick
                    }
                }
            }
            FwUpdateState::Recovery => {
                // Recovery state, wait for the next attempt to recover the device
                self.fw_update_ticker.lock().await.next().await;
                debug!("FW update ticker ticked");
                EventCfu::RecoveryTick
            }
        }
    }
}
