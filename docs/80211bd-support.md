# 802.11bd support

This patch adds an 802.11bd/NGV vehicular Wi-Fi path to the automotive module.
It is intended for V2X simulations that already use the ns-3 WAVE/OCB stack and
need to compare 802.11p, 802.11bd native NGV transmissions, and 802.11bd
legacy-compatible fallback modes.

## What is available

- A new `WIFI_STANDARD_80211bd` Wi-Fi standard entry for the vehicular OCB path.
- 802.11bd TXVECTOR metadata for PPDU format, NGV-MCS, spatial streams,
  midamble periodicity, NGV-LTF type, LDPC, and `NON_NGV_10` repetitions.
- Native NGV PPDU handling for 10 MHz and 20 MHz channel profiles.
- Legacy-compatible `NON_NGV_10` and `NON_NGV_20_DUPLICATE` fallback modes.
- Basic 802.11p/802.11bd interoperability behavior:
  - 802.11bd receivers can receive supported non-NGV fallback transmissions.
  - 802.11p receivers do not decode native NGV transmissions.
  - 802.11p receivers can decode compatible non-NGV fallback transmissions.
- 802.11bd profile helpers for configuring PHY, remote station manager, and
  metrics consistently.
- A bounded fallback controller for switching between native NGV, duplicated
  non-NGV 20 MHz fallback, and primary-only non-NGV 10 MHz fallback.
- A modular rate controller that can select 802.11bd NGV-MCS values and can
  reuse the same SNR-threshold policy shape for 802.11p OFDM data rates.
- CBR and DCC accounting paths updated to handle 802.11bd Wi-Fi devices.

The implementation is a network-simulation model integrated into the existing
Yans/WAVE-style automotive stack. It does not attempt to provide a full
waveform-level 802.11bd PHY.

## Helper API

Use `VehicularWifiProfile` from `ns3/vehicular-wifi-helper.h` to configure
common vehicular Wi-Fi profiles:

```cpp
VehicularWifiProfile profile = VehicularWifiProfile::Ieee80211bd ();
profile.ConfigurePhy (wifiPhy);
profile.ConfigureRemoteStationManager (wifi);
```

Useful factory methods include:

- `VehicularWifiProfile::Ieee80211p()`
- `VehicularWifiProfile::Ieee80211bd()`
- `VehicularWifiProfile::Ieee80211bd20()`
- `VehicularWifiProfile::Ieee80211bdNonNgv10()`
- `VehicularWifiProfile::Ieee80211bdNonNgv20Duplicate()`

For explicit fallback selection, use `VehicularWifiFallbackController`:

```cpp
auto mode = VehicularWifiFallbackController::SelectForLinkQuality (policy);
VehicularWifiProfile profile = VehicularWifiFallbackController::MakeProfile (mode);
```

For MCS/rate decisions, use `VehicularWifiRateController`:

```cpp
VehicularWifiRateController::Policy policy;
policy.linkQuality.estimatedSnrDb = 30.0;

auto decision = VehicularWifiRateController::Select (policy);
VehicularWifiProfile profile = VehicularWifiRateController::MakeProfile (decision);
```

The default 802.11bd rate policy selects native NGV MCS 7, 3, or 0 according to
explicit SNR thresholds, then falls back to `NON_NGV_20_DUPLICATE` or
`NON_NGV_10` through the fallback controller.

## Examples

The following examples exercise the new 802.11bd path:

- `v2v-simple-cam-exchange-80211bd`
- `v2v-congestion-80211bd`
- `v2v-marginal-snr-80211bd`
- `v2v-nss2-comparison-80211bd`
- `v2v-runtime-fallback-80211bd`
- `v2v-link-quality-fallback-80211bd`
- `v2v-observed-fallback-80211bd`
- `v2v-trace-observed-fallback-80211bd`
- `v2v-rate-control-80211bd`
- `v2v-coexistence-80211bd-nrv2x`

Example runs:

```bash
./ns3 run "v2v-simple-cam-exchange-80211bd --sumo-gui=false --sim-time=1"
./ns3 run "v2v-rate-control-80211bd"
./ns3 run "v2v-runtime-fallback-80211bd"
```

## Tests

The `vehicular-wifi-profile` test suite covers the core 802.11bd behavior:

```bash
./build/utils/ns3-dev-test-runner-optimized --suite=vehicular-wifi-profile --verbose
```

The suite includes tests for:

- profile metadata and installed PHY settings;
- NGV-MCS table validity;
- TXVECTOR metadata;
- `NON_NGV_10` repetition behavior;
- 802.11p/802.11bd interoperability;
- NGV error-rate model behavior;
- CBR/DCC integration;
- fallback and rate-control decisions.

## Modeling limitations

The current model uses static NGV PER and midamble/Doppler parameters in
`src/automotive/model/SignalInfo/WiFi/ngv/ngv-calibration.h`. These values are
modeling assumptions, not a calibrated link-level reference.

The following items are intentionally left for future work:

- calibrated PER-vs-SNR curves for NGV modes;
- waveform-level duplicate combining;
- full receiver-capability discovery;
- failed-reception evidence for closed-loop rate control;
- CBR-aware rate-control constraints;
- a detailed MIMO PHY model for NSS greater than one.
