"""
rov_sensor.py - Realistic IMU/AHRS sensor model for the SMC feedback path.

The SMC controller in rov_smc.py normally closes the loop on the *true* simulated
state, which is unrealistically clean. In reality the controller only sees the
output of an onboard attitude/heading reference system (AHRS) that fuses a triax
gyroscope, accelerometer and magnetometer through an EKF running at 500 Hz.

This module turns the true (attitude, rates) coming out of the plant integrator
into the *measured* (attitude, rates) that the controller actually receives.
Every error source in the sensor data-sheet is modelled so that closing the loop
on the corrupted feedback degrades the controller the way it would on the real
ROV.

Modelled error sources (all derived from the supplied sensor data-sheet):

  Gyroscope (rate channel)
    - Broadband white noise           rate noise density 0.0005 deg/s/rtHz
    - Total RMS noise floor           0.06 deg/s-rms
    - Turn-on bias                    drawn per axis, slowly drifting
    - Bias temperature drift          up to 20 deg/s across -40..+85 C
    - Sensitivity (scale) temp drift  0.004 %/C
    - Scale factor nonlinearity       0.2 % of full scale (2000 deg/s)
    - Cross-axis sensitivity          +/-2 %
    - Saturation                      +/-2000 deg/s
    - Output quantization             configurable

  Attitude (EKF output, fuses gyro + accel + mag)
    - Static accuracy                 +/-1 deg roll/pitch, +/-3 deg yaw
    - Dynamic accuracy                +/-3 deg roll/pitch, +/-5 deg yaw
    - Repeatability (bias-like floor) 0.5 deg
    - Resolution / quantization       < 0.01 deg
    - Output rate                     500 Hz EKF (resampled to sim dt)

  Accelerometer (informational - drives attitude accuracy above)
    - Rate noise density              400 ug/rtHz
    - Bias temp drift                 0.75 mg/C (X,Y), 1.50 mg/C (Z)
    - Dynamic range                   +/-8 g

  Magnetometer (informational - drives yaw accuracy above)
    - Dynamic range                   +/-1200 uT
    - Initial scale factor tol        +/-4 %
    - Initial bias tolerance          +/-300 uT

The public API is:

    SensorParams    - dataclass holding every tunable sensor spec
    SensorModel     - stateful model; call .measure(attitude, rates, t) per step
                      to obtain (attitude_meas, rates_meas)

All angles are stored in radians internally to match rov_smc.py conventions.
The data-sheet values are exposed in degrees in SensorParams for readability.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional, Tuple

import numpy as np


# ──────────────────────────────────────────────
# Sensor parameters
# ──────────────────────────────────────────────

@dataclass
class SensorParams:
    """All tunable sensor specifications, in engineering units.

    Defaults reproduce the supplied AHRS data-sheet. Every field can be
    overridden from the GUI or programmatically to perform sensitivity studies.
    """

    # ── Output / EKF ──
    # Internal EKF update rate [Hz]. Used only to compute the discrete noise
    # statistics when the simulation runs at a different dt.
    ekf_rate_hz: float = 500.0
    # Operating temperature [degC]. Bias drift is referenced to 25 degC.
    temperature_c: float = 25.0
    # Reference (calibration) temperature [degC].
    ref_temperature_c: float = 25.0

    # ── Attitude (EKF output) accuracy, 1-sigma per axis (roll, pitch, yaw) [deg] ──
    # Static accuracy from data-sheet: +/-1 deg RP, +/-3 deg yaw.
    attitude_static_std_deg: Tuple[float, float, float] = (1.0, 1.0, 3.0)
    # Dynamic accuracy from data-sheet: +/-3 deg RP, +/-5 deg yaw.
    attitude_dynamic_std_deg: Tuple[float, float, float] = (3.0, 3.0, 5.0)
    # Rate magnitude above which we treat the channel as 'dynamic' [deg/s].
    dynamic_rate_threshold_deg_per_s: float = 5.0
    # Repeatability (slowly varying attitude bias floor) [deg, 1-sigma].
    attitude_repeatability_deg: float = 0.5
    # Output resolution / quantization step [deg]. Spec: < 0.01 deg.
    attitude_resolution_deg: float = 0.01
    # Disable attitude quantization if False (useful for sensitivity studies).
    enable_attitude_quantization: bool = True

    # ── Gyroscope (rate channel) ──
    # Broadband rate noise density [deg/s/rtHz].
    gyro_rate_noise_density_deg_per_s_per_rtHz: float = 5.0e-4
    # Integrated RMS noise floor at the sensor bandwidth [deg/s-rms].
    # Used as a hard cap on the discrete white-noise std.
    gyro_total_rms_noise_deg_per_s: float = 0.06
    # Full-scale range [deg/s]. Saturation applied per sample.
    gyro_dynamic_range_deg_per_s: float = 2000.0
    # Turn-on bias 1-sigma per axis [deg/s]. Factory-calibrated residual.
    gyro_turn_on_bias_std_deg_per_s: float = 0.05
    # In-run bias random walk coefficient [deg/s/rt(s)] (Allan variance floor).
    gyro_bias_instability_deg_per_s_per_rt_s: float = 0.005
    # Maximum bias drift across the full temperature span (-40..+85 degC) [deg/s].
    gyro_bias_temp_drift_max_deg_per_s: float = 20.0
    # Span of the temperature range used for the drift_max spec [degC].
    gyro_bias_temp_span_degC: float = 125.0
    # Sensitivity (scale factor) change vs temperature [%/degC].
    gyro_sensitivity_temp_pct_per_degC: float = 0.004
    # Initial scale factor tolerance (turn-on) [%]. Spec only gives temp drift,
    # so we use a small default for full scale-factor error modelling.
    gyro_scale_factor_tol_pct: float = 0.1
    # Scale factor nonlinearity, fraction of full scale [fraction].
    # Spec: 0.2 % FS.
    gyro_nonlinearity_pct_fs: float = 0.2
    # Cross-axis sensitivity magnitude (off-diagonal coupling) [fraction].
    gyro_cross_axis_sensitivity: float = 0.02
    # Output resolution / quantization step [deg/s]. Typical MEMS is well below
    # the noise floor; left configurable.
    gyro_rate_resolution_deg_per_s: float = 0.001
    enable_rate_quantization: bool = True

    # ── Accelerometer (informational; modelled as a contributor to attitude error) ──
    accel_rate_noise_density_ug_per_rtHz: float = 400.0
    accel_dynamic_range_g: float = 8.0
    accel_bias_temp_drill_mg_per_degC_xy: float = 0.75
    accel_bias_temp_drill_mg_per_degC_z: float = 1.50
    accel_sensitivity_temp_pct_per_degC: float = 0.02

    # ── Magnetometer (informational; modelled as a contributor to yaw error) ──
    mag_dynamic_range_ut: float = 1200.0
    mag_scale_factor_tol_pct: float = 4.0
    mag_bias_tol_ut: float = 300.0

    # ── Master switches ──
    # If False, the model degrades to a noise-free passthrough regardless of
    # the individual enable flags below.
    enable: bool = True
    enable_rate_noise: bool = True
    enable_rate_bias: bool = True
    enable_rate_bias_drift: bool = True
    enable_rate_scale_factor: bool = True
    enable_rate_cross_axis: bool = True
    enable_rate_nonlinearity: bool = True
    enable_rate_saturation: bool = True
    enable_attitude_noise: bool = True
    enable_attitude_bias: bool = True

    # ── RNG ──
    seed: Optional[int] = 7


# ──────────────────────────────────────────────
# Helpers
# ──────────────────────────────────────────────

def _as_rad_array(values, default):
    """Coerce a scalar / tuple / array of degree values to a 3-vector in radians."""
    if values is None:
        values = default
    arr = np.atleast_1d(np.asarray(values, dtype=float)).reshape(-1)
    if arr.size == 1:
        arr = np.repeat(arr, 3)
    elif arr.size != 3:
        raise ValueError(f"Expected a 3-vector, got shape {arr.shape}.")
    return np.deg2rad(arr)


def _quantize(values: np.ndarray, step: np.ndarray) -> np.ndarray:
    """Quantize `values` to integer multiples of `step` (per-element)."""
    if np.any(step <= 0.0):
        return values
    return np.round(values / step) * step


# ──────────────────────────────────────────────
# Sensor model
# ──────────────────────────────────────────────

class SensorModel:
    """Stateful IMU/AHRS sensor model.

    Construct once with a SensorParams instance and the simulation time-step,
    then call ``measure(attitude_true, rates_true, t)`` once per simulation
    step. The model carries running state (bias random walk, filtered noise)
    so consecutive calls are time-correlated exactly like a real sensor.

    Parameters
    ----------
    params : SensorParams
        Sensor specifications.
    dt : float
        Simulation time-step in seconds. The discrete white-noise statistics are
        derived from the continuous-time noise densities at this rate.
    seed : int, optional
        Overrides ``params.seed`` if provided (useful when the same physical
        sensor should be re-used across two simulations with different seeds).
    """

    def __init__(self, params: SensorParams, dt: float, seed: Optional[int] = None):
        self.params = params
        self.dt = float(max(dt, 1.0e-6))
        seed = params.seed if seed is None else seed
        self.rng = np.random.RandomState(seed)

        # ── Derived noise statistics ──
        # Effective noise bandwidth at the simulation rate (Nyquist).
        bw_hz = 0.5 / self.dt
        # Continuous-time noise density -> discrete white noise std.
        rate_noise_std_rad_s = math.radians(
            params.gyro_rate_noise_density_deg_per_s_per_rtHz * math.sqrt(bw_hz)
        )
        # Cap by the integrated RMS noise floor so the model never exceeds it.
        total_rms_rad_s = math.radians(params.gyro_total_rms_noise_deg_per_s)
        if total_rms_rad_s > 0.0:
            rate_noise_std_rad_s = min(rate_noise_std_rad_s, total_rms_rad_s)
        self._rate_noise_std = rate_noise_std_rad_s

        # Attitude noise std (static and dynamic) in rad.
        self._att_noise_std_static = _as_rad_array(
            params.attitude_static_std_deg, (1.0, 1.0, 3.0)
        )
        self._att_noise_std_dynamic = _as_rad_array(
            params.attitude_dynamic_std_deg, (3.0, 3.0, 5.0)
        )
        self._dynamic_rate_threshold = math.radians(params.dynamic_rate_threshold_deg_per_s)

        # ── Persistent bias / scale states ──
        # Gyro turn-on bias (constant per power-cycle, per axis).
        self.rate_bias = self.rng.normal(
            0.0,
            math.radians(params.gyro_turn_on_bias_std_deg_per_s),
            size=3,
        )
        # Attitude bias (slowly varying offset, drawn from repeatability spec).
        self.attitude_bias = self.rng.normal(
            0.0,
            math.radians(params.attitude_repeatability_deg),
            size=3,
        )

        # Rate scale factor: product of turn-on tolerance and temperature sensitivity.
        dT = params.temperature_c - params.ref_temperature_c
        sf_tol = 1.0 + self.rng.uniform(
            -params.gyro_scale_factor_tol_pct,
            params.gyro_scale_factor_tol_pct,
            size=3,
        ) / 100.0
        sf_temp = 1.0 + params.gyro_sensitivity_temp_pct_per_degC * dT / 100.0
        self.rate_scale_factor = sf_tol * sf_temp

        # Cross-axis sensitivity matrix (3x3). Diagonal = scale factor; off-diagonal
        # are small random couplings drawn from the +/-cross_axis_sensitivity band.
        ca = params.gyro_cross_axis_sensitivity
        cross = self.rng.uniform(-ca, ca, size=(3, 3))
        np.fill_diagonal(cross, 1.0)
        self.rate_cross_axis = cross

        # Nonlinearity coefficient per axis (fraction of full scale).
        nl_fs = params.gyro_nonlinearity_pct_fs / 100.0
        self._rate_nonlinearity_fs = nl_fs
        self._rate_full_scale = math.radians(params.gyro_dynamic_range_deg_per_s)

        # Saturation limit.
        self._rate_sat = math.radians(params.gyro_dynamic_range_deg_per_s)

        # Quantization steps.
        self._att_quant = math.radians(max(0.0, params.attitude_resolution_deg))
        self._rate_quant = math.radians(max(0.0, params.gyro_rate_resolution_deg_per_s))

        # Bias drift random-walk increment per step.
        # Discrete sigma = continuous_density * sqrt(dt).
        self._rate_bias_drift_step_std = math.radians(
            params.gyro_bias_instability_deg_per_s_per_rt_s
        ) * math.sqrt(self.dt)

        # Temperature-driven bias drift bound (max drift over the full temp span).
        # Per-degree drift coefficient so that |bias_temp(T)| <= drift_max when
        # |T - ref| <= temp_span/2. Drawn once and held constant.
        per_deg = params.gyro_bias_temp_drift_max_deg_per_s / max(
            params.gyro_bias_temp_span_degC, 1.0
        )
        signs = self.rng.choice([-1.0, 1.0], size=3)
        self._rate_bias_temp_coeff = math.radians(per_deg) * signs
        self._ref_temperature_c = params.ref_temperature_c

        # Sliding-window attitude noise filter to mimic the EKF's smoothing.
        # The EKF does not produce pure white noise on attitude; it low-pass
        # filters the gyro integration. A short moving average approximates this.
        self._att_noise_window = 4
        self._att_noise_buffer = np.zeros((self._att_noise_window, 3))
        self._att_noise_idx = 0

        # Running time for diagnostics.
        self._t_last = 0.0
        self._sample_count = 0

    # ──────────────────────────────────────────────
    # Public API
    # ──────────────────────────────────────────────

    def reset(self, seed: Optional[int] = None) -> None:
        """Re-initialize the model state (e.g. between simulations)."""
        if seed is not None:
            self.params.seed = seed
        self.__init__(self.params, self.dt, seed=self.params.seed)

    def set_temperature(self, temperature_c: float) -> None:
        """Update the operating temperature; scale factor and bias drift react."""
        self.params.temperature_c = float(temperature_c)

    def measure(
        self,
        attitude_true: np.ndarray,
        rates_true: np.ndarray,
        t: Optional[float] = None,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Return the sensor-corrupted (attitude_meas, rates_meas).

        Parameters
        ----------
        attitude_true : (3,) array
            True roll/pitch/yaw from the plant integrator [rad].
        rates_true : (3,) array
            True p/q/r from the plant integrator [rad/s].
        t : float, optional
            Current simulation time [s]. Used only for diagnostics; the bias
            random walk is driven by dt, not t.

        Returns
        -------
        attitude_meas, rates_meas : (3,) arrays in [rad] and [rad/s].
        """
        p = self.params
        att = np.asarray(attitude_true, dtype=float).reshape(3)
        rate = np.asarray(rates_true, dtype=float).reshape(3)

        if not p.enable:
            return att.copy(), rate.copy()

        # ──────────────────────────────────────────────
        # Rate channel: gyro chain
        # ──────────────────────────────────────────────
        rate_meas = rate.copy()

        # 1) Cross-axis sensitivity (matrix multiply).
        if p.enable_rate_cross_axis:
            rate_meas = self.rate_cross_axis @ rate_meas

        # 2) Scale factor error (per-axis multiply).
        if p.enable_rate_scale_factor:
            rate_meas = rate_meas * self.rate_scale_factor

        # 3) Nonlinearity (cubic term scaled to full scale).
        if p.enable_rate_nonlinearity and self._rate_nonlinearity_fs > 0.0:
            rate_meas = rate_meas + self._rate_nonlinearity_fs * (
                (rate_meas / self._rate_full_scale) ** 3
            ) * self._rate_full_scale

        # 4) Bias: turn-on + temperature drift + in-run random walk.
        if p.enable_rate_bias:
            # Temperature-driven bias offset (held constant per temperature).
            dT = p.temperature_c - self._ref_temperature_c
            bias_temp = self._rate_bias_temp_coeff * dT
            if p.enable_rate_bias_drift:
                # Random walk on top of the constant offsets.
                self.rate_bias += self.rng.normal(
                    0.0, self._rate_bias_drift_step_std, size=3
                )
            rate_meas = rate_meas + self.rate_bias + bias_temp

        # 5) Broadband white noise.
        if p.enable_rate_noise and self._rate_noise_std > 0.0:
            rate_meas = rate_meas + self.rng.normal(
                0.0, self._rate_noise_std, size=3
            )

        # 6) Saturation (per-axis clip to dynamic range).
        if p.enable_rate_saturation:
            rate_meas = np.clip(rate_meas, -self._rate_sat, self._rate_sat)

        # 7) Output quantization.
        if p.enable_rate_quantization:
            rate_meas = _quantize(rate_meas, self._rate_quant)

        # ──────────────────────────────────────────────
        # Attitude channel: EKF chain
        # ──────────────────────────────────────────────
        att_meas = att.copy()

        # Use the *measured* rate magnitude to decide static vs dynamic regime,
        # exactly as the EKF would (it cannot see the true rate either).
        rate_mag = np.abs(rate_meas)
        att_noise_std = np.where(
            rate_mag > self._dynamic_rate_threshold,
            self._att_noise_std_dynamic,
            self._att_noise_std_static,
        )

        # 1) Bias offset (slowly varying).
        if p.enable_attitude_bias:
            att_meas = att_meas + self.attitude_bias

        # 2) Coloured noise (moving average of white noise -> EKF smoothing).
        if p.enable_attitude_noise and np.any(att_noise_std > 0.0):
            raw = self.rng.normal(0.0, 1.0, size=3) * att_noise_std
            self._att_noise_buffer[self._att_noise_idx] = raw
            self._att_noise_idx = (self._att_noise_idx + 1) % self._att_noise_window
            smoothed = self._att_noise_buffer.mean(axis=0)
            att_meas = att_meas + smoothed

        # 3) Output quantization (resolution).
        if p.enable_attitude_quantization:
            att_meas = _quantize(att_meas, self._att_quant)

        self._sample_count += 1
        if t is not None:
            self._t_last = float(t)

        return att_meas, rate_meas

    # ──────────────────────────────────────────────
    # Convenience batch method (for offline tests / plotting)
    # ──────────────────────────────────────────────

    def measure_batch(
        self,
        attitude_true: np.ndarray,
        rates_true: np.ndarray,
        t: Optional[np.ndarray] = None,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Apply the model sequentially to whole trajectories.

        Useful for post-processing a clean simulation through the sensor model.
        State (bias drift, noise buffer) advances sample-by-sample exactly as
        it would online.
        """
        attitude_true = np.asarray(attitude_true, dtype=float)
        rates_true = np.asarray(rates_true, dtype=float)
        if attitude_true.ndim != 2 or attitude_true.shape[1] != 3:
            raise ValueError("attitude_true must have shape (N, 3).")
        if rates_true.shape != attitude_true.shape:
            raise ValueError("rates_true must have the same shape as attitude_true.")

        n = attitude_true.shape[0]
        att_meas = np.zeros_like(attitude_true)
        rate_meas = np.zeros_like(rates_true)
        for i in range(n):
            ti = float(t[i]) if t is not None else None
            att_meas[i], rate_meas[i] = self.measure(attitude_true[i], rates_true[i], ti)
        return att_meas, rate_meas

    # ──────────────────────────────────────────────
    # Diagnostics
    # ──────────────────────────────────────────────

    def diagnostics(self) -> dict:
        """Return a snapshot of the current sensor state for logging/plotting."""
        return {
            "rate_bias_deg_per_s": np.rad2deg(self.rate_bias),
            "attitude_bias_deg": np.rad2deg(self.attitude_bias),
            "rate_scale_factor": self.rate_scale_factor.copy(),
            "rate_cross_axis": self.rate_cross_axis.copy(),
            "rate_noise_std_deg_per_s": np.rad2deg(self._rate_noise_std),
            "att_noise_std_static_deg": np.rad2deg(self._att_noise_std_static),
            "att_noise_std_dynamic_deg": np.rad2deg(self._att_noise_std_dynamic),
            "temperature_c": self.params.temperature_c,
            "dt": self.dt,
            "sample_count": self._sample_count,
        }


# ──────────────────────────────────────────────
# Factory helpers
# ──────────────────────────────────────────────

def make_sensor_from_specs(
    dt: float,
    enable: bool = True,
    seed: Optional[int] = None,
    **overrides,
) -> SensorModel:
    """Build a SensorModel from the data-sheet defaults with optional overrides."""
    params = SensorParams(enable=enable, **overrides)
    if seed is not None:
        params.seed = seed
    return SensorModel(params, dt=dt)


def passthrough_sensor(dt: float) -> SensorModel:
    """Return a sensor model that reproduces the input exactly (no errors).

    Handy for A/B comparisons: code paths stay identical, only the sensor
    degradation is disabled.
    """
    params = SensorParams(enable=False)
    return SensorModel(params, dt=dt)
