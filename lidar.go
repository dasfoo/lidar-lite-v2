package lidar

import (
	"errors"
	"fmt"
	"time"

	"github.com/dasfoo/i2c"
)

// TODO: add mutex

// HealthError is returned when LIDAR has unhealthy status
type HealthError struct {
	healthFlags byte
}

// HealthError GetHealthFlags() bit fields
const (
	UnhealthyReference     byte = 1 << 1
	UnhealthyTransmitPower      = 1 << 2
	UnhealthyDC                 = 1 << 3
)

// GetHealthFlags returns bitmask of which parts of LIDAR are unhealthy
func (e *HealthError) GetHealthFlags() byte {
	return e.healthFlags
}

// Error description, human-readable
func (e *HealthError) Error() string {
	if (e.healthFlags & UnhealthyReference) != 0 {
		return "LIDAR unhealthy: reference signal failure"
	}
	if (e.healthFlags & UnhealthyTransmitPower) != 0 {
		return "LIDAR unhealthy: transmit power failure"
	}
	if (e.healthFlags & UnhealthyDC) != 0 {
		return "LIDAR unhealthy: DC (preamplifier) failure"
	}
	return "LIDAR unhealthy, measurement result may be inaccurate"
}

// Lidar is a structure to access basic functions of LIDAR-Lite V2 "Blue Label".
// Documentation at http://lidarlite.com/docs/v2/specs_and_hardware
// Tested on model LL-905-PIN-02.
type Lidar struct {
	bus         i2c.Bus
	address     byte
	WaitTimeout time.Duration
}

// DefaultAddress is a default i2c slave address of LIDAR-Lite v2
const DefaultAddress = 0x62

const (
	customAcquisitionInterval = 1 << 5
	velocityModeEnabled       = 1 << 7
	modeControlRegister       = 0x04
	defaultIntervalValue      = 0xc8
	minIntervalValue          = 0x02 // minimal interval value for proper operation
	maxIntervalValue          = 0xff
)

// Values for use with SetDistanceAndVelocityMode and SetContinuousMode.
const (
	DefaultAcquisitionInterval = (defaultIntervalValue >> 1) * time.Millisecond
	MinAcquisitionInterval     = (minIntervalValue >> 1) * time.Millisecond
	MaxAcquisitionInterval     = (maxIntervalValue >> 1) * time.Millisecond
)

// InfiniteAcquisitions passed to SetContinuousMode makes LIDAR measure distance infinitely
const InfiniteAcquisitions = 0xff

// GetStatus() bits
const (
	// Busy with acquisition
	Busy = 1 << iota

	// ReferenceOverflow indicates that the Maximum Acquisition Count (register 0x02) has not been
	// reached because the signal received has reached maximum strength
	ReferenceOverflow = 1 << iota

	// SignalOverflow - see description for ReferenceOverflow
	SignalOverflow = 1 << iota

	// SignalNotDetected - signal correlation peak is equal to or below correlation record threshold
	SignalNotDetected = 1 << iota

	// SecondReturn - second peak above correlation noise floor threshold has been detected
	SecondReturn = 1 << iota

	// Healthy status indicates that preamplifier (DC) is operating properly,
	// transmit power is active and a reference pulse has been processed and has been stored
	Healthy = 1 << iota

	// ErrorDetected and measurement is invalid
	ErrorDetected = 1 << iota

	// EyeSafetyActivated is when safe average power use has been exceeded and limit is in place
	EyeSafetyActivated = 1 << iota
)

// NewLidar resets the sensor and returns all registers to defaults
func NewLidar(bus i2c.Bus, addr byte) *Lidar {
	return &Lidar{
		bus:         bus,
		address:     addr,
		WaitTimeout: 2 * time.Second,
	}
}

func (ls *Lidar) waitReadyStatus() (status byte, err error) {
	startedAt := time.Now()
	backoff := time.Millisecond
	for {
		status, err = ls.GetStatus()
		if err == nil && (status&Busy) == 0 {
			return
		}
		if time.Since(startedAt) >= ls.WaitTimeout {
			break
		}
		time.Sleep(backoff)
		backoff *= 2
	}
	if err == nil {
		err = errors.New("Timed out waiting for non-Busy LIDAR status")
	}
	return
}

func (ls *Lidar) waitReadyForCommand() error {
	_, err := ls.waitReadyStatus()
	return err
}

func (ls *Lidar) waitAcquisitionReady() error {
	status, err := ls.waitReadyStatus()
	if err == nil {
		if (status & Healthy) == 0 {
			healthFlags, _ := ls.bus.ReadByteFromReg(ls.address, 0x48)
			err = &HealthError{healthFlags: healthFlags}
		}
		if (status & ErrorDetected) != 0 {
			err = errors.New("LIDAR has detected an error during measurement")
		}
		if (status & SignalNotDetected) != 0 {
			err = errors.New("LIDAR has not received it's signal")
		}
	}
	return err
}

// Reset re-loads FPGA from internal Flash memory:
// run a self-test, reset all registers to default values, go into sleep mode (< 10mA).
func (ls *Lidar) Reset() error {
	// Make sure the LIDAR is awake and accepts the next command.
	if err := ls.Wake(); err != nil {
		return err
	}
	if err := ls.bus.WriteByteToReg(ls.address, 0x00, 0x00); err != nil {
		return err
	}
	return ls.waitReadyForCommand()
}

// Sleep puts LIDAR into low power consumption mode.
// Use Wake() before sending any other command.
func (ls *Lidar) Sleep() error {
	return ls.bus.WriteByteToReg(ls.address, 0x65, 0x0f)
}

// Wake LIDAR from the sleep state by sending dummy command and enabling sensors
func (ls *Lidar) Wake() error {
	_, _ = ls.GetStatus()
	return ls.bus.WriteByteToReg(ls.address, 0x65, 0x00)
}

// GetStatus gets Mode/Status of sensor
func (ls *Lidar) GetStatus() (byte, error) {
	value, err := ls.bus.ReadByteFromReg(ls.address, 0x01)
	return value, err
}

func (ls *Lidar) setAcquisitionInterval(interval time.Duration, velocity bool) error {
	control, err := ls.bus.ReadByteFromReg(ls.address, modeControlRegister)
	if err != nil {
		return err
	}
	if velocity {
		control |= velocityModeEnabled
	} else {
		control &^= velocityModeEnabled
	}
	if interval == DefaultAcquisitionInterval {
		if err := ls.bus.WriteByteToReg(ls.address, modeControlRegister,
			control&^customAcquisitionInterval); err != nil {
			return err
		}
	} else {
		// 0xc8 corresponds to 10Hz, 0x14 corresponds to 100Hz.
		translatedInterval := interval.Nanoseconds() * 2 / 1000000
		if translatedInterval < minIntervalValue || translatedInterval > maxIntervalValue {
			return fmt.Errorf("Specified measurement interval %v is not achievable", interval)
		}
		if err := ls.bus.WriteByteToReg(ls.address, 0x45, byte(translatedInterval)); err != nil {
			return err
		}
		if err := ls.bus.WriteByteToReg(ls.address, modeControlRegister,
			control|customAcquisitionInterval); err != nil {
			return err
		}
	}
	return nil
}

func (ls *Lidar) setAcquisitionCount(count byte) error {
	return ls.bus.WriteByteToReg(ls.address, 0x11, count)
}

// SetDistanceOnlyMode sets LIDAR to single distance value acquisition mode.
// Call sequence:
//   SetDistanceOnlyMode()
//   Acquire(...)
//   ReadDistance()
func (ls *Lidar) SetDistanceOnlyMode() error {
	if err := ls.setAcquisitionInterval(DefaultAcquisitionInterval, false); err != nil {
		return err
	}
	return ls.setAcquisitionCount(0)
}

// SetContinuousMode prepares LIDAR registers for continuous distance and velocity measurement.
// Call sequence:
//   SetContinuousMode(...)
//   Acquire(...)
//   in a loop: ReadDistance() / ReadVelocity()
func (ls *Lidar) SetContinuousMode(total byte, interval time.Duration) error {
	if err := ls.setAcquisitionInterval(interval, false); err != nil {
		return err
	}
	return ls.setAcquisitionCount(total)
}

// SetDistanceAndVelocityMode prepares LIDAR for distance and velocity measurement.
// Lower window values decrease velocity precision.
// Call sequence:
//   SetDistanceAndVelocityMode(...)
//   Acquire(...)
//   ReadDistance()
//   ReadVelocity()
func (ls *Lidar) SetDistanceAndVelocityMode(window time.Duration) error {
	if err := ls.setAcquisitionInterval(window, true); err != nil {
		return err
	}
	return ls.setAcquisitionCount(0)
}

// Acquire instructs LIDAR to acquire a measurement.
// Set stabilizePreamp to enable DC correction; otherwise it will be faster,
// but you need to stabilize DC, i.e. Acquire(true), ~ 1 out of every 100 readings.
// The result might be fetched with ReadDistance().
func (ls *Lidar) Acquire(stablizePreamp bool) error {
	if err := ls.waitReadyForCommand(); err != nil {
		return err
	}
	command := byte(0x03)
	if stablizePreamp {
		command = 0x04
	}
	return ls.bus.WriteByteToReg(ls.address, 0x00, command)
}

// ReadDistance waits until acquisition is complete and reads distance. The unit is meters.
// If error is HealthError, the result is read but may be inaccurate.
func (ls *Lidar) ReadDistance() (uint16, error) {
	healthError := ls.waitAcquisitionReady()
	if healthError != nil {
		if _, ok := healthError.(*HealthError); !ok {
			return 0, healthError
		}
	}
	value, err := ls.bus.ReadWordFromReg(ls.address, 0x8f)
	if err != nil {
		return 0, err
	}
	return value, healthError
}

// GetDistance is a convenience method to get a single distance measurement from the LIDAR.
// It is basically Acquire(true) and ReadDistance(), in an error retry loop until WaitTimeout.
func (ls *Lidar) GetDistance() (value uint16, err error) {
	startedAt := time.Now()
	if err = ls.setAcquisitionCount(0); err != nil {
		return
	}
	for {
		if err = ls.Acquire(true); err == nil {
			if value, err = ls.ReadDistance(); err == nil {
				return
			}
		}
		if time.Since(startedAt) >= ls.WaitTimeout {
			break
		}
	}
	return
}

// ReadVelocity waits until acquisition is complete and reads velocity. Unit is cm/s.
// If error implements HealthError, the result is read but may be inaccurate.
func (ls *Lidar) ReadVelocity() (int16, error) {
	healthError := ls.waitAcquisitionReady()
	if healthError != nil {
		if _, isHealthError := healthError.(*HealthError); !isHealthError {
			return 0, healthError
		}
	}
	control, err := ls.bus.ReadByteFromReg(ls.address, modeControlRegister)
	if err != nil {
		return 0, err
	}
	scale := byte(defaultIntervalValue)
	if (control & customAcquisitionInterval) != 0 {
		if scale, err = ls.bus.ReadByteFromReg(ls.address, 0x45); err != nil {
			return 0, err
		}
	}
	valueUnscaled, err := ls.bus.ReadByteFromReg(ls.address, 0x09)
	if err != nil {
		return 0, err
	}
	// valueUnscaled is read as byte (uint8), but in fact is signed (int8)
	value := int16(int(int8(valueUnscaled)) * int(scale) / 20)
	return value, healthError
}

// GetVersion gets hardware and software revision of the LIDAR-Lite.
func (ls *Lidar) GetVersion() (hw byte, sw byte, err error) {
	if hw, err = ls.bus.ReadByteFromReg(ls.address, 0x41); err == nil {
		sw, err = ls.bus.ReadByteFromReg(ls.address, 0x4f)
	}
	return
}
