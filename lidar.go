package lidar

import (
	"errors"
	"fmt"
	"log"
	"time"

	"github.com/dasfoo/i2c"
)

// TODO: add mutex

// Lidar is a structure to access basic functions of LIDAR-Lite V2 "Blue Label".
// Documentation at http://lidarlite.com/docs/v2/specs_and_hardware
// Tested on model LL-905-PIN-02.
type Lidar struct {
	bus         *i2c.Bus
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

const (
	detailedHealthReference     = 1 << 1
	detailedHealthTransmitPower = 1 << 2
	detailedHealthDC            = 1 << 3
)

// NewLidar resets the sensor and returns all registers to defaults
func NewLidar(bus *i2c.Bus, addr byte) *Lidar {
	ls := &Lidar{
		bus:         bus,
		address:     addr,
		WaitTimeout: 2 * time.Second,
	}
	return ls
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
	log.Println("waitReadyStatus:", err)
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
			err = errors.New("LIDAR has failed to reach Healthy status")
			if detailedHealth, hErr := ls.bus.ReadByteFromReg(ls.address, 0x48); hErr == nil {
				if (detailedHealth & detailedHealthReference) == 0 {
					err = errors.New("LIDAR unhealthy: reference signal failure")
				}
				if (detailedHealth & detailedHealthTransmitPower) == 0 {
					err = errors.New("LIDAR unhealthy: transmit power failure")
				}
				if (detailedHealth & detailedHealthDC) == 0 {
					err = errors.New("LIDAR unhealthy: DC (preamplifier) failure")
				}
			}
		}
		if (status & ErrorDetected) != 0 {
			err = errors.New("LIDAR has detected an error during measurement")
		}
		if (status & SignalNotDetected) != 0 {
			err = errors.New("LIDAR has not received it's signal")
		}
	}
	log.Println("waitAcquisitionReady:", err)
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
	log.Printf("GetStatus: %.8b\n", value)
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
func (ls *Lidar) ReadDistance() (uint16, error) {
	if err := ls.waitAcquisitionReady(); err != nil {
		return 0, err
	}
	return ls.bus.ReadWordFromReg(ls.address, 0x8f)
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
func (ls *Lidar) ReadVelocity() (value int16, err error) {
	if err = ls.waitAcquisitionReady(); err != nil {
		return
	}
	control, err := ls.bus.ReadByteFromReg(ls.address, modeControlRegister)
	if err != nil {
		return
	}
	scale := byte(defaultIntervalValue)
	if (control & customAcquisitionInterval) != 0 {
		scale, err = ls.bus.ReadByteFromReg(ls.address, 0x45)
		if err != nil {
			return
		}
	}
	valueUnscaled, err := ls.bus.ReadByteFromReg(ls.address, 0x09)
	// valueUnscaled is read as byte (uint8), but in fact is signed (int8)
	value = int16(int(int8(valueUnscaled)) * int(scale) / 20)
	return
}

// GetVersion gets hardware and software revision of the LIDAR-Lite.
func (ls *Lidar) GetVersion() (hw byte, sw byte, err error) {
	if hw, err = ls.bus.ReadByteFromReg(ls.address, 0x41); err == nil {
		sw, err = ls.bus.ReadByteFromReg(ls.address, 0x4f)
	}
	return
}
