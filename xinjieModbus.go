/*
 * @Author: tang
 * @Date: 2026-05-23
 * @GitHub: Mr-tang0/CTSystem
 * @Description: 信捷PLC Modbus通信客户端，实现与PLC的Modbus TCP通信
 */
package services

import (
	"encoding/binary"
	"fmt"
	"math"
	"strconv"
	"sync"
	"time"

	"github.com/goburrow/modbus"
)

// DataType 定义数据类型枚举
type DataType int

const (
	// Int16 单字整数 (16位)
	Int16 DataType = iota
	// UInt16 单字无符号整数 (16位)
	UInt16
	// Int32 双字整数 (32位)
	Int32
	// UInt32 双字无符号整数 (32位)
	UInt32
	// Int64 四字整数 (64位)
	Int64
	// UInt64 四字无符号整数 (64位)
	UInt64
	// Float32 单精度浮点数 (32位)
	Float32
	// Float64 双精度浮点数 (64位)
	Float64
)

// XinjieClient 信捷PLC客户端
type XinjieClient struct {
	handler *modbus.TCPClientHandler
	client  modbus.Client
	address string
	mu      sync.Mutex
}

// NewXinjieClient 创建信捷PLC客户端实例
func NewXinjieClient() *XinjieClient {
	return &XinjieClient{}
}

// OpenTCP 连接到指定 IP 和 端口 (默认 502)
func (x *XinjieClient) OpenTCP(address string, slaveID byte) error {
	x.mu.Lock()
	defer x.mu.Unlock()

	if x.handler != nil {
		x.handler.Close()
	}

	x.handler = modbus.NewTCPClientHandler(address)
	x.handler.Timeout = 2 * time.Second
	x.handler.SlaveId = slaveID

	err := x.handler.Connect()
	if err != nil {
		return err
	}

	x.client = modbus.NewClient(x.handler)
	x.address = address
	return nil
}

// Close 关闭连接
func (x *XinjieClient) Close() {
	if x.handler != nil {
		x.handler.Close()
	}
}

// ConvertOctalToDecimal 将八进制地址转换为十进制
func (x *XinjieClient) ConvertOctalToDecimal(octalAddr int) (uint16, error) {
	s := strconv.Itoa(octalAddr)
	val, err := strconv.ParseUint(s, 8, 16)
	if err != nil {
		return 0, fmt.Errorf("非法八进制地址: %d (不能包含8或9)", octalAddr)
	}
	return uint16(val), nil
}

// ==================== 线圈读写 (Coil) ====================

// ReadCoil 读取单个线圈 (功能码 02 - 离散输入) 或 (功能码 01 - 线圈状态)
func (x *XinjieClient) ReadCoil(address uint16, isInput bool) (bool, error) {
	if x.client == nil {
		return false, fmt.Errorf("客户端未初始化")
	}
	x.mu.Lock()
	defer x.mu.Unlock()

	var results []byte
	var err error

	if isInput {
		// 功能码 02: 读取离散输入
		results, err = x.client.ReadDiscreteInputs(address, 1)
	} else {
		// 功能码 01: 读取线圈状态
		results, err = x.client.ReadCoils(address, 1)
	}

	if err != nil {
		return false, err
	}

	return (results[0] & 0x01) == 1, nil
}

// ReadCoils 读取多个线圈
func (x *XinjieClient) ReadCoils(address uint16, count uint16, isInput bool) ([]bool, error) {
	if x.client == nil {
		return nil, fmt.Errorf("客户端未初始化")
	}
	x.mu.Lock()
	defer x.mu.Unlock()

	var results []byte
	var err error

	if isInput {
		results, err = x.client.ReadDiscreteInputs(address, count)
	} else {
		results, err = x.client.ReadCoils(address, count)
	}

	if err != nil {
		return nil, err
	}

	coils := make([]bool, count)
	for i := uint16(0); i < count; i++ {
		byteIndex := i / 8
		bitIndex := i % 8
		res := (results[byteIndex] >> bitIndex) & 0x01
		coils[i] = (res == 1)
	}

	return coils, nil
}

// WriteSingleCoil 写入单个线圈 (功能码 05)
func (x *XinjieClient) WriteSingleCoil(address uint16, value bool) error {
	if x.client == nil {
		return fmt.Errorf("客户端未初始化")
	}
	x.mu.Lock()
	defer x.mu.Unlock()

	var coilValue uint16
	if value {
		coilValue = 0xFF00
	} else {
		coilValue = 0x0000
	}
	_, err := x.client.WriteSingleCoil(address, coilValue)
	return err
}

// WriteMultipleCoils 写入多个线圈 (功能码 15)
func (x *XinjieClient) WriteMultipleCoils(address uint16, values []bool) error {
	if x.client == nil {
		return fmt.Errorf("客户端未初始化")
	}

	count := uint16(len(values))
	byteCount := (count + 7) / 8
	payload := make([]byte, byteCount)

	for i, v := range values {
		if v {
			payload[i/8] |= (1 << (uint(i) % 8))
		}
	}

	x.mu.Lock()
	defer x.mu.Unlock()

	_, err := x.client.WriteMultipleCoils(address, count, payload)
	return err
}

// ==================== X输入继电器 (输入线圈) ====================

// ReadXCoil 读取单个X输入线圈
func (x *XinjieClient) ReadXCoil(address uint16) (bool, error) {
	// X输入继电器地址偏移: 0x5000
	return x.ReadCoil(address+0x5000, true)
}

// ReadXCoils 读取多个X输入线圈
func (x *XinjieClient) ReadXCoils(address uint16, count uint16) ([]bool, error) {
	return x.ReadCoils(address+0x5000, count, true)
}

// ==================== Y输出继电器 ====================

// ReadYCoil 读取单个Y输出线圈
func (x *XinjieClient) ReadYCoil(address uint16) (bool, error) {
	// Y输出继电器地址偏移: 0x6000，地址为八进制
	convertedAddr, err := x.ConvertOctalToDecimal(int(address))
	if err != nil {
		return false, err
	}
	return x.ReadCoil(convertedAddr+0x6000, false)
}

// ReadYCoils 读取多个Y输出线圈
func (x *XinjieClient) ReadYCoils(address uint16, count uint16) ([]bool, error) {
	convertedAddr, err := x.ConvertOctalToDecimal(int(address))
	if err != nil {
		return nil, err
	}
	return x.ReadCoils(convertedAddr+0x6000, count, false)
}

// WriteYCoil 写入单个Y输出线圈
func (x *XinjieClient) WriteYCoil(address uint16, value bool) error {
	convertedAddr, err := x.ConvertOctalToDecimal(int(address))
	if err != nil {
		return err
	}
	return x.WriteSingleCoil(convertedAddr+0x6000, value)
}

// WriteYCoils 写入多个Y输出线圈
func (x *XinjieClient) WriteYCoils(address uint16, values []bool) error {
	convertedAddr, err := x.ConvertOctalToDecimal(int(address))
	if err != nil {
		return err
	}
	return x.WriteMultipleCoils(convertedAddr+0x6000, values)
}

// ==================== M辅助继电器 ====================

// ReadMCoil 读取单个M辅助继电器
func (x *XinjieClient) ReadMCoil(address uint16) (bool, error) {
	return x.ReadCoil(address, false)
}

// ReadMCoils 读取多个M辅助继电器
func (x *XinjieClient) ReadMCoils(address uint16, count uint16) ([]bool, error) {
	return x.ReadCoils(address, count, false)
}

// WriteMCoil 写入单个M辅助继电器
func (x *XinjieClient) WriteMCoil(address uint16, value bool) error {
	return x.WriteSingleCoil(address, value)
}

// WriteMCoils 写入多个M辅助继电器
func (x *XinjieClient) WriteMCoils(address uint16, values []bool) error {
	return x.WriteMultipleCoils(address, values)
}

// ==================== SM特殊辅助继电器 ====================

// ReadSMCoil 读取单个SM特殊辅助继电器
func (x *XinjieClient) ReadSMCoil(address uint16) (bool, error) {
	// SM特殊辅助继电器地址偏移: 0x4000
	return x.ReadCoil(address+0x4000, false)
}

// ReadSMCoils 读取多个SM特殊辅助继电器
func (x *XinjieClient) ReadSMCoils(address uint16, count uint16) ([]bool, error) {
	return x.ReadCoils(address+0x4000, count, false)
}

// WriteSMCoil 写入单个SM特殊辅助继电器
func (x *XinjieClient) WriteSMCoil(address uint16, value bool) error {
	return x.WriteSingleCoil(address+0x4000, value)
}

// WriteSMCoils 写入多个SM特殊辅助继电器
func (x *XinjieClient) WriteSMCoils(address uint16, values []bool) error {
	return x.WriteMultipleCoils(address+0x4000, values)
}

// ==================== 寄存器读写 (Register) ====================

// ReadSingleRegister 读取单个保持寄存器 (功能码 03)
func (x *XinjieClient) ReadSingleRegister(address uint16) (uint16, error) {
	if x.client == nil {
		return 0, fmt.Errorf("客户端未初始化")
	}
	x.mu.Lock()
	defer x.mu.Unlock()

	results, err := x.client.ReadHoldingRegisters(address, 1)
	if err != nil {
		return 0, err
	}

	return binary.BigEndian.Uint16(results), nil
}

// ReadRegisters 读取多个保持寄存器 (功能码 03)
func (x *XinjieClient) ReadRegisters(address uint16, quantity uint16) ([]byte, error) {
	if x.client == nil {
		return nil, fmt.Errorf("客户端未初始化")
	}
	x.mu.Lock()
	defer x.mu.Unlock()

	return x.client.ReadHoldingRegisters(address, quantity)
}

// WriteSingleRegister 写入单个保持寄存器 (功能码 06)
func (x *XinjieClient) WriteSingleRegister(address uint16, value uint16) error {
	if x.client == nil {
		return fmt.Errorf("客户端未初始化")
	}
	x.mu.Lock()
	defer x.mu.Unlock()

	_, err := x.client.WriteSingleRegister(address, value)

	return err
}

// WriteMultipleRegisters 写入多个保持寄存器 (功能码 16)
func (x *XinjieClient) WriteMultipleRegisters(address uint16, quantity uint16, value []byte) error {
	if x.client == nil {
		return fmt.Errorf("客户端未初始化")
	}
	x.mu.Lock()
	defer x.mu.Unlock()

	_, err := x.client.WriteMultipleRegisters(address, quantity, value)
	return err
}

// ==================== D数据寄存器 ====================

// ReadDRegister 读取单个D寄存器，返回指定类型数据
func (x *XinjieClient) ReadDRegister(address uint16, dataType DataType) (interface{}, error) {
	return x.readRegister(address, dataType, false)
}

// ReadDRegisters 读取多个D寄存器，返回指定类型数据数组
func (x *XinjieClient) ReadDRegisters(address uint16, count uint16, dataType DataType) ([]interface{}, error) {
	return x.readRegisters(address, count, dataType, false)
}

// WriteDRegister 写入单个D寄存器
func (x *XinjieClient) WriteDRegister(address uint16, value interface{}, dataType DataType) error {
	return x.writeRegister(address, value, dataType, false)
}

// WriteDRegisters 写入多个D寄存器
func (x *XinjieClient) WriteDRegisters(address uint16, values []interface{}, dataType DataType) error {
	return x.writeRegisters(address, values, dataType, false)
}

// ==================== HD高速寄存器 ====================

// ReadHDRegister 读取单个HD高速寄存器
func (x *XinjieClient) ReadHDRegister(address uint16, dataType DataType) (interface{}, error) {
	return x.readRegister(address+0xA080, dataType, true)
}

// ReadHDRegisters 读取多个HD高速寄存器
func (x *XinjieClient) ReadHDRegisters(address uint16, count uint16, dataType DataType) ([]interface{}, error) {
	return x.readRegisters(address+0xA080, count, dataType, true)
}

// WriteHDRegister 写入单个HD高速寄存器
func (x *XinjieClient) WriteHDRegister(address uint16, value interface{}, dataType DataType) error {
	return x.writeRegister(address+0xA080, value, dataType, true)
}

// WriteHDRegisters 写入多个HD高速寄存器
func (x *XinjieClient) WriteHDRegisters(address uint16, values []interface{}, dataType DataType) error {
	return x.writeRegisters(address+0xA080, values, dataType, true)
}

// ==================== HSD超高速寄存器 ====================

// ReadHSDRegister 读取单个HSD超高速寄存器
func (x *XinjieClient) ReadHSDRegister(address uint16, dataType DataType) (interface{}, error) {
	return x.readRegister(address+0xB080, dataType, true)
}

// ReadHSDRegisters 读取多个HSD超高速寄存器
func (x *XinjieClient) ReadHSDRegisters(address uint16, count uint16, dataType DataType) ([]interface{}, error) {
	return x.readRegisters(address+0xB080, count, dataType, true)
}

// WriteHSDRegister 写入单个HSD超高速寄存器
func (x *XinjieClient) WriteHSDRegister(address uint16, value interface{}, dataType DataType) error {
	return x.writeRegister(address+0xB080, value, dataType, true)
}

// WriteHSDRegisters 写入多个HSD超高速寄存器
func (x *XinjieClient) WriteHSDRegisters(address uint16, values []interface{}, dataType DataType) error {
	return x.writeRegisters(address+0xB080, values, dataType, true)
}

// ==================== 内部辅助函数 ====================

// readRegister 读取单个寄存器并按指定类型解析
func (x *XinjieClient) readRegister(address uint16, dataType DataType, isHiSpeed bool) (interface{}, error) {
	var regCount uint16
	switch dataType {
	case Int16, UInt16:
		regCount = 1
	case Int32, UInt32, Float32:
		regCount = 2
	case Int64, UInt64, Float64:
		regCount = 4
	default:
		return nil, fmt.Errorf("不支持的数据类型")
	}

	data, err := x.ReadRegisters(address, regCount)

	if err != nil {
		return nil, err
	}

	return x.parseData(data, dataType, isHiSpeed)
}

// readRegisters 读取多个寄存器并按指定类型解析
func (x *XinjieClient) readRegisters(address uint16, count uint16, dataType DataType, isHiSpeed bool) ([]interface{}, error) {
	var regCountPerValue uint16
	switch dataType {
	case Int16, UInt16:
		regCountPerValue = 1
	case Int32, UInt32, Float32:
		regCountPerValue = 2
	case Int64, UInt64, Float64:
		regCountPerValue = 4
	default:
		return nil, fmt.Errorf("不支持的数据类型")
	}

	data, err := x.ReadRegisters(address, count*regCountPerValue)

	if err != nil {
		return nil, err
	}

	results := make([]interface{}, count)
	for i := uint16(0); i < count; i++ {
		startIdx := i * regCountPerValue * 2
		endIdx := startIdx + regCountPerValue*2
		if endIdx > uint16(len(data)) {
			break
		}
		results[i], _ = x.parseData(data[startIdx:endIdx], dataType, isHiSpeed)
	}

	return results, nil
}

// writeRegister 写入单个寄存器
func (x *XinjieClient) writeRegister(address uint16, value interface{}, dataType DataType, isHiSpeed bool) error {
	data, err := x.encodeData(value, dataType, isHiSpeed)
	if err != nil {
		return err
	}

	regCount := uint16(len(data)) / 2
	if regCount == 1 {
		return x.WriteSingleRegister(address, binary.BigEndian.Uint16(data))
	}
	return x.WriteMultipleRegisters(address, regCount, data)
}

// writeRegisters 写入多个寄存器
func (x *XinjieClient) writeRegisters(address uint16, values []interface{}, dataType DataType, isHiSpeed bool) error {
	var regCountPerValue uint16
	switch dataType {
	case Int16, UInt16:
		regCountPerValue = 1
	case Int32, UInt32, Float32:
		regCountPerValue = 2
	case Int64, UInt64, Float64:
		regCountPerValue = 4
	default:
		return fmt.Errorf("不支持的数据类型")
	}

	totalBytes := uint16(len(values)) * regCountPerValue * 2
	data := make([]byte, totalBytes)

	for i, v := range values {
		encoded, err := x.encodeData(v, dataType, isHiSpeed)
		if err != nil {
			return err
		}
		copy(data[i*int(regCountPerValue)*2:], encoded)
	}

	return x.WriteMultipleRegisters(address, uint16(len(values))*regCountPerValue, data)
}

// parseData 解析原始数据为指定类型
func (x *XinjieClient) parseData(data []byte, dataType DataType, isHiSpeed bool) (interface{}, error) {
	switch dataType {
	case Int16:
		return int16(binary.BigEndian.Uint16(data)), nil
	case UInt16:
		return binary.BigEndian.Uint16(data), nil
	case Int32:
		if isHiSpeed {
			// 高速寄存器: 低字在前，高字在后
			low := binary.BigEndian.Uint16(data[0:2])
			high := binary.BigEndian.Uint16(data[2:4])
			return int32(uint32(high)<<16 | uint32(low)), nil
		}
		return int32(binary.BigEndian.Uint32(data)), nil
	case UInt32:
		if isHiSpeed {
			low := binary.BigEndian.Uint16(data[0:2])
			high := binary.BigEndian.Uint16(data[2:4])
			return uint32(high)<<16 | uint32(low), nil
		}
		return binary.BigEndian.Uint32(data), nil
	case Int64:
		if isHiSpeed {
			w0 := binary.BigEndian.Uint16(data[0:2])
			w1 := binary.BigEndian.Uint16(data[2:4])
			w2 := binary.BigEndian.Uint16(data[4:6])
			w3 := binary.BigEndian.Uint16(data[6:8])
			return int64(uint64(w3)<<48 | uint64(w2)<<32 | uint64(w1)<<16 | uint64(w0)), nil
		}
		return int64(binary.BigEndian.Uint64(data)), nil
	case UInt64:
		if isHiSpeed {
			w0 := binary.BigEndian.Uint16(data[0:2])
			w1 := binary.BigEndian.Uint16(data[2:4])
			w2 := binary.BigEndian.Uint16(data[4:6])
			w3 := binary.BigEndian.Uint16(data[6:8])
			return uint64(w3)<<48 | uint64(w2)<<32 | uint64(w1)<<16 | uint64(w0), nil
		}
		return binary.BigEndian.Uint64(data), nil
	case Float32:
		// 信捷PLC统一使用低字在前，高字在后
		low := binary.BigEndian.Uint16(data[0:2])
		high := binary.BigEndian.Uint16(data[2:4])
		bits := uint32(high)<<16 | uint32(low)
		return math.Float32frombits(bits), nil
	case Float64:
		if isHiSpeed {
			w0 := binary.BigEndian.Uint16(data[0:2])
			w1 := binary.BigEndian.Uint16(data[2:4])
			w2 := binary.BigEndian.Uint16(data[4:6])
			w3 := binary.BigEndian.Uint16(data[6:8])
			bits := uint64(w3)<<48 | uint64(w2)<<32 | uint64(w1)<<16 | uint64(w0)
			return math.Float64frombits(bits), nil
		}
		return math.Float64frombits(binary.BigEndian.Uint64(data)), nil
	default:
		return nil, fmt.Errorf("不支持的数据类型")
	}
}

// encodeData 将值编码为字节数组
func (x *XinjieClient) encodeData(value interface{}, dataType DataType, isHiSpeed bool) ([]byte, error) {
	switch dataType {
	case Int16:
		v, ok := value.(int16)
		if !ok {
			return nil, fmt.Errorf("类型不匹配，期望 int16")
		}
		data := make([]byte, 2)
		binary.BigEndian.PutUint16(data, uint16(v))
		return data, nil
	case UInt16:
		v, ok := value.(uint16)
		if !ok {
			return nil, fmt.Errorf("类型不匹配，期望 uint16")
		}
		data := make([]byte, 2)
		binary.BigEndian.PutUint16(data, v)
		return data, nil
	case Int32:
		v, ok := value.(int32)
		if !ok {
			return nil, fmt.Errorf("类型不匹配，期望 int32")
		}
		data := make([]byte, 4)
		if isHiSpeed {
			// 低字在前，高字在后
			low := uint16(v & 0xFFFF)
			high := uint16(v >> 16)
			binary.BigEndian.PutUint16(data[0:2], low)
			binary.BigEndian.PutUint16(data[2:4], high)
		} else {
			binary.BigEndian.PutUint32(data, uint32(v))
		}
		return data, nil
	case UInt32:
		v, ok := value.(uint32)
		if !ok {
			return nil, fmt.Errorf("类型不匹配，期望 uint32")
		}
		data := make([]byte, 4)
		if isHiSpeed {
			low := uint16(v & 0xFFFF)
			high := uint16(v >> 16)
			binary.BigEndian.PutUint16(data[0:2], low)
			binary.BigEndian.PutUint16(data[2:4], high)
		} else {
			binary.BigEndian.PutUint32(data, v)
		}
		return data, nil
	case Int64:
		v, ok := value.(int64)
		if !ok {
			return nil, fmt.Errorf("类型不匹配，期望 int64")
		}
		data := make([]byte, 8)
		if isHiSpeed {
			w0 := uint16(v & 0xFFFF)
			w1 := uint16((v >> 16) & 0xFFFF)
			w2 := uint16((v >> 32) & 0xFFFF)
			w3 := uint16((v >> 48) & 0xFFFF)
			binary.BigEndian.PutUint16(data[0:2], w0)
			binary.BigEndian.PutUint16(data[2:4], w1)
			binary.BigEndian.PutUint16(data[4:6], w2)
			binary.BigEndian.PutUint16(data[6:8], w3)
		} else {
			binary.BigEndian.PutUint64(data, uint64(v))
		}
		return data, nil
	case UInt64:
		v, ok := value.(uint64)
		if !ok {
			return nil, fmt.Errorf("类型不匹配，期望 uint64")
		}
		data := make([]byte, 8)
		if isHiSpeed {
			w0 := uint16(v & 0xFFFF)
			w1 := uint16((v >> 16) & 0xFFFF)
			w2 := uint16((v >> 32) & 0xFFFF)
			w3 := uint16((v >> 48) & 0xFFFF)
			binary.BigEndian.PutUint16(data[0:2], w0)
			binary.BigEndian.PutUint16(data[2:4], w1)
			binary.BigEndian.PutUint16(data[4:6], w2)
			binary.BigEndian.PutUint16(data[6:8], w3)
		} else {
			binary.BigEndian.PutUint64(data, v)
		}
		return data, nil
	case Float32:
		v, ok := value.(float32)
		if !ok {
			return nil, fmt.Errorf("类型不匹配，期望 float32")
		}
		bits := math.Float32bits(v)
		data := make([]byte, 4)
		// 信捷PLC统一使用低字在前，高字在后
		low := uint16(bits & 0xFFFF)
		high := uint16(bits >> 16)
		binary.BigEndian.PutUint16(data[0:2], low)
		binary.BigEndian.PutUint16(data[2:4], high)
		return data, nil
	case Float64:
		v, ok := value.(float64)
		if !ok {
			return nil, fmt.Errorf("类型不匹配，期望 float64")
		}
		bits := math.Float64bits(v)
		data := make([]byte, 8)
		if isHiSpeed {
			w0 := uint16(bits & 0xFFFF)
			w1 := uint16((bits >> 16) & 0xFFFF)
			w2 := uint16((bits >> 32) & 0xFFFF)
			w3 := uint16((bits >> 48) & 0xFFFF)
			binary.BigEndian.PutUint16(data[0:2], w0)
			binary.BigEndian.PutUint16(data[2:4], w1)
			binary.BigEndian.PutUint16(data[4:6], w2)
			binary.BigEndian.PutUint16(data[6:8], w3)
		} else {
			binary.BigEndian.PutUint64(data, bits)
		}
		return data, nil
	default:
		return nil, fmt.Errorf("不支持的数据类型")
	}
}
