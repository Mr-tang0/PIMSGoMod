/*

 */
package protocol

import (
	"bytes"
	"fmt"
	"sync"
	"time"

	"go.bug.st/serial"
)

// SerialCommunicator 串口通讯实现
type SerialCommunicator struct {
	Port     string
	BaudRate int

	// 重试配置
	MaxRetries int           // 最大重试次数
	Timeout    time.Duration // 总超时时间（等待回复）

	mu          sync.Mutex
	isConnected bool
	mode        *serial.Mode
	portConn    serial.Port // 实际的串口连接对象
}

// ----------------------------------------------------------------
// 1. 基础连接管理
// ----------------------------------------------------------------

// ListAvailablePorts 遍历所有可用的串口设备
func (s *SerialCommunicator) ListAvailablePorts() ([]string, error) {
	ports, err := serial.GetPortsList()
	if err != nil {
		return nil, fmt.Errorf("failed to enumerate ports: %v", err)
	}
	return ports, nil
}

// Connect 连接具体设备
func (s *SerialCommunicator) Connect(comPort string, baudRate int) error {
	s.mu.Lock()
	defer s.mu.Unlock()

	if s.isConnected {
		return fmt.Errorf("already connected to %s", s.Port)
	}

	s.mode = &serial.Mode{
		BaudRate: baudRate,
		DataBits: 8,
		Parity:   serial.NoParity,
		StopBits: serial.OneStopBit,
	}

	p, err := serial.Open(comPort, s.mode)
	if err != nil {
		return fmt.Errorf("failed to open port %s: %v", comPort, err)
	}

	s.portConn = p
	s.BaudRate = baudRate
	s.Port = comPort
	s.isConnected = true

	fmt.Printf("Successfully connected to %s at %d baud\n", s.Port, s.BaudRate)
	return nil
}

// Disconnect 断开连接
func (s *SerialCommunicator) Disconnect() error {
	s.mu.Lock()
	defer s.mu.Unlock()

	if s.portConn != nil {
		s.portConn.Close()
	}
	s.isConnected = false
	return nil
}

func (s *SerialCommunicator) IsConnected() bool {
	return s.isConnected
}

// ----------------------------------------------------------------
// 2. 字符串通讯逻辑 (String Mode)
// ----------------------------------------------------------------

// SendString 发送字符串（通常以 \n 结尾）并等待含 \n 的回复
func (s *SerialCommunicator) SendString(data string) (string, error) {
	dataWithNewLine := data + "\r\n"

	resp, err := s.executeTransfer([]byte(dataWithNewLine), false)
	if err != nil {
		return "", err
	}
	return string(resp), nil
}

// readUntilDelimiter 持续读取直至找到换行符
func (s *SerialCommunicator) readUntilDelimiter() ([]byte, error) {
	var fullFrame []byte
	startTime := time.Now()
	tmpBuf := make([]byte, 128)

	for {
		if time.Since(startTime) > s.Timeout {
			return nil, fmt.Errorf("string read timeout (%v)", s.Timeout)
		}

		n, err := s.portConn.Read(tmpBuf)

		if err != nil {
			return nil, err
		}

		if n > 0 {
			fullFrame = append(fullFrame, tmpBuf[:n]...)
			if bytes.Contains(tmpBuf[:n], []byte{'\r'}) {
				return fullFrame, nil
			}
		} else {
			time.Sleep(10 * time.Millisecond)
		}
	}
}

// ----------------------------------------------------------------
// 3. 3E Bus 电机协议通讯逻辑 (3E Bus Mode)
// ----------------------------------------------------------------

// Send3EBus 发送 3E Bus 电机协议请求
// data: 原始数据（不含校验字节）
// expectedLen: 期望的回复长度（包含校验字节）
// startByte: 帧头字节，用于帧同步
func (s *SerialCommunicator) Send3EBus(data []byte, expectedLen int, startByte byte) ([]byte, error) {
	// 添加校验字节
	fullCmd := s.calculate3ECheckSum(data)

	// 执行传输
	return s.execute3EBusTransfer(fullCmd, expectedLen, startByte)
}

// Calculate3ECheckSum 计算 3E Bus 协议校验和（公开方法）
func (s *SerialCommunicator) Calculate3ECheckSum(data []byte) []byte {
	return s.calculate3ECheckSum(data)
}

// calculate3ECheckSum 计算 3E Bus 协议校验和
func (s *SerialCommunicator) calculate3ECheckSum(data []byte) []byte {
	var sum uint16 = 0
	for _, b := range data {
		sum += uint16(b)
	}
	return append(data, uint8(sum))
}

// Send3EBusRaw 发送已包含校验和的 3E Bus 数据
// 用于需要手动计算校验和的场景（如分段发送）
func (s *SerialCommunicator) Send3EBusRaw(data []byte, expectedLen int, startByte byte) ([]byte, error) {
	return s.execute3EBusTransfer(data, expectedLen, startByte)
}

// execute3EBusTransfer 执行 3E Bus 协议传输
func (s *SerialCommunicator) execute3EBusTransfer(data []byte, expectedLen int, startByte byte) ([]byte, error) {
	s.mu.Lock()
	defer s.mu.Unlock()

	if !s.isConnected {
		return nil, fmt.Errorf("serial port not connected")
	}

	// 设置读取超时
	s.portConn.SetReadTimeout(5 * time.Millisecond)

	// 发送前清空缓冲区
	s.portConn.ResetInputBuffer()

	// 发送请求
	if _, err := s.portConn.Write(data); err != nil {
		return nil, err
	}

	// 循环读取，直到读够字节或超时
	result := make([]byte, 0, expectedLen)
	deadline := time.Now().Add(200 * time.Millisecond)

	for time.Now().Before(deadline) && len(result) < expectedLen {
		buf := make([]byte, expectedLen)
		n, err := s.portConn.Read(buf)
		if err != nil {
			break
		}
		if n > 0 {
			result = append(result, buf[:n]...)
		}

		// 寻找帧头：如果第一个字节不是预期的，丢弃前面的字节
		for len(result) > 0 && result[0] != startByte {
			result = result[1:]
		}
	}

	if len(result) < expectedLen {
		return nil, fmt.Errorf("3E Bus read timeout or insufficient length")
	}

	return result[:expectedLen], nil
}

// ----------------------------------------------------------------
// 4. Modbus RTU 通讯逻辑 (Modbus Mode)
// ----------------------------------------------------------------

// SendModbus 发送标准 Modbus RTU 请求
// slaveID: 从站地址, funcCode: 功能码, startAddr: 起始地址, count: 数量
func (s *SerialCommunicator) SendModbus(slaveID byte, funcCode byte, startAddr uint16, count uint16) ([]byte, error) {
	// 组装 Modbus 协议帧 (不含CRC)
	req := []byte{
		slaveID,
		funcCode,
		byte(startAddr >> 8), byte(startAddr & 0xFF),
		byte(count >> 8), byte(count & 0xFF),
	}
	// 计算并添加 CRC
	crc := s.CalculateCRC(req)
	req = append(req, byte(crc&0xFF), byte(crc>>8))

	// 执行传输（Modbus 模式）
	return s.executeTransfer(req, true)
}

// readModbusFrame 依靠 CRC 校验和最小帧长判定 Modbus 包结束
func (s *SerialCommunicator) readModbusFrame() ([]byte, error) {
	var fullFrame []byte
	startTime := time.Now()
	tmpBuf := make([]byte, 256)

	for {
		if time.Since(startTime) > s.Timeout {
			// fmt.Printf("串口 %s Modbus 读取超时\n", s.Port)
			return nil, fmt.Errorf("modbus read timeout (%v)", s.Timeout)
		}

		n, err := s.portConn.Read(tmpBuf)
		if err != nil {
			return nil, err
		}

		if n > 0 {
			fullFrame = append(fullFrame, tmpBuf[:n]...)
			// Modbus 响应最短通常为 5 字节
			if len(fullFrame) >= 5 {
				if s.verifyCRC(fullFrame) {
					return fullFrame, nil
				}
			}
		} else {
			// 如果有数据但 CRC 还没过，微等一下可能存在的后续包
			if len(fullFrame) > 0 {
				time.Sleep(20 * time.Millisecond)
			}
			time.Sleep(10 * time.Millisecond)
		}
	}
}

// ----------------------------------------------------------------
// 4. 核心调度与辅助工具
// ----------------------------------------------------------------

// executeTransfer 内部统一调度函数
func (s *SerialCommunicator) executeTransfer(data []byte, isModbus bool) ([]byte, error) {
	s.mu.Lock()
	defer s.mu.Unlock()

	if !s.isConnected {
		return nil, fmt.Errorf("serial port not connected")
	}

	// 设置底层物理读取超时，防止 Read 永远阻塞
	s.portConn.SetReadTimeout(100 * time.Millisecond)

	var lastErr error
	for i := 0; i <= s.MaxRetries; i++ {
		if i > 0 {
			time.Sleep(100 * time.Millisecond)
		}

		s.portConn.ResetInputBuffer()
		if _, err := s.portConn.Write(data); err != nil {
			lastErr = err
			continue
		}

		var resp []byte
		var err error
		if isModbus {
			resp, err = s.readModbusFrame()
		} else {
			resp, err = s.readUntilDelimiter()
		}

		if err != nil {
			lastErr = err
			continue
		}
		return resp, nil
	}
	return nil, fmt.Errorf("failed after %d retries: %v", s.MaxRetries, lastErr)
}

// CalculateCRC 计算 Modbus CRC16
func (s *SerialCommunicator) CalculateCRC(data []byte) uint16 {
	var crc uint16 = 0xFFFF
	for _, b := range data {
		crc ^= uint16(b)
		for i := 0; i < 8; i++ {
			if (crc & 0x0001) != 0 {
				crc = (crc >> 1) ^ 0xA001
			} else {
				crc >>= 1
			}
		}
	}
	return crc
}

// verifyCRC 验证数据包末尾的 CRC 是否正确
func (s *SerialCommunicator) verifyCRC(data []byte) bool {
	if len(data) < 3 {
		return false
	}
	payload := data[:len(data)-2]
	expected := s.CalculateCRC(payload)
	actual := uint16(data[len(data)-2]) | (uint16(data[len(data)-1]) << 8)
	return expected == actual
}
