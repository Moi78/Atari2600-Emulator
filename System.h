#pragma once
#ifndef _SYSTEM_H__
#define _SYSTEM_H__

#include <iostream>
#include <fstream>
#include <array>
#include <vector>
#include <filesystem>
#include <cstddef>

#define HIGHLOW(x, y) ((x) << 8) + (y)

template<class T>
void DispBinary(T number) {
	std::cout << std::bitset<sizeof(T) * 8>(number) << std::endl;
}

struct Registers {
	std::uint8_t A;
	std::uint8_t X;
	std::uint8_t Y;
	std::uint8_t FLAG;

	std::uint8_t SP;
	std::uint16_t PC;
};

class Processor {
public:
	Processor() {
		Reset();
	}

	void Reset() {
		m_regs.A = 0;
		m_regs.X = 0;
		m_regs.Y = 0;
		m_regs.FLAG = 0;
		m_regs.SP = 0;
		m_regs.PC = 0xFFFC;
	}

	std::uint16_t GetProgramCounter() {
		return m_regs.PC;
	}

	void SetProgramCounter(std::uint16_t PC) {
		m_regs.PC = PC;
	}

	void Step(int step_size = 1) {
		m_regs.PC += step_size;
	}

	void JumpAbsolute(std::vector<std::uint8_t> rom_data) {
		std::uint16_t addr_low = rom_data[(m_regs.PC + 1) - 0xF000];
		std::uint16_t addr_high = rom_data[(m_regs.PC + 2) - 0xF000];

		std::uint16_t new_addr = (addr_high << 8) + addr_low;
		std::cout << "Jumped to " << new_addr << std::endl;

		m_regs.PC = new_addr;
	}

	void Jump(std::vector<std::uint8_t> rom_data) {
		//Getting the indirect address pointing to the Zero Page, at PC+1
		std::uint16_t ind_addr_low = rom_data[(m_regs.PC + 1) - 0xF000];

		//Getting the effective address from the Zero Page, at 00, IAL; 00, IAL+1
		std::uint16_t eff_addr_low = rom_data[ind_addr_low];
		std::uint16_t eff_addr_high = rom_data[ind_addr_low + 1];

		std::uint16_t eff_addr = (eff_addr_high << 8) + eff_addr_low;
		std::cout << "Jumped to " << eff_addr << std::endl;

		m_regs.PC = eff_addr;
	}

	void ClearDec() {
		m_regs.FLAG &= 0b00010000;

		Step();
	}

	void LoadXMem(std::vector<std::uint8_t> rom_data, std::uint8_t addr_mode) {
		if (addr_mode == 0b000) { //Immediate
			m_regs.X = rom_data[(m_regs.PC + 1) - 0xF000];
			std::cout << "X = " << (int)m_regs.X << std::endl;

			Step(2);
		}
		else if (addr_mode == 0b001) { //Zero-Page
			std::uint16_t zp_addr = rom_data[(m_regs.PC + 1) - 0xF000];

			m_regs.X = rom_data[zp_addr];
			std::cout << "X = " << (int)m_regs.X << std::endl;

			Step(2);

		}
		else if (addr_mode == 0b011) { //Absolute
			std::uint16_t addr_low = rom_data[(m_regs.PC + 1) - 0xF000];
			std::uint16_t addr_high = rom_data[(m_regs.PC + 2) - 0xF000];

			std::uint16_t addr = (addr_high << 8) + addr_low;
			m_regs.X = rom_data[addr];
			std::cout << "X = " << (int)m_regs.X << std::endl;

			Step(3);
		}
		else if (addr_mode == 0b101) { //Zero-Page, X/Y
			std::cout << "INDEXED ZERO PAGE (LDX)" << std::endl;

			Step(3);
		}
		else if (addr_mode == 0b111) { //Absolute, X/Y
			std::cout << "INDEXED ABSOLUTE (LDX)" << std::endl;

			Step(3);
		}
	}

	void TransferXA() {
		m_regs.A = m_regs.X;

		Step(1);
	}

	void StoreA(std::vector<std::uint8_t> rom_data, std::array<std::uint8_t, 256> ram, std::uint8_t addr_mode) {
		if (addr_mode == 0b000) { // (Zero page, X) - Indexed Indirect
			std::uint16_t base_low = rom_data[(m_regs.PC + 1) - 0xF000];

			std::uint16_t data_low = rom_data[base_low + m_regs.X];
			std::uint16_t data_high = rom_data[base_low + m_regs.X + 1];

			std::uint16_t addr_addr = HIGHLOW(data_high, data_low);
			std::uint16_t data_addr = rom_data[addr_addr];

			ram[data_addr] = m_regs.A;
			std::cout << "A to address " << (int)data_addr << std::endl;

			Step(2);
		}
		else if (addr_mode == 0b001) { // Zero-Page
			std::uint16_t zp_addr = rom_data[(m_regs.PC + 1) - 0xF000];

			ram[zp_addr] = m_regs.A;
			std::cout << "A to address " << (int)zp_addr << std::endl;

			Step(2);
		}
		else if (addr_mode == 0b011) { // Absolute
			std::uint16_t addr_low = rom_data[(m_regs.PC + 1) - 0xF000];
			std::uint16_t addr_high = rom_data[(m_regs.PC + 2) - 0xF000];

			std::uint16_t addr_addr = HIGHLOW(addr_high, addr_low);
			std::uint16_t data_addr = rom_data[addr_addr];

			ram[data_addr] = m_regs.A;

			std::cout << "A to address " << (int)data_addr << std::endl;

			Step(3);
		}
		else if (addr_mode == 0b100) { // (Zero-Page), Y - Indirect Indexed
			std::uint16_t indir_addr = rom_data[(m_regs.PC + 1) - 0xF000];

			std::uint16_t base_low = rom_data[indir_addr];
			std::uint16_t base_high = rom_data[indir_addr + 1];

			base_low += m_regs.Y;
			if (base_low >= 0xFF) {
				base_high++;
				base_low -= 0xFF;
			}

			std::uint16_t addr_addr = HIGHLOW(base_high, base_low);
			std::uint16_t data_addr = rom_data[addr_addr];
			ram[data_addr] = m_regs.A;

			std::cout << "A to address " << (int)data_addr << std::endl;

			Step(2);
		}
		else if (addr_mode == 0b101) { // Zero-Page, X
			std::uint16_t base_low = rom_data[(m_regs.PC + 1) - 0xF000];
			base_low += m_regs.X;

			std::uint16_t data_addr = rom_data[base_low];
			ram[data_addr] = m_regs.A;

			std::cout << "A to address " << (int)data_addr << std::endl;

			Step(2);
		}
		else if (addr_mode == 0b110) { // Absolute, Y
			std::uint16_t base_low = rom_data[(m_regs.PC + 1) - 0xF000];
			std::uint16_t base_high = rom_data[(m_regs.PC + 2) - 0xF000];

			base_low += m_regs.Y;
			if (base_low >= 0xFF) {
				m_regs.FLAG |= 0b10000000;
				base_high++;
				base_low -= 0xFF;
			}

			std::uint16_t data_addr = HIGHLOW(base_high, base_low);
			ram[data_addr] = m_regs.A;

			std::cout << "A to address " << (int)data_addr << std::endl;

			Step(3);
		}
		else if (addr_mode == 0b111) { // Absolute, X
			std::uint16_t base_low = rom_data[(m_regs.PC + 1) - 0xF000];
			std::uint16_t base_high = rom_data[(m_regs.PC + 2) - 0xF000];

			base_low += m_regs.X;
			if (base_low >= 0xFF) {
				m_regs.FLAG |= 0b10000000;
				base_high++;
				base_low -= 0xFF;
			}

			std::uint16_t data_addr = HIGHLOW(base_high, base_low);
			ram[data_addr] = m_regs.A;

			std::cout << "A to address " << (int)data_addr << std::endl;

			Step(3);
		}
	}

	void DecX() {
		if (m_regs.X == 0) {
			m_regs.FLAG |= 0b00000001;
		}

		m_regs.X--;

		if (m_regs.X == 0) {
			m_regs.FLAG |= 0b01000000;
		}
		else {
			m_regs.FLAG &= 0b00000000;
		}

		std::cout << "X = " << (int)m_regs.X << std::endl;

		Step(1);
	}

	void BranchNEq(std::vector<std::uint8_t> rom_data) {
		std::uint8_t Z_flag = m_regs.FLAG & 0b01000000;
		if (!Z_flag) {
			std::int8_t offset = rom_data[(m_regs.PC + 1) - 0xF000];
			m_regs.PC += offset + 1;

			std::cout << "Branch taken to " << (int)m_regs.PC << std::endl;
		}
		else {
			std::cout << "Branch not taken" << std::endl;
			Step(2);
		}
	}

private:
	Registers m_regs;
};

class System {
public:
	System() {
		m_ram = {};
		m_proc = Processor();
	}

	bool OpenRom(std::string path) {
		std::ifstream rom;
		rom.open(path, std::ios::beg | std::ios::binary);
		if (!rom.is_open()) {
			return false;
		}

		std::uint16_t fsize = std::filesystem::file_size(path);
		m_rom.resize(fsize);
		rom.read((char*)m_rom.data(), fsize);

		return true;
	}

	std::uint8_t ReadOPAtAddress(uint16_t pointer) {
		return m_rom[pointer - 0xF000];
	}

	void Init() {
		//Reading reset vector
		m_proc.Reset();
		
		std::uint16_t FFFC = ReadOPAtAddress(m_proc.GetProgramCounter());
		m_proc.Step();
		std::uint16_t FFFD = ReadOPAtAddress(m_proc.GetProgramCounter());

		std::uint16_t reset_vector = (FFFD << 8) + FFFC;

		std::cout << "Reset vector " << std::hex << reset_vector << std::endl;
		m_proc.SetProgramCounter(reset_vector);
	}

	void DecodeExecuteOP() {
		std::uint8_t current_op = ReadOPAtAddress(m_proc.GetProgramCounter());

		std::cout << "-----------------" << std::endl;
		std::cout << "Full OP ";
		DispBinary(current_op);

		//Checking for single-bytes instructions
		bool is_single_byte = false;
		switch (current_op)
		{
		case 0x08:
			std::cout << "Single byte : PHP" << std::endl;
			is_single_byte = true;
			break;
		case 0x28:
			std::cout << "Single byte : PLP" << std::endl;
			is_single_byte = true;
			break;
		case 0x48:
			std::cout << "Single byte : PHA" << std::endl;
			is_single_byte = true;
			break;
		case 0x68:
			std::cout << "Single byte : PLA" << std::endl;
			is_single_byte = true;
			break;
		case 0x88:
			std::cout << "Single byte : DEY" << std::endl;
			is_single_byte = true;
			break;
		case 0xA8:
			std::cout << "Single byte : TAY" << std::endl;
			is_single_byte = true;
			break;
		case 0xC8:
			std::cout << "Single byte : INY" << std::endl;
			is_single_byte = true;
			break;
		case 0xE8:
			std::cout << "Single byte : INX" << std::endl;
			is_single_byte = true;
			break;
		case 0x18:
			std::cout << "Single byte : CLC" << std::endl;
			is_single_byte = true;
			break;
		case 0x38:
			std::cout << "Single byte : SEC" << std::endl;
			is_single_byte = true;
			break;
		case 0x58:
			std::cout << "Single byte : CLI" << std::endl;
			is_single_byte = true;
			break;
		case 0x78:
			std::cout << "Single byte : SEI" << std::endl;
			is_single_byte = true;
			break;
		case 0x98:
			std::cout << "Single byte : TYA" << std::endl;
			is_single_byte = true;
			break;
		case 0xB8:
			std::cout << "Single byte : CLV" << std::endl;
			is_single_byte = true;
			break;
		case 0xD8:
			std::cout << "Single byte : CLD" << std::endl;
			m_proc.ClearDec();

			is_single_byte = true;
			break;
		case 0xF8:
			std::cout << "Single byte : SED" << std::endl;
			is_single_byte = true;
			break;
		case 0x8A:
			std::cout << "Single byte : TXA" << std::endl;
			m_proc.TransferXA();

			is_single_byte = true;
			break;
		case 0x9A:
			std::cout << "Single byte : TXS" << std::endl;
			is_single_byte = true;
			break;
		case 0xAA:
			std::cout << "Single byte : TAX" << std::endl;
			is_single_byte = true;
			break;
		case 0xBA:
			std::cout << "Single byte : TSX" << std::endl;
			is_single_byte = true;
			break;
		case 0xCA:
			std::cout << "Single byte : DEX" << std::endl;
			m_proc.DecX();

			is_single_byte = true;
			break;
		case 0xEA:
			std::cout << "Single byte : NOP" << std::endl;
			is_single_byte = true;
			break;
		case 0x10:
			std::cout << "Single byte : BPL" << std::endl;
			is_single_byte = true;
			break;
		case 0x30:
			std::cout << "Single byte : BMI" << std::endl;
			is_single_byte = true;
			break;
		case 0x50:
			std::cout << "Single byte : BVC" << std::endl;
			is_single_byte = true;
			break;
		case 0x70:
			std::cout << "Single byte : BVS" << std::endl;
			is_single_byte = true;
			break;
		case 0x90:
			std::cout << "Single byte : BCC" << std::endl;
			is_single_byte = true;
			break;
		case 0xB0:
			std::cout << "Single byte : BCS" << std::endl;
			is_single_byte = true;
			break;
		case 0xD0:
			std::cout << "Single byte : BNE" << std::endl;
			m_proc.BranchNEq(m_rom);

			is_single_byte = true;
			break;
		case 0xF0:
			std::cout << "Single byte : BEQ" << std::endl;
			is_single_byte = true;
			break;
		default:
			break;
		}
		
		if (is_single_byte) {
			return;
		}

		std::uint8_t CC = current_op & 0x03;
		std::cout << "OP Group ";
		DispBinary(CC);

		std::uint8_t AAA = (current_op & 0xE0) >> 5;
		std::cout << "AAA OP ";
		DispBinary(AAA);

		std::uint8_t BBB = (current_op & 0x2B5C) >> 2;
		std::cout << "Addressing mode ";
		DispBinary(BBB);

		if (CC == 0x01) {
			switch (AAA)
			{
			case 0x00:
				std::cout << "ORA" << std::endl;
				break;
			case 0x01:
				std::cout << "AND" << std::endl;
				break;
			case 0x02:
				std::cout << "EOR" << std::endl;
				break;
			case 0x03:
				std::cout << "ADC" << std::endl;
				break;
			case 0x04:
				std::cout << "STA" << std::endl;
				m_proc.StoreA(m_rom, m_ram, BBB);
				break;
			case 0x05:
				std::cout << "LDA" << std::endl;
				break;
			case 0x06:
				std::cout << "CMP" << std::endl;
				break;
			case 0x07:
				std::cout << "SBC" << std::endl;
				break;
			default:
				break;
			}
		} else if(CC == 0x02) {
			switch (AAA)
			{
			case 0x00:
				std::cout << "ASL" << std::endl;
				break;
			case 0x01:
				std::cout << "ROL" << std::endl;
				break;
			case 0x02:
				std::cout << "LSR" << std::endl;
				break;
			case 0x03:
				std::cout << "ROR" << std::endl;
				break;
			case 0x04:
				std::cout << "STX" << std::endl;
				break;
			case 0x05:
				std::cout << "LDX" << std::endl;
				m_proc.LoadXMem(m_rom, BBB);
				break;
			case 0x06:
				std::cout << "DEC" << std::endl;
				break;
			case 0x07:
				std::cout << "INC" << std::endl;
				break;
			default:
				break;
			}
		}
		else if (CC == 0x00) {
			switch (AAA)
			{
			case 0x01:
				std::cout << "BIT" << std::endl;
				break;
			case 0x02:
				std::cout << "JMP" << std::endl;
				m_proc.Jump(m_rom);
				break;
			case 0x03:
				std::cout << "JMP(ABSOLUTE)" << std::endl;
				m_proc.JumpAbsolute(m_rom);
				break;
			case 0x04:
				std::cout << "STY" << std::endl;
				break;
			case 0x05:
				std::cout << "LDY" << std::endl;
				break;
			case 0x06:
				std::cout << "CPY" << std::endl;
				break;
			case 0x07:
				std::cout << "CPX" << std::endl;
				break;
			default:
				break;
			}
		}
	}

private:
	std::array<std::uint8_t, 256> m_ram;
	Processor m_proc;

	std::vector<std::uint8_t> m_rom;
};

#endif //_SYSTEM_H__