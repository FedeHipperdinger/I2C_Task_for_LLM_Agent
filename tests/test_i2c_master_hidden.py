from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, FallingEdge, Timer, ReadOnly
from cocotb_tools.runner import get_runner


LANGUAGE = os.getenv("HDL_TOPLEVEL_LANG", "verilog").lower().strip()


@dataclass
class OpenDrainBus:
    """Open-drain SDA model: line is low if master OR slave drives low, else high."""
    dut: object
    slave_drive_low: bool = False

    async def drive_loop(self):
        """Continuously resolve SDA line and drive dut.sda_i."""
        while True:
            await ReadOnly()
            master_drive_low = int(self.dut.sda_oe.value) == 1  # master can only drive low
            line = 0 if (master_drive_low or self.slave_drive_low) else 1
            self.dut.sda_i.value = line
            await Timer(1, units="ns")  # small time step to keep line updated


async def wait_for_start(dut) -> None:
    """Detect START: SDA 1->0 while SCL is high."""
    last_sda = int(dut.sda_i.value)
    while True:
        await RisingEdge(dut.clk)
        scl = int(dut.scl.value)
        sda = int(dut.sda_i.value)
        if scl == 1 and last_sda == 1 and sda == 0:
            return
        last_sda = sda


async def wait_for_stop(dut) -> None:
    """Detect STOP: SDA 0->1 while SCL is high."""
    last_sda = int(dut.sda_i.value)
    while True:
        await RisingEdge(dut.clk)
        scl = int(dut.scl.value)
        sda = int(dut.sda_i.value)
        if scl == 1 and last_sda == 0 and sda == 1:
            return
        last_sda = sda


async def recv_byte_from_master(dut, bus: OpenDrainBus, *, expect_master_release: bool) -> int:
    """
    Slave receives 8 bits MSB-first, sampling at SCL rising edge.
    During these 8 data bits, master must not drive SDA (must release).
    """
    val = 0
    for _ in range(8):
        await RisingEdge(dut.scl)
        if expect_master_release:
            assert int(dut.sda_oe.value) == 0, "Master must release SDA during slave sampling of master->slave bits"
        bit = int(dut.sda_i.value)
        val = ((val << 1) | bit) & 0xFF
        await FallingEdge(dut.scl)
    return val


async def slave_ack_address(dut, bus: OpenDrainBus, *, ack: bool) -> None:
    """
    Address ACK bit (9th clock):
    - Master must release SDA.
    - Slave drives ACK=0 (drive low) or NACK=1 (release).
    """
    # Ensure we start during SCL low phase before the ACK rising edge
    if int(dut.scl.value) == 1:
        await FallingEdge(dut.scl)

    bus.slave_drive_low = bool(ack)  # ACK => drive low, NACK => release (False)
    await RisingEdge(dut.scl)

    assert int(dut.sda_oe.value) == 0, "Master must release SDA during address ACK/NACK bit"

    # Hold through SCL high
    await FallingEdge(dut.scl)
    bus.slave_drive_low = False  # release after ACK bit


async def slave_drive_read_byte_and_check_master_ack(
    dut,
    bus: OpenDrainBus,
    data_byte: int,
    *,
    expect_master_ack: bool,
) -> None:
    """
    Slave drives 8 data bits, master samples at SCL rising edges.
    Then master drives ACK/NACK on 9th clock:
      - ACK => master drives low
      - NACK => master releases
    """
    # Drive 8 data bits MSB-first; only change during SCL low
    for bitpos in range(7, -1, -1):
        if int(dut.scl.value) == 1:
            await FallingEdge(dut.scl)

        bit = (data_byte >> bitpos) & 1
        bus.slave_drive_low = (bit == 0)

        await RisingEdge(dut.scl)
        # During read data bits, master must release SDA (slave owns it)
        assert int(dut.sda_oe.value) == 0, "Master must release SDA during slave-driven read data bits"
        await FallingEdge(dut.scl)

    # 9th clock: master ACK/NACK, slave releases SDA
    bus.slave_drive_low = False
    if int(dut.scl.value) == 1:
        await FallingEdge(dut.scl)

    await RisingEdge(dut.scl)

    if expect_master_ack:
        # ACK = 0 => master drives low
        assert int(dut.sda_oe.value) == 1, "Master must drive SDA low for ACK"
        assert int(dut.sda_i.value) == 0, "ACK bit must be low (0)"
    else:
        # NACK = 1 => master releases
        assert int(dut.sda_oe.value) == 0, "Master must release SDA for NACK"
        assert int(dut.sda_i.value) == 1, "NACK bit must be high (1)"

    await FallingEdge(dut.scl)


async def wait_done(dut, timeout_cycles: int = 200000) -> None:
    """Wait for done pulse with timeout."""
    for _ in range(timeout_cycles):
        await RisingEdge(dut.clk)
        if int(dut.done.value) == 1:
            return
    raise AssertionError("Timeout waiting for dut.done")


@cocotb.test()
async def i2c_read_1byte(dut):
    """READ 1 byte: expect NACK on the only byte, then STOP, rvalid once."""
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start(start_high=False))

    # Reset
    dut.rst_n.value = 0
    dut.cmd_start.value = 0
    dut.cmd_addr.value = 0x12
    dut.cmd_len.value = 1
    await Timer(200, units="ns")
    dut.rst_n.value = 1
    await Timer(200, units="ns")

    bus = OpenDrainBus(dut)
    cocotb.start_soon(bus.drive_loop())

    # Start command
    dut.cmd_start.value = 1
    await RisingEdge(dut.clk)
    dut.cmd_start.value = 0

    await wait_for_start(dut)

    # Address+R from master
    addr_byte = await recv_byte_from_master(dut, bus, expect_master_release=False)
    exp_addr_byte = ((int(dut.cmd_addr.value) & 0x7F) << 1) | 1
    assert addr_byte == exp_addr_byte, f"Address+R mismatch: got 0x{addr_byte:02X}, expected 0x{exp_addr_byte:02X}"

    # ACK address
    await slave_ack_address(dut, bus, ack=True)

    # Slave returns 1 byte
    expected = [0xA5]
    observed = []

    # Watch rvalid pulses
    async def rvalid_watcher():
        while len(observed) < 1:
            await RisingEdge(dut.clk)
            if int(dut.rvalid.value) == 1:
                observed.append(int(dut.rdata.value) & 0xFF)

    cocotb.start_soon(rvalid_watcher())

    await slave_drive_read_byte_and_check_master_ack(dut, bus, expected[0], expect_master_ack=False)

    await wait_for_stop(dut)
    await wait_done(dut)

    assert int(dut.ack_error.value) == 0, "ack_error should be 0 on successful read"
    assert observed == expected, f"Read bytes mismatch: observed={observed}, expected={expected}"


@cocotb.test()
async def i2c_read_2bytes_ack_then_nack(dut):
    """READ 2 bytes: expect ACK after byte0, NACK after byte1, then STOP."""
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start(start_high=False))

    # Reset
    dut.rst_n.value = 0
    dut.cmd_start.value = 0
    dut.cmd_addr.value = 0x2A
    dut.cmd_len.value = 2
    await Timer(200, units="ns")
    dut.rst_n.value = 1
    await Timer(200, units="ns")

    bus = OpenDrainBus(dut)
    cocotb.start_soon(bus.drive_loop())

    dut.cmd_start.value = 1
    await RisingEdge(dut.clk)
    dut.cmd_start.value = 0

    await wait_for_start(dut)

    addr_byte = await recv_byte_from_master(dut, bus, expect_master_release=False)
    exp_addr_byte = ((int(dut.cmd_addr.value) & 0x7F) << 1) | 1
    assert addr_byte == exp_addr_byte

    await slave_ack_address(dut, bus, ack=True)

    expected = [0x11, 0x22]
    observed = []

    async def rvalid_watcher():
        while len(observed) < 2:
            await RisingEdge(dut.clk)
            if int(dut.rvalid.value) == 1:
                observed.append(int(dut.rdata.value) & 0xFF)

    cocotb.start_soon(rvalid_watcher())

    # byte0 -> expect master ACK
    await slave_drive_read_byte_and_check_master_ack(dut, bus, expected[0], expect_master_ack=True)
    # byte1 -> expect master NACK
    await slave_drive_read_byte_and_check_master_ack(dut, bus, expected[1], expect_master_ack=False)

    await wait_for_stop(dut)
    await wait_done(dut)

    assert int(dut.ack_error.value) == 0
    assert observed == expected, f"Observed {observed}, expected {expected}"


@cocotb.test()
async def i2c_read_4bytes(dut):
    """READ 4 bytes: ACK ACK ACK NACK sequence required."""
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start(start_high=False))

    dut.rst_n.value = 0
    dut.cmd_start.value = 0
    dut.cmd_addr.value = 0x3C
    dut.cmd_len.value = 4
    await Timer(200, units="ns")
    dut.rst_n.value = 1
    await Timer(200, units="ns")

    bus = OpenDrainBus(dut)
    cocotb.start_soon(bus.drive_loop())

    dut.cmd_start.value = 1
    await RisingEdge(dut.clk)
    dut.cmd_start.value = 0

    await wait_for_start(dut)

    addr_byte = await recv_byte_from_master(dut, bus, expect_master_release=False)
    exp_addr_byte = ((int(dut.cmd_addr.value) & 0x7F) << 1) | 1
    assert addr_byte == exp_addr_byte

    await slave_ack_address(dut, bus, ack=True)

    expected = [0xDE, 0xAD, 0xBE, 0xEF]
    observed = []

    async def rvalid_watcher():
        while len(observed) < 4:
            await RisingEdge(dut.clk)
            if int(dut.rvalid.value) == 1:
                observed.append(int(dut.rdata.value) & 0xFF)

    cocotb.start_soon(rvalid_watcher())

    for i, b in enumerate(expected):
        await slave_drive_read_byte_and_check_master_ack(
            dut, bus, b, expect_master_ack=(i < len(expected) - 1)
        )

    await wait_for_stop(dut)
    await wait_done(dut)

    assert int(dut.ack_error.value) == 0
    assert observed == expected, f"Observed {observed}, expected {expected}"


@cocotb.test()
async def i2c_address_nack_sets_ack_error(dut):
    """Slave NACKs address: DUT must set ack_error, issue STOP, no rvalid pulses."""
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start(start_high=False))

    dut.rst_n.value = 0
    dut.cmd_start.value = 0
    dut.cmd_addr.value = 0x55
    dut.cmd_len.value = 3
    await Timer(200, units="ns")
    dut.rst_n.value = 1
    await Timer(200, units="ns")

    bus = OpenDrainBus(dut)
    cocotb.start_soon(bus.drive_loop())

    dut.cmd_start.value = 1
    await RisingEdge(dut.clk)
    dut.cmd_start.value = 0

    await wait_for_start(dut)

    addr_byte = await recv_byte_from_master(dut, bus, expect_master_release=False)
    exp_addr_byte = ((int(dut.cmd_addr.value) & 0x7F) << 1) | 1
    assert addr_byte == exp_addr_byte

    # NACK the address
    await slave_ack_address(dut, bus, ack=False)

    # Ensure no rvalid pulses appear for a while
    for _ in range(5000):
        await RisingEdge(dut.clk)
        assert int(dut.rvalid.value) == 0, "rvalid must not pulse when address is NACKed"

    await wait_for_stop(dut)
    await wait_done(dut)

    assert int(dut.ack_error.value) == 1, "ack_error must be set when address is NACKed"


def test_i2c_master_hidden_runner():
    sim = os.getenv("SIM", "icarus")

    proj_path = Path(__file__).resolve().parent.parent

    # IMPORTANT: package first for compilation order
    sources = [
        proj_path / "sources/i2c_pkg.sv",
        proj_path / "sources/i2c_bit_engine.sv",
        proj_path / "sources/i2c_shift.sv",
        proj_path / "sources/i2c_master_top.sv",
    ]

    runner = get_runner(sim)
    runner.build(
        sources=sources,
        hdl_toplevel="i2c_master_top",
        always=True,
    )

    runner.test(
        hdl_toplevel="i2c_master_top",
        test_module="test_i2c_master_hidden",
    )
