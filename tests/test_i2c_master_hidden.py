from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, FallingEdge, Timer, ReadOnly
from cocotb_tools.runner import get_runner


LANGUAGE = os.getenv("HDL_TOPLEVEL_LANG", "verilog").lower().strip()


def sig_int(sig, default=None) -> int:
    """Safe conversion of cocotb signal handle to int."""
    try:
        return int(sig.value)
    except Exception:
        return default


def dbg_snapshot(dut, tag="DBG") -> None:
    """Log a compact snapshot of key DUT pins/flags. Also tries to log dut.st if visible."""
    st = None
    try:
        st = sig_int(dut.st, default=None)  # only if the internal signal is visible
    except Exception:
        st = None

    cocotb.log.info(
        f"[{tag}] t={cocotb.utils.get_sim_time(units='ns')}ns "
        f"scl={sig_int(dut.scl)} sda_i={sig_int(dut.sda_i)} sda_oe={sig_int(dut.sda_oe)} "
        f"busy={sig_int(dut.busy)} done={sig_int(dut.done)} ack_error={sig_int(dut.ack_error)} "
        f"rvalid={sig_int(dut.rvalid)} rdata=0x{sig_int(dut.rdata, 0):02X} "
        f"{'' if st is None else f'st={st}'}"
    )


@dataclass
class OpenDrainBus:
    """Open-drain SDA model: line is low if master OR slave drives low, else high."""
    dut: object
    slave_drive_low: bool = False

    async def drive_loop(self):
        """
        Continuously resolve SDA line and drive dut.sda_i.

        NOTE: We assign dut.sda_i during ReadOnly to reduce edge/timing ambiguity.
        """
        while True:
            await ReadOnly()
            master_drive_low = sig_int(self.dut.sda_oe, 0) == 1  # master can only drive low
            line = 0 if (master_drive_low or self.slave_drive_low) else 1
            await Timer(1, units="ns")  # small time step to keep line updated
            self.dut.sda_i.value = line


async def wait_for_start(dut, timeout_cycles: int = 200_000) -> None:
    """Detect START: SDA 1->0 while SCL is high."""
    last_sda = sig_int(dut.sda_i, 1)
    for i in range(timeout_cycles):
        await RisingEdge(dut.clk)
        scl = sig_int(dut.scl, 0)
        sda = sig_int(dut.sda_i, 1)

        if (i % 20_000) == 0:
            dbg_snapshot(dut, tag="WAIT_START")

        if scl == 1 and last_sda == 1 and sda == 0:
            cocotb.log.info("START detected")
            return
        last_sda = sda

    dbg_snapshot(dut, tag="WAIT_START_TIMEOUT")
    raise AssertionError("Timeout waiting for START condition")


async def wait_for_stop(dut, timeout_cycles: int = 200_000) -> None:
    """Detect STOP: SDA 0->1 while SCL is high."""
    last_sda = sig_int(dut.sda_i, 1)
    for i in range(timeout_cycles):
        await RisingEdge(dut.clk)
        scl = sig_int(dut.scl, 0)
        sda = sig_int(dut.sda_i, 1)

        if (i % 20_000) == 0:
            dbg_snapshot(dut, tag="WAIT_STOP")

        if scl == 1 and last_sda == 0 and sda == 1:
            cocotb.log.info("STOP detected")
            return
        last_sda = sda

    dbg_snapshot(dut, tag="WAIT_STOP_TIMEOUT")
    raise AssertionError("Timeout waiting for STOP condition")


async def recv_byte_from_master(dut, bus: OpenDrainBus, *, expect_master_release: bool) -> int:
    """
    Slave receives 8 bits MSB-first, sampling at SCL rising edge.
    Optionally asserts that master releases SDA during these bits.
    """
    val = 0
    for bit_idx in range(8):
        await RisingEdge(dut.scl)
        if expect_master_release:
            assert sig_int(dut.sda_oe, 0) == 0, (
                "Master must release SDA during slave sampling of master->slave bits"
            )
        bit = sig_int(dut.sda_i, 1)
        val = ((val << 1) | bit) & 0xFF
        cocotb.log.debug(f"recv_bit[{bit_idx}]= {bit}")
        await FallingEdge(dut.scl)
    cocotb.log.info(f"Received byte from master: 0x{val:02X}")
    return val


async def slave_ack_address(dut, bus: OpenDrainBus, *, ack: bool) -> None:
    """
    Address ACK bit (9th clock):
    - Master must release SDA.
    - Slave drives ACK=0 (drive low) or NACK=1 (release).
    """
    if sig_int(dut.scl, 1) == 1:
        await FallingEdge(dut.scl)

    bus.slave_drive_low = bool(ack)  # ACK => drive low, NACK => release (False)

    await RisingEdge(dut.scl)
    dbg_snapshot(dut, tag="ADDR_ACK_EDGE")

    assert sig_int(dut.sda_oe, 0) == 0, "Master must release SDA during address ACK/NACK bit"

    await FallingEdge(dut.scl)
    bus.slave_drive_low = False


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
    cocotb.log.info(
        f"Slave driving byte 0x{data_byte:02X}, expecting master {'ACK' if expect_master_ack else 'NACK'}"
    )

    # 8 data bits
    for bitpos in range(7, -1, -1):
        if sig_int(dut.scl, 1) == 1:
            await FallingEdge(dut.scl)

        bit = (data_byte >> bitpos) & 1
        bus.slave_drive_low = (bit == 0)

        await RisingEdge(dut.scl)
        assert sig_int(dut.sda_oe, 0) == 0, "Master must release SDA during slave-driven read data bits"
        await FallingEdge(dut.scl)

    # 9th bit: master ACK/NACK, slave releases
    bus.slave_drive_low = False
    if sig_int(dut.scl, 1) == 1:
        await FallingEdge(dut.scl)

    await RisingEdge(dut.scl)
    dbg_snapshot(dut, tag="DATA_ACK_EDGE")

    if expect_master_ack:
        assert sig_int(dut.sda_oe, 0) == 1, "Master must drive SDA low for ACK"
        assert sig_int(dut.sda_i, 1) == 0, "ACK bit must be low (0)"
    else:
        assert sig_int(dut.sda_oe, 0) == 0, "Master must release SDA for NACK"
        assert sig_int(dut.sda_i, 1) == 1, "NACK bit must be high (1)"

    await FallingEdge(dut.scl)


async def wait_done(dut, timeout_cycles: int = 200_000) -> None:
    """Wait for done pulse with timeout."""
    for i in range(timeout_cycles):
        await RisingEdge(dut.clk)
        if (i % 20_000) == 0:
            dbg_snapshot(dut, tag="WAIT_DONE")
        if sig_int(dut.done, 0) == 1:
            cocotb.log.info("DONE observed")
            return
    dbg_snapshot(dut, tag="WAIT_DONE_TIMEOUT")
    raise AssertionError("Timeout waiting for dut.done")


@cocotb.test()
async def i2c_read_1byte(dut):
    """READ 1 byte: expect NACK on the only byte, then STOP, rvalid once."""
    cocotb.log.info("=== TEST: i2c_read_1byte ===")
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
    exp_addr_byte = ((sig_int(dut.cmd_addr, 0) & 0x7F) << 1) | 1
    assert addr_byte == exp_addr_byte, (
        f"Address+R mismatch: got 0x{addr_byte:02X}, expected 0x{exp_addr_byte:02X}"
    )

    await slave_ack_address(dut, bus, ack=True)

    expected = [0xA5]
    observed = []

    async def rvalid_watcher():
        while len(observed) < 1:
            await RisingEdge(dut.clk)
            if sig_int(dut.rvalid, 0) == 1:
                b = sig_int(dut.rdata, 0) & 0xFF
                observed.append(b)
                cocotb.log.info(f"rvalid pulse: rdata=0x{b:02X}")

    cocotb.start_soon(rvalid_watcher())

    await slave_drive_read_byte_and_check_master_ack(dut, bus, expected[0], expect_master_ack=False)

    await wait_for_stop(dut)
    await wait_done(dut)

    assert sig_int(dut.ack_error, 0) == 0, "ack_error should be 0 on successful read"
    assert observed == expected, f"Read bytes mismatch: observed={observed}, expected={expected}"


@cocotb.test()
async def i2c_read_2bytes_ack_then_nack(dut):
    """READ 2 bytes: expect ACK after byte0, NACK after byte1, then STOP."""
    cocotb.log.info("=== TEST: i2c_read_2bytes_ack_then_nack ===")
    clock = Clock(dut.clk, 10, unit="ns")
    cocotb.start_soon(clock.start(start_high=False))

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
    exp_addr_byte = ((sig_int(dut.cmd_addr, 0) & 0x7F) << 1) | 1
    assert addr_byte == exp_addr_byte

    await slave_ack_address(dut, bus, ack=True)

    expected = [0x11, 0x22]
    observed = []

    async def rvalid_watcher():
        while len(observed) < 2:
            await RisingEdge(dut.clk)
            if sig_int(dut.rvalid, 0) == 1:
                b = sig_int(dut.rdata, 0) & 0xFF
                observed.append(b)
                cocotb.log.info(f"rvalid pulse: rdata=0x{b:02X}")

    cocotb.start_soon(rvalid_watcher())

    await slave_drive_read_byte_and_check_master_ack(dut, bus, expected[0], expect_master_ack=True)
    await slave_drive_read_byte_and_check_master_ack(dut, bus, expected[1], expect_master_ack=False)

    await wait_for_stop(dut)
    await wait_done(dut)

    assert sig_int(dut.ack_error, 0) == 0
    assert observed == expected, f"Observed {observed}, expected {expected}"


@cocotb.test()
async def i2c_read_4bytes(dut):
    """READ 4 bytes: ACK ACK ACK NACK sequence required."""
    cocotb.log.info("=== TEST: i2c_read_4bytes ===")
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
    exp_addr_byte = ((sig_int(dut.cmd_addr, 0) & 0x7F) << 1) | 1
    assert addr_byte == exp_addr_byte

    await slave_ack_address(dut, bus, ack=True)

    expected = [0xDE, 0xAD, 0xBE, 0xEF]
    observed = []

    async def rvalid_watcher():
        while len(observed) < 4:
            await RisingEdge(dut.clk)
            if sig_int(dut.rvalid, 0) == 1:
                b = sig_int(dut.rdata, 0) & 0xFF
                observed.append(b)
                cocotb.log.info(f"rvalid pulse: rdata=0x{b:02X}")

    cocotb.start_soon(rvalid_watcher())

    for i, b in enumerate(expected):
        await slave_drive_read_byte_and_check_master_ack(
            dut, bus, b, expect_master_ack=(i < len(expected) - 1)
        )

    await wait_for_stop(dut)
    await wait_done(dut)

    assert sig_int(dut.ack_error, 0) == 0
    assert observed == expected, f"Observed {observed}, expected {expected}"


@cocotb.test()
async def i2c_address_nack_sets_ack_error(dut):
    """Slave NACKs address: DUT must set ack_error, issue STOP, no rvalid pulses."""
    cocotb.log.info("=== TEST: i2c_address_nack_sets_ack_error ===")
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
    exp_addr_byte = ((sig_int(dut.cmd_addr, 0) & 0x7F) << 1) | 1
    assert addr_byte == exp_addr_byte

    # NACK address
    await slave_ack_address(dut, bus, ack=False)

    # Ensure no rvalid pulses appear for a while
    for i in range(5000):
        await RisingEdge(dut.clk)
        if (i % 1000) == 0:
            dbg_snapshot(dut, tag="POST_NACK")
        assert sig_int(dut.rvalid, 0) == 0, "rvalid must not pulse when address is NACKed"

    await wait_for_stop(dut)
    await wait_done(dut)

    assert sig_int(dut.ack_error, 0) == 1, "ack_error must be set when address is NACKed"


def test_i2c_master_hidden_runner():
    """
    Pytest-visible runner. This is what makes pytest collect and execute the cocotb tests.
    If this function is missing or the file name doesn't match pytest patterns, you'll get 'collected 0 items'.
    """
    sim = os.getenv("SIM", "icarus")

    proj_path = Path(__file__).resolve().parent.parent

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
