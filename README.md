# S32X_MiSTer — RetroAchievements Fork

This is a fork of the official [Sega 32X core for MiSTer](https://github.com/MiSTer-devel/S32X_MiSTer) with modifications to support **RetroAchievements** on MiSTer FPGA.

> **Status:** Experimental / Proof of Concept — works together with the [modified Main_MiSTer binary](https://github.com/odelot/Main_MiSTer).

## What's Different from the Original

The upstream S32X core emulates the Sega 32X add-on (with full Genesis/Mega Drive compatibility). This fork adds two new FPGA modules and modifies several existing files so the ARM side (Main_MiSTer) can read both 68K Work RAM and SH2 SDRAM for achievement evaluation. **No emulation logic was changed** — the core plays all games identically to the original.

### Added Files

| File | Purpose |
|------|--------|
| `rtl/ra_ram_mirror_s32x.sv` | Selective Address (Option C) state machine — reads the ARM-provided address list from DDRAM, fetches each value from BRAM (68K RAM) or DDR3 (SH2 SDRAM), and writes the result cache back to DDRAM |
| `rtl/ddram_arb_s32x.sv` | DDR3 bus arbiter — inserted as an interposer between `ddram.sv` and the physical DDR3 pins; grants the RA mirror read/write access when the primary master is idle |

### Modified Files

| File | Change |
|------|--------|
| `S32X.sv` | Wires RA BRAM read ports from the GEN module, inserts `ddram_arb_s32x` between `ddram.sv` and physical DDR3, instantiates `ra_ram_mirror_s32x` |
| `files.qip` | Adds `ra_ram_mirror_s32x.sv` and `ddram_arb_s32x.sv` to the Quartus project |
| GEN module (internal) | Exposes a secondary BRAM read port via `RA_BRAM_ADDR[14:0]`, `RA_BRAM_U[7:0]`, `RA_BRAM_L[7:0]` signals for 68K Work RAM |

## How the RAM Mirror Works

The Sega 32X exposes two RAM regions from two different physical backends:

| rcheevos Range | CPU Region | Size | Physical Source |
|---------------|-----------|------|-----------------|
| `0x00000–0x0FFFF` | 68K Work RAM | 64 KB | On-chip BRAM (GEN module Port B, 2-cycle latency) |
| `0x10000–0x4FFFF` | 32X SH2 SDRAM | 256 KB | DDR3 via `ddram_arb_s32x` interposer |

**Total exposed: 320 KB**

### Protocol: Selective Address (Option C)

Because the address space is too large for a full bulk copy, this core uses the Selective Address protocol (same as SNES, Genesis, GBA, and others):

1. The ARM binary (`achievements_s32x.cpp`) runs `rc_client_do_frame()` once in *collect mode* to discover which addresses rcheevos needs.
2. The ARM sorts and deduplicates the list, then writes it to DDRAM at offset `0x40000`.
3. The FPGA state machine reads the list each VBlank and routes every address:
   - Addresses `< 0x10000` → BRAM path (reads Port B of the GEN 68K Work RAM BRAM, 2-cycle pipeline).
   - Addresses `0x10000–0x4FFFF` → DDR3 path (steals an idle cycle from `ddram.sv` via the arbiter).
4. The FPGA writes the collected byte values into the cache at DDRAM offset `0x48000`.
5. The ARM reads the cache and feeds it to rcheevos for evaluation.

### DDR3 Arbiter Architecture

The S32X core already uses DDR3 (via `ddram.sv`) for SH2 SDRAM. Rather than modifying `ddram.sv` to add a new channel, `ddram_arb_s32x.sv` is placed as an **interposer**:

```
Physical DDR3 pins
        ↑
  ddram_arb_s32x  (clk_ram)
     /         \
ddram.sv     ra_ram_mirror_s32x
(primary)     (secondary, clk_sys toggle signals)
```

The arbiter passes all `ddram.sv` traffic transparently (`S_PASSTHRU` state) and steals the bus only when `ddram.sv` has no pending request and no burst is in flight. Clock domain crossing between `clk_sys` (RA mirror) and `clk_ram` (DDR3 controller) uses two-stage flip-flop synchronisers on all toggle req/ack signals.

### Endianness Handling

The SH2 processor is big-endian: byte 0 of a memory word is at bits [63:56] of the 64-bit DDR3 read. rcheevos expects little-endian byte order within each 16-bit halfword. The RA mirror corrects this with a byte-lane XOR:

```verilog
case (s32x_offset[2:0] ^ 3'd1)
  3'd0: fetch_byte <= rd_data[63:56];
  3'd1: fetch_byte <= rd_data[55:48];
  // ...
  3'd7: fetch_byte <= rd_data[7:0];
endcase
```

### DDRAM Layout

```
0x00000  Header:       magic "RACH" (32-bit) + flags (busy bit)
0x00008  Frame counter: increments each VBlank
0x00010  Debug word:   dispatch count, BRAM hit / DDR3 hit / out-of-range counters
0x40000  Address list  [ARM→FPGA]: addr_count (32-bit) + request_id (32-bit),
                       then up to 4096 addresses packed 2 per 64-bit word
0x48000  Value cache   [FPGA→ARM]: response_id (32-bit) + response_frame (32-bit),
                       then byte values packed 8 per 64-bit word
```

All data flows through shared DDRAM at ARM physical address **0x3D000000**.

### Architecture Diagram

```
┌─────────────────────────────────────────────────────┐
│              S32X FPGA Core                         │
│                                                     │
│  GEN module: 68K Work RAM (BRAM, Port B read)       │
│  ddram.sv:   SH2 SDRAM (DDR3 primary consumer)      │
└──────────────┬──────────────────────────────────────┘
               │  VBlank trigger
               ▼
┌─────────────────────────────────────────────────────┐
│   ra_ram_mirror_s32x.sv  (clk_sys)                  │
│   Reads address list from DDRAM 0x40000             │
│   Routes per address:                               │
│     < 0x10000 → BRAM Port B (68K Work RAM)          │
│     0x10000+  → DDR3 via ddram_arb_s32x             │
│   Writes value cache to DDRAM 0x48000               │
└──────────────┬──────────────────────────────────────┘
               │  DDRAM @ 0x3D000000 (shared)
               ▼
┌─────────────────────────────────────────────────────┐
│   Main_MiSTer ARM binary                            │
│   achievements_s32x.cpp — writes address list       │
│   ra_ramread.cpp — reads value cache                │
│   rcheevos — evaluates achievement conditions       │
└─────────────────────────────────────────────────────┘
```

## How to Try It

1. Download the latest S32X core binary (`S32X_*.rbf`) from the [Releases](https://github.com/odelot/S32X_MiSTer/releases) page.
2. Copy the `.rbf` file to `/media/fat/_Console/` on your MiSTer SD card (replacing or alongside the stock S32X core).
3. You will also need the **modified Main_MiSTer binary** from [odelot/Main_MiSTer](https://github.com/odelot/Main_MiSTer) — follow the setup instructions there to configure your RetroAchievements credentials.
4. Load a Sega 32X ROM (`.32x`) and open a game that has achievements on [retroachievements.org](https://retroachievements.org/).

## Building from Source

Open the project in Quartus Prime (use the same version as the upstream MiSTer S32X core) and compile. The `ra_ram_mirror_s32x.sv` and `ddram_arb_s32x.sv` files are already included in `files.qip`.

## Links

- Original S32X core: [MiSTer-devel/S32X_MiSTer](https://github.com/MiSTer-devel/S32X_MiSTer)
- Modified Main binary (required): [odelot/Main_MiSTer](https://github.com/odelot/Main_MiSTer)
- RetroAchievements: [retroachievements.org](https://retroachievements.org/)

---

# Original S32X Core Documentation

*Everything below is from the upstream [S32X_MiSTer](https://github.com/MiSTer-devel/S32X_MiSTer) README and applies unchanged to this fork.*

# [Sega 32X](https://en.wikipedia.org/wiki/32X) for [MiSTer Platform](https://github.com/MiSTer-devel/Main_MiSTer/wiki)
by [Sergey Dvodnenko](https://github.com/srg320)

[![Patreon](https://img.shields.io/website?label=patreon&logo=patreon&style=social&url=https%3A%2F%2Fwww.patreon.com%2Fsrg320%2F)](https://www.patreon.com/srg320)

## Status
* Most games are playable and run without major issues.
* Some bugs present with audio playback and graphics.
* Some features like cheats and game saves are not implemented at this time. Please don't submit an issue to request cheats or saves, this is known.

Check the issues page to see if the issue you are experiencing is already known before submitting a new issue.

## Region detection
Region detection is known to work for all commercially released 32X games except the following. These are not bugs, this is due to the way the games were developed and due to the nature of the region code system in Sega Genesis games in general:

* Shadow Squadron ~ Stellar Assault (USA, Europe) always initially boots to Europe region. The ROM header has "E" (Europe) for the region, so the developers set it to the european region, you will have to set the region manually.
* FIFA Soccer 96 (Europe) and Mortal Kombat II (Europe) boot to US/JP Region. Header has the "JUE" (Region-Free) region code, so just set your region priority.
